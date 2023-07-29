use crate::control::radio::{Radio, RadioCommand};
use crate::math::functions::{
    cartesian_to_polar, cartesian_to_polar_magnitude, cartesian_to_polar_theta,
};
use crate::sensors::imu::ICM_20948;
use crate::sensors::imu::{Accelerometer, Gyroscope, Sensor};
use core::u16;
use defmt::info;
use hal::pwm::{Channel, FreeRunning, Pwm0, Pwm1, Pwm2, Pwm3, A};
use hal::sio::Spinlock0 as CoreStateLock;

// i could use intercore fifo but im too lazy
use hal::multicore::{Core, Stack};
use rp2040_hal as hal;

#[derive(Clone, Copy)]
pub struct DroneCoreState {
    last_input: RadioCommand,
    last_acc: ([f32; 3], u64),
    last_gyr: ([f32; 3], u64),
    current_command: DroneCommand,
}

#[derive(Clone, Copy)]
pub struct DronePeripheralState {}

#[derive(Clone, Copy)]
pub enum DroneCommand {
    Hover,
    Land,
    Takeoff,
    Calibrate,
    PilotControl,
    FallOutOfTheSky, // probably better than going up
}

static mut CORESTATE: Option<DroneCoreState> = None;

static mut CORE1_STACK: Stack<4096> = Stack::new();

// arbitrary max values
const MAX_TWIST: f32 = 10.0; //dps
const MAX_TILT: f32 = 20.0; //degrees
const CAL_LENGTH: u32 = 100; //number of cycles
const RADIO_TEMPORARY_FAILURE_THRESHOLD: u16 = 100;
const RADIO_FULL_FAILURE_THRESHOLD: u16 = 2000;
const MAX_THROTTLE: u16 = 0x00FF;

//percent difference for manual control
const MAX_THROTTLE_DIFFERENCE: f32 = 0.3;

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,
    theta: [f32; 3],
    mu_theta: [f32; 3],
    a: [f32; 3],
    mu_a: [f32; 3],
    v: [f32; 3],
    p: [f32; 3], // no way this works
    prevgyr: u64,
    prevacc: u64,
    g: f32,
    last_acc_time: u64,
    last_gyr_time: u64,
}

impl FlightSystem {
    pub fn new(
        fl: Channel<Pwm0, FreeRunning, A>,
        bl: Channel<Pwm1, FreeRunning, A>,
        br: Channel<Pwm2, FreeRunning, A>,
        fr: Channel<Pwm3, FreeRunning, A>,
    ) -> Self {
        Self {
            fl,
            bl,
            br,
            fr,
            theta: [0.0, 0.0, 0.0],
            prevgyr: 0,
            mu_theta: [0.0, 0.0, 0.0],
            a: [0.0, 0.0, 0.0],
            mu_a: [0.0, 0.0, 0.0],
            v: [0.0, 0.0, 0.0],
            p: [0.0, 0.0, 0.0],
            prevacc: 0,
            g: 0.0,
            last_acc_time: 0,
            last_gyr_time: 0,
        }
    }
    fn core0_task(&mut self) -> ! {
        loop {
            let mut current_state = None;
            unsafe {
                let _clock = CoreStateLock::claim();
                current_state = CORESTATE.clone();
            }
            let current_state = current_state.unwrap();
            let a = current_state.last_acc.0;
            let mut dtheta = current_state.last_gyr.0;
            for i in 0..3 {
                dtheta[i] -= self.mu_theta[i];
            }
            let adt = current_state.last_acc.1 - self.last_acc_time;
            let gdt = current_state.last_gyr.1 - self.last_gyr_time;
            match current_state.current_command {
                DroneCommand::PilotControl => self.full_manual(current_state.last_input),
                _ => self.hover(),
            }
        }
    }
    fn full_manual(&mut self, command: RadioCommand) {
        // full manual control
        let mut speeds: [f32; 4] = [command.z_throttle; 4];
        speeds[0] += MAX_THROTTLE_DIFFERENCE * (command.x_throttle + command.y_throttle);
        speeds[1] += MAX_THROTTLE_DIFFERENCE * (command.x_throttle + command.y_throttle);
        speeds[2] += MAX_THROTTLE_DIFFERENCE * (command.x_throttle + command.y_throttle);
        speeds[3] += MAX_THROTTLE_DIFFERENCE * (command.x_throttle + command.y_throttle);
        let max = *speeds
            .iter()
            .max_by(|&a, &b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
            .unwrap_or(&0.0);
        if max == 0.0 {
            self.set_speeds([0.0; 4]);
        } else {
            for i in 0..4 {
                speeds[i] /= max;
            }
            self.set_speeds(speeds);
        }
    }
    fn hover(&mut self) {}
    fn fall_out_of_the_sky(&mut self) -> ! {
        self.set_speeds([0.0; 4]);
        loop {
            panic!("AAAAAAAAAAAAAaaa");
        }
    }
    pub fn start(
        &mut self,
        mut delay: cortex_m::delay::Delay,
        timer: hal::Timer,
        mut imu: ICM_20948,
        mut radio: Radio,
        core1: &mut Core,
    ) -> ! {
        let mut sig_theta: [f64; 3] = [0.0, 0.0, 0.0];
        let mut sig_a: [f64; 3] = [0.0, 0.0, 0.0];
        imu.update_all(&mut delay, &timer).unwrap();
        let a = imu.get_acc();
        self.prevgyr = timer.get_counter().ticks();

        for _ in 0..CAL_LENGTH {
            imu.update_all(&mut delay, &timer).unwrap();
            let new_dtheta = imu.get_gyr().0;
            let new_da = imu.get_acc().0;
            for i in 0..3 {
                sig_theta[i] += new_dtheta[i] as f64;
                sig_a[i] += new_da[i] as f64;
            }
        }
        for i in 0..3 {
            sig_theta[i] /= CAL_LENGTH as f64;
            self.mu_theta[i] = sig_theta[i] as f32;
            sig_a[i] /= CAL_LENGTH as f64;
            self.mu_a[i] = sig_a[i] as f32;
        }
        self.g = cartesian_to_polar_magnitude(self.mu_a);
        self.prevgyr = imu.get_gyr().1;
        self.prevacc = imu.get_acc().1;
        let _core1task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, || {
            Self::core1_task(delay, timer, imu, radio)
        });
        self.core0_task();
    }
    fn calc_speeds(&self) -> [u16; 4] {
        [0, 0, 0, 0]
    }
    fn set_speeds(&mut self, speeds: [f32; 4]) {
        // clockwise starting at FL
        let mut speedsu16: [u16; 4] = [0; 4]; //inefficient!!
        for i in 0..4 {
            speedsu16[i] = (speeds[i] * MAX_THROTTLE as f32) as u16;
        }
        info!(
            "--------\n{} {}\n{} {}\n--------",
            speeds[0], speeds[1], speeds[2], speeds[3]
        );
        // self.fl.set_duty(speedsu16[0]);
        // self.fr.set_duty(speedsu16[1]);
        // self.br.set_duty(speedsu16[2]);
        // self.bl.set_duty(speedsu16[3]);
    }
    #[inline(always)]
    fn theta_from_dt(&mut self, reading: ([f32; 3], u64)) {
        let dt: f32 = ((reading.1 - self.prevgyr) as f32) / 1000000.0;
        self.theta[0] += dt * (reading.0[0] - self.mu_theta[0]);
        self.theta[1] += dt * (reading.0[1] - self.mu_theta[1]);
        self.theta[2] += dt * (reading.0[2] - self.mu_theta[2]);
        self.prevgyr = reading.1;
    }
    #[inline(always)]
    fn p_from_da(&mut self, reading: ([f32; 3], u64)) {
        let dt: f32 = ((reading.1 - self.prevgyr) as f32) / 1000000.0;
        for i in 0..3 {
            let a = reading.0[i] - self.mu_a[i];
            self.a[i] = a;
            self.v[i] += dt * a;
            self.p[i] += dt * dt * a * 0.5f32 + self.v[i] * dt;
        }
        self.prevgyr = reading.1;
    }
    fn core1_task(
        mut delay: cortex_m::delay::Delay,
        timer: hal::Timer,
        mut imu: ICM_20948,
        mut radio: Radio,
    ) -> ! {
        let mut failed_imu: u8 = 0;
        let mut failed_radio: u16 = 0;
        let mut newimu = false;
        let mut newradio = false;
        loop {
            newimu = match imu.update_all(&mut delay, &timer) {
                Ok(()) => {
                    failed_imu = 0;
                    true
                }
                Err(e) => {
                    failed_imu += 1;
                    false
                }
            };
            newradio = match radio.read() {
                Ok(()) => {
                    failed_radio = 0;
                    true
                }
                Err(e) => {
                    failed_radio += 1;
                    false
                }
            };
            unsafe {
                let _clock = CoreStateLock::claim();
                if newimu {
                    CORESTATE.unwrap().last_acc = imu.get_acc();
                }
                if newradio {
                    CORESTATE.unwrap().last_input = radio.get_command();
                }
                if failed_radio > RADIO_FULL_FAILURE_THRESHOLD {
                    CORESTATE.unwrap().current_command = DroneCommand::Land;
                } else if failed_radio > RADIO_TEMPORARY_FAILURE_THRESHOLD {
                    CORESTATE.unwrap().current_command = DroneCommand::FallOutOfTheSky;
                } // cant call function because core1 doesnt own self
            }
        }
    }
}
