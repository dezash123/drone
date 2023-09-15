use crate::control::radio::{Radio, RadioCommand};
use crate::math::functions::{
    cartesian_to_polar, cartesian_to_polar_magnitude, cartesian_to_polar_theta,
};
use crate::sensors::imu::ICM_20948;
use crate::sensors::imu::{Accelerometer, Gyroscope, Sensor};
use core::u16;
use defmt::info;
use defmt::Format;
use embedded_hal::PwmPin;
use hal::pwm::{Channel, FreeRunning, Pwm0, Pwm1, Pwm2, Pwm3, A};
use hal::sio::Spinlock0 as CoreStateLock;

// i could use intercore fifo but im too lazy
use hal::multicore::{Core, Stack};
use rp2040_hal as hal;

#[derive(Clone, Copy, Debug, Format)]
pub struct DroneCoreState {
    last_input: RadioCommand,
    last_acc: ([f32; 3], u64),
    last_gyr: ([f32; 3], u64),
    current_command: DroneCommand,
}

#[derive(Clone, Copy)]
pub struct DronePeripheralState {}

#[derive(Clone, Copy, Format, Debug)]
pub enum DroneCommand {
    Hover,
    Land,
    Takeoff,
    Calibrate,
    FullManual,
    NormalControl,
    FallOutOfTheSky, // probably better than going up
}

static mut CORESTATE: Option<DroneCoreState> = None;

static mut CORE1_STACK: Stack<8192> = Stack::new();

// arbitrary max values
const MAX_TWIST: f32 = 10.0; //dps
const MAX_TILT: f32 = 20.0; //degrees
const CAL_LENGTH: u32 = 100; //number of cycles
const IMU_FAILURE_THRESHOLD: u8 = 100;
const RADIO_TEMPORARY_FAILURE_THRESHOLD: u16 = 100;
const RADIO_FULL_FAILURE_THRESHOLD: u16 = 2000;
// 1-2ms PWM
const MAX_THROTTLE: u16 = 0xCCCC;
const MIN_THROTTLE: u16 = 0x6666;
// "multishot" (30kHz pwm) couldn't get it to work
// const MAX_THROTTLE: u16 = 50000;
// const MIN_THROTTLE: u16 = 10000;

const KPX: f32 = 0.0;
const KIX: f32 = 0.0;
const KDX: f32 = 0.0;
const KPY: f32 = 0.0;
const KIY: f32 = 0.0;
const KDY: f32 = 0.0;
const KPTWIST: f32 = 0.0;
const KITWIST: f32 = 0.0;
const KDTWIST: f32 = 0.0;

//percent difference for manual control
const MAX_THROTTLE_DIFFERENCE: f32 = 0.1;

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,
    mu_theta: [f32; 3],
    mu_a: [f32; 3],
    prevgyr: u64,
    prevacc: u64,
    g: f32,
    last_acc_time: u64,
    last_gyr_time: u64,
    last_integral_x: f32,
    last_integral_y: f32,
    last_integral_twist: f32,
    last_error_x: f32,
    last_error_y: f32,
    last_error_twist: f32,
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
            prevgyr: 0,
            mu_theta: [0.0, 0.0, 0.0],
            mu_a: [0.0, 0.0, 0.0],
            prevacc: 0,
            g: 0.0,
            last_acc_time: 0,
            last_gyr_time: 0,
            last_error_x: 0.0,
            last_error_y: 0.0,
            last_integral_x: 0.0,
            last_integral_y: 0.0,
            last_integral_twist: 0.0,
            last_error_twist: 0.0,
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
        // self.set_speeds([1.0; 4]);
        // delay.delay_ms(4000);
        // self.set_speeds([0.0; 4]);

        imu.init(&mut delay).unwrap();

        let initial_command = radio.get_command();
        unsafe {
            let _clock = CoreStateLock::claim();
            CORESTATE = Some(DroneCoreState {
                last_input: initial_command,
                last_acc: imu.get_acc(),
                last_gyr: imu.get_gyr(),
                current_command: DroneCommand::FallOutOfTheSky,
            })
        }
        info!("asdf");
        let _core1task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, || {
            Self::core1_task(delay, timer, imu, radio)
        });
        self.core0_task();
    }

    fn core1_task(
        //read from stuff and update states
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
                    e.info();
                    false
                }
            };
            // newradio = match radio.read() {
            //     Ok(()) => {
            //         failed_radio = 0;
            //         true
            //     }
            //     Err(e) => {
            //         failed_radio += 0;
            //         false
            //     }
            // };
            unsafe {
                let mut newstate = CORESTATE.clone().unwrap();
                if newimu {
                    newstate.last_acc = imu.get_acc();
                    newstate.last_gyr = imu.get_gyr();
                } else if failed_imu > IMU_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::FallOutOfTheSky;
                }
                if newradio {
                    newstate.last_input = radio.get_command();
                    // info!("{}", newstate.last_input.mode_select);
                    newstate.current_command = match newstate.last_input.mode_select {
                        1 => DroneCommand::FullManual,
                        0 => DroneCommand::Calibrate,
                        2 => DroneCommand::FallOutOfTheSky,
                        _ => newstate.current_command,
                    }
                }
                // if failed_radio > RADIO_FULL_FAILURE_THRESHOLD {
                //     newstate.current_command = DroneCommand::Land;
                // } else
                if failed_radio > RADIO_TEMPORARY_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::FallOutOfTheSky;
                }
                let _clock = CoreStateLock::claim();
                CORESTATE = Some(newstate);
            }
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
            let adt = (current_state.last_acc.1 - self.last_acc_time) as f32 / 1000000.0;
            self.last_acc_time = current_state.last_acc.1;
            let tdt = (current_state.last_gyr.1 - self.last_gyr_time) as f32 / 1000000.0;
            self.last_gyr_time = current_state.last_gyr.1;
            let throttle = current_state.last_input.z_throttle;
            // info!("{}", throttle);
            match current_state.current_command {
                DroneCommand::FullManual => self.full_manual(current_state.last_input),
                DroneCommand::NormalControl => {
                    let desired_angle: [f32; 2] = [
                        current_state.last_input.x_throttle * MAX_TILT,
                        current_state.last_input.y_throttle * MAX_TILT,
                    ];
                    // ignored for now
                    let desired_twist: f32 = current_state.last_input.twist_throttle * MAX_TWIST;
                    self.normal_control(throttle, desired_angle, desired_twist, a, dtheta, adt, tdt)
                }
                DroneCommand::Hover => {
                    self.normal_control(throttle, [0.0; 2], 0.0, a, dtheta, adt, tdt)
                }
                DroneCommand::Calibrate => self.set_speeds([throttle; 4]),
                _ => self.fall_out_of_the_sky(),
            }
        }
    }

    fn normal_control(
        &mut self,
        throttle: f32,
        desired_angle: [f32; 2],
        desired_twist: f32,
        a: [f32; 3],
        dtheta: [f32; 3],
        adt: f32,
        tdt: f32,
    ) {
        let mut new_speeds = [throttle; 4];
        // probably need kalman filter and dtheta influence for v_xy > 0
        let measured_angle = cartesian_to_polar_theta(a, true);

        let error_x = desired_angle[0] - measured_angle[0];
        let error_y = desired_angle[1] - measured_angle[1];
        let error_twist = desired_twist - dtheta[2];

        let p_x = KPX * error_x;
        let p_y = KPY * error_x;
        let p_twist = KPTWIST * error_twist;

        // use setpoint velocity and measured omega instead????
        let d_x = KDX * (error_x - self.last_error_x) / adt;
        let d_y = KDY * (error_y - self.last_error_y) / adt;
        let d_twist = KDTWIST * (error_twist - self.last_error_twist) / tdt;

        self.last_error_x = error_x;
        self.last_error_y = error_y;
        self.last_error_twist = error_twist;

        let i_x = KIX * error_x * adt + self.last_integral_x;
        let i_y = KIY * error_y * adt + self.last_integral_y;
        let i_twist = KITWIST * error_twist * tdt + self.last_integral_twist;

        self.last_integral_x = i_x;
        self.last_integral_y = i_y;
        self.last_integral_twist = i_twist;

        let all_x = p_x + d_x + i_x;
        let all_y = p_y + d_y + i_y;
        let all_twist = p_twist + d_twist + i_twist;

        new_speeds[0] += all_x + all_y + all_twist;
        new_speeds[1] += -all_x + all_y - all_twist;
        new_speeds[2] += -all_x - all_y + all_twist;
        new_speeds[3] += all_x - all_y - all_twist;

        self.set_speeds(new_speeds);
    }

    fn set_speeds(&mut self, speeds: [f32; 4]) {
        // clockwise starting at Front Left
        let mut speedsu16: [u16; 4] = [0; 4]; //inefficient!!
        for i in 0..4 {
            speedsu16[i] = (speeds[i] * (MAX_THROTTLE - MIN_THROTTLE) as f32) as u16 + MIN_THROTTLE;
        }
        self.fl.set_duty(speedsu16[0]);
        self.fr.set_duty(speedsu16[1]);
        self.br.set_duty(speedsu16[2]);
        self.bl.set_duty(speedsu16[3]);
    }

    fn full_manual(&mut self, command: RadioCommand) {
        let mut speeds: [f32; 4] = [command.z_throttle; 4];

        speeds[0] += MAX_THROTTLE_DIFFERENCE
            * (command.x_throttle - command.y_throttle + command.twist_throttle);
        speeds[1] += MAX_THROTTLE_DIFFERENCE
            * (-command.x_throttle - command.y_throttle - command.twist_throttle);
        speeds[2] += MAX_THROTTLE_DIFFERENCE
            * (-command.x_throttle + command.y_throttle + command.twist_throttle);
        speeds[3] += MAX_THROTTLE_DIFFERENCE
            * (command.x_throttle + command.y_throttle - command.twist_throttle);
        let mut max = speeds[0];
        for i in 1..4 {
            if speeds[i] > max {
                max = speeds[i];
            }
        }
        if max == 0.0 {
            self.set_speeds([0.0; 4]);
        } else if max > 1.0 {
            for i in 0..4 {
                speeds[i] /= max;
            }
            self.set_speeds(speeds);
        } else {
            self.set_speeds(speeds);
        }
    }

    fn fall_out_of_the_sky(&mut self) {
        self.set_speeds([0.0; 4]);
    }
}
