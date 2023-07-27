use crate::io::radio::{Radio, RadioCommand};
use crate::math::functions::{
    cartesian_to_polar, cartesian_to_polar_magnitude, cartesian_to_polar_theta,
};
use crate::sensors::imu::{Accelerometer, Gyroscope, IMUError, Sensor};
use crate::sensors::{
    distance::DistanceSensor,
    imu::{ICM_20948, MPU_6050},
};
use core::u16;
use cortex_m::delay::Delay;
use cortex_m::peripheral::SYST;
use defmt::info;
use embedded_hal::PwmPin;
use hal::pwm::{Channel, FreeRunning, InputHighRunning, Pwm0, Pwm1, Pwm2, Pwm3, Slices, A};
use hal::sio::Spinlock0 as Inputlock;
use hal::sio::Spinlock1 as Delaylock;
use hal::sio::Spinlock2 as Timerlock;
use hal::sio::Spinlock3 as Motorlock;

// i could use intercore fifo but its too scary
use hal::Timer;
use hal::{
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
};
use rp2040_hal as hal;

pub struct DroneCoreState {
    last_input: RadioCommand,
    last_acc: ([f32; 3], u64),
    last_gyr: ([f32; 3], u64),
    current_command: Option<DroneCommand>,
}
pub struct DronePeripheralState {}

pub enum DroneCommand {
    Hover,
    Land,
    Takeoff,
    Calibrate,
    PilotControl,
    FallOutOfTheSky, // probably better than going up
}

static mut CORESTATE: Option<DroneCoreState> = None;
static TIMER: Option<hal::Timer> = None;
static mut DELAY: Option<cortex_m::delay::Delay> = None;

static mut CORE1_STACK: Stack<4096> = Stack::new();

// arbitrary max values
const MAX_TWIST: f32 = 10.0; //dps
const MAX_TILT: f32 = 20.0; //degrees
const CAL_LENGTH: u32 = 100; //number of cycles

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,

    target_linear_velocity: [f32; 3],
    target_angular_velocity: [f32; 3],
    theta: [f32; 3],
    mu_theta: [f32; 3],
    a: [f32; 3],
    mu_a: [f32; 3],
    v: [f32; 3],
    p: [f32; 3], // no way this works
    prevgyr: u64,
    prevacc: u64,
    g: f32,
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
            target_linear_velocity: [0.0, 0.0, 5.0],
            target_angular_velocity: [0.0, 0.0, 0.0],
            theta: [0.0, 0.0, 0.0],
            prevgyr: 0,
            mu_theta: [0.0, 0.0, 0.0],
            a: [0.0, 0.0, 0.0],
            mu_a: [0.0, 0.0, 0.0],
            v: [0.0, 0.0, 0.0],
            p: [0.0, 0.0, 0.0],
            prevacc: 0,
            g: 0.0,
        }
    }
    fn core0_task() -> ! {
        loop {}
    }
    fn core1_task(imu: ICM_20948, radio: Radio) -> ! {
        let mut failed_imu = 0;
        let mut failed_radio = 0;
        let mut newimu = false;
        let mut newradio = false;
        loop {
            unsafe {
                let _dlock = Delaylock::claim();
                let _tlock = Timerlock::claim();
                newimu = match imu.update_all(&mut DELAY.unwrap(), &TIMER.unwrap()) {
                    Ok(()) => true,
                    Err(e) => {
                        failed_imu = 0;
                        false
                    }
                };
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
        }
    }
    fn fall_out_of_the_sky(&mut self) -> ! {
        let _mlock = Motorlock::claim();
        self.set_speeds([0; 4]);
        loop {
            panic!("AAAAAAAAAAAAAaaa");
        }
    }
    pub fn start(
        &mut self,
        fl: Channel<Pwm0, FreeRunning, A>,
        bl: Channel<Pwm1, FreeRunning, A>,
        br: Channel<Pwm2, FreeRunning, A>,
        fr: Channel<Pwm3, FreeRunning, A>,
        imu: ICM_20948,
        radio: Radio,
        syst: SYST,
        resets: pac::RESETS,
        delay_clock_hz: u32,
    ) -> ! {
        let mut sig_theta: [f64; 3] = [0.0, 0.0, 0.0];
        let mut sig_a: [f64; 3] = [0.0, 0.0, 0.0];
        unsafe {
            let _dlock = Delaylock::claim();
            let _tlock = Timerlock::claim();
            DELAY = Some(cortex_m::delay::Delay::new(syst, delay_clock_hz));
            imu.update_all(&mut DELAY.unwrap(), &TIMER.unwrap())
                .unwrap();
            let a = imu.get_acc();
            self.prevgyr = TIMER.unwrap().get_counter().ticks();

            for j in 0..CAL_LENGTH {
                imu.update_all(&mut DELAY.unwrap(), &TIMER.unwrap())
                    .unwrap();
                let new_dtheta = imu.get_gyr().0;
                let new_da = imu.get_acc().0;
                for i in 0..3 {
                    sig_theta[i] += new_dtheta[i] as f64;
                    sig_a[i] += new_da[i] as f64;
                }
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
    }
    fn calc_speeds(&self) -> [u16; 4] {
        [0, 0, 0, 0]
    }
    fn set_speeds(&mut self, speeds: [u16; 4]) {
        // clockwise starting at FL
        self.fl.set_duty(speeds[0]);
        self.fr.set_duty(speeds[1]);
        self.br.set_duty(speeds[2]);
        self.bl.set_duty(speeds[3]);
    }
    pub fn update_target(&mut self, linear: [f32; 3], angular: [f32; 3]) {
        self.target_angular_velocity = angular;
        self.target_linear_velocity = linear;
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
}
