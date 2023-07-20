use crate::sensors::imu::{Accelerometer, Gyroscope, IMUError, Sensor};
use crate::sensors::{
    distance::DistanceSensor,
    imu::{ICM_20948, MPU_6050},
};
use core::u16;
use cortex_m::delay::Delay;
use defmt::info;
use embedded_hal::PwmPin;
use hal::Timer;
use libm::atanf;
use rp2040_hal as hal;
use rp2040_hal::pwm::{Channel, FreeRunning, InputHighRunning, Pwm0, Pwm1, Pwm2, Pwm3, Slices, A};
const FR_PORT: u8 = 1;
const FL_PORT: u8 = 2;
const BR_PORT: u8 = 3;
const BL_PORT: u8 = 4;

const PI: f32 = 3.141592653589793238462643383279506939937f32; // how far ive memorized
const RAD2DEG: f32 = 180.0 / PI;
const DEG2RAD: f32 = PI / 180.0;
const LIFTOFF_SPEED: f32 = 5.0;

const CAL_LENGTH: u32 = 100;

pub struct FlightSystem {
    // front_left: Channel<Pwm0, FreeRunning, A>,
    // back_left: Channel<Pwm1, FreeRunning, A>,
    // front_right: Channel<Pwm2, FreeRunning, A>,
    // back_right: Channel<Pwm3, FreeRunning, A>,
    imu: ICM_20948,
    // distance: DistanceSensor,
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
}
impl FlightSystem {
    pub fn new(
        // p0: Channel<Pwm0, FreeRunning, A>,
        // p1: Channel<Pwm1, FreeRunning, A>,
        // p2: Channel<Pwm2, FreeRunning, A>,
        // p3: Channel<Pwm3, FreeRunning, A>,
        imu: ICM_20948,
        // distance: DistanceSensor,
    ) -> Self {
        Self {
            // front_left: p0,
            // back_left: p1,
            // front_right: p2,
            // back_right: p3,
            imu,
            // distance,
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
        }
    }
    pub fn init(
        &mut self,
        delay: &mut cortex_m::delay::Delay,
        timer: &hal::Timer,
    ) -> Result<(), IMUError> {
        self.imu.update_all(delay, timer)?;
        let a = self.imu.get_acc();
        let nums = FlightSystem::get_sf_orientation(a.0);
        self.prevgyr = timer.get_counter().ticks();
        let mut sig_theta: [f64; 3] = [0.0, 0.0, 0.0];
        let mut sig_a: [f64; 3] = [0.0, 0.0, 0.0];
        for j in 0..CAL_LENGTH {
            match self.imu.update_all(delay, timer) {
                Ok(()) => (),
                Err(e) => e.info(),
            };
            let new_dtheta = self.imu.get_gyr().0;
            let new_da = self.imu.get_acc().0;
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
        self.prevgyr = self.imu.get_gyr().1;
        self.prevacc = self.imu.get_acc().1;
        Ok(())
    }
    fn calc_speeds(&self) -> [u16; 4] {
        [0, 0, 0, 0]
    }
    // fn update(&mut self) {
    //     let speeds: [u16; 4] = Self::calc_speeds(self);
    //     self.front_left.set_duty(speeds[0]);
    //     self.front_right.set_duty(speeds[1]);
    //     self.back_right.set_duty(speeds[2]);
    //     self.back_left.set_duty(speeds[3]);
    // }
    pub fn update_target(&mut self, linear: [f32; 3], angular: [f32; 3]) {
        self.target_angular_velocity = angular;
        self.target_linear_velocity = linear;
    }
    pub fn run(&mut self, delay: &mut cortex_m::delay::Delay, timer: &hal::Timer) -> ! {
        let mut failrow: u8 = 0;
        loop {
            match self.imu.update_all(delay, timer) {
                Ok(()) => (),
                Err(e) => {
                    e.info();
                    failrow += 1;
                }
            };
            if failrow > 100 {
                panic!();
            }
            let gyr = self.imu.get_gyr();
            self.theta_from_dt(gyr);
            info!(
                "{}, {}, {}",
                self.theta[0] as i16, self.theta[1] as i16, self.theta[2] as i16
            );
        }
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
