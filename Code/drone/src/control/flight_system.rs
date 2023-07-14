use crate::sensors::{
    distance::DistanceSensor,
    imu::{ICM_20948, MPU_6050},
};
use core::u16;
use embedded_hal::PwmPin;
use rp2040_hal::pwm::{Channel, FreeRunning, InputHighRunning, Pwm0, Pwm1, Pwm2, Pwm3, Slices, A};

const FR_PORT: u8 = 1;
const FL_PORT: u8 = 2;
const BR_PORT: u8 = 3;
const BL_PORT: u8 = 4;

const PI: f32 = 3.141592653589793238462643383279506939937f32; // how far ive memorized
const RAD2DEG: f32 = 180.0 / PI;
const DEG2RAD: f32 = PI / 180.0;
const LIFTOFF_SPEED: f32 = 5.0;

pub struct FlightSystem {
    // front_left: Channel<Pwm0, FreeRunning, A>,
    // back_left: Channel<Pwm1, FreeRunning, A>,
    // front_right: Channel<Pwm2, FreeRunning, A>,
    // back_right: Channel<Pwm3, FreeRunning, A>,
    imu: MPU_6050,
    // distance: DistanceSensor,
    target_linear_velocity: [f32; 3],
    target_angular_velocity: [f32; 3],
}
impl FlightSystem {
    pub fn new(
        // p0: Channel<Pwm0, FreeRunning, A>,
        // p1: Channel<Pwm1, FreeRunning, A>,
        // p2: Channel<Pwm2, FreeRunning, A>,
        // p3: Channel<Pwm3, FreeRunning, A>,
        imu: MPU_6050,
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
        }
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
    pub fn run() {
        loop {}
    }
    pub fn get_orientation(acc: [f32; 3]) -> [f32; 2] {
        let r = acc[0] / acc[2];
        let ty = r + r * r * r * (1.0 + r * r / 20.0) / 6.0;
        let r = acc[1] / acc[2];
        let tx = r + r * r * r * (1.0 + r * r / 20.0) / 6.0;
        [tx * RAD2DEG, ty * RAD2DEG]
    }
}
