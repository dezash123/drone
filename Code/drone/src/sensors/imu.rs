use std::io::Error;

use crate::math::vector3::Vector3;
use rp2040_hal::gpio::DynPin;
use rp2040_hal::i2c::{Peripheral, I2C};
use rp_pico::pac::I2C1;
const IMU_SDA: u8 = 24;
const IMU_SCL: u8 = 25;
pub struct IMU {
    pub a: Vector3<f32>,
    pub theta: Vector3<f32>,
    pub time: u128,
    a_sd: Vector3<f32>,
    theta_sd: Vector3<f32>,
    pub g: f32,
    hardware: I2C<I2C1, (, DynPin), Peripheral>,
}
impl IMU {
    pub fn new(call_time: u128) -> Self {
        Self {
            a: Vector3::new(0.0, 0.0, 0.0),
            theta: Vector3::new(0.0, 0.0, 0.0),
            time: call_time,
        }
    }
    fn raw_a(&self) -> (Vector3<Result<f32, Error>>, u128) {
        let mut readbuf: [u8; 1] = [0; 1];
        self.hardware.read(0x69)
    }
    fn raw_theta() -> (Vector3<Result<f32>>, u128) {
        (0.0, 0)
    }
    pub fn calibrate(time: u128) {}
    pub fn update() {}
}
