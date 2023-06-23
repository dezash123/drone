use crate::math::vector3::Vector3;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio2, Gpio3};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::Error;
use hal::pac::I2C1;
use hal::{i2c::I2C, pac};
use rp2040_hal as hal;
const IMU_ADDRESS: u8 = 0x68;
pub struct IMU {
    pub a: Vector3<i16>,
    pub theta: Vector3<i16>,
    a_sd: Vector3<i16>,
    theta_sd: Vector3<i16>,
    i2c: I2C<
        I2C1,
        (
            Pin<Gpio2, Function<hal::gpio::I2C>>,
            Pin<Gpio3, Function<hal::gpio::I2C>>,
        ),
    >,
}
impl IMU {
    pub fn new(
        i2c1: pac::I2C1,
        sda_pin: Pin<Gpio2, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio3, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c1(i2c1, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        Self {
            a: Vector3::new(0, 0, 0),
            theta: Vector3::new(0, 0, 0),
            a_sd: Vector3::new(0, 0, 0),
            theta_sd: Vector3::new(0, 0, 0),
            i2c,
        }
    }
    pub fn az(&mut self) -> i16 {
        let mut buf_h: [u8; 1] = [0; 1];
        let mut buf_l: [u8; 1] = [0; 1];
        self.i2c.write_read(IMU_ADDRESS, &[0x3B], &mut buf_h);
        self.i2c.write_read(IMU_ADDRESS, &[0x3C], &mut buf_l);
        let be: [u8; 2] = [buf_h[0], buf_l[0]];
        i16::from_be_bytes(be)
    }
    fn raw_a(&mut self) -> Vector3<i16> {
        let mut buffer: [u8; 6] = [0; 6];

        self.i2c.write_read(IMU_ADDRESS, &[0x3B], &mut buffer[0]);
        self.i2c.write_read(IMU_ADDRESS, &[0x3C], &mut buffer[1]);
        self.i2c.write_read(IMU_ADDRESS, &[0x3B], &mut buffer[2]);
        self.i2c.write_read(IMU_ADDRESS, &[0x3C], &mut buffer[3]);
        self.i2c.write_read(IMU_ADDRESS, &[0x3B], &mut buffer[4]);
        self.i2c.write_read(IMU_ADDRESS, &[0x3C], &mut buffer[5]);

        Vector3::new(
            i16::from_be_bytes([buffer[0], buffer[1]]),
            i16::from_be_bytes([buffer[2], buffer[3]]),
            i16::from_be_bytes([buffer[4], buffer[5]]),
        )
    }
    fn raw_theta(&mut self) -> Vector3<i16> {}

    pub fn calibrate(&mut self) {
        self.i2c.write(IMU_ADDRESS, &[0x1C, 0b11100000]);
        self.i2c.write(IMU_ADDRESS, &[0x23, 0b00001000]);
        self.i2c.write(IMU_ADDRESS, &[0x6B, 0x01]).unwrap();
    }
    pub fn update() {}
}
