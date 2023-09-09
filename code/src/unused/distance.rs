use cortex_m::delay::Delay;
use defmt::info;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio18, Gpio19};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::Error;
use hal::pac::I2C1;
use hal::{i2c::I2C, pac};
use rp2040_hal as hal;
pub enum DistanceSensorError {
    InitFailed,
}

pub struct DistanceSensor {
    i2c: I2C<
        I2C1,
        (
            Pin<Gpio18, Function<hal::gpio::I2C>>,
            Pin<Gpio19, Function<hal::gpio::I2C>>,
        ),
    >,
}

impl DistanceSensor {
    pub fn new(
        i2c1: pac::I2C1,
        sda_pin: Pin<Gpio18, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio19, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c1(i2c1, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        Self { i2c }
    }
    pub fn init(
        &mut self,
        delay: &mut cortex_m::delay::Delay,
        timer: &hal::Timer,
    ) -> Result<(), DistanceSensorError> {
        let mut buf: [u8; 1] = [0; 1];
        self.i2c.write_read(0x52, &[0x01, 0x0F], &mut buf);
        info!("{:#X}", buf);
        Ok(())
    }
}
