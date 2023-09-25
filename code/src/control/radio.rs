// FlySky iA6B ibus interface
// only rx for now
// TODO: program will freeze if radio is disconnected; should crash (very bad)
use cortex_m::prelude::_embedded_hal_serial_Read;
use defmt::info;
use defmt::Format;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio4, Gpio5};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::{
    pac,
    uart::{DataBits, Reader, StopBits, UartConfig, UartPeripheral},
};
use rp2040_hal as hal;

pub enum RadioError {
    ChecksumError,
    ReadError(Option<hal::uart::ReadErrorType>),
    NoNewData,
}

pub struct Radio {
    uart: UartPeripheral<
        hal::uart::Enabled,
        pac::UART1,
        (
            Pin<Gpio4, Function<hal::gpio::Uart>>,
            Pin<Gpio5, Function<hal::gpio::Uart>>,
        ),
    >,
    pub buf: [u8; 31],
}

impl Radio {
    pub fn new(
        uart: pac::UART1,
        mosi: Pin<Gpio4, Function<hal::gpio::Uart>>,
        miso: Pin<Gpio5, Function<hal::gpio::Uart>>,
        resets: &mut pac::RESETS,
        peripheral_clock_freq: fugit::HertzU32,
    ) -> Self {
        // language server = dumb because this works
        let mut uart = UartPeripheral::new(uart, (mosi, miso), resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                peripheral_clock_freq,
            )
            .unwrap();
        uart.set_fifos(true);
        Self { uart, buf: [0; 31] }
    }
    pub fn read(&mut self) -> Result<(), RadioError> {
        let mut newdata: [u8; 31] = [0; 31];
        // if self.uart.uart_is_readable() {
        let mut first = match nb::block!(self.uart.read()) {
            Ok(n) => n,
            Err(e) => {
                return Err(RadioError::ReadError(Some(e)));
            }
        };
        while first != 0x20 {
            first = match nb::block!(self.uart.read()) {
                Ok(n) => n,
                Err(e) => {
                    return Err(RadioError::ReadError(Some(e)));
                }
            }
        }
        match self.uart.read_full_blocking(&mut newdata) {
            Ok(()) => (),
            Err(e) => return Err(RadioError::ReadError(Some(e))),
        };
        let mut checksum: u16 = 0xffdf;
        for i in &newdata[0..29] {
            checksum -= *i as u16;
        }
        if checksum != u16::from_be_bytes([newdata[30], newdata[29]]) {
            return Err(RadioError::ChecksumError);
        }
        self.buf = newdata;
        Ok(())
        // } else {
        //     Err(RadioError::NoNewData)
        // }
    }
    #[inline(always)]
    pub fn get_command(&self) -> RadioCommand {
        let mut channels: [f32; 6] = [0.0; 6];
        for i in 1..7 {
            let cval = u16::from_be_bytes([self.buf[i * 2], self.buf[i * 2 - 1]]);
            channels[i - 1] = if cval < 1000 {
                0.0
            } else if cval > 2000 {
                1.0
            } else {
                (cval - 1000) as f32 / 1000.0
            };
        }
        if channels[2] > 0.95 {
            channels[2] = 1.0;
        }
        RadioCommand {
            z_throttle: channels[2],
            y_throttle: (channels[1] * 2.0 - 1.0),
            x_throttle: (channels[0] * 2.0 - 1.0),
            twist_throttle: (channels[3] * 2.0 - 1.0),
            mode_select: (channels[4] * 2.0) as u8,
            aux: channels[5],
        }
    }
}

#[derive(Debug, Format, Clone, Copy)]
pub struct RadioCommand {
    pub z_throttle: f32,
    pub y_throttle: f32,
    pub x_throttle: f32,
    pub twist_throttle: f32,
    pub mode_select: u8,
    pub aux: f32,
}
