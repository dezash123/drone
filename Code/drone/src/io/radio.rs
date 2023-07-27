// FlySky iA6B ibus interface
// only rx for now
use cortex_m::delay::Delay;
use cortex_m::prelude::_embedded_hal_serial_Read;
use defmt::info;
use defmt::Format;
use fugit::HertzU32;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio0, Gpio1, Gpio16, Gpio17};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::I2C;
use hal::uart::ValidUartPinout;
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionUart, Pins},
    pac::{self, UART0},
    sio::Sio,
    uart::{
        self, DataBits, Parity, ReadError, ReadErrorType, Reader, StopBits, UartConfig,
        UartPeripheral,
    },
    watchdog::Watchdog,
    Clock,
};
use nb::Error;
use nb::Error::Other;
use rp2040_hal as hal;

pub enum RadioError {
    ChecksumError,
    ReadError(Option<hal::uart::ReadErrorType>),
}

pub struct Radio {
    reader: Reader<
        pac::UART0,
        (
            Pin<Gpio0, Function<hal::gpio::Uart>>,
            Pin<Gpio1, Function<hal::gpio::Uart>>,
        ),
    >,
    pub buf: [u8; 31],
}

impl Radio {
    pub fn new(
        uart0: pac::UART0,
        miso: Pin<Gpio0, Function<hal::gpio::Uart>>,
        mosi: Pin<Gpio1, Function<hal::gpio::Uart>>,
        resets: &mut pac::RESETS,
        peripheral_clock_freq: fugit::HertzU32,
    ) -> Self {
        let mut uart = UartPeripheral::new(uart0, (miso, mosi), resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                peripheral_clock_freq,
            )
            .unwrap();
        uart.set_fifos(true);
        Self {
            reader: uart.split().0,
            buf: [0; 31],
        }
    }
    pub fn read(&mut self) -> Result<(), RadioError> {
        let mut newdata: [u8; 31] = [0; 31];
        let first = match nb::block!(self.reader.read()) {
            Ok(n) => n,
            Err(e) => {
                return Err(RadioError::ReadError(Some(e)));
            }
        };
        if first != 0x20 {
            return Err(RadioError::ReadError(None));
        }
        match self.reader.read_full_blocking(&mut newdata) {
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
        return Ok(());
    }
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
        RadioCommand {
            z_throttle: channels[2],
            y_throttle: channels[1] * 2.0 - 1.0,
            x_throttle: channels[0] * 2.0 - 1.0,
            twist_throttle: channels[3] * 2.0 - 1.0,
            mode_select: channels[4],
            total_throttle: channels[5],
        }
    }
}

#[derive(Debug, Format)]
pub struct RadioCommand {
    z_throttle: f32,
    y_throttle: f32,
    x_throttle: f32,
    twist_throttle: f32,
    mode_select: f32,
    total_throttle: f32,
}
