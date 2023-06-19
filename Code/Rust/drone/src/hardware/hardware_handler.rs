use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::timer::{Cancel, CountDown};
use fugit::{ExtU32, RateExtU32};
use hal::gpio::bank0::{Gpio0, Gpio1};
use hal::gpio::Function;
use hal::i2c::{SclPin, SdaPin};
use hal::pac::I2C0;
use hal::{
    gpio::{Pin, Pins},
    i2c::I2C,
    pac, Sio,
};
use io::lcd;
use panic_halt as _;
use rp2040_hal as hal;

pub struct Hardware_Handler {
    pac: pac::Peripherals,
    core: pac::CorePeripherals,
    watchdog: hal::Watchdog,
    clocks: hal::Clocks,
    timer: hal::Timer,
    delay: hal::timer::CountDown,
}
impl Hardware_Handler {
    fn new() -> Result<Self, InitErr> {
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            EXT_CLK_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS);
        let mut delay = timer.count_down();
        Self {
            pac,
            core,
            watchdog,
            clocks,
            timer,
            delay,
        }
    }
}

pub enum DataRequest {
    Read,
    Write,
    ReadWrite,
}
pub enum HardwareType {
    I2C,
    UART,
    SPI,
    ADC,
    PWM,
    GPIO,
    Clock,
}
pub trait HardwareRequest {
    fn device() -> HardwareType;
    fn process_request() -> Result<Optional<&[u8]>, Error>;
}
pub struct I2CRequest {
    request_type: DataRequest,
}
impl HardwareRequest for I2CRequest {
    fn device() -> HardwareType {
        HardwareType.I2C
    }
}
