#![no_std]
#![no_main]
mod io;
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

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const EXT_CLK_FREQ: u32 = 12000000;

#[rp2040_hal::entry]
fn main() -> ! {
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
    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut lcd_i2c = hal::I2C::i2c0(
        pac.I2C0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        led_pin.set_high().unwrap();
        delay.start(200.millis());
        let _ = nb::block!(delay.wait());
        led_pin.set_low().unwrap();
        delay.start(200.millis());
        let _ = nb::block!(delay.wait());
    }
}
