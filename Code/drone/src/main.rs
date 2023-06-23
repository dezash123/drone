#![no_std]
#![no_main]
mod io;
mod math;
mod sensors;
use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use hal::clocks::Clock;
use hal::pac;
use hal::{
    prelude::*,
    pwm::{InputHighRunning, Slices},
};
use heapless::String;
use io::lcd::Lcd;
use panic_halt as _;
use rp2040_hal as hal;
use sensors::imu::IMU;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const EXT_CLK_HZ: u32 = 12_000_000;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let clocks = hal::clocks::init_clocks_and_plls(
        EXT_CLK_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut lcd = Lcd::new(
        pac.I2C0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        &mut pac.RESETS,
    );
    let mut imu = IMU::new(
        pac.I2C1,
        pins.gpio2.into_mode(),
        pins.gpio3.into_mode(),
        &mut pac.RESETS,
    );
    lcd.init(&mut delay);
    let mut led_pin = pins.gpio25.into_push_pull_output();
    imu.calibrate();
    loop {
        let az = imu.az();
        let mut data = String::<32>::new();
        let _ = write!(data, "az:{az}");
        lcd.clear(&mut delay);
        lcd.lcd_display_string(&data, &mut delay);
        delay.delay_ms(500);
    }
}
