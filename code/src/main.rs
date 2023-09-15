#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
mod control;
mod math;
mod sensors;
use crate::sensors::imu::Accelerometer;
use crate::sensors::imu::Gyroscope;
use control::radio::Radio;
use control::FlightSystem;
use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::PwmPin;
use hal::pac;
use hal::pwm::Slices;
use hal::Clock;
use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::multicore::Multicore;
use sensors::imu::{Sensor, ICM_20948};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const EXT_CLK_HZ: u32 = 12_000_000;
const MOTOR_HZ: u32 = 1_000_000;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut sio = hal::Sio::new(pac.SIO);
    let slices = Slices::new(pac.PWM, &mut pac.RESETS);
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
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut imu = ICM_20948::new(
        pac.I2C0,
        pins.gpio24.into_mode(),
        pins.gpio25.into_mode(),
        &mut pac.RESETS,
    );
    let mut radio = Radio::new(
        pac.UART1,
        pins.gpio4.into_mode(),
        pins.gpio5.into_mode(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let mut p0 = slices.pwm0;
    let mut p1 = slices.pwm1;
    let mut p2 = slices.pwm2;
    let mut p3 = slices.pwm3;
    // standard 1-2 ms pwm
    let pwm_int: u8 = 4;
    let pwm_frac: u8 = 12;

    // multishot
    // let pwm_int: u8 = 0;
    // let pwm_frac: u8 = 1;

    p0.set_div_int(pwm_int);
    p1.set_div_int(pwm_int);
    p2.set_div_int(pwm_int);
    p3.set_div_int(pwm_int);
    p0.set_div_frac(pwm_frac);
    p1.set_div_frac(pwm_frac);
    p2.set_div_frac(pwm_frac);
    p3.set_div_frac(pwm_frac);
    p0.enable();
    p1.enable();
    p2.enable();
    p3.enable();
    let mut m0 = p0.channel_a;
    let mut m1 = p1.channel_a;
    let mut m2 = p2.channel_a;
    let mut m3 = p3.channel_a;
    let _ = m0.output_to(pins.gpio0);
    let _ = m1.output_to(pins.gpio2);
    let _ = m2.output_to(pins.gpio20);
    let _ = m3.output_to(pins.gpio22);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let mut flight_system = FlightSystem::new(m0, m1, m2, m3);
    flight_system.start(delay, timer, imu, radio, core1);
}
