#![no_std]
#![no_main]
mod control;
mod io;
mod math;
mod sensors;
mod tests;
use crate::sensors::imu::Accelerometer;
use control::{flight_system, FlightSystem};
use core::fmt::Write;
use cortex_m::asm::delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use embedded_hal::PwmPin;
use fugit::RateExtU32;
use hal::gpio::{Function, FunctionUart};
use hal::pwm::{InputHighRunning, Slices};
use hal::{
    clocks::init_clocks_and_plls,
    uart::{
        self, DataBits, Parity, ReadError, ReadErrorType, Reader, StopBits, UartConfig,
        UartPeripheral,
    },
    watchdog::Watchdog,
    Clock,
};
use hal::{pac, I2C};
use heapless::String;
use io::lcd::Lcd;
use io::radio;
use io::radio::Radio;
use io::router::Server;
use panic_probe as _;
use rp2040_hal as hal;
use rp2040_hal::multicore::{Multicore, Stack};
use sensors::distance::DistanceSensor;
use sensors::imu::IMUError;
use sensors::imu::IMUHardwareType;
use sensors::imu::Sensor;
use sensors::imu::ICM_20948;
use sensors::imu::MPU_6050;

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
    let sio = hal::Sio::new(pac.SIO);
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
    let mut timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut imu = ICM_20948::new(
        pac.I2C0,
        pins.gpio24.into_mode(),
        pins.gpio25.into_mode(),
        &mut pac.RESETS,
    );
    let mut p0 = slices.pwm0;
    let mut p1 = slices.pwm1;
    let mut p2 = slices.pwm2;
    let mut p3 = slices.pwm3;
    let pwm_ratio: u8 = (clocks.system_clock.freq().raw() / MOTOR_HZ)
        .try_into()
        .unwrap();
    p0.set_div_int(pwm_ratio);
    p1.set_div_int(pwm_ratio);
    p2.set_div_int(pwm_ratio);
    p3.set_div_int(pwm_ratio);
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
    let mut flight_system = FlightSystem::new();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    flight_system.start();
}
