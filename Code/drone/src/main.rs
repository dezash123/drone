#![no_std]
#![no_main]
mod control;
mod io;
mod math;
mod sensors;
use control::{flight_system, FlightSystem};
use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use fugit::RateExtU32;
use hal::clocks::Clock;
use hal::gpio::{Function, FunctionUart};
use hal::pwm::{InputHighRunning, Slices};
use hal::{pac, I2C};
use heapless::String;
use io::lcd::Lcd;
// use io::router::Server;
use panic_halt as _;
use rp2040_hal as hal;
use sensors::distance::DistanceSensor;
use sensors::imu::IMU;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const EXT_CLK_HZ: u32 = 12_000_000;
const MOTOR_HZ: u32 = 100_000;

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut lcd = Lcd::new(
        pac.I2C1,
        pins.gpio6.into_mode(),
        pins.gpio7.into_mode(),
        &mut pac.RESETS,
    );
    lcd.init(&mut delay);
    lcd.lcd_display_string("hello world", &mut delay);
    // loop {
    delay.delay_ms(1000);
    //     lcd.clear(&mut delay);
    //     lcd.lcd_display_string("hello", &mut delay);
    //     delay.delay_ms(1000);
    //     lcd.clear(&mut delay);
    //     lcd.lcd_display_string("world", &mut delay);
    // }
    // let mut imu = IMU::new(
    //     pac.I2C0,
    //     pins.gpio24.into_mode(),
    //     pins.gpio25.into_mode(),
    //     &mut pac.RESETS,
    // );
    let mut i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );
    let IMU_ADDRESS = 0x68;
    i2c.write(IMU_ADDRESS, &[0x1C, 0b11100000]);
    i2c.write(IMU_ADDRESS, &[0x23, 0b00001000]);
    i2c.write(IMU_ADDRESS, &[0x6B, 0x01]);
    // let mut server = Server::new(
    //     pac.UART0,
    //     pins.gpio16.into_mode::<FunctionUart>(),
    //     pins.gpio17.into_mode::<FunctionUart>(),
    //     &mut pac.RESETS,
    //     clocks.peripheral_clock.freq(),
    // );
    // let mut distance = DistanceSensor::new(
    //     pac.I2C1,
    //     pins.gpio18.into_mode(),
    //     pins.gpio19.into_mode(),
    //     &mut pac.RESETS,
    // );

    // let mut p0 = slices.pwm0;
    // let mut p1 = slices.pwm1;
    // let mut p2 = slices.pwm2;
    // let mut p3 = slices.pwm3;

    // let pwm_ratio: u8 = (clocks.system_clock.freq().raw() / MOTOR_HZ)
    //     .try_into()
    //     .unwrap();
    // p0.set_div_int(pwm_ratio);
    // p1.set_div_int(pwm_ratio);
    // p2.set_div_int(pwm_ratio);
    // p3.set_div_int(pwm_ratio);
    // p0.enable();
    // p1.enable();
    // p2.enable();
    // p3.enable();
    // let mut m0 = p0.channel_a;
    // let mut m1 = p1.channel_a;
    // let mut m2 = p2.channel_a;
    // let mut m3 = p3.channel_a;
    // let cp0 = m0.output_to(pins.gpio0);
    // let cp1 = m1.output_to(pins.gpio2);
    // let cp2 = m2.output_to(pins.gpio20);
    // let cp3 = m3.output_to(pins.gpio22);

    // let mut flight_system = FlightSystem::new(m0, m1, m2, m3, imu, distance);
    //lcd.init(&mut delay);
    //let mut led_pin = pins.gpio25.into_push_pull_output();
    //imu.calibrate();
    // let mut data = String::<32>::new();
    // use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
    // for i in 0..=127 {
    //     let mut readbuf: [u8; 1] = [0; 1];
    //     let result = i2c.read(i, &mut readbuf);
    //     if let Ok(d) = result {
    //         let _ = write!(data, "Dev @ {i:#?}");
    //     }
    // }
    // imu.init(&mut delay);
    loop {
        let mut buf: [u8; 4] = [0; 4];
        //let mut buf_l: [u8; 1] = [0; 1];
        i2c.write_read(0x68, &[0x3B], &mut buf);
        //self.i2c.write_read(IMU_ADDRESS, &[0x3C], &mut buf_l);
        let be: [u8; 2] = [buf[2], buf[3]];
        let results = i16::from_be_bytes(be);
        lcd.clear(&mut delay);
        let mut data = String::<32>::new();
        let _ = write!(data, "I am {results}");
        lcd.lcd_display_string(&data, &mut delay);
        delay.delay_ms(500);
    }
}
