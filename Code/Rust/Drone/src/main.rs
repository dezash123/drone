#![no_std]
#![no_main]
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    loop {
        short(led_pin, count_down)
    }
}
fn short(led_pin: Pin<Gpio25, Output<PushPull>>, timer: CountDown<'_>) {
    led_pin.set_high().unwrap();
    count_down.start(200.millis());
    let _ = nb::block!(count_down.wait());
    led_pin.set_low().unwrap();
    count_down.start(200.millis());
    let _ = nb::block!(count_down.wait());
}
fn long(led_pin: Pin<Gpio25, Output<PushPull>>) {
    led_pin.set_high().unwrap();
    count_down.start(1000.millis());
    let _ = nb::block!(count_down.wait());
    led_pin.set_low().unwrap();
    count_down.start(1000.millis());
    let _ = nb::block!(count_down.wait());
}
