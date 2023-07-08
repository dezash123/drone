use fugit::HertzU32;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio16, Gpio17};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::I2C;
use rp2040_hal as hal;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionUart, Pins},
    pac::{self, UART0},
    sio::Sio,
    uart::{self, DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
    Clock,
};
const WIFI_SSID: &str = "DESMOD DRONE!!";
const WIFI_PASS: &str = "balls";

pub struct Server {
    uart: UartPeripheral<
        hal::uart::Enabled,
        UART0,
        (
            Pin<Gpio16, Function<hal::gpio::Uart>>,
            Pin<Gpio17, Function<hal::gpio::Uart>>,
        ),
    >,
}
impl Server {
    pub fn new(
        uart0: pac::UART0,
        miso: Pin<Gpio16, Function<hal::gpio::Uart>>,
        mosi: Pin<Gpio17, Function<hal::gpio::Uart>>,
        resets: &mut pac::RESETS,
        peripheral_clock_freq: fugit::HertzU32,
    ) -> Self {
        let uart = UartPeripheral::new(uart0, (miso, mosi), resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                peripheral_clock_freq,
            )
            .unwrap();
        Self { uart }
    }
}
