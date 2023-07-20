use cortex_m::delay::Delay;
use defmt::info;
use fugit::HertzU32;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio0, Gpio1, Gpio16, Gpio17};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::I2C;
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionUart, Pins},
    pac::{self, UART0},
    sio::Sio,
    uart::{
        self, DataBits, Parity, ReadError, ReadErrorType, StopBits, UartConfig, UartPeripheral,
    },
    watchdog::Watchdog,
    Clock,
};
use nb::Error;
use nb::Error::Other;
use rp2040_hal as hal;
const WIFI_SSID: &str = "DRONE!!";
const WIFI_PASS: &str = "password";

pub struct Server {
    uart: UartPeripheral<
        hal::uart::Enabled,
        UART0,
        (
            Pin<Gpio0, Function<hal::gpio::Uart>>,
            Pin<Gpio1, Function<hal::gpio::Uart>>,
        ),
    >,
    readbuf: [u8; 512],

    dlen: usize,
}
impl Server {
    pub fn new(
        uart0: pac::UART0,
        miso: Pin<Gpio0, Function<hal::gpio::Uart>>,
        mosi: Pin<Gpio1, Function<hal::gpio::Uart>>,
        resets: &mut pac::RESETS,
        peripheral_clock_freq: fugit::HertzU32,
    ) -> Self {
        let uart = UartPeripheral::new(uart0, (miso, mosi), resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                peripheral_clock_freq,
            )
            .unwrap();
        Self {
            uart,
            readbuf: [0; 512],
            dlen: 0,
        }
    }
    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) {
        self.uart.write_full_blocking(b"AT+CWMODE=3\r\n");
        delay.delay_ms(1000);
        self.uart
            .write_full_blocking(b"AT+CWSAP=\"DRONE!\",\"password\",11,0,3\r\n");
        delay.delay_ms(1000);
        self.uart.write_full_blocking(b"AT+CIPMUX=1\r\n");
        delay.delay_ms(1000);
        self.uart.write_full_blocking(b"AT+CIPSERVER=1,80\r\n");
        delay.delay_ms(2000);
    }
    fn read(&mut self, last_command: &str) -> usize {
        self.dlen = last_command.len() + 5;
        let mut n = 1;
        while !self.uart.uart_is_readable() {
            info!("uart not readable {}", n);
            // delay.delay_ms(100);
            n = n + 1;
        }
        let num = match self.uart.read_raw(&mut self.readbuf) {
            Ok(size) => size,
            Err(e) => match e {
                Other(e) => {
                    info!("{}", e.discarded);
                    match e.err_type {
                        ReadErrorType::Overrun => panic!("overrun"),
                        ReadErrorType::Break => panic!("break"),
                        ReadErrorType::Parity => panic!("parity"),
                        ReadErrorType::Framing => panic!("framing"),
                    }
                }
                _ => panic!("not other?"),
            },
        };
        num
    }
}
