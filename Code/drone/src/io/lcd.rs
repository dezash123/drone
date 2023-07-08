#[allow(unused)]
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio6, Gpio7};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::pac::I2C1;
use hal::{i2c::I2C, pac};
use rp2040_hal as hal;

const LCD_ADDRESS: u8 = 0x27;
const LCD_CLEARDISPLAY: u8 = 0x01;
const LCD_RETURNHOME: u8 = 0x02;
const LCD_ENTRYMODESET: u8 = 0x04;
const LCD_DISPLAYCONTROL: u8 = 0x08;
const LCD_CURSORSHIFT: u8 = 0x10;
const LCD_FUNCTIONSET: u8 = 0x20;
const LCD_SETCGRAMADDR: u8 = 0x40;
const LCD_SETDDRAMADDR: u8 = 0x80;
const LCD_ENTRYRIGHT: u8 = 0x00;
const LCD_ENTRYLEFT: u8 = 0x02;
const LCD_ENTRYSHIFTINCREMENT: u8 = 0x01;
const LCD_ENTRYSHIFTDECREMENT: u8 = 0x00;
const LCD_DISPLAYON: u8 = 0x04;
const LCD_DISPLAYOFF: u8 = 0x00;
const LCD_CURSORON: u8 = 0x02;
const LCD_CURSOROFF: u8 = 0x00;
const LCD_BLINKON: u8 = 0x01;
const LCD_BLINKOFF: u8 = 0x00;
const LCD_DISPLAYMOVE: u8 = 0x08;
const LCD_CURSORMOVE: u8 = 0x00;
const LCD_MOVERIGHT: u8 = 0x04;
const LCD_MOVELEFT: u8 = 0x00;
const LCD_8BITMODE: u8 = 0x10;
const LCD_4BITMODE: u8 = 0x00;
const LCD_2LINE: u8 = 0x08;
const LCD_1LINE: u8 = 0x00;
const LCD_5X10DOTS: u8 = 0x04;
const LCD_5X8DOTS: u8 = 0x00;
const LCD_BACKLIGHT: u8 = 0x08;
const LCD_NOBACKLIGHT: u8 = 0x00;
const EN: u8 = 0b00000100;
const RW: u8 = 0b00000010;
const RS: u8 = 0b00000001;
pub struct Lcd {
    i2c: I2C<
        I2C1,
        (
            Pin<Gpio6, Function<hal::gpio::I2C>>,
            Pin<Gpio7, Function<hal::gpio::I2C>>,
        ),
    >,
}

impl Lcd {
    pub fn new(
        i2c0: pac::I2C1,
        sda_pin: Pin<Gpio6, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio7, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c1(i2c0, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        Self { i2c }
    }
    pub fn init(&mut self, timer: &mut cortex_m::delay::Delay) {
        self.lcd_write(0x03, 0x00, timer);
        self.lcd_write(0x03, 0x00, timer);
        self.lcd_write(0x03, 0x00, timer);
        self.lcd_write(0x02, 0x00, timer);
        self.lcd_write(
            LCD_FUNCTIONSET | LCD_2LINE | LCD_5X8DOTS | LCD_4BITMODE,
            0x00,
            timer,
        );
        self.lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON, 0x00, timer);
        self.lcd_write(LCD_CLEARDISPLAY, 0x00, timer);
        self.lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0x00, timer);
        timer.delay_ms(200);
    }
    fn write_cmd(&mut self, data: u8) {
        self.i2c.write(LCD_ADDRESS, &[data]).unwrap();
    }
    fn lcd_strobe(&mut self, data: u8, timer: &mut cortex_m::delay::Delay) {
        self.write_cmd(data | EN | LCD_BACKLIGHT);
        timer.delay_us(1500);
        self.write_cmd((data & !EN) | LCD_BACKLIGHT);
        timer.delay_us(300);
    }
    fn lcd_write_four_bits(&mut self, data: u8, timer: &mut cortex_m::delay::Delay) {
        self.write_cmd(data | LCD_BACKLIGHT);
        self.lcd_strobe(data, timer);
    }
    fn lcd_write(&mut self, cmd: u8, mode: u8, timer: &mut cortex_m::delay::Delay) {
        self.lcd_write_four_bits(mode | (cmd & 0xF0), timer);
        self.lcd_write_four_bits(mode | ((cmd << 4) & 0xF0), timer);
    }
    pub fn lcd_display_string(&mut self, string: &str, timer: &mut cortex_m::delay::Delay) {
        for char in string.as_bytes().iter() {
            self.lcd_write(*char, RS, timer)
        }
    }
    pub fn clear(&mut self, timer: &mut cortex_m::delay::Delay) {
        self.lcd_write(LCD_CLEARDISPLAY, 0x00, timer);
        self.lcd_write(LCD_RETURNHOME, 0x00, timer);
    }
}
