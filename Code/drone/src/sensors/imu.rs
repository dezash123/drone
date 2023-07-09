use crate::math::vector3::Vector3;
use arrform::{arrform, ArrForm};
use cortex_m::asm::delay;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio24, Gpio25};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::pac::I2C0;
use hal::{i2c::I2C, pac};
use rp2040_hal as hal;

#[derive(Debug)]
enum IMUError {
    ReadError(&'a str),
    WriteError(&'a str),
    SetupError(&'a str),
}

const CHIP_ID: u8 = 0xEA;
const IMU_ADDR: u8 = 0x68;
const I2C_ADDR_ALT: u8 = 0x69;
const BANK_SEL: u8 = 0x7f;

const I2C_MST_ODR_CONFIG: u8 = 0x00;
const I2C_MST_CTRL: u8 = 0x01;
const I2C_MST_DELAY_CTRL: u8 = 0x02;
const I2C_SLV0_ADDR: u8 = 0x03;
const I2C_SLV0_REG: u8 = 0x04;
const I2C_SLV0_CTRL: u8 = 0x05;
const I2C_SLV0_DO: u8 = 0x06;
const EXT_SLV_SENS_DATA_00: u8 = 0x3B;

const GYRO_SMPLRT_DIV: u8 = 0x00;
const GYRO_CONFIG_1: u8 = 0x01;
const GYRO_CONFIG_2: u8 = 0x02;

const WHO_AM_I: u8 = 0x00;
const USER_CTRL: u8 = 0x03;
const PWR_MGMT_1: u8 = 0x06;
const PWR_MGMT_2: u8 = 0x07;
const INT_PIN_CFG: u8 = 0x0F;

const ACC_SMPLRT_DIV_1: u8 = 0x10;
const ACC_SMPLRT_DIV_2: u8 = 0x11;
const ACC_INTEL_CTRL: u8 = 0x12;
const ACC_WOM_THR: u8 = 0x13;
const ACC_CONFIG: u8 = 0x14;
const ACC_START: u8 = 0x2D;
const GYR_START: u8 = 0x33;

const TEMP_START: u8 = 0x39;
const TEMPERATURE_DEGREES_OFFSET: u8 = 21;
const TEMPERATURE_SENSITIVITY: f32 = 333.87;
const ROOM_TEMP_OFFSET: u8 = 21;

const MAG_I2C_ADDR: u8 = 0x0c;
const MAG_CHIP_ID: u8 = 0x09;
const MAG_WIA: u8 = 0x01;
const MAG_ST1: u8 = 0x10;
const MAG_ST1_DOR: u8 = 0b00000010; // Data overflow bit
const MAG_ST1_DRDY: u8 = 0b00000001; // Data self.ready bit
const MAG_HXL: u8 = 0x11;
const MAG_ST2: u8 = 0x18;
const MAG_ST2_HOFL: u8 = 0b00001000; // mag overflow bit
const MAG_CNTL2: u8 = 0x31;
const MAG_CNTL2_MODE: u8 = 0b00001111;
const MAG_CNTL2_MODE_OFF: u8 = 0;
const MAG_CNTL2_MODE_SINGLE: u8 = 1;
const MAG_CNTL2_MODE_CONT1: u8 = 2;
const MAG_CNTL2_MODE_CONT2: u8 = 4;
const MAG_CNTL2_MODE_CONT3: u8 = 6;
const MAG_CNTL2_MODE_CONT4: u8 = 8;
const MAG_CNTL2_MODE_TEST: u8 = 16;
const MAG_CNTL3: u8 = 0x32;

pub struct IMU {
    bank: u8,
    raw_acc: [u8; 6],
    raw_gyr: [u8; 6],
    raw_temp: [u8; 2],
    raw_mag: [u8; 6],
    i2c: I2C<
        I2C0,
        (
            Pin<Gpio24, Function<hal::gpio::I2C>>,
            Pin<Gpio25, Function<hal::gpio::I2C>>,
        ),
    >,
}
impl IMU {
    pub fn new(
        i2c0: pac::I2C0,
        sda_pin: Pin<Gpio24, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio25, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c0(i2c0, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        Self {
            bank: 69,
            raw_acc: [0; 6],
            raw_gyr: [0; 6],
            raw_temp: [0; 2],
            raw_mag: [0; 6],
            i2c,
        }
    }

    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.switch_bank(0, delay)?;
        if self.imu_read(WHO_AM_I)? != CHIP_ID {
            return Err(IMUError::SetupError("ICM-2069240 not found!!!"));
        }
        self.imu_write(PWR_MGMT_1, 0x80, delay)?; // reset
        delay.delay_ms(10);
        self.imu_write(PWR_MGMT_1, 0x01, delay)?; // select best clock
        self.imu_write(PWR_MGMT_2, 0x00, delay)?; // enable gyro and accelerometer

        self.switch_bank(2, delay)?;
        self.set_gyro_srate(100.0, delay)?;
        self.set_gyro_low_pass(true, 5, delay)?;
        self.set_gyro_full_scale(250, delay)?;

        self.set_acc_srate(125.0, delay)?;
        self.set_acc_low_pass(true, 5, delay)?;
        self.set_acc_full_scale(16, delay)?;

        self.switch_bank(0, delay)?;
        self.imu_write(INT_PIN_CFG, 0x30, delay)?;

        self.switch_bank(3, delay)?;
        self.imu_write(I2C_MST_CTRL, 0x4D, delay)?; // maybe make 0b1001111
        self.imu_write(I2C_MST_DELAY_CTRL, 0x01, delay)?; // maybe make 0

        if self.mag_read(MAG_WIA, delay)? != MAG_CHIP_ID {
            return Err(IMUError::SetupError("Unable to find magnetometer"));
        }

        self.mag_write(MAG_CNTL3, 0x01, delay)?; // mag reset
        while self.mag_read(MAG_CNTL3, delay)? == 0x01 {
            delay.delay_us(100);
        }
        Ok(())
    }

    #[inline(always)]
    fn switch_bank(
        &mut self,
        bank: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        if bank > 3 {
            return Err(IMUError::SetupError(
                arrform!(128, "bank cannot switch to {bank}; 0<=bank<=3").as_str(),
            ));
        }
        if bank != self.bank {
            match self.imu_write(BANK_SEL, bank << 4, delay) {
                Ok(()) => (),
                Err(e) => {
                    let arr = arrform!(
                        128,
                        "{e:#?}\nfailed to switch from bank {0} to {bank}",
                        self.bank
                    );
                    let errormsg: &str = arr.as_str();
                    return Err(IMUError::WriteError(errormsg));
                }
            };
            self.bank = bank;
        }
        Ok(())
    }

    #[inline(always)]
    fn imu_write(
        &mut self,
        register: u8,
        data: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        let result = match self.i2c.write(IMU_ADDR, &[register, data]) {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::WriteError(
                arrform!(128, "{e:#?}\nfailed to write {data:#b} to {register:#X}").as_str(),
            )),
        };
        delay.delay_us(100);
        result
    }

    #[inline(always)]
    fn imu_read(&mut self, register: u8) -> Result<u8, IMUError> {
        let mut buf: [u8; 1] = [0; 1];
        match self.i2c.write_read(IMU_ADDR, &[register], &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(IMUError::ReadError(
                arrform!(128, "{e:#?}\nfailed to read from {register:#X}").as_str(),
            )),
        }
    }

    fn set_gyro_srate(
        &mut self,
        rate_hz: f32,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(2, delay)?;
        // 125Hz sample rate -> 1.125 kHz / (1 + rate)
        let rate: u8 = (((1125.0f32 / rate_hz) - 1.0f32).clamp(0.0f32, 255.0f32) + 0.5f32) as u8;
        self.imu_write(GYRO_SMPLRT_DIV, rate, delay)
    }
    fn set_gyro_full_scale(
        &mut self,
        scale: u32,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(2, delay)?;
        let mut value = self.imu_read(GYRO_CONFIG_1)?;
        value &= 0b11111001;
        value |= match scale {
            250 => 0b00,
            500 => 0b01,
            1000 => 0b10,
            2000 => 0b11,
            _ => return Err(IMUError::SetupError("invalid gyro scale!")),
        } << 1;
        self.imu_write(GYRO_CONFIG_1, value, delay)
    }

    fn set_gyro_low_pass(
        &mut self,
        enabled: bool,
        mode: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        if mode > 7 {
            return Err(IMUError::SetupError("invalid gyro low pass mode!!"));
        }
        self.switch_bank(2, delay)?;
        let mut buf = self.imu_read(GYRO_CONFIG_1)?;
        buf &= 0b10001110;
        if enabled {
            buf |= 0b1;
        }
        buf |= (mode & 0x07) << 4;
        self.imu_write(GYRO_CONFIG_1, buf, delay)
    }

    fn set_acc_srate(
        &mut self,
        rate_hz: f32,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(2, delay)?;
        // 125Hz - 1.125 kHz / (1 + rate)
        let rate: u8 = (((1125.0f32 / rate_hz) - 1.0f32).clamp(0.0f32, 255.0f32) + 0.5f32) as u8;
        self.imu_write(ACC_SMPLRT_DIV_1, (rate >> 8) & 0xff, delay)?;
        self.imu_write(ACC_SMPLRT_DIV_2, rate & 0xff, delay)
    }

    fn set_acc_full_scale(
        &mut self,
        scale: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(2, delay)?;
        let mut value = self.imu_read(ACC_CONFIG)? & 0b11111001;
        value |= match scale {
            2 => 0b00,
            4 => 0b01,
            8 => 0b10,
            16 => 0b11,
            _ => return Err(IMUError::SetupError("invalid accelerometer scale!")),
        } << 1;
        self.imu_write(ACC_CONFIG, value, delay)
    }

    fn set_acc_low_pass(
        &mut self,
        enabled: bool,
        mode: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        if mode > 7 {
            return Err(IMUError::SetupError(
                "invalid accelerometer low pass mode!!!",
            ));
        }
        self.switch_bank(2, delay)?;
        let mut value = self.imu_read(ACC_CONFIG)? & 0b10001110;
        if enabled {
            value |= 0b1;
        }
        value |= (mode & 0x07) << 4;
        self.imu_write(ACC_CONFIG, value, delay)
    }

    //figure out what this all does
    fn mag_ready(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<bool, IMUError> {
        Ok(self.mag_read(MAG_ST1, delay)? & 0x01 > 0)
    }
    fn trigger_mag_io(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        let user = self.imu_read(USER_CTRL)?;
        self.imu_write(USER_CTRL, user | 0x20, delay)?;
        delay.delay_ms(5);
        self.imu_write(USER_CTRL, user, delay)
    }
    fn mag_write(
        &mut self,
        register: u8,
        data: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(3, delay)?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR, delay)?;
        self.imu_write(I2C_SLV0_REG, register, delay)?;
        self.imu_write(I2C_SLV0_DO, data, delay)?;
        self.switch_bank(0, delay)?;
        Ok(self.trigger_mag_io(delay)?)
    }
    fn mag_read(
        &mut self,
        register: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<u8, IMUError> {
        self.switch_bank(3, delay)?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80, delay)?;
        self.imu_write(I2C_SLV0_REG, register, delay)?;
        self.imu_write(I2C_SLV0_DO, 0xff, delay)?;
        self.imu_write(I2C_SLV0_CTRL, 0x80 | 1, delay)?; // read 1 byte
        self.switch_bank(0, delay)?;
        self.trigger_mag_io(delay)?;
        Ok(self.imu_read(EXT_SLV_SENS_DATA_00)?)
    }
    fn mag_read_bytes<const length: usize>(
        &mut self,
        register: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<[u8; length], IMUError> {
        if length > 24 {
            return Err(IMUError::SetupError(arrform!(256,
                "length {length} too long to read from magnetometer register {register:#X} in single transaction").as_str()));
        }
        let mut buf: [u8; length] = [0; length];
        self.switch_bank(3, delay)?;
        self.imu_write(
            I2C_SLV0_CTRL,
            (0x80 | 0x08 | length).try_into().unwrap(),
            delay,
        )?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80, delay)?;
        self.imu_write(I2C_SLV0_REG, register, delay)?;
        self.imu_write(I2C_SLV0_DO, 0xff, delay)?;
        self.switch_bank(0, delay)?;
        self.trigger_mag_io(delay)?;

        match self
            .i2c
            .write_read(IMU_ADDR, &[EXT_SLV_SENS_DATA_00], &mut buf)
        {
            Ok(()) => Ok(buf),
            Err(e) => Err(IMUError::ReadError(
                arrform!(128,
                "{e:#?}\nfailed to read {length} bytes starting at {register:#X} from magnetometer"
            )
                .as_str(),
            )),
        }
    }
    #[inline(always)]
    pub fn update_raw_acc(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.switch_bank(0, &mut delay)?;
        match self
            .i2c
            .write_read(IMU_ADDR, &[ACC_START], &mut self.raw_acc)
        {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::ReadError(
                arrform!(128, "{e:#?}\nfailed to read acceleration").as_str(),
            )),
        }
    }

    #[inline(always)]
    pub fn update_raw_gyr(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.switch_bank(0, delay)?;
        match self
            .i2c
            .write_read(IMU_ADDR, &[GYR_START], &mut self.raw_gyr)
        {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::ReadError(
                arrform!(128, "{e:#?}\nfailed to read gyro").as_str(),
            )),
        }
    }
    #[inline]
    pub fn update_raw_temp(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.switch_bank(0, delay)?;
        match self
            .i2c
            .write_read(IMU_ADDR, &[TEMP_START], &mut self.raw_temp)
        {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::ReadError(
                arrform!(128, "{e:#?}\nfailed to read temperature").as_str(),
            )),
        }
    }
    #[inline]
    pub fn get_acc(self) -> Vector3<f32> {
        Vector3::from(&[0.0, 0.0, 0.0])
    }

    pub fn update() {}
}
