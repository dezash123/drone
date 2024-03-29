use defmt::*;
use defmt_rtt as _;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio24, Gpio25, Gpio4, Gpio5};
use hal::gpio::Function;
use hal::gpio::Pin;
use hal::i2c::Error as I2CError;
use hal::pac::I2C0;
use hal::{i2c::I2C, pac};
use panic_probe as _;

use rp2040_hal as hal;

pub trait Accelerometer {
    fn get_acc(&self) -> [f32; 3];
    fn update_raw_acc(&mut self) -> Result<(), IMUError>;
}

pub trait Gyroscope {
    fn get_gyr(&self) -> [f32; 3];
    fn update_raw_gyr(&mut self) -> Result<(), IMUError>;
}

pub trait TemperatureSensor {
    fn get_temp_c(&self) -> f32;
    fn update_raw_temp(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError>;
}

pub trait Magnetometer {
    fn get_mag(&self) -> [f32; 3];
    fn update_raw_mag(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError>;
}

pub trait Sensor {
    fn update_all(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError>;
}
#[derive(Debug)]
pub enum IMUHardwareType {
    Gyro,
    Accelerometer,
    Magnetometer,
    Temperature,
    Memory,
    Unknown,
}

#[derive(Debug)]
pub enum IMUError {
    ReadError(IMUHardwareType, u8, I2CError),
    WriteError(IMUHardwareType, u8, u8, I2CError),
    SetupError(&'static str),
    SyntaxError(&'static str, u8),
}

impl IMUError {
    pub fn info(self) {
        match self {
            IMUError::ReadError(typ, reg, err) => {
                IMUError::i2c_error_info(err);
                match typ {
                    IMUHardwareType::Gyro => info!("failed to read gyro data"),
                    IMUHardwareType::Accelerometer => {
                        info!("failed to read accelerometer data");
                    }
                    IMUHardwareType::Temperature => {
                        info!("failed to read temperature data")
                    }
                    _ => info!("failed to read from {0:#X}", reg),
                };
            }
            IMUError::WriteError(typ, data, reg, err) => {
                IMUError::i2c_error_info(err);
                match typ {
                    IMUHardwareType::Memory => {
                        info!("failed to switch from bank {0} to {1}", data, reg)
                    }
                    _ => info!("failed to write {0:#b} to register {1:#X}", data, reg),
                };
            }
            IMUError::SyntaxError(msg, arg) => info!("{0} for input {1}", msg, arg),
            IMUError::SetupError(msg) => info!("{}", msg),
        }
    }
    fn i2c_error_info(err: I2CError) {
        match err {
            I2CError::Abort(v) => {
                if v & 1 << 12 != 0 {
                    info!("ArbitrationLoss")
                } else if v & 1 << 7 != 0 {
                    info!("ABRT_SBYTE_ACKDET")
                }
                // ha::i2c::ErrorKind::Bus,
                // if v & 1<<6 != 0 // ABRT_HS_ACKDET
                // ha::i2c::ErrorKind::Bus,
                // if v & 1<<4 != 0 // ABRT_GCALL_NOACK
                // ha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
                // if v & 1<<3 != 0 // ABRT_TXDATA_NOACK
                // ha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Data),
                // if v & 1<<2 != 0 // ABRT_10ADDR2_NOACK
                // ha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
                // if v & 1<<1 != 0 // ABRT_10ADDR1_NOACK
                // ha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
                else if v & 1 << 0 != 0 {
                    // ABRT_7B_ADDR_NOACK
                    info!("no acknowledge from device")
                } else {
                    info!("Other abort")
                };
            }
            I2CError::AddressReserved(addr) => info!("address {0} out of range", addr),
            I2CError::AddressOutOfRange(addr) => info!("address {0} out of range", addr),
            I2CError::InvalidReadBufferLength => info!("Invalid read buffer length"),
            I2CError::InvalidWriteBufferLength => info!("Invalid write buffer length"),
            _ => info!("other error"),
        }
    }
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

const GYRO_SR8_DIV: u8 = 0x00;
const GYRO_CONFIG_1: u8 = 0x01;

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

pub enum AccelerometerSetting {
    r2g,
    r4g,
    r8g,
    r16g,
}

impl AccelerometerSetting {
    pub fn get_mult(&self) -> f32 {
        match self {
            Self::r2g => 2.0 / 32768.0,
            Self::r4g => 4.0 / 32768.0,
            Self::r8g => 8.0 / 32768.0,
            Self::r16g => 16.0 / 32768.0,
        }
    }
}

pub enum GyroSetting {
    r250dps,
    r500dps,
    r1000dps,
    r2000dps,
}

impl GyroSetting {
    pub fn get_mult(&self) -> f32 {
        match self {
            Self::r250dps => 250.0 / 32768.0,
            Self::r500dps => 500.0 / 32768.0,
            Self::r1000dps => 1000.0 / 32768.0,
            Self::r2000dps => 2000.0 / 32768.0,
        }
    }
}

pub struct ICM_20948 {
    accelerometer_range: AccelerometerSetting,
    gyro_range: GyroSetting,
    bank: u8,
    raw_acc: [u8; 6],
    last_acc: u64,
    raw_gyr: [u8; 6],
    last_gyr: u64,
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

impl ICM_20948 {
    pub fn new(
        accelerometer_range: AccelerometerSetting,
        gyro_range: GyroSetting,
        i2c0: pac::I2C0,
        sda_pin: Pin<Gpio24, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio25, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c0(i2c0, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        let mut firstbank: [u8; 1] = [69; 1];
        match i2c.write_read(IMU_ADDR, &[BANK_SEL], &mut firstbank) {
            Ok(()) => (),
            Err(e) => info!("no read imu"),
        };
        Self {
            accelerometer_range,
            gyro_range,
            bank: firstbank[0] << 2 >> 6,
            raw_acc: [0; 6],
            last_acc: 0,
            raw_gyr: [0; 6],
            last_gyr: 0,
            raw_temp: [0; 2],
            raw_mag: [0; 6],
            i2c,
        }
    }

    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        delay.delay_ms(100);
        self.switch_bank(0)?;
        self.imu_write(PWR_MGMT_1, 0x80)?; // reset
        delay.delay_ms(10);
        if self.imu_read(WHO_AM_I)? != CHIP_ID {
            return Err(IMUError::SetupError("IMU not found!!!"));
        }

        self.imu_write(PWR_MGMT_1, 0x01)?; // select best clock
        self.imu_write(PWR_MGMT_2, 0x00)?; // enable gyro and accelerometer

        self.switch_bank(2)?;
        // chatgpt said these were good values for NBW
        self.imu_write(GYRO_SR8_DIV, 0x00)?;
        // f = 1.125 kHz / (1 + div as u8)
        self.imu_write(GYRO_CONFIG_1, 0b00_111_00_1 | (match &self.gyro_range {
            GyroSetting::r250dps => 0,
            GyroSetting::r500dps => 1,
            GyroSetting::r1000dps => 2,
            GyroSetting::r2000dps => 3,
        } << 1))?;
        // 7:6 resv
        // 5:3 look at table for NBW, 111 = 375Hz NBW
        // 2:1 00 = +-250, 01 = 500, 10 = 1000, 11 = 2000
        // 0   1 = enable low pass

        self.imu_write(ACC_SMPLRT_DIV_1, 0x00)?; // msb
        self.imu_write(ACC_SMPLRT_DIV_2, 0x00)?;
        // f = 1.125 kHz / (1 + div as u16)
        self.imu_write(ACC_CONFIG, 0b00_011_00_1 | (match &self.accelerometer_range {
            AccelerometerSetting::r2g => 0,
            AccelerometerSetting::r4g => 1,
            AccelerometerSetting::r8g => 2,
            AccelerometerSetting::r16g => 3,
        } << 1))?;
        // 7:6 resv
        // 5:3 look at table for NBW, 011 = 69Hz NBW
        // 2:1 00 = +-2g, 01 = 4, 10 = 8, 11 = 16
        // 0   1 = enable low pass

        self.switch_bank(0)?;
        self.imu_write(INT_PIN_CFG, 0x30)?;

        self.switch_bank(3)?;
        self.imu_write(I2C_MST_CTRL, 0x4D)?; // maybe make 0b1001111
        self.imu_write(I2C_MST_DELAY_CTRL, 0x01)?; // maybe make 0

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
    fn switch_bank(&mut self, bank: u8) -> Result<(), IMUError> {
        if bank > 3 {
            return Err(IMUError::SyntaxError("invalid bank", bank));
        }
        if bank != self.bank {
            match self.imu_write(BANK_SEL, bank << 4) {
                Ok(()) => (),
                Err(e) => {
                    return Err(IMUError::WriteError(
                        IMUHardwareType::Memory,
                        self.bank,
                        bank,
                        match e {
                            IMUError::WriteError(_, _, _, r) => r,
                            _ => self::panic!(),
                        },
                    ));
                }
            };
            self.bank = bank;
        }
        Ok(())
    }

    #[inline(always)]
    fn imu_write(&mut self, register: u8, data: u8) -> Result<(), IMUError> {
        let result = match self.i2c.write(IMU_ADDR, &[register, data]) {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::WriteError(
                IMUHardwareType::Unknown,
                data,
                register,
                e,
            )),
        };
        result
    }

    #[inline(always)]
    fn imu_read(&mut self, register: u8) -> Result<u8, IMUError> {
        let mut buf: [u8; 1] = [0; 1];
        match self.i2c.write_read(IMU_ADDR, &[register], &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(IMUError::ReadError(IMUHardwareType::Unknown, register, e)),
        }
    }

    fn mag_ready(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<bool, IMUError> {
        Ok(self.mag_read(MAG_ST1, delay)? & 0x01 > 0)
    }
    fn trigger_mag_io(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        let user = self.imu_read(USER_CTRL)?;
        self.imu_write(USER_CTRL, user | 0x20)?;
        delay.delay_ms(5);
        self.imu_write(USER_CTRL, user)
    }
    fn mag_write(
        &mut self,
        register: u8,
        data: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        self.switch_bank(3)?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR)?;
        self.imu_write(I2C_SLV0_REG, register)?;
        self.imu_write(I2C_SLV0_DO, data)?;
        self.switch_bank(0)?;
        Ok(self.trigger_mag_io(delay)?)
    }
    fn mag_read(
        &mut self,
        register: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<u8, IMUError> {
        self.switch_bank(3)?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80)?;
        self.imu_write(I2C_SLV0_REG, register)?;
        self.imu_write(I2C_SLV0_DO, 0xff)?;
        self.imu_write(I2C_SLV0_CTRL, 0x80 | 1)?; // read 1 byte
        self.switch_bank(0)?;
        self.trigger_mag_io(delay)?;
        Ok(self.imu_read(EXT_SLV_SENS_DATA_00)?)
    }
    fn mag_read_bytes<const length: usize>(
        &mut self,
        register: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<[u8; length], IMUError> {
        if length > 24 {
            return Err(IMUError::SyntaxError(
                "invalid magnetometer read length!!",
                length as u8,
            ));
        }
        let mut buf: [u8; length] = [0; length];
        self.switch_bank(3)?;
        self.imu_write(I2C_SLV0_CTRL, (0x80 | 0x08 | length).try_into().unwrap())?;
        self.imu_write(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80)?;
        self.imu_write(I2C_SLV0_REG, register)?;
        self.imu_write(I2C_SLV0_DO, 0xff)?;
        self.switch_bank(0)?;
        self.trigger_mag_io(delay)?;

        match self
            .i2c
            .write_read(IMU_ADDR, &[EXT_SLV_SENS_DATA_00], &mut buf)
        {
            Ok(()) => Ok(buf),
            Err(e) => Err(IMUError::ReadError(
                IMUHardwareType::Magnetometer,
                register,
                e,
            )),
        }
    }
    #[inline]
    pub fn update_raw_temp(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.switch_bank(0)?;
        match self
            .i2c
            .write_read(IMU_ADDR, &[TEMP_START], &mut self.raw_temp)
        {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::ReadError(
                IMUHardwareType::Temperature,
                TEMP_START,
                e,
            )),
        }
    }
}

impl Accelerometer for ICM_20948 {
    fn update_raw_acc(&mut self) -> Result<(), IMUError> {
        self.switch_bank(0)?;
        let result = match self
            .i2c
            .write_read(IMU_ADDR, &[ACC_START], &mut self.raw_acc)
        {
            Ok(()) => Ok(()),
            Err(e) => {
                return Err(IMUError::ReadError(
                    IMUHardwareType::Accelerometer,
                    ACC_START,
                    e,
                ))
            }
        };
        // delay.delay_us(100);
        result
    }
    fn get_acc(&self) -> [f32; 3] {
        let mult = self.accelerometer_range.get_mult();
        // meters per sec
        [
            f32::from(i16::from_be_bytes([self.raw_acc[0], self.raw_acc[1]])) * mult,
            f32::from(i16::from_be_bytes([self.raw_acc[2], self.raw_acc[3]])) * mult,
            f32::from(i16::from_be_bytes([self.raw_acc[4], self.raw_acc[5]])) * mult,
        ]
    }
}

impl Gyroscope for ICM_20948 {
    fn update_raw_gyr(&mut self) -> Result<(), IMUError> {
        self.switch_bank(0)?;
        let result = match self
            .i2c
            .write_read(IMU_ADDR, &[GYR_START], &mut self.raw_gyr)
        {
            Ok(()) => Ok(()),
            Err(e) => return Err(IMUError::ReadError(IMUHardwareType::Gyro, GYR_START, e)),
        };
        // delay.delay_us(100);
        result
    }
    fn get_gyr(&self) -> [f32; 3] {
        let mult = self.gyro_range.get_mult();
        // degrees per sec
        [
            f32::from(i16::from_be_bytes([self.raw_gyr[0], self.raw_gyr[1]])) * mult,
            f32::from(i16::from_be_bytes([self.raw_gyr[2], self.raw_gyr[3]])) * mult,
            f32::from(i16::from_be_bytes([self.raw_gyr[4], self.raw_gyr[5]])) * mult,
        ]
    }
}
impl Sensor for ICM_20948 {
    fn update_all(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.update_raw_acc()?;
        self.update_raw_gyr()
    }
}

const MPU_ADDR: u8 = 0x68;
const MPU_ID: u8 = 0x68;
const MPU_WHOAMI: u8 = 0x75;
const MPU_GYR_START: u8 = 0x43;
const MPU_ACC_START: u8 = 0x3B;
const MPU_ACC_DIV: f32 = 4.0 * 9.81;
const MPU_GYR_DIV: f32 = 125.0;

pub struct MPU_6050 {
    raw_acc: [u8; 6],
    last_acc: u64,
    raw_gyr: [u8; 6],
    last_gyr: u64,
    raw_temp: [u8; 2],
    i2c: I2C<
        I2C0,
        (
            Pin<Gpio4, Function<hal::gpio::I2C>>,
            Pin<Gpio5, Function<hal::gpio::I2C>>,
        ),
    >,
}

impl MPU_6050 {
    // i was too lazy to make nice errors for this one
    pub fn new(
        i2c0: pac::I2C0,
        sda_pin: Pin<Gpio4, Function<hal::gpio::I2C>>,
        scl_pin: Pin<Gpio5, Function<hal::gpio::I2C>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let mut i2c = I2C::i2c0(i2c0, sda_pin, scl_pin, 400.kHz(), resets, 125_000_000.Hz());
        let mut firstbank: [u8; 1] = [69; 1];
        Self {
            raw_acc: [0; 6],
            last_acc: 0,
            raw_gyr: [0; 6],
            last_gyr: 0,
            raw_temp: [0; 2],
            i2c,
        }
    }
    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.write(0x6B, 0b10000000, delay)?;
        delay.delay_ms(200);
        self.write(0x6B, 0, delay)?;
        if self.read_out::<1>(MPU_WHOAMI)?[0] != MPU_ID {
            return Err(IMUError::SetupError("IMU not found!!!"));
        }
        self.write(0x37, 0b0110000, delay)?;
        self.write(0x1B, 0, delay)?; // +- 250 dps
        self.write(0x1C, 0b00001000, delay)?; // +- 4g

        Ok(())
    }

    fn read_out<const len: usize>(&mut self, reg: u8) -> Result<[u8; len], IMUError> {
        let mut buf: [u8; len] = [0; len];
        match self.i2c.write_read(MPU_ADDR, &[reg], &mut buf) {
            Ok(()) => Ok(buf),
            Err(e) => Err(IMUError::ReadError(IMUHardwareType::Unknown, reg, e)),
        }
    }

    #[inline(always)]
    fn read_buf<const len: usize>(&mut self, reg: u8, buf: &mut [u8; len]) -> Result<(), IMUError> {
        match self.i2c.write_read(MPU_ADDR, &[reg], buf) {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::ReadError(IMUHardwareType::Unknown, reg, e)),
        }
    }

    #[inline(always)]
    fn write(
        &mut self,
        reg: u8,
        data: u8,
        delay: &mut cortex_m::delay::Delay,
    ) -> Result<(), IMUError> {
        let results = match self.i2c.write(MPU_ADDR, &[reg, data]) {
            Ok(()) => Ok(()),
            Err(e) => Err(IMUError::WriteError(IMUHardwareType::Unknown, data, reg, e)),
        };
        delay.delay_us(100);
        results
    }
}

impl Accelerometer for MPU_6050 {
    fn get_acc(&self) -> [f32; 3] {
        // meters per sec
        [
            f32::from(i16::from_be_bytes([self.raw_acc[0], self.raw_acc[1]])) * MPU_ACC_DIV,
            f32::from(i16::from_be_bytes([self.raw_acc[2], self.raw_acc[3]])) * MPU_ACC_DIV,
            f32::from(i16::from_be_bytes([self.raw_acc[4], self.raw_acc[5]])) * MPU_ACC_DIV,
        ]
    }
    fn update_raw_acc(&mut self) -> Result<(), IMUError> {
        match self
            .i2c
            .write_read(MPU_ADDR, &[MPU_ACC_START], &mut self.raw_acc)
        {
            Ok(()) => (),
            Err(e) => {
                return Err(IMUError::ReadError(
                    IMUHardwareType::Accelerometer,
                    ACC_START,
                    e,
                ))
            }
        };
        Ok(())
    }
}

impl Gyroscope for MPU_6050 {
    fn get_gyr(&self) -> [f32; 3] {
        // degrees per sec
        [
            f32::from(i16::from_be_bytes([self.raw_gyr[0], self.raw_gyr[1]])) * MPU_GYR_DIV,
            f32::from(i16::from_be_bytes([self.raw_gyr[2], self.raw_gyr[3]])) * MPU_GYR_DIV,
            f32::from(i16::from_be_bytes([self.raw_gyr[4], self.raw_gyr[5]])) * MPU_GYR_DIV,
        ]
    }
    fn update_raw_gyr(&mut self) -> Result<(), IMUError> {
        match self
            .i2c
            .write_read(MPU_ADDR, &[MPU_GYR_START], &mut self.raw_gyr)
        {
            Ok(()) => (),
            Err(e) => return Err(IMUError::ReadError(IMUHardwareType::Gyro, GYR_START, e)),
        };

        Ok(())
    }
}

impl Sensor for MPU_6050 {
    fn update_all(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), IMUError> {
        self.update_raw_acc()?;
        self.update_raw_gyr()
    }
}
