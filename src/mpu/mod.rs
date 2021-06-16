use crate::logger::Logger;
use alloc::sync::Arc;
use esp32_hal::i2c::{self, Error};
use xtensa_lx::mutex::{SpinLockMutex, Mutex};
use esp32_hal::target::{ I2C0, DPORT};
use esp32_hal::gpio::{OutputPin, InputPin};
use core::borrow::Borrow;
use esp32_hal::delay::Delay;
use esp32_hal::hal::blocking::delay::DelayMs;
mod bits;

/// Describes a bit block from bit number 'bit' to 'bit'+'length'
pub struct BitBlock {
    pub bit: u8,
    pub length: u8
}

/// Slave address of Mpu6050
pub const SLAVE_ADDR: u8 = 0x68;
/// Internal register to check slave addr
pub const WHOAMI: u8 = 0x75;

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 107: Power Management 1
pub struct PWR_MGMT_1;

impl PWR_MGMT_1 {
    /// Base Address
    pub const ADDR: u8 = 0x6b;
    /// Device Reset bit
    pub const DEVICE_RESET: u8 = 7;
    /// Sleep mode bit (Should be called "Low Power", doesn't actually sleep)
    pub const SLEEP: u8 = 6;
    /// Cycle bit for wake operations
    pub const CYCLE: u8 = 5;
    /// Temperature sensor enable/disable bit
    pub const TEMP_DIS: u8 = 3;

}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Accelerometer High Pass Filter Values
pub enum ACCEL_HPF {
    /// Cut off frequency: None
    _RESET = 0,
    /// Cut off frequency: 5 Hz
    _5 = 1,
    /// Cut off frequency: 2.5 Hz
    _2P5 = 2,
    /// Cut off frequency: 1.25 Hz
    _1P25 = 3,
    /// Cut off frequency: 0.63 Hz
    _0P63 = 4,
    /// When triggered, the filter holds the present sample. The filter output will be the
    /// difference between the input sample and the held sample
    _HOLD = 7
}
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 27: Gyro Config
pub struct GYRO_CONFIG;

impl GYRO_CONFIG {
    pub const ADDR: u8 = 0x1b;
    /// Gyro x axis self test bit
    pub const XG_ST: u8 = 7;
    /// Gyro y axis self test bit
    pub const YG_ST: u8 = 6;
    /// Gyro z axis self test bit
    pub const ZG_ST: u8 = 5;
    /// Gyro Config FS_SEL
    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 28: Accel Config
pub struct ACCEL_CONFIG;

impl ACCEL_CONFIG {
    /// Base Address
    pub const ADDR: u8 = 0x1c;
    /// Accel x axis self test bit
    pub const XA_ST: u8 = 7;
    /// Accel y axis self test bit
    pub const YA_ST: u8 = 6;
    /// Accel z axis self test bit
    pub const ZA_ST: u8 = 5;
    /// Accel Config FS_SEL
    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2};
    /// Accel Config ACCEL_HPF
    pub const ACCEL_HPF: BitBlock = BitBlock { bit: 2, length: 3};
}

/// Defines accelerometer range/sensivity
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum AccelRange {
    /// 2G
    G2 = 0,
    /// 4G
    G4,
    /// 8G
    G8,
    /// 16G
    G16,
}

/// Defines gyro range/sensitivity
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum GyroRange {
    /// 250 degrees
    D250 = 0,
    /// 500 degrees
    D500,
    /// 1000 degrees
    D1000,
    /// 2000 degrees
    D2000,
}

pub const ACCEL_SENS: (f32, f32, f32, f32) = (16384., 8192., 4096., 2048.);
impl AccelRange {
    // Converts accelerometer range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            AccelRange::G2 => ACCEL_SENS.0,
            AccelRange::G4 => ACCEL_SENS.1,
            AccelRange::G8 => ACCEL_SENS.2,
            AccelRange::G16 => ACCEL_SENS.3,
        }
    }
}

pub const GYRO_SENS: (f32, f32, f32, f32) = (131., 65.5, 32.8, 16.4);
impl GyroRange {
    // Converts gyro range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            GyroRange::D250 => GYRO_SENS.0,
            GyroRange::D500 => GYRO_SENS.1,
            GyroRange::D1000 => GYRO_SENS.2,
            GyroRange::D2000 => GYRO_SENS.3,
        }
    }
}


pub enum MpuError {
    /// I2C bus error
    I2c(Error),

    /// Invalid chip ID was read
    InvalidChipId(u8),
}

pub struct Mpu {
    logger: Arc<Logger>,
    i2c: SpinLockMutex<i2c::I2C<I2C0>>,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}



impl Mpu{

    pub fn new<SDA: InputPin + OutputPin, SCL: InputPin + OutputPin>
    (logger:  Arc<Logger>, i2c0: I2C0, sda:SDA, scl: SCL, mut dport: DPORT) -> Mpu{
        let i2c = i2c::I2C::new(
            i2c0,
            i2c::Pins { sda, scl },
            400_000,
            &mut dport, //registers that control power/clock/routing things
        );
        Mpu {
            logger,
            i2c: SpinLockMutex::new(i2c),
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    pub fn init(&mut self, delay: &mut DelayMs<u8>) -> Result<(), MpuError>{
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.logger.info("init");
        self.wake(delay)?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        self.set_accel_hpf(ACCEL_HPF::_RESET)?;
        Ok(())
    }

    /// Wakes MPU6050 with all sensors enabled (default)
    fn wake(&mut self, delay: &mut DelayMs<u8>) -> Result<(), MpuError> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        delay.delay_ms(100u8);
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    fn verify(&mut self) -> Result<(), MpuError> {
        let address = self.read_byte(WHOAMI)?;
        if address != SLAVE_ADDR {
            return Err(MpuError::InvalidChipId(address));
        }
        Ok(())
    }

    /// set accel range, and update sensitivy accordingly
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), MpuError> {
        self.write_bits(ACCEL_CONFIG::ADDR,
                        ACCEL_CONFIG::FS_SEL.bit,
                        ACCEL_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// Set gyro range, and update sensitivity accordingly
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), MpuError> {
        self.write_bits(GYRO_CONFIG::ADDR,
                        GYRO_CONFIG::FS_SEL.bit,
                        GYRO_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// set accel high pass filter mode
    pub fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), MpuError> {
        Ok(
            self.write_bits(ACCEL_CONFIG::ADDR,
                            ACCEL_CONFIG::ACCEL_HPF.bit,
                            ACCEL_CONFIG::ACCEL_HPF.length,
                            mode as u8)?
        )
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub fn write_bits(&mut self, reg: u8, start_bit: u8, length: u8, data: u8) -> Result<(), MpuError> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Reads series of bytes into buf from specified reg
    pub fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), MpuError> {
        return self.i2c.borrow().lock( |bus| {
             bus.write_read(SLAVE_ADDR, &[reg], buf)
                .map_err(MpuError::I2c)
        });

    }

    /// Writes byte to register
    fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), MpuError>{
        return self.i2c.borrow().lock( |bus| {
            bus.write(SLAVE_ADDR, &[reg, byte]).map_err(MpuError::I2c)
        });
    }

    /// Reads series of bytes into buf from specified reg
    fn read_byte(&mut self, reg: u8) -> Result<u8, MpuError> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.borrow().lock( |bus| {
            bus.write_read(SLAVE_ADDR, &[reg],  &mut byte)
                .map_err(MpuError::I2c)
        });
        Ok(byte[0])
    }




    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }

    pub fn read_temperature(& self)-> f64{
        self.logger.info("read temperature");
        return 24.00;
    }
}