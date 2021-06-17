mod bits;
mod device;
use crate::mpu::device::{ACCEL_SENS, GYRO_SENS, AccelRange, GyroRange, ACCEL_HPF, PWR_MGMT_1, SLAVE_ADDR, WHOAMI, ACCEL_CONFIG, GYRO_CONFIG, TEMP_OUT_H, TEMP_SENSITIVITY, TEMP_OFFSET};
use crate::logger::Logger;
use alloc::sync::Arc;
use esp32_hal::i2c::{self, Error};
use xtensa_lx::mutex::{SpinLockMutex, Mutex};
use esp32_hal::target::{ I2C0, DPORT};
use esp32_hal::gpio::{OutputPin, InputPin};
use core::borrow::Borrow;
use esp32_hal::delay::Delay;
use esp32_hal::hal::blocking::delay::DelayMs;




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

    pub fn init(&mut self, delay: &mut esp32_hal::delay::Delay) -> Result<(), Error>{
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
    fn wake(&mut self, delay: &mut esp32_hal::delay::Delay) -> Result<(), Error> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?; //0x6B=0x00
        delay.delay_ms(100u8);
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    fn verify(&mut self) -> Result<(), Error> {
        let address = self.read_byte(WHOAMI)?;
        if address != SLAVE_ADDR {
            self.logger.info("Mpu device not connected");
            return Err(Error::Transmit);
        }
        Ok(())
    }

    /// set accel range, and update sensitivy accordingly
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error> {
        self.write_bits(ACCEL_CONFIG::ADDR,
                        ACCEL_CONFIG::FS_SEL.bit,
                        ACCEL_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// Set gyro range, and update sensitivity accordingly
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error> {
        self.write_bits(GYRO_CONFIG::ADDR,
                        GYRO_CONFIG::FS_SEL.bit,
                        GYRO_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// set accel high pass filter mode
    pub fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Error> {
        Ok(
            self.write_bits(ACCEL_CONFIG::ADDR,
                            ACCEL_CONFIG::ACCEL_HPF.bit,
                            ACCEL_CONFIG::ACCEL_HPF.length,
                            mode as u8)?
        )
    }

    /// Write bits data at reg from start_bit to start_bit+length
    fn write_bits(&mut self, reg: u8, start_bit: u8, length: u8, data: u8) -> Result<(), Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Reads series of bytes into buf from specified reg
    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        return self.i2c.borrow().lock( |bus| {
             bus.write_read(SLAVE_ADDR, &[reg], buf)
        });

    }

    /// Writes byte to register
    fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Error>{
        return self.i2c.borrow().lock( |bus| {
            bus.write(SLAVE_ADDR, &[reg, byte])
        });
    }

    /// Reads series of bytes into buf from specified reg
    fn read_byte(&mut self, reg: u8) -> Result<u8, Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.borrow().lock( |bus| {
            bus.write_read(SLAVE_ADDR, &[reg],  &mut byte).unwrap();
        });
        Ok(byte[0])
    }

    /// Converts 2 bytes number in 2 compliment
    fn read_word_2c(&self, byte: &[u8]) -> i16 {
        let high: i16 = byte[0] as i16;
        let low: i16 = byte[1] as i16;
        let word: i16 = (high << 8) | low;
        return word;
    }




    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }



    /// Sensor Temp in degrees celcius
    pub fn read_temperature(&mut self) -> Result<f32,Error>{
        self.logger.info("read temperature");
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_word_test() {
        let b0:u8= 0b0000;
        let b1:u8= 0b0001;
        let byte = [b0,b0];
        assert_eq!(read_word_2c(&byte), 0);
        let byte = [b0,b1];
        assert_eq!(read_word_2c(&byte), 1);
        let byte = [b1,b0];
        assert_eq!(read_word_2c(&byte), 256);
        let byte = [b1,b1];
        assert_eq!(read_word_2c(&byte), 257);

    }
}