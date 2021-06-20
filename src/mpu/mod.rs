mod bits;
mod device;
use nalgebra::{Vector3};
use crate::mpu::device::{ACCEL_SENS, GYRO_SENS, AccelRange, GyroRange, ACCEL_HPF, PWR_MGMT_1,SMPLRT_DIV,CONFIG,BANDWITH,
SLAVE_ADDR, WHOAMI,SIGNAL_PATH_RESET, ACCEL_CONFIG, GYRO_CONFIG, TEMP_OUT_H, TEMP_SENSITIVITY, TEMP_OFFSET, ACC_REGX_H, GYRO_REGX_H};
use crate::logger::Logger;
use core::fmt::Write;
use alloc::sync::Arc;
use esp32_hal::i2c::{self, Error};
use xtensa_lx::mutex::{SpinLockMutex, Mutex, CriticalSectionSpinLockMutex};
use esp32_hal::target::{ I2C0, DPORT};
use esp32_hal::gpio::{OutputPin, InputPin};
use core::borrow::Borrow;
use esp32_hal::delay::Delay;
use esp32_hal::hal::blocking::delay::DelayMs;
use core::ops::Deref;


/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;


pub struct Mpu {
    logger: Arc<CriticalSectionSpinLockMutex<Logger>>,
    i2c: SpinLockMutex<i2c::I2C<I2C0>>,
    delay: esp32_hal::delay::Delay,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}



impl Mpu{

    pub fn new<SDA: InputPin + OutputPin, SCL: InputPin + OutputPin>
    (logger:  Arc<CriticalSectionSpinLockMutex<Logger>>, i2c0: I2C0, sda:SDA, scl: SCL, mut dport: DPORT) -> Mpu{
        let i2c = i2c::I2C::new(
            i2c0,
            i2c::Pins { sda, scl },
            400_000,
            &mut dport, //registers that control power/clock/routing things
        );
        Mpu {
            logger,
            i2c: SpinLockMutex::new(i2c),
            delay: Delay::new(),
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    pub fn init(&mut self) -> Result<(), Error>{
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )

        self.reset_device()?;
        self.wake()?;
        self.verify()?;
        self.set_sample_rate_division(0)?;

        self.set_filter_bandwidth(BANDWITH::_260_HZ)?;

        self.set_gyro_range(GyroRange::D500)?;
        self.set_accel_range(AccelRange::G2)?;
       
        
        //self.set_accel_hpf(ACCEL_HPF::_RESET)?;
       
        Ok(())
    }

    /***
    //Wite byte 10000000 to Register 0x6B to reset device 
     _____________ _______ _______ ______ _________ _________ _________ __________
    |     bit 7   | bit 6 | bit 5 | bit4 |   bit3  |   bit2  |   bit1  |    bit0  | 
     ------------- ------- ------- ------ --------- --------- --------- ---------- 
     _____________ _______ _______ ______ _________ _________ _________ __________
    |DEVICE_RESET | SLEEP | CYCLE |  --  | TEMP_DIS|         CLKSEL[2:0]          |
     ------------- ------- ------- ------ --------- --------- --------- ---------- 
    //Note: Reset sets sleep to true! Section register map: resets PWR_MGMT to 0x40 

    //Wite byte 00000111 Register 0x68 to reset sensor digital path
     ______ ______ ______ ______ ______ ____________ ___________ ____________
    |  --  |  --  |  --  |  --  |  --  | GYRO_RESET | ACC_RESET | TEMP_RESET |
     ------ ------ ------ ------ ------ ------------ ----------- ------------
    ***/
    pub fn reset_device(&mut self) -> Result<(),Error> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)?;
        let default_value: u8 = 0x40;
        loop {
            let result = self.read_byte(PWR_MGMT_1::ADDR)?;
            self.logger.deref().lock(|logger| { uprint!(logger, " reset device result")}); self.print_bits(result);
            if result == default_value {
                break;
            }
            self.delay.delay_ms(1u8);
            
        }
        self.delay.delay_ms(100u8);
        let reset_all_sensor_digital_path: u8 = 0x7;
        self.write_byte(SIGNAL_PATH_RESET, reset_all_sensor_digital_path)?;
        self.delay.delay_ms(100u8);
        Ok(())
    }

    /// Wakes MPU6050 with all sensors enabled (default)
    fn wake(&mut self) -> Result<(), Error> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x00)?; //0x6B=0x00
        self.delay.delay_ms(100u8);
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    fn verify(&mut self) -> Result<(), Error> {
        let address = self.read_byte(WHOAMI)?;
        self.logger.deref().lock(|logger| { uprint!(logger, " mpu address")}); self.print_bits(address);
        if address != SLAVE_ADDR {
            return Err(Error::Transmit);
        }
        Ok(())
    }

    fn print_bits(&mut self, byte_var: u8){
        self.logger.deref().lock(|logger| {
            uprintln!(logger, " in binary = {}{}{}{}{}{}{}{}", bits::get_bit(byte_var,7),bits::get_bit(byte_var,6), bits::get_bit(byte_var,5),
            bits::get_bit(byte_var,4) , bits::get_bit(byte_var,3), bits::get_bit(byte_var,2), bits::get_bit(byte_var,1), bits::get_bit(byte_var,0));
        });

    }

    /// Sets the divisor used to divide the base clock rate into a measurement rate
    /// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    fn set_sample_rate_division(&mut self, divisor: u8) -> Result<(), Error> {
        self.write_byte( SMPLRT_DIV,divisor)?;
        Ok(())
    }

    /***
    //Wite byte xxxxx000 Register 0x1a to set filter bandwith
     ______ ______ ______ ______ ______ ______ ______ ______
    |  --  |  --  |     EXT_SYNC_SET   |      DLPF_CFG      |
     ------ ------ ------ ------ ------ ------ ------ ------ 
    ***/
    fn set_filter_bandwidth(&mut self, bandwith: BANDWITH) -> Result<(), Error> {
        self.write_bits(CONFIG::ADDR,
            CONFIG::DLPF_CFG.bit,
            CONFIG::DLPF_CFG.length,
            bandwith as u8)?;
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

    /// Enables bit n at register address reg
    pub fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0])?)
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



    /// Sensor Temp in degrees celcius
    pub fn read_temperature(&mut self) -> Result<f32,Error>{
        self.logger.deref().lock(|logger| {
            uprintln!(logger, "read temperature");
        });
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    /// Accelerometer readings in g
    pub fn read_acceleration(&mut self) -> Result<Vector3<f32>, Error> {
        let mut acc = self.read_rot(ACC_REGX_H)?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub fn read_gyro(&mut self) -> Result<Vector3<f32>, Error> {
        let mut gyro = self.read_rot(GYRO_REGX_H)?;

        gyro *= PI_180 * self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Reads rotation (gyro/acc) from specified register
    fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Error> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;
        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32
        ))
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