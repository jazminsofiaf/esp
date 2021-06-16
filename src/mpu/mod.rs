use crate::logger::Logger;
use alloc::sync::Arc;
use esp32_hal::i2c::{self, Error};
use xtensa_lx::mutex::{SpinLockMutex, Mutex};
use esp32_hal::target::{ I2C0, DPORT};
use esp32_hal::gpio::{OutputPin, InputPin};
use core::borrow::Borrow;


pub const SLAVE_ADDR: u8 = 0x68;
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


pub struct Mpu {
    logger: Arc<Logger>,
    i2c: SpinLockMutex<i2c::I2C<I2C0>>,
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
            i2c: SpinLockMutex::new(i2c)
        }
    }

    pub fn init(&mut self){
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.logger.info("init");
        self.write_byte(PWR_MGMT_1::ADDR, 0x01);
    }

    /// Writes byte to register
    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Error>{
        return self.i2c.borrow().lock( |bus| {
            self.logger.info("writing to i2c bus");
            bus.write(SLAVE_ADDR, &[reg, byte])
        });
    }


    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }

    pub fn read_temperature(& self)-> f64{
        self.logger.info("read temperature");
        return 24.00;
    }
}