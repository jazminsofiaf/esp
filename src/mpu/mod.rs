use crate::logger::Logger;
use alloc::sync::Arc;
use esp32_hal::i2c::{self};
use xtensa_lx::mutex::SpinLockMutex;
use esp32_hal::target::{Peripherals, I2C0, DPORT};
use esp32_hal::dport::Split;
use esp32_hal::gpio::{OutputPin, InputPin, GpioExt, Parts};


pub struct Mpu {
    logger: Arc<Logger>,
    i2c_bus: SpinLockMutex<i2c::I2C<I2C0>>,
}


impl Mpu {

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
            i2c_bus: SpinLockMutex::new(i2c)
        }
    }


    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }
}