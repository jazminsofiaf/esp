use crate::logger::Logger;
use alloc::sync::Arc;

pub struct Mpu {
    logger: Arc<Logger>
}

impl Mpu {
    pub fn new(logger:  Arc<Logger>) -> Mpu{
        Mpu {
            logger
        }
    }

    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }
}