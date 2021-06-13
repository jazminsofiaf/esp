

/*
use crate::logger::Logger;
use core::borrow::Borrow;

'a elided output lifetime
// to compiles without lifetime annotations every reference needed an explicit lifetime
pub struct Mpu<'a> {
    logger:  &'a mut Logger<'a>
}

impl Mpu<'_> {
    pub fn new(logger: &'a mut Logger) -> Mpu{
        Mpu {
            logger
        }
    }

    pub fn read(& self) {
        self.logger.info("info from mpu sensor")
    }
}*/