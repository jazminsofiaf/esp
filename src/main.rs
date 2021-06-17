
// to flash: cargo espflash --chip esp32 --features="xtensa-lx-rt/lx6,xtensa-lx/lx6" /dev/tty.usbserial-0001
// to see:  screen /dev/tty.usbserial-0001 9600
// to exit: ctr+a ctr+k

// or inside a spressif project
// to see: idf.py -p /dev/cu.usbserial-0001  monitor -B 9600
// to exit: ctr+]

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(default_alloc_error_handler)]


use esp32_hal::prelude::*;


use esp32_hal::target;
use esp32_hal::dport::Split;
use xtensa_lx::timer::delay;
use esp32_hal::hal::digital::v2::OutputPin;

#[macro_use]
extern crate alloc;
use esp32_hal::alloc::{Allocator, DEFAULT_ALLOCATOR};
use esp32_hal::delay::Delay;


#[global_allocator]
pub static GLOBAL_ALLOCATOR: Allocator = DEFAULT_ALLOCATOR;

mod logger;
mod mpu;


/// The default clock source is the onboard crystal
/// In most cases 40mhz (but can be as low as 2mhz depending on the board)
const CORE_HZ: u32 = 40_000_000;


#[entry] // entry point
fn main() -> ! {

    // take the peripherials
    let peripherals = target::Peripherals::take().expect("Failed to obtain Peripherals");

    let mut timg0 = peripherals.TIMG0;
    let mut timg1 = peripherals.TIMG1;

    // (https://github.com/espressif/openocd-esp3H/blob/97ba3a6bb9eaa898d91df923bbedddfeaaaf28c9/src/target/esp32.c#L431)
    // openocd disables the watchdog timers on halt
    // we will do it manually on startup
    disable_timg_wdts(&mut timg0, &mut timg1);

    //General Purpose Input/Output pins
    let pins = peripherals.GPIO.split();
    let mut led = pins.gpio2.into_push_pull_output();

    let (dport, dport_clock_control) = peripherals.DPORT.split();

    let serial_port_logger = alloc::sync::Arc::new(logger::Logger::new(dport_clock_control, peripherals.RTCCNTL, peripherals.APB_CTRL, peripherals.UART0, pins.gpio1, pins.gpio3));

    let mut mpu = mpu::Mpu::new(serial_port_logger.clone(), peripherals.I2C0, pins.gpio21, pins.gpio22, dport);
    let mut delay_i2c = Delay::new();
    mpu.init(&mut delay_i2c).unwrap();

    loop {
        serial_port_logger.clone().info("info from main loop");
        let temp = mpu.read_temperature().unwrap();
        serial_port_logger.clone().info_float(temp);
        led.set_high().unwrap();
        delay(CORE_HZ); // timer
    }
}

#[panic_handler]
fn my_panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

const WDT_WKEY_VALUE: u32 = 0x50D83AA1;

fn disable_timg_wdts(timg0: &mut target::TIMG0, timg1: &mut target::TIMG1) {
    /* ESP32 ignores writes to any register if WDTWPROTECT doesn't contain the
    * magic value of TIMG_WDT_WKEY_VALUE.  The datasheet recommends unsealing,
    * making modifications, and sealing for every watchdog modification.
    */
    timg0
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });
    timg1
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });

    timg0.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
    timg1.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
}
