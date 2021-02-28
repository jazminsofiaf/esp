
#![no_std] // no import standar library 
#![no_main] // no main used


use core::fmt::Write;


//import the esp32 hardware abstraction layer
use esp32_hal::target; 
use esp32_hal as hal;

//import the serial port and its config
use esp32_hal::serial::{config::Config, Serial};
use esp32_hal::clock_control::{CPUSource::PLL, ClockControl};
use esp32_hal::dport::Split;
use hal::prelude::*;
use xtensa_lx::timer::delay;
use panic_halt as _;


/// The default clock source is the onboard crystal
/// In most cases 40mhz (but can be as low as 2mhz depending on the board)
const CORE_HZ: u32 = 40_000_000;

#[entry] // entry point
fn main() -> ! {

    // take the peripherials
    let peripherals = target::Peripherals::take().expect("Failed to obtain Peripherals");

    let mut timg0 = peripherals.TIMG0;
    let mut timg1 = peripherals.TIMG1;

    // (https://github.com/espressif/openocd-esp32/blob/97ba3a6bb9eaa898d91df923bbedddfeaaaf28c9/src/target/esp32.c#L431)
    // openocd disables the watchdog timers on halt
    // we will do it manually on startup
    disable_timg_wdts(&mut timg0, &mut timg1);

    //General Purpose Input/Output pins
    let pins = peripherals.GPIO.split();
    let mut led = pins.gpio2.into_push_pull_output();

    //This peripheral contains many registers, which are used for various different functions
    let (_, dport_clock_control) = peripherals.DPORT.split();

    // setup clocks & watchdog
    let mut clock_control = ClockControl::new(
        peripherals.RTCCNTL, //Real Time Controls
        peripherals.APB_CTRL, // Advanced Peripheral Bus Control
        dport_clock_control,
        esp32_hal::clock_control::XTAL_FREQUENCY_AUTO,
    ).unwrap();

    // set desired clock frequencies
    //PPL is a control system that generates an output signal whose phase is related to the phase of an input signal
    clock_control.set_cpu_frequencies(PLL, 240.MHz(), PLL, 240.MHz(), PLL, 240.MHz()).unwrap();

    //Freeze clock settings and return ClockControlConfig
    let (clock_control_config, mut watchdog) = clock_control.freeze().unwrap();

    watchdog.start(20.s());

    // setup serial controller  
    let mut uart0: Serial<_, _, _> = Serial::new(
        peripherals.UART0, // UART port is the universal asynchronous receiver-transmitter
        esp32_hal::serial::Pins {
            tx: pins.gpio1, //needs two pins
            rx: pins.gpio3,
            cts: None,
            rts: None,
        },
        Config::default(), //default config
        clock_control_config,
    ).unwrap();

  //uart0.change_baudrate(115200).unwrap(); //set signals per seconds
  //uart0.change_baudrate(74880 ).unwrap();
    uart0.change_baudrate(9600  ).unwrap();

    // print startup message
    let b = [b'R', b'E', b'B', b'O', b'O',b'T'];
    uart0.write_str(core::str::from_utf8(&b[..]).unwrap()).unwrap();

    loop {
        writeln!(uart0, "En el loop.....").unwrap();
        
        //red led blick
        led.set_high().unwrap();
        delay(CORE_HZ); // timer
    }
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
