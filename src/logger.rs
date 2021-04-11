

//use core::fmt::Write;


//import the esp32 hardware abstraction layer
//use esp32_hal::target; 
use esp32_hal as hal;

//import the serial port and its config
use esp32_hal::serial::{config::Config, Serial};
use esp32_hal::clock_control::{CPUSource::PLL, ClockControl};
//use esp32_hal::dport::Split;
use hal::prelude::*;
//use xtensa_lx::timer::delay;
use panic_halt as _;
use esp32_hal::esp32::UART0;
use esp32_hal::target::{RTCCNTL,APB_CTRL};


pub mod logger {
    pub struct Logger<T> {
        //mut uart0: Serial<_, _, _>
        //uart0: Serial<T, T, T>
        test_field: T,
    }

    impl<T> Logger<T> {
        pub fn new(dport_clock_control: esp32_hal::dport::ClockControl,
                   real_time_control: RTCCNTL,
                   advanced_peripherial_bus_control: APB_CTRL,
                   uart: UART0,
                   gpio1: Gpio1,
                   gpio3: Gpio3,
        ) -> Logger<T> {


            // setup clocks & watchdog
            let mut clock_control = ClockControl::new(
                real_time_control, //RTCCNTL
                advanced_peripherial_bus_control, // APB_CTRL
                dport_clock_control,
                esp32_hal::clock_control::XTAL_FREQUENCY_AUTO,
            ).unwrap();

            // set desired clock frequencies
            //PPL is a control system that generates an output signal whose phase is related to the phase of an input signal
            clock_control.set_cpu_frequencies(PLL, 240.MHz(), PLL, 240.MHz(), PLL, 240.MHz()).unwrap();

            //Freeze clock settings and return ClockControlConfig
            let (clock_control_config, mut watchdog) = clock_control.freeze().unwrap();

            watchdog.start(20.s());

            //uart0.change_baudrate(115200).unwrap(); //set signals per seconds
            //uart0.change_baudrate(74880 ).unwrap();
            //uart0.change_baudrate(9600  ).unwrap();

            let mut uart0: Serial<_, _, _> = Serial::new(
                uart, // UART port is the universal asynchronous receiver-transmitter
                esp32_hal::serial::Pins {
                    tx: gpio1, //needs two pins
                    rx: gpio3,
                    cts: None,
                    rts: None,
                },
                Config::default(), //default config
                clock_control_config,
            ).unwrap();

            Logger {
                test_field: 3
            }
        }

        pub fn info(&self, msg: &str) {}
    }
}
