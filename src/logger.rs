pub mod serial_port {
    use esp32_hal::clock_control::{CPUSource::PLL, ClockControl};
    use esp32_hal::target::{RTCCNTL,APB_CTRL, UART0};
    use esp32_hal::gpio::{Gpio1,Gpio3};
    use esp32_hal::serial::{config::Config, Serial};

    use core::fmt::Write;
    use esp32_hal as hal;
    use hal::prelude::*;
    use panic_halt as _;



    pub struct Logger<T> {
        serial: Serial<UART0, Gpio1<T>, Gpio3<T>>,
    }

    impl<T> Logger<T> {
        pub fn new(dport_clock_control: esp32_hal::dport::ClockControl,
                   real_time_control: RTCCNTL,
                   advanced_peripherial_bus_control: APB_CTRL,
                   uart: UART0,
                   gpio1: Gpio1<T>,
                   gpio3: Gpio3<T>,
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

            uart0.change_baudrate(115200).unwrap(); //set signals per seconds
            //uart0.change_baudrate(74880 ).unwrap();
            //uart0.change_baudrate(9600  ).unwrap();

            Logger {
                serial: uart0
            }
        }

        pub fn info(&mut self, msg: &'static str) {
            writeln!(self.serial, "{}", msg).unwrap();
        }
    }
}
