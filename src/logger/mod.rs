use esp32_hal::serial::Serial;
use esp32_hal::target::{UART0, RTCCNTL, APB_CTRL};
use esp32_hal::gpio::{Gpio1, Gpio3, FromValueType};
use esp32_hal::clock_control::ClockControl;
use esp32_hal::clock_control::CPUSource::PLL;
use esp32_hal::hal::watchdog::WatchdogEnable;
use esp32_hal::serial::config::Config;
use core::fmt;

macro_rules! uprint {
    ($serial:expr, $($arg:tt)*) => {
        $serial.write_fmt(format_args!($($arg)*)).ok()
    };
}

macro_rules! uprintln {
    ($serial:expr, $fmt:expr) => {
        uprint!($serial, concat!($fmt, "\n"))
    };
    ($serial:expr, $fmt:expr, $($arg:tt)*) => {
        uprint!($serial, concat!($fmt, "\n"), $($arg)*)
    };
}

pub struct Logger {
    serial: Serial<UART0, Gpio1<esp32_hal::gpio::Unknown>, Gpio3<esp32_hal::gpio::Unknown>>,
}

impl Logger {
    pub fn new(dport_clock_control: esp32_hal::dport::ClockControl,
               real_time_control: RTCCNTL,
               advanced_peripherial_bus_control: APB_CTRL,
               uart: UART0,
               gpio1: Gpio1<esp32_hal::gpio::Unknown>,
               gpio3: Gpio3<esp32_hal::gpio::Unknown>,
    ) -> Logger {

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

}

impl fmt::Write for Logger {
    fn write_str(& mut self, msg: &str) -> fmt::Result {
        write!(self.serial, "{}", msg).unwrap();
        Ok(())
    }
}



