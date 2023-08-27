//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod sensor;

use arrayvec::ArrayString;
use bsp::hal::timer::Alarm;
use bsp::hal::uart::UartPeripheral;
use bsp::hal::Timer;
use critical_section::Mutex;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use embedded_sdmmc::{TimeSource, Timestamp};
use sensor::lsm303d::{configuration::*, lsm303d::Measurements};

use bsp::hal::pac::interrupt;

use bsp::{
    entry,
    hal::{
        gpio::PullDown,
        timer::Instant,
        uart::{DataBits, StopBits, UartConfig},
    },
};

use core::{cell::RefCell, fmt::Write};
use embedded_hal::digital::v2::OutputPin;
use fugit::{MicrosDurationU32, MicrosDurationU64, RateExtU32};
use {defmt_rtt as _, panic_probe as _};

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Recording,
}

struct SharedDelay {
    inner: RefCell<cortex_m::delay::Delay>,
}

impl SharedDelay {
    fn new(delay: cortex_m::delay::Delay) -> Self {
        Self {
            inner: delay.into(),
        }
    }
}

impl DelayMs<u32> for &SharedDelay {
    fn delay_ms(&mut self, ms: u32) {
        self.inner.borrow_mut().delay_ms(ms);
    }
}

impl DelayUs<u8> for &SharedDelay {
    fn delay_us(&mut self, us: u8) {
        self.inner.borrow_mut().delay_us(us as u32);
    }
}

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

type ButtonPin = gpio::Pin<gpio::bank0::Gpio18, gpio::Input<PullDown>>;
type UartLogger = UartPeripheral<
    bsp::hal::uart::Enabled,
    bsp::pac::UART0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::Function<gpio::Uart>>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::Function<gpio::Uart>>,
    ),
>;
static GLOBAL_PINS: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static LOGGER_STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::Idle));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
static DEV_ALARM: Mutex<RefCell<Option<bsp::hal::timer::Alarm0>>> = Mutex::new(RefCell::new(None));
static UART: Mutex<RefCell<Option<UartLogger>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut delay = &SharedDelay::new(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    ));

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<gpio::FunctionUart>(),
    );
    let uart = bsp::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let sda_pin = pins.gpio14.into_mode::<bsp::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio15.into_mode::<bsp::hal::gpio::FunctionI2C>();

    let i2c = bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut sensor = sensor::lsm303d::lsm303d::LSM303D::new(i2c);

    sensor
        .configure(
            Configuration::default()
                .configure_accelerometer(
                    true,
                    true,
                    true,
                    AccelerationDataRate::Hz50,
                    AccelerationFullScale::Acc2G,
                )
                .configure_magnetometer(
                    MagnetometerDataRate::Hz50,
                    MagneticSensorMode::ContinuousConversion,
                    MagnetometerFullScale::Mag2Gauss,
                    MagnetometerResolution::Low,
                )
                .configure_temperature(true),
        )
        .unwrap();

    let mut led_pin = pins.led.into_push_pull_output();

    let mut idle_led = pins.gpio16.into_push_pull_output();
    let mut recording_led = pins.gpio17.into_push_pull_output();

    let mut buffer = ArrayString::<64>::new();
    let mut timer = bsp::hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    let begin = Instant::from_ticks(timer.get_counter().ticks());

    // Set up the GPIO pin that will be our input
    let in_pin = pins.gpio18.into_pull_down_input();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    in_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    in_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    unsafe {
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::TIMER_IRQ_0);
    };
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio13.into_push_pull_output();

    // Create an SPI driver instance for the SPI1 device
    let spi = bsp::hal::spi::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        400.kHz(), // card initialization happens at low baud rate
        &embedded_hal::spi::MODE_0,
    );

    let sdcard = embedded_sdmmc::SdCard::new(spi, spi_cs, delay);

    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource {});

    let mut volume0 = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    let root_dir = volume_mgr.open_root_dir(&volume0).unwrap();
    let mut csv_file = volume_mgr
        .open_file_in_dir(
            &mut volume0,
            &root_dir,
            "a_0.csv",
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )
        .unwrap();
    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        // Schedule an alarm in 1 second
        let _ = alarm.schedule(MicrosDurationU32::Hz(1));
        // Enable generating an interrupt on alarm
        alarm.enable_interrupt();
        DEV_ALARM.replace(cs, Some(alarm));
        TIMER.replace(cs, Some(timer));
        GLOBAL_PINS.borrow(cs).replace(Some(in_pin));
        UART.borrow(cs).replace(Some(uart));
    });

    loop {
        let measurements = sensor.read_measurements().unwrap();
        let current_state = critical_section::with(|cs| *LOGGER_STATE.borrow(cs).borrow());
        match current_state {
            State::Idle => {
                idle_led.set_high().unwrap();
                led_pin.set_high().unwrap();
                delay.delay_ms(50);
                idle_led.set_low().unwrap();
                led_pin.set_low().unwrap();
                delay.delay_ms(50);
            }
            State::Recording => {
                recording_led.set_high().unwrap();
                led_pin.set_high().unwrap();
                delay.delay_ms(50);
                recording_led.set_low().unwrap();
                led_pin.set_low().unwrap();
                delay.delay_ms(50);
            }
        }
        let measure_time = Instant::from_ticks(critical_section::with(|cs| {
            TIMER
                .borrow_ref(cs)
                .as_ref()
                .map_or(0, |v| v.get_counter().ticks())
        }));
        format_measurements(
            &mut buffer,
            &measurements,
            measure_time
                .checked_duration_since(begin)
                .unwrap_or(MicrosDurationU64::from_ticks(0)),
        );

        critical_section::with(|cs| {
            UART.borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write_str(&buffer)
                .unwrap();
        });
        if current_state == State::Recording {
            volume_mgr
                .write(&mut volume0, &mut csv_file, buffer.as_str().as_bytes())
                .unwrap();
        }
        buffer.clear();
    }
}

fn format_measurements<const SIZE: usize>(
    mut message: &mut ArrayString<SIZE>,
    measurements: &Measurements,
    elapsed: MicrosDurationU64,
) {
    message.clear();
    write!(
        &mut message,
        "{:6},{:+03.4},{:+03.4},{:+03.4},{:+03.3},{:+04.3},{:+04.3}\r\n",
        elapsed.to_millis(),
        measurements.accelerometer.x,
        measurements.accelerometer.y,
        measurements.accelerometer.z,
        measurements.magnetometer.x,
        measurements.magnetometer.y,
        measurements.magnetometer.z,
    )
    .unwrap();
}

#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        let mut buffer = ArrayString::<64>::new();
        let measure_time = Instant::from_ticks(critical_section::with(|cs| {
            TIMER
                .borrow_ref(cs)
                .as_ref()
                .map_or(0, |v| v.get_counter().ticks())
        }));
        let _ = write!(
            buffer,
            "Hello from timer! {}\r\n",
            measure_time
                .checked_duration_since(Instant::from_ticks(0))
                .unwrap()
                .to_millis()
        );
        UART.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str(&buffer)
            .unwrap();
        let mut alarm = DEV_ALARM.borrow(cs).borrow_mut();
        alarm.as_mut().unwrap().clear_interrupt();
        let _ = alarm.as_mut().unwrap().schedule(MicrosDurationU32::Hz(1));
    });
}

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let mut pin = GLOBAL_PINS.borrow(cs).borrow_mut();
        let button = pin.as_mut();
        if let Some(button) = button {
            let mut s = LOGGER_STATE.borrow(cs).borrow_mut();
            if button.interrupt_status(gpio::Interrupt::EdgeHigh) {
                button.clear_interrupt(gpio::Interrupt::EdgeHigh);
            }

            if button.interrupt_status(gpio::Interrupt::EdgeLow) {
                *s = match *s {
                    State::Idle => State::Recording,
                    State::Recording => State::Idle,
                };
                button.clear_interrupt(gpio::Interrupt::EdgeLow);
            }
        }
    });
}
