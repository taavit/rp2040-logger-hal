//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod sensor;

use arrayvec::{ArrayString, ArrayVec};
use bsp::hal::timer::Alarm;
use bsp::hal::uart::UartPeripheral;
use bsp::hal::Timer;
use cortex_m::asm;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use embedded_sdmmc::{Directory, File, SdCard, TimeSource, Timestamp, Volume, VolumeManager};
use sensor::lsm303d::LSM303D;
use sensor::lsm303d::{configuration::*, Measurements};

use bsp::hal::pac::interrupt;

use bsp::{
    entry,
    hal::{
        gpio::PullDown,
        timer::Instant,
        uart::{DataBits, StopBits, UartConfig},
    },
};

use core::panic::PanicInfo;
use core::{cell::RefCell, fmt::Write};
use embedded_hal::digital::v2::OutputPin;
use fugit::{MicrosDurationU32, MicrosDurationU64, RateExtU32};

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

impl core::ops::Not for State {
    type Output = Self;

    fn not(self) -> Self {
        match self {
            Self::Idle => Self::Recording,
            Self::Recording => Self::Idle,
        }
    }
}

impl State {
    pub fn toggle(&mut self) {
        *self = !*self;
    }
}

struct IrqDelayer {}

impl DelayUs<u8> for IrqDelayer {
    fn delay_us(&mut self, us: u8) {
        critical_section::with(|cs| {
            DELAYER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .delay_us(us as u32);
        })
    }
}

impl DelayMs<u32> for IrqDelayer {
    fn delay_ms(&mut self, ms: u32) {
        critical_section::with(|cs| {
            DELAYER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .delay_ms(ms);
        });
    }
}

type SdCardType = SdCard<
    bsp::hal::Spi<bsp::hal::spi::Enabled, pac::SPI1, 8>,
    gpio::Pin<gpio::bank0::Gpio13, gpio::Output<gpio::PushPull>>,
    IrqDelayer,
>;

type VolumeManagerType = VolumeManager<SdCardType, DummyTimesource>;
struct SdWriter {
    file: Option<File>,
    volume_mgr: VolumeManagerType,
    volume: Volume,
    root_dir: Directory,
    file_idx: u16,
    base_filename: ArrayString<16>,
    filename: ArrayString<32>,
}

impl SdWriter {
    pub fn new(file: &str, mut volume_mgr: VolumeManagerType) -> Self {
        let volume = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
        let root_dir = volume_mgr.open_root_dir(&volume).unwrap();
        let filename = ArrayString::new();
        Self {
            volume,
            volume_mgr,
            file: None,
            file_idx: 1,
            base_filename: ArrayString::from(file).unwrap(),
            filename,
            root_dir,
        }
    }
    pub fn write(&mut self, data: &str) {
        critical_section::with(|cs| {
            if let Some(ref mut file) = self.file {
                let mut cache = CACHE.borrow(cs).borrow_mut();
                cache.push(ArrayString::from(data).unwrap());
                if cache.is_full() {
                    let mut full_string = ArrayString::<640>::new();
                    for e in cache.drain(..) {
                        full_string.push_str(e.as_str());
                    }
                    self.volume_mgr
                        .write(&mut self.volume, file, full_string.as_bytes())
                        .unwrap();
                }
            }
        });
    }

    pub fn start_recording(&mut self) {
        self.generate_filename();

        critical_section::with(|cs| {
            CACHE.borrow(cs).borrow_mut().clear();
            let csv_file = self
                .volume_mgr
                .open_file_in_dir(
                    &mut self.volume,
                    &self.root_dir,
                    self.filename.as_str(),
                    embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
                )
                .unwrap();
            self.file = Some(csv_file);
        });
    }

    pub fn stop_recording(&mut self) {
        critical_section::with(|cs| {
            if let Some(mut file) = self.file.take() {
                let mut cache = CACHE.borrow(cs).borrow_mut();

                let mut full_string = ArrayString::<640>::new();
                for e in cache.drain(..) {
                    full_string.push_str(e.as_str());
                }
                self.volume_mgr
                    .write(&mut self.volume, &mut file, full_string.as_bytes())
                    .unwrap();
                self.volume_mgr.close_file(&self.volume, file).unwrap();
            };
            self.file_idx += 1;
        })
    }

    fn generate_filename(&mut self) {
        self.filename.clear();
        write!(
            self.filename,
            "{}_{:05}.csv",
            self.base_filename, self.file_idx
        )
        .unwrap();
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
type ControlLed = gpio::Pin<gpio::bank0::Gpio25, gpio::Output<gpio::PushPull>>;
type IdleLed = gpio::Pin<gpio::bank0::Gpio16, gpio::Output<gpio::PushPull>>;
type RecordingLed = gpio::Pin<gpio::bank0::Gpio17, gpio::Output<gpio::PushPull>>;

type LedSet = (ControlLed, IdleLed, RecordingLed, RefCell<bool>);

type I2CPin = bsp::hal::i2c::I2C<
    bsp::pac::I2C1,
    (
        gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2C>,
        gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2C>,
    ),
>;

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
static SENSOR: Mutex<RefCell<Option<LSM303D<I2CPin>>>> = Mutex::new(RefCell::new(None));
static SDWRITER: Mutex<RefCell<Option<SdWriter>>> = Mutex::new(RefCell::new(None));
static DELAYER: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static LED_SET: Mutex<RefCell<Option<LedSet>>> = Mutex::new(RefCell::new(None));
static CACHE: Mutex<RefCell<ArrayVec<ArrayString<64>, 10>>> =
    Mutex::new(RefCell::new(ArrayVec::new_const()));

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

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    let mut sensor = sensor::lsm303d::LSM303D::new(i2c);

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
                    MagnetometerFullScale::Mag2,
                    MagnetometerResolution::Low,
                )
                .configure_temperature(true),
        )
        .unwrap();

    let control_led = pins.led.into_push_pull_output();
    let idle_led = pins.gpio16.into_push_pull_output();
    let recording_led = pins.gpio17.into_push_pull_output();

    let mut timer = bsp::hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

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

    let sdcard: SdCard<
        bsp::hal::Spi<bsp::hal::spi::Enabled, pac::SPI1, 8>,
        gpio::Pin<gpio::bank0::Gpio13, gpio::Output<gpio::PushPull>>,
        IrqDelayer,
    > = embedded_sdmmc::SdCard::new(spi, spi_cs, IrqDelayer {});

    let volume_mgr: VolumeManager<SdCardType, DummyTimesource> =
        embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource {});

    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(MicrosDurationU32::Hz(10));
        alarm.enable_interrupt();
        DEV_ALARM.replace(cs, Some(alarm));
        TIMER.replace(cs, Some(timer));
        GLOBAL_PINS.borrow(cs).replace(Some(in_pin));
        UART.borrow(cs).replace(Some(uart));
        SENSOR.borrow(cs).replace(Some(sensor));
        DELAYER.borrow(cs).replace(Some(delay));
        LED_SET.borrow(cs).replace(Some((
            control_led,
            idle_led,
            recording_led,
            RefCell::new(false),
        )));
    });

    critical_section::with(|cs| {
        SDWRITER
            .borrow(cs)
            .replace(Some(SdWriter::new("d", volume_mgr)));
    });

    loop {
        asm::wfi();
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
        let current_state: State = *LOGGER_STATE.borrow_ref(cs);
        let measurements = SENSOR
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .read_measurements()
            .unwrap();
        let begin = Instant::from_ticks(0);
        let mut buffer = ArrayString::<64>::new();
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

        UART.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str(&buffer)
            .unwrap();
        let mut leds = LED_SET.borrow_ref_mut(cs);
        let leds = leds.as_mut().unwrap();
        if current_state == State::Recording {
            let sd_writer_raw = &mut SDWRITER.borrow_ref_mut(cs);
            sd_writer_raw.as_mut().unwrap().write(buffer.as_str());

            leds.1.set_low().unwrap();
            if *leds.3.borrow() {
                leds.2.set_high().unwrap();
                leds.0.set_high().unwrap();
                leds.3.replace(false);
            } else {
                leds.2.set_low().unwrap();
                leds.0.set_low().unwrap();
                leds.3.replace(true);
            }
        } else {
            leds.2.set_low().unwrap();
            if *leds.3.borrow() {
                leds.0.set_high().unwrap();
                leds.1.set_high().unwrap();
                leds.3.replace(false);
            } else {
                leds.0.set_low().unwrap();
                leds.1.set_low().unwrap();
                leds.3.replace(true);
            }
        }
        buffer.clear();
        UART.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str(&buffer)
            .unwrap();
        let mut alarm = DEV_ALARM.borrow(cs).borrow_mut();
        alarm.as_mut().unwrap().clear_interrupt();
        let _ = alarm
            .as_mut()
            .unwrap()
            .schedule(MicrosDurationU32::millis(50));
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
                s.toggle();
                if *s == State::Idle {
                    SDWRITER
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .stop_recording();
                } else {
                    SDWRITER
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .start_recording();
                }
                button.clear_interrupt(gpio::Interrupt::EdgeLow);
            }
        }
    });
}

fn uart_write(a: &str) {
    critical_section::with(|cs| {
        UART.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str(a)
            .unwrap();
    });
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut a = ArrayString::<4096>::new();
    writeln!(a, "{}", info).ok();
    uart_write(a.as_str());

    loop {}
}
