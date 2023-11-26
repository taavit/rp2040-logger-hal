//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod sd_writer;
mod sensor;

use bsp::hal::gpio::{Pin, PullNone, PullUp};
use bsp::hal::timer::Alarm;
use bsp::hal::uart::UartPeripheral;
use bsp::hal::Timer;
use critical_section::Mutex;
use embedded_sdmmc::{SdCard, VolumeManager};
use heapless::{String, Vec};
use sd_writer::{DummyTimesource, SdWriter};
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
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

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

type SdCardType = SdCard<
    bsp::hal::Spi<
        bsp::hal::spi::Enabled,
        pac::SPI1,
        (
            Pin<gpio::bank0::Gpio11, gpio::FunctionSpi, PullNone>,
            Pin<gpio::bank0::Gpio12, gpio::FunctionSpi, PullUp>,
            Pin<gpio::bank0::Gpio10, gpio::FunctionSpi, PullNone>,
        ),
    >,
    Pin<gpio::bank0::Gpio13, gpio::FunctionSio<gpio::SioOutput>, PullDown>,
    Timer,
>;

type ButtonPin = Pin<gpio::bank0::Gpio18, gpio::FunctionSio<gpio::SioInput>, PullDown>;
type ControlLed = Pin<gpio::bank0::Gpio25, gpio::FunctionSio<gpio::SioOutput>, PullDown>;
type IdleLed = Pin<gpio::bank0::Gpio16, gpio::FunctionSio<gpio::SioOutput>, PullDown>;
type RecordingLed = Pin<gpio::bank0::Gpio17, gpio::FunctionSio<gpio::SioOutput>, PullDown>;

type LedSet = (ControlLed, IdleLed, RecordingLed, RefCell<bool>);

type I2CPin = bsp::hal::i2c::I2C<
    bsp::pac::I2C1,
    (
        Pin<gpio::bank0::Gpio14, gpio::FunctionI2C, PullDown>,
        Pin<gpio::bank0::Gpio15, gpio::FunctionI2C, PullDown>,
    ),
>;

type UartLogger = UartPeripheral<
    bsp::hal::uart::Enabled,
    pac::UART0,
    (
        Pin<gpio::bank0::Gpio0, gpio::FunctionUart, PullDown>,
        Pin<gpio::bank0::Gpio1, gpio::FunctionUart, PullDown>,
    ),
>;
static GLOBAL_PINS: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static LOGGER_STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::Idle));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
static DEV_ALARM: Mutex<RefCell<Option<bsp::hal::timer::Alarm0>>> = Mutex::new(RefCell::new(None));
static UART: Mutex<RefCell<Option<UartLogger>>> = Mutex::new(RefCell::new(None));
static SENSOR: Mutex<RefCell<Option<LSM303D<I2CPin>>>> = Mutex::new(RefCell::new(None));
static SDWRITER: Mutex<RefCell<Option<SdWriter>>> = Mutex::new(RefCell::new(None));
static LED_SET: Mutex<RefCell<Option<LedSet>>> = Mutex::new(RefCell::new(None));
static CACHE: Mutex<RefCell<Vec<String<64>, 10>>> = Mutex::new(RefCell::new(Vec::new()));

static USB_CACHE: Mutex<RefCell<Vec<String<64>, 100>>> = Mutex::new(RefCell::new(Vec::new()));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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

    // let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut delay = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function::<gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function::<gpio::FunctionUart>(),
    );
    let uart = bsp::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let sda_pin = pins.gpio14.into_function::<bsp::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio15.into_function::<bsp::hal::gpio::FunctionI2C>();

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

    // Set up the GPIO pin that will be our input
    let in_pin = pins.gpio18.into_pull_down_input();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    in_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    in_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Init usb device
    let force_vbus_detect_bit = true;

    let usb_bus = bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        force_vbus_detect_bit,
        &mut pac.RESETS,
    );

    let bus_allocator = UsbBusAllocator::new(usb_bus);
    let mut usb_serial = SerialPort::new(&bus_allocator);

    let mut usb_dev = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
        .product("AccMag tracker")
        .device_class(USB_CLASS_CDC)
        .build();

    unsafe {
        // Interrupt for USBCTRL disables for unknown reason IO_IRQ_BANK0 and buttons,
        // thefore polling will be in global loop.
        // pac::NVIC::unmask(bsp::hal::pac::Interrupt::USBCTRL_IRQ);
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::TIMER_IRQ_0);
    };

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio10.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio11.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio12.reconfigure();

    let spi_cs = pins.gpio13.into_push_pull_output();

    // Create an SPI driver instance for the SPI1 device
    let spi = bsp::hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        400.kHz(), // card initialization happens at low baud rate
        embedded_hal::spi::MODE_0,
    );

    let sdcard = embedded_sdmmc::SdCard::new(spi, spi_cs, delay);

    let volume_mgr: VolumeManager<SdCardType, DummyTimesource> =
        embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource {});

    critical_section::with(|cs| {
        let mut alarm = delay.alarm_0().unwrap();
        let _ = alarm.schedule(MicrosDurationU32::Hz(10));
        alarm.enable_interrupt();
        DEV_ALARM.replace(cs, Some(alarm));
        TIMER.replace(cs, Some(delay));
        GLOBAL_PINS.borrow(cs).replace(Some(in_pin));
        UART.borrow(cs).replace(Some(uart));
        SENSOR.borrow(cs).replace(Some(sensor));
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
        if usb_dev.poll(&mut [&mut usb_serial]) {
            critical_section::with(|cs| {
                let mut vec = USB_CACHE.borrow_ref_mut(cs);
                for row in vec.iter() {
                    let e = usb_serial.write(row.as_bytes());
                    if e.is_err() {
                        let mut buf = String::<128>::new();
                        let _ = write!(buf, "Problem with writing: {}\r\n", &row[..16]);
                        uart_write(buf.as_str());
                    }
                }
                vec.clear();
            });
        }
    }
}

fn format_measurements<const SIZE: usize>(
    mut message: &mut String<SIZE>,
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

fn handle_timer() {
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
        let mut buffer = String::<64>::new();
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

        uart_write(buffer.as_str());
        let mut cache = USB_CACHE.borrow_ref_mut(cs);
        if cache.is_full() {
            cache.remove(0);
        }
        cache.push(buffer.clone()).unwrap();
        drop(cache);
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
fn TIMER_IRQ_0() {
    handle_timer();
}

fn handle_button() {
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

#[interrupt]
fn IO_IRQ_BANK0() {
    handle_button();
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
    let mut a = String::<4096>::new();
    writeln!(a, "{}", info).ok();
    uart_write(a.as_str());

    loop {}
}
