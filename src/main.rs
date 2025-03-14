//! This example shows how to send messages between the two cores in the RP2040 chip.
//!
//! The LED on the RP Pico W board is connected differently. See wifi_blinky.rs.

#![no_std]
#![no_main]

mod draw;

use defmt::*;
use embassy_executor::Executor;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Timer};
use gc9a01::prelude::*;
use static_cell::StaticCell;
use tinyrand::Probability;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, [i32; 10], 1> = Channel::new();

type Spi1Mutex = embassy_sync::blocking_mutex::CriticalSectionMutex<
    core::cell::RefCell<
        embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
    >,
>;
pub type BufferedDriver = gc9a01::Gc9a01<
    gc9a01::prelude::SPIInterface<
        embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<
            'static,
            embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
            embassy_rp::gpio::Output<'static>,
        >,
        embassy_rp::gpio::Output<'static>,
    >,
    gc9a01::prelude::DisplayResolution240x240,
    gc9a01::mode::BufferedGraphics<gc9a01::prelude::DisplayResolution240x240>,
>;
static SPI1_MUTEX: StaticCell<Spi1Mutex> = StaticCell::new();
static LCD: StaticCell<BufferedDriver> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let mut p_lcd_backlight = Output::new(p.PIN_25, Level::Low);
    let p_lcd_dc_select = Output::new(p.PIN_8, Level::Low);
    let p_lcd_cs = Output::new(p.PIN_9, Level::Low);
    let mut p_lcd_reset = Output::new(p.PIN_12, Level::Low);

    let p_lcd_sck = p.PIN_10;
    let p_lcd_mosi = p.PIN_11;

    info!("Initializing SPI...");
    let mut spi_config = embassy_rp::spi::Config::default();
    spi_config.frequency = 40_000_000;
    spi_config.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
    spi_config.polarity = embassy_rp::spi::Polarity::IdleLow;
    let spi =
        embassy_rp::spi::Spi::new_txonly(p.SPI1, p_lcd_sck, p_lcd_mosi, p.DMA_CH0, spi_config);
    let spi_driver = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(
        SPI1_MUTEX.init(embassy_sync::blocking_mutex::CriticalSectionMutex::new(
            core::cell::RefCell::new(spi),
        )),
        p_lcd_cs,
    );
    let spi_device = gc9a01::SPIDisplayInterface::new(spi_driver, p_lcd_dc_select);

    info!("Initializing lcd driver...");
    let mut lcd = gc9a01::Gc9a01::new(
        spi_device,
        DisplayResolution240x240,
        DisplayRotation::Rotate0,
    );
    lcd.reset(&mut p_lcd_reset, &mut Delay).unwrap();
    lcd.init_with_addr_mode(&mut Delay).unwrap();

    info!("Init LCD framebuffer...");
    let fb = LCD.init(lcd.into_buffered_graphics());

    info!("Activating lcd...");
    p_lcd_backlight.set_high();

    let _display = spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                unwrap!(spawner.spawn(draw::lcd_task(fb)));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(core0_task()));
    });
}

#[embassy_executor::task]
async fn core0_task() {
    info!("Hello from core 0");
    use tinyrand::{Rand, RandRange, StdRand};
    let mut rand = StdRand::default();
    let mut points: [i32; 10] = [0i32; 10];
    let prob = Probability::new(0.5);
    loop {
        for p in points.iter_mut() {
            let m: u16 = rand.next_range(0..8);
            let d: bool = rand.next_bool(prob);
            if d {
                *p += m as i32;
                if *p > 40 {
                    *p = 40;
                }
            } else {
                *p -= m as i32;
                if *p < -40 {
                    *p = -40;
                }
            }
        }
        CHANNEL.send(points).await;

        Timer::after_millis(100).await;
    }
}
