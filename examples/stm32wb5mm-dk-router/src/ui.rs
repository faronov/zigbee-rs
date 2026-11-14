//! Indicator, display and touch task: owns SPI1 (OLED + RGB LED, which
//! share PA7) and the TSC touch key, so their bus rules stay in one place.
//!
//! Every [`TICK`] the task:
//! - (`touch`) takes one charge-transfer acquisition, classifies it with
//!   the product's [`TouchDetector`] and [`ButtonGestures`]: a tap shows
//!   the next page, a hold returns to the network page;
//! - (`rgb-led`) maps the router status and permit-join state to a colour
//!   with the product palette, writing the LED only when it changes;
//! - (`lcd`) redraws the current page once per [`REDRAW`] or at once after
//!   a page change.
//!
//! Each LED write takes ~0.4 ms with interrupts masked and each OLED flush
//! ~1.5 ms of blocking SPI; both are well inside one router receive slice.

#[cfg(feature = "lcd")]
use embassy_time::Duration;
use embassy_time::{Instant, Timer};
#[cfg(feature = "lcd")]
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
#[cfg(feature = "rgb-led")]
use router_app::StatusIndicator;
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
use stm32wb5mm_dk::spi1::{Spi1Bus, Spi1Pins};
#[cfg(feature = "touch")]
use stm32wb5mm_dk::touch::{self, Acquisition, TouchKey, TouchPins};
#[cfg(feature = "lcd")]
use stm32wb5mm_dk_router_product::screen::{self, Page};
#[cfg(feature = "touch")]
use stm32wb5mm_dk_router_product::{
    actions::{self, Action, Input},
    button::ButtonGestures,
    touch::TouchDetector,
};
#[cfg(feature = "lcd")]
use zigbee_display::PageFrame;

use crate::shared;

/// Indicator and touch sampling period.
const TICK: u64 = 50;
/// Display refresh period.
#[cfg(feature = "lcd")]
const REDRAW: Duration = Duration::from_secs(1);
/// Longest charge-transfer acquisition before it counts as failed.
#[cfg(feature = "touch")]
const TOUCH_TIMEOUT_US: u64 = 20_000;
/// Text rows are 10 px apart (FONT_6X10), starting 2 px down.
#[cfg(feature = "lcd")]
const ROW_PITCH: i32 = 10;

#[cfg(feature = "lcd")]
type Frame = PageFrame<[u8; 1024]>;

/// The task's resources, initialised before the radio starts (the OLED
/// reset and init sequence uses blocking delays).
pub struct Ui {
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    bus: Spi1Bus,
    #[cfg(feature = "lcd")]
    frame: Option<Frame>,
    #[cfg(feature = "touch")]
    key: Option<TouchKey>,
}

impl Ui {
    pub fn new(
        #[cfg(any(feature = "rgb-led", feature = "lcd"))] spi1: Spi1Pins,
        #[cfg(feature = "touch")] touch: TouchPins,
    ) -> Self {
        #[cfg(any(feature = "rgb-led", feature = "lcd"))]
        #[allow(unused_mut)]
        let mut bus = Spi1Bus::new(spi1);
        #[cfg(feature = "lcd")]
        let frame = match bus.lcd().init(&mut stm32wb5mm_dk::CycleDelay) {
            Ok(()) => Frame::new([0; 1024], 128, 64),
            Err(error) => {
                log::warn!("[WB] OLED init failed: {error:?}");
                None
            }
        };
        #[cfg(feature = "touch")]
        let key = TouchKey::new(touch)
            .inspect_err(|error| log::warn!("[WB] touch key unavailable: {error:?}"))
            .ok();
        Self {
            #[cfg(any(feature = "rgb-led", feature = "lcd"))]
            bus,
            #[cfg(feature = "lcd")]
            frame,
            #[cfg(feature = "touch")]
            key,
        }
    }
}

#[embassy_executor::task]
pub async fn run(mut ui: Ui) -> ! {
    #[cfg(feature = "rgb-led")]
    let mut indicator = StatusIndicator::new(
        &stm32wb5mm_dk_router_product::indication::STATUS_PALETTE,
        now_ms() as u32,
    );
    #[cfg(feature = "rgb-led")]
    let mut shown: Option<router_app::Rgb> = None;
    #[cfg(feature = "touch")]
    let (mut detector, mut gestures) = (TouchDetector::new(), ButtonGestures::new());
    #[cfg(feature = "lcd")]
    #[cfg_attr(not(feature = "touch"), allow(unused_mut))]
    let (mut page, mut next_draw) = (Page::Network, Instant::now());

    loop {
        Timer::after_millis(TICK).await;

        #[cfg(feature = "touch")]
        if let Some(key) = ui.key.as_mut()
            && let Some(count) = acquire(key).await
        {
            let now = now_ms();
            let touched = detector.update(count, now);
            if let Some(gesture) = gestures.sample(touched, now) {
                match actions::action(Input::Touch(gesture)) {
                    Some(Action::NextPage) => page = page.next(),
                    Some(Action::FirstPage) => page = Page::Network,
                    _ => {}
                }
                next_draw = Instant::now();
            }
        }

        #[cfg(feature = "rgb-led")]
        {
            let now = now_ms() as u32;
            if let Some(status) = shared::STATUS.get() {
                indicator.set_status(status, now);
            }
            indicator.set_permit_join(shared::NETWORK.get().permit_join_secs.is_some());
            let colour = indicator.update(now);
            if shown != Some(colour) {
                // At most one write per tick: the LED select idle time
                // (10 ms) has always elapsed, so this never busy-waits.
                match ui.bus.set_rgb([colour.r, colour.g, colour.b]) {
                    Ok(()) => shown = Some(colour),
                    Err(error) => log::warn!("[WB] RGB LED write failed: {error:?}"),
                }
            }
        }

        #[cfg(feature = "lcd")]
        if Instant::now() >= next_draw
            && let Some(frame) = ui.frame.as_mut()
        {
            next_draw = Instant::now() + REDRAW;
            let mut network = shared::NETWORK.get();
            if let Some(status) = shared::STATUS.get() {
                network.state = screen::State::from_status(status);
            }
            let lines = screen::render(page, &network, &shared::SENSORS.get());
            draw(frame, &lines);
            if let Err(error) = ui.bus.lcd().flush(frame) {
                log::warn!("[WB] OLED flush failed: {error:?}");
            }
        }
    }
}

#[cfg(any(feature = "rgb-led", feature = "touch"))]
fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// One charge-transfer acquisition; `None` on a max-count error or timeout.
#[cfg(feature = "touch")]
async fn acquire(key: &mut TouchKey) -> Option<u16> {
    key.discharge();
    Timer::after_micros(u64::from(touch::DISCHARGE_US)).await;
    let started = Instant::now();
    key.start();
    loop {
        match key.poll() {
            Acquisition::Count(count) => return Some(count),
            Acquisition::MaxCount => return None,
            Acquisition::Busy if started.elapsed().as_micros() >= TOUCH_TIMEOUT_US => return None,
            Acquisition::Busy => Timer::after_micros(250).await,
        }
    }
}

#[cfg(feature = "lcd")]
fn draw(frame: &mut Frame, lines: &[screen::Line; screen::ROWS]) {
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = frame.clear(BinaryColor::Off);
    for (row, line) in (0i32..).zip(lines) {
        let origin = Point::new(1, 2 + row * ROW_PITCH);
        let _ = Text::with_baseline(line.as_str(), origin, style, Baseline::Top).draw(frame);
    }
}
