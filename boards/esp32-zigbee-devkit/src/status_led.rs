//! The board's addressable status LED.
//!
//! All three supported boards fit one WS2812-class RGB LED with its data
//! input on GPIO8 (ESP32-C6-DevKitC-1, ESP32-H2-DevKitM-1, Waveshare
//! ESP32-H2-Zero: WS2812B-0807 with a 10 kΩ resistor on GPIO8 and VDD on
//! 3V3). It is not a plain GPIO LED: driving GPIO8 high/low cannot light
//! it. The LED is driven by one RMT TX channel ([`StatusLed`], feature
//! `status-led`, target only). The colour and bit encoding come from the
//! platform-neutral `zigbee-ws2812` crate (zigbee-rs-drivers); this module
//! only maps its frame onto RMT symbols.
//!
//! The colour byte order depends on the fitted part (`color_order`): the
//! Espressif devkits take WS2812B GRB, the Waveshare ESP32-H2-Zero
//! (feature `waveshare-h2-zero`) takes RGB; sent GRB, its green shows red
//! (seen on hardware).
//!
//! The VDD of the H2-Zero LED is not switched, so the LED draws its
//! quiescent current even when dark. Sleepy images must not construct
//! [`StatusLed`]; they leave GPIO8 to the product's pad isolation.

/// GPIO carrying the WS2812 data input on every supported board.
pub const STATUS_LED_GPIO: u8 = 8;

/// RMT tick rate used for the LED: ESP32-H2 XTAL 32 MHz / 2 and
/// ESP32-C6 PLL 80 MHz / 5 (the RMT source clocks `esp-hal` selects), with
/// a channel divider of 1. 62.5 ns per tick.
pub const RMT_TICK_HZ: u32 = 16_000_000;

#[cfg(feature = "status-led")]
pub use zigbee_ws2812::ColorOrder;

/// Colour byte order of the LED on [`crate::BOARD`], fixed at build time
/// so the encoder folds it away.
#[cfg(feature = "status-led")]
pub const COLOR_ORDER: ColorOrder = color_order(crate::BOARD);

/// Colour byte order of the LED fitted to `board`.
#[cfg(feature = "status-led")]
pub const fn color_order(board: crate::Board) -> ColorOrder {
    match board {
        crate::Board::EspressifC6DevKitC1 | crate::Board::EspressifH2DevKitM1 => ColorOrder::Grb,
        // Same as the S3/C3/C6-Zero (arduino-esp32 variants:
        // `LED_COLOR_ORDER_RGB`); confirmed on an H2-Zero.
        crate::Board::WaveshareH2Zero => ColorOrder::Rgb,
    }
}

/// WS2812 bit and latch timing in RMT ticks: the `zigbee-ws2812`
/// WS2812B timing (0.3/0.9 µs bit halves, the ESP-IDF `led_strip` RMT
/// timing used on these boards, and a 300 µs latch for WS2812B parts that
/// need more than 280 µs) at [`RMT_TICK_HZ`].
#[cfg(feature = "status-led")]
pub const WS2812_TIMING: zigbee_ws2812::Timing =
    zigbee_ws2812::Timing::from_ns(RMT_TICK_HZ, &zigbee_ws2812::WS2812B_NS);

// An RMT symbol half holds 15 bits of ticks; one frame (24 data bits and
// the latch symbol that also ends the transmission) must fit the 48-word
// ESP32-C6/H2 RMT channel memory block, so no refill (and no interrupt
// latency) is ever involved.
#[cfg(feature = "status-led")]
const _: () = {
    assert!(
        WS2812_TIMING.max_ticks() <= 0x7FFF,
        "RMT symbol length out of range"
    );
    assert!(
        zigbee_ws2812::FRAME_SYMBOLS <= 48,
        "frame must fit one RMT memory block"
    );
};

#[cfg(all(target_os = "none", feature = "status-led"))]
pub use driver::{StatusLed, StatusLedError};

#[cfg(all(target_os = "none", feature = "status-led"))]
mod driver {
    use esp_hal::Blocking;
    use esp_hal::gpio::Level;
    use esp_hal::peripherals::{GPIO8, RMT};
    use esp_hal::rmt::{Channel, Error, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
    use esp_hal::time::Rate;

    use zigbee_ws2812::{FRAME_SYMBOLS, encode};

    use super::{COLOR_ORDER, WS2812_TIMING};

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum StatusLedError {
        Rmt(Error),
        /// An earlier start error consumed the RMT channel.
        ChannelLost,
    }

    impl From<Error> for StatusLedError {
        fn from(error: Error) -> Self {
            Self::Rmt(error)
        }
    }

    /// The WS2812 on GPIO8, owned through RMT channel 0 (blocking mode, no
    /// interrupt). Owns the RMT peripheral and GPIO8.
    pub struct StatusLed {
        /// `None` only after the driver lost the channel (an RMT start
        /// error); the LED then stays as it was.
        channel: Option<Channel<'static, Blocking, Tx>>,
    }

    impl StatusLed {
        /// Configure RMT channel 0 on GPIO8 with the line idling low. The
        /// LED keeps its power-on state (dark) until the first
        /// [`Self::write`].
        pub fn new(rmt: RMT<'static>, pin: GPIO8<'static>) -> Result<Self, StatusLedError> {
            let rmt = Rmt::new(rmt, Rate::from_hz(super::RMT_TICK_HZ))?;
            let channel = rmt.channel0.configure_tx(
                pin,
                TxChannelConfig::default()
                    .with_clk_divider(1)
                    .with_idle_output(true)
                    .with_idle_output_level(Level::Low),
            )?;
            Ok(Self {
                channel: Some(channel),
            })
        }

        /// Send one colour and wait for the frame and its latch time
        /// (about 330 µs, busy wait with interrupts enabled).
        pub fn write(&mut self, r: u8, g: u8, b: u8) -> Result<(), StatusLedError> {
            let channel = self.channel.take().ok_or(StatusLedError::ChannelLost)?;
            // 24 data bits, then the latch low time; the zero second half
            // of the latch symbol ends the transmission (the same layout
            // as `zigbee_ws2812::encode_frame`).
            let mut frame = [PulseCode::end_marker(); FRAME_SYMBOLS];
            for (code, bit) in frame
                .iter_mut()
                .zip(encode(COLOR_ORDER, r, g, b, &WS2812_TIMING))
            {
                *code = PulseCode::new(Level::High, bit.high, Level::Low, bit.low);
            }
            frame[FRAME_SYMBOLS - 1] =
                PulseCode::new(Level::Low, WS2812_TIMING.reset, Level::Low, 0);
            match channel.transmit(&frame)?.wait() {
                Ok(channel) => {
                    self.channel = Some(channel);
                    Ok(())
                }
                Err((error, channel)) => {
                    self.channel = Some(channel);
                    Err(error.into())
                }
            }
        }
    }
}

#[cfg(all(test, feature = "status-led"))]
mod tests {
    use super::*;

    #[test]
    fn board_color_orders() {
        assert_eq!(
            color_order(crate::Board::EspressifC6DevKitC1),
            ColorOrder::Grb
        );
        assert_eq!(
            color_order(crate::Board::EspressifH2DevKitM1),
            ColorOrder::Grb
        );
        assert_eq!(color_order(crate::Board::WaveshareH2Zero), ColorOrder::Rgb);
    }

    #[test]
    fn rmt_timing_at_16_mhz() {
        // 62.5 ns ticks: '0' 312.5/875 ns, '1' 875/312.5 ns, 300 µs latch.
        assert_eq!(
            WS2812_TIMING.zero,
            zigbee_ws2812::Pulse { high: 5, low: 14 }
        );
        assert_eq!(WS2812_TIMING.one, zigbee_ws2812::Pulse { high: 14, low: 5 });
        assert_eq!(WS2812_TIMING.reset, 4_800);
    }
}
