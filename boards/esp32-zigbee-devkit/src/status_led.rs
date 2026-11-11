//! The board's addressable status LED.
//!
//! All three supported boards fit one WS2812-class RGB LED with its data
//! input on GPIO8 (ESP32-C6-DevKitC-1, ESP32-H2-DevKitM-1, Waveshare
//! ESP32-H2-Zero: WS2812B-0807 with a 10 kΩ resistor on GPIO8 and VDD on
//! 3V3). It is not a plain GPIO LED: driving GPIO8 high/low cannot light
//! it. The LED is driven by one RMT TX channel ([`StatusLed`], feature
//! `status-led`, target only); the bit encoding ([`encode`]) is plain
//! code that the host tests cover.
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

/// Number of RMT symbols in one LED frame: 24 data bits plus the latch
/// (reset) symbol that also ends the transmission. Fits one 48-word RMT
/// channel memory block, so no refill (and no interrupt latency) is ever
/// involved.
pub const FRAME_SYMBOLS: usize = 25;

// ESP32-C6/H2 RMT channel memory block: 48 symbols.
const _: () = assert!(FRAME_SYMBOLS <= 48, "frame must fit one RMT memory block");

/// One RMT symbol: `high` ticks high, then `low` ticks low.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pulse {
    pub high: u16,
    pub low: u16,
}

/// WS2812 bit and latch timing in RMT ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ws2812Timing {
    pub zero: Pulse,
    pub one: Pulse,
    /// Low time after the last bit that latches the frame.
    pub reset: u16,
}

const fn ticks(ns: u32, tick_hz: u32) -> u16 {
    let ticks = (ns as u64 * tick_hz as u64 + 500_000_000) / 1_000_000_000;
    assert!(
        ticks > 0 && ticks <= 0x7FFF,
        "RMT symbol length out of range"
    );
    ticks as u16
}

impl Ws2812Timing {
    /// Round nanosecond timings to the nearest tick at `tick_hz`; panics
    /// (at compile time in a `const`) if a length does not fit an RMT
    /// symbol.
    pub const fn from_ns(
        tick_hz: u32,
        t0h_ns: u32,
        t0l_ns: u32,
        t1h_ns: u32,
        t1l_ns: u32,
        reset_ns: u32,
    ) -> Self {
        Self {
            zero: Pulse {
                high: ticks(t0h_ns, tick_hz),
                low: ticks(t0l_ns, tick_hz),
            },
            one: Pulse {
                high: ticks(t1h_ns, tick_hz),
                low: ticks(t1l_ns, tick_hz),
            },
            reset: ticks(reset_ns, tick_hz),
        }
    }
}

/// 0.3/0.9 µs bit halves (the ESP-IDF `led_strip` RMT timing used on these
/// boards) and a 300 µs latch, long enough for WS2812B parts that need
/// more than 280 µs.
pub const WS2812_TIMING: Ws2812Timing =
    Ws2812Timing::from_ns(RMT_TICK_HZ, 300, 900, 900, 300, 300_000);

/// Encode one colour as 24 WS2812 bits: green, red, blue, MSB first.
pub const fn encode(r: u8, g: u8, b: u8, timing: &Ws2812Timing) -> [Pulse; 24] {
    let grb = (g as u32) << 16 | (r as u32) << 8 | b as u32;
    let mut bits = [timing.zero; 24];
    let mut i = 0;
    while i < 24 {
        if grb & (1 << (23 - i)) != 0 {
            bits[i] = timing.one;
        }
        i += 1;
    }
    bits
}

#[cfg(all(target_os = "none", feature = "status-led"))]
pub use driver::{StatusLed, StatusLedError};

#[cfg(all(target_os = "none", feature = "status-led"))]
mod driver {
    use esp_hal::Blocking;
    use esp_hal::gpio::Level;
    use esp_hal::peripherals::{GPIO8, RMT};
    use esp_hal::rmt::{Channel, Error, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
    use esp_hal::time::Rate;

    use super::{FRAME_SYMBOLS, WS2812_TIMING, encode};

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
            let mut frame = [PulseCode::end_marker(); FRAME_SYMBOLS];
            for (code, bit) in frame.iter_mut().zip(encode(r, g, b, &WS2812_TIMING)) {
                *code = PulseCode::new(Level::High, bit.high, Level::Low, bit.low);
            }
            // Latch low time; the zero second half ends the transmission.
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

#[cfg(test)]
mod tests {
    use super::*;

    fn ns(ticks: u16) -> u32 {
        (u64::from(ticks) * 1_000_000_000 / u64::from(RMT_TICK_HZ)) as u32
    }

    #[test]
    fn timing_is_within_ws2812b_high_time_windows() {
        let t = WS2812_TIMING;
        assert_eq!(t.zero, Pulse { high: 5, low: 14 });
        assert_eq!(t.one, Pulse { high: 14, low: 5 });
        // Decoders sample the high time: '0' 220..380 ns, '1' 580..1000 ns.
        assert!((220..=380).contains(&ns(t.zero.high)));
        assert!((580..=1_000).contains(&ns(t.one.high)));
        assert!((580..=1_000).contains(&ns(t.zero.low)));
        // Bit period about 1.2 µs, latch over 280 µs.
        assert_eq!(ns(t.zero.high + t.zero.low), ns(t.one.high + t.one.low));
        assert!(ns(t.reset) >= 280_000);
    }

    #[test]
    fn encodes_grb_msb_first() {
        let t = WS2812_TIMING;
        let bits = encode(0x80, 0x01, 0xA5, &t);
        let as_bits: u32 = bits
            .iter()
            .fold(0, |acc, p| (acc << 1) | u32::from(*p == t.one));
        assert_eq!(as_bits, 0x01_80_A5);
        assert!(bits.iter().all(|p| *p == t.zero || *p == t.one));
    }

    #[test]
    fn black_and_white_frames() {
        let t = WS2812_TIMING;
        assert!(encode(0, 0, 0, &t).iter().all(|p| *p == t.zero));
        assert!(encode(255, 255, 255, &t).iter().all(|p| *p == t.one));
    }

    #[test]
    fn other_tick_rates_round_to_nearest() {
        let t = Ws2812Timing::from_ns(80_000_000, 300, 900, 900, 300, 300_000);
        assert_eq!(t.zero, Pulse { high: 24, low: 72 });
        assert_eq!(t.reset, 24_000);
        let t = Ws2812Timing::from_ns(10_000_000, 350, 800, 700, 600, 50_000);
        assert_eq!(t.zero, Pulse { high: 4, low: 8 });
        assert_eq!(t.one, Pulse { high: 7, low: 6 });
    }
}
