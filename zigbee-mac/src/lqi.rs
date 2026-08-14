//! Link Quality Indicator normalisation.
//!
//! Every LQI that reaches the stack — [`PhyRxFrame::lqi`](crate::PhyRxFrame),
//! [`PanDescriptor::lqi`](crate::primitives::PanDescriptor) and the
//! `McpsDataIndication`/MLME indications — is defined to be an **IEEE
//! 802.15.4 LQI in `0..=255`**, where larger is better. NWK link cost
//! (R22 §3.6.3.1) and rejoin parent selection (R22 §3.6.1.4.2) are calibrated
//! against that scale, so a backend that reports a raw hardware value on a
//! different scale silently biases routing and parent choice.
//!
//! Backends whose hardware reports something else convert here, exactly once,
//! at the point where the value enters the stack.
//!
//! Two conversions live here:
//!
//! * [`nrf_from_hardware`] — Nordic's energy-detect byte, which has a
//!   vendor-documented scale factor.
//! * [`from_rssi_dbm`] — the RSSI-derived estimate used by backends whose
//!   radios expose no IEEE-normalized quality byte at all (EFR32, EFR32S2,
//!   PHY6222, CC2340). A raw modem value on an undocumented scale is *not* an
//!   acceptable substitute and must never reach the stack.

/// Convert a Nordic nRF5x hardware LQI byte into an IEEE 802.15.4 LQI.
///
/// The nRF52 radio appends an energy-detect value (`ED`) after the payload in
/// IEEE 802.15.4 mode; `embassy_nrf::radio::ieee802154::Packet::lqi()` returns
/// that raw byte unchanged. The Nordic product specification defines the
/// conversion as
///
/// ```text
/// LQI = ED * ED_RSSISCALE   with ED_RSSISCALE = 4, saturating at 255
/// ```
///
/// so a raw byte of `63` already maps to `252` and anything from `64` upwards
/// saturates at `255`. Without this multiply the stack sees roughly a quarter
/// of the real link quality: a healthy link is reported as poor, link cost is
/// inflated, and R22 parent selection rejects perfectly good parents through
/// the link-cost gate.
///
/// Saturating multiply in 8-bit, per the Nordic definition.
pub const fn nrf_from_hardware(hardware_lqi: u8) -> u8 {
    hardware_lqi.saturating_mul(4)
}

/// Lowest RSSI still distinguished by [`from_rssi_dbm`]; anything at or below
/// this maps to LQI `0`.
pub const RSSI_FLOOR_DBM: i8 = -100;

/// Highest RSSI still distinguished by [`from_rssi_dbm`]; anything at or above
/// this maps to LQI `255`.
pub const RSSI_CEILING_DBM: i8 = -20;

/// Derive an IEEE 802.15.4 LQI in `0..=255` from a received-signal strength in
/// dBm.
///
/// # Why RSSI at all
///
/// R22 §3.6.3.1 leaves the LQI computation implementation-defined: a radio may
/// derive it from signal strength, from a chip-correlation/SNR estimate, or
/// from both. Backends whose hardware exposes no IEEE-normalized correlation
/// value — EFR32, EFR32S2, PHY6222 and CC2340 — use signal strength, which is
/// the only calibrated, documented per-frame quality metric those radios
/// report.
///
/// # Policy
///
/// This is *this stack's* initial LQI estimate policy, not a hardware-derived
/// curve: RSSI is clamped to
/// [`RSSI_FLOOR_DBM`]`..=`[`RSSI_CEILING_DBM`] and mapped linearly onto
/// `0..=255`.
///
/// ```text
/// LQI = (clamp(rssi, -100, -20) + 100) * 255 / 80
/// ```
///
/// The endpoints are conventional: roughly the 802.15.4 receiver sensitivity
/// floor, and a level well above which no additional link margin is useful.
/// The mapping is monotonic and saturating, so it never inverts the ordering
/// of two links and never produces an out-of-range LQI.
///
/// # Calibration gate
///
/// The linear curve is deliberately the behaviour this repository already
/// shipped for EFR32/EFR32S2/PHY6222; it is centralised here rather than
/// re-tuned, because changing the curve changes NWK link cost thresholds and
/// R22 parent selection on every affected backend. Real per-radio calibration
/// — measuring PER against RSSI on hardware and fitting the LQI bands so that
/// link cost 1..7 lands where the radio's actual packet delivery does — is
/// still outstanding and must be done per backend before these numbers can be
/// called production-accurate. Do not substitute a different curve without
/// that hardware evidence.
pub const fn from_rssi_dbm(rssi_dbm: i8) -> u8 {
    let floor = RSSI_FLOOR_DBM as i16;
    let ceiling = RSSI_CEILING_DBM as i16;
    let raw = rssi_dbm as i16;

    // `i16::clamp` is not a `const fn`, so branch explicitly.
    let clamped = if raw < floor {
        floor
    } else if raw > ceiling {
        ceiling
    } else {
        raw
    };

    (((clamped - floor) as u16) * 255 / 80) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nrf_lqi_is_a_saturating_times_four() {
        // No signal stays no signal.
        assert_eq!(nrf_from_hardware(0), 0);
        // Mid-scale hardware readings scale linearly.
        assert_eq!(nrf_from_hardware(1), 4);
        assert_eq!(nrf_from_hardware(32), 128);
        // Largest raw value that does not saturate.
        assert_eq!(nrf_from_hardware(63), 252);
        // ED_RSSISCALE saturates at 255 from raw 64 upwards.
        assert_eq!(nrf_from_hardware(64), 255);
        assert_eq!(nrf_from_hardware(65), 255);
        assert_eq!(nrf_from_hardware(128), 255);
        assert_eq!(nrf_from_hardware(u8::MAX), 255);
    }

    #[test]
    fn nrf_lqi_normalisation_is_monotonic() {
        let mut previous = 0u8;
        for raw in 0..=u8::MAX {
            let normalized = nrf_from_hardware(raw);
            assert!(
                normalized >= previous,
                "raw {raw} produced {normalized}, below the previous {previous}"
            );
            previous = normalized;
        }
        assert_eq!(previous, 255);
    }

    /// The bug this helper fixes: raw hardware bytes look like poor links.
    ///
    /// A raw byte of 55 is a strong link (220 on the IEEE scale, link cost 1),
    /// but used unconverted it lands in the weak band and inflates NWK link
    /// cost.
    #[test]
    fn raw_hardware_lqi_would_understate_link_quality() {
        let raw = 55u8;
        let normalized = nrf_from_hardware(raw);
        assert_eq!(normalized, 220);
        assert!(u16::from(normalized) > u16::from(raw) * 3);
    }

    // ── from_rssi_dbm ────────────────────────────────────────────────

    #[test]
    fn rssi_mapping_hits_both_endpoints() {
        assert_eq!(from_rssi_dbm(RSSI_FLOOR_DBM), 0);
        assert_eq!(from_rssi_dbm(RSSI_CEILING_DBM), 255);
    }

    #[test]
    fn rssi_mapping_saturates_outside_the_calibrated_window() {
        // Below the floor is still "no usable signal", not a wrapped value.
        assert_eq!(from_rssi_dbm(i8::MIN), from_rssi_dbm(RSSI_FLOOR_DBM));
        assert_eq!(from_rssi_dbm(-128), 0);
        assert_eq!(from_rssi_dbm(-101), 0);
        // Above the ceiling adds no further link margin.
        assert_eq!(from_rssi_dbm(-19), 255);
        assert_eq!(from_rssi_dbm(0), 255);
        assert_eq!(from_rssi_dbm(i8::MAX), 255);
    }

    #[test]
    fn rssi_mapping_is_monotonic_over_the_whole_i8_range() {
        let mut previous = 0u8;
        for raw in i8::MIN..=i8::MAX {
            let normalized = from_rssi_dbm(raw);
            assert!(
                normalized >= previous,
                "rssi {raw} dBm produced {normalized}, below the previous {previous}"
            );
            previous = normalized;
        }
        assert_eq!(previous, 255);
    }

    #[test]
    fn rssi_mapping_is_strictly_increasing_inside_the_window() {
        let mut previous = from_rssi_dbm(RSSI_FLOOR_DBM);
        for raw in (RSSI_FLOOR_DBM + 1)..=RSSI_CEILING_DBM {
            let normalized = from_rssi_dbm(raw);
            assert!(
                normalized > previous,
                "rssi {raw} dBm did not improve on {previous}"
            );
            previous = normalized;
        }
    }

    /// The mapping this repository already shipped for EFR32/EFR32S2/PHY6222,
    /// reproduced here so centralising it stays provably behaviour-preserving.
    #[test]
    fn rssi_mapping_matches_the_previous_per_backend_formula() {
        for raw in i8::MIN..=i8::MAX {
            let clamped = (raw as i16).clamp(-100, -20);
            let expected = (((clamped + 100) as u16) * 255 / 80) as u8;
            assert_eq!(from_rssi_dbm(raw), expected, "mismatch at {raw} dBm");
        }
    }

    #[test]
    fn rssi_mapping_is_usable_in_const_context() {
        const STRONG: u8 = from_rssi_dbm(-30);
        const WEAK: u8 = from_rssi_dbm(-95);
        const { assert!(STRONG > WEAK) };
    }
}
