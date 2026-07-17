//! Explicit raw test state machine: cycle channels 11 -> 18 -> 26, send a
//! Beacon Request on each, listen for Beacon responses within a fixed
//! window, and record everything in the SRAM diagnostic record.
//!
//! This is deliberately *not* a general MAC/MLME implementation — no
//! association, no data frames, no retries beyond the bounded RX window.
//! It exists to prove PHY/DMA/channel bring-up end-to-end on hardware with
//! externally-visible, checksum-protected results.

#![cfg(target_arch = "tc32")]

use crate::diag;
use crate::platform;
use crate::radio::{self, RxOutcome, frame::CoordAddress};

/// Channels to cycle, in order. Chosen to span the 2.4 GHz band's three
/// power-band boundaries used by `radio::phy::set_channel` (11-16, 17-22,
/// 23-26), so a working cycle exercises all three band constants.
#[cfg(not(feature = "control-channel-15"))]
pub const CHANNELS: &[u8] = &[11, 18, 26];

#[cfg(feature = "control-channel-15")]
pub const CHANNELS: &[u8] = &[15];

/// Upper bound on frames classified per RX window. Bounds the window's CPU
/// work independent of the timer deadline (belt-and-suspenders alongside
/// `radio::RX_WINDOW_TICKS`).
pub const MAX_FRAMES_PER_WINDOW: u16 = 4;

/// Fixed settle time between finishing one channel's RX window and
/// programming the next channel.
pub const INTER_CHANNEL_SETTLE_TICKS: u32 = platform::timer::TICKS_PER_MS * 2;

pub const SCAN_DURATION_EXPONENT: u8 = 3;
pub const SCAN_DURATION_TICKS: u32 = 960 * ((1u32 << SCAN_DURATION_EXPONENT) + 1) * 16 * 24;
pub const SCAN_DWELL_TOLERANCE_TICKS: u32 = platform::timer::TICKS_PER_MS;

/// Run the channel-cycle state machine forever. Never returns — this *is*
/// the firmware's `main`. Every wait inside is bounded (see `radio::phy`
/// and `platform::timer::wait_until`); only the outer supervisory loop is
/// deliberately infinite ("repeatedly cycle channels 11, 18, and 26").
pub fn run() -> ! {
    #[cfg(feature = "active-scan")]
    {
        run_active_scan()
    }
    #[cfg(not(feature = "active-scan"))]
    {
        run_raw_cycle()
    }
}

#[cfg(not(feature = "active-scan"))]
fn run_raw_cycle() -> ! {
    diag::init();
    radio::init();

    let mut seq: u8 = 0;

    loop {
        for (idx, &channel) in CHANNELS.iter().enumerate() {
            diag::update(|r| {
                r.state = diag::state::SET_CHANNEL;
                r.channel = channel;
                r.channel_index = idx as u8;
                r.uptime_ticks = platform::timer::now_ticks();
            });
            radio::set_channel(channel);

            transmit_beacon_request(&mut seq);

            diag::update(|r| r.state = diag::state::RX_WINDOW);
            radio::rx_window(MAX_FRAMES_PER_WINDOW, |outcome| {
                record_outcome(channel, outcome);
            });

            // Cheap, frequent externally-visible proof that a cache overrun
            // (the historical TLSR8258 bug class this bring-up guards
            // against) has not silently corrupted `.data`.
            diag::verify_cache_canary();

            platform::timer::sleep_ticks(INTER_CHANNEL_SETTLE_TICKS);
        }

        diag::update(|r| {
            r.cycles_completed = r.cycles_completed.wrapping_add(1);
            r.state = diag::state::NEXT_CHANNEL;
            r.uptime_ticks = platform::timer::now_ticks();
        });
    }
}

#[cfg(feature = "active-scan")]
fn run_active_scan() -> ! {
    diag::init();
    radio::init();

    let mut seq = 0u8;
    loop {
        for channel in 11u8..=26 {
            diag::update(|r| {
                r.state = diag::state::SET_CHANNEL;
                r.channel = channel;
                r.channel_index = channel - 11;
                r.uptime_ticks = platform::timer::now_ticks();
                r.scan_duration_exponent = SCAN_DURATION_EXPONENT;
            });
            radio::set_channel(channel);
            transmit_beacon_request(&mut seq);

            diag::update(|r| r.state = diag::state::ACTIVE_SCAN);
            let elapsed = radio::rx_window_for(SCAN_DURATION_TICKS, u16::MAX, |outcome| {
                record_outcome(channel, outcome);
            });
            let dwell_ok = elapsed >= SCAN_DURATION_TICKS
                && elapsed <= SCAN_DURATION_TICKS + SCAN_DWELL_TOLERANCE_TICKS;
            diag::update(|r| {
                r.last_scan_ticks = elapsed;
                r.scan_dwell_ok = if dwell_ok {
                    diag::DIAG_TRUE
                } else {
                    diag::DIAG_FALSE
                };
                r.uptime_ticks = platform::timer::now_ticks();
            });
            diag::verify_cache_canary();
        }
        diag::update(|r| {
            r.cycles_completed = r.cycles_completed.wrapping_add(1);
            r.state = diag::state::NEXT_CHANNEL;
        });
    }
}

fn transmit_beacon_request(seq: &mut u8) {
    diag::update(|r| r.state = diag::state::TX_BEACON_REQUEST);
    *seq = seq.wrapping_add(1);
    let outcome = radio::send_beacon_request(*seq);
    let csma = radio::csma_stats();
    diag::update(|r| {
        r.last_seq = *seq;
        match outcome {
            radio::TxOutcome::Sent => {
                r.tx_success_count = r.tx_success_count.wrapping_add(1);
            }
            radio::TxOutcome::InvalidFrame => {
                r.tx_invalid_frame_count = r.tx_invalid_frame_count.wrapping_add(1);
            }
            radio::TxOutcome::ChannelAccessFailure => {}
            radio::TxOutcome::Timeout => {
                r.tx_timeout_count = r.tx_timeout_count.wrapping_add(1);
            }
        }
        r.cca_attempt_count = csma.cca_attempts;
        r.cca_busy_count = csma.cca_busy;
        r.channel_access_failure_count = csma.channel_access_failures;
    });
}

fn record_outcome(channel: u8, outcome: RxOutcome) {
    match outcome {
        RxOutcome::Beacon {
            info,
            len,
            lqi,
            rssi,
        } => {
            diag::update(|r| {
                match channel {
                    11 => r.beacons_ch11 = r.beacons_ch11.wrapping_add(1),
                    18 => r.beacons_ch18 = r.beacons_ch18.wrapping_add(1),
                    26 => r.beacons_ch26 = r.beacons_ch26.wrapping_add(1),
                    _ => r.beacons_control = r.beacons_control.wrapping_add(1),
                }
                r.last_frame_len = len;
                r.last_frame_lqi = lqi;
                r.last_rssi = rssi;
                r.last_valid_beacon = diag::DIAG_TRUE;
                r.last_pan_id = info.pan_id;
                match info.coord_address {
                    CoordAddress::Short(short) => {
                        r.last_coord_short = short;
                        r.last_coord_ext = [0; 8];
                    }
                    CoordAddress::Extended(ext) => {
                        r.last_coord_short = 0xFFFF;
                        r.last_coord_ext = ext;
                    }
                }
                r.last_association_permit = if info.association_permit {
                    diag::DIAG_TRUE
                } else {
                    diag::DIAG_FALSE
                };
                if let Some(zigbee) = info.zigbee {
                    r.scan_descriptors_found = r.scan_descriptors_found.wrapping_add(1);
                    r.last_extended_pan_id = zigbee.extended_pan_id;
                    r.last_protocol_id = zigbee.protocol_id;
                    r.last_stack_profile = zigbee.stack_profile;
                    r.last_protocol_version = zigbee.protocol_version;
                    r.last_device_depth = zigbee.device_depth;
                    r.last_router_capacity = if zigbee.router_capacity {
                        diag::DIAG_TRUE
                    } else {
                        diag::DIAG_FALSE
                    };
                    r.last_end_device_capacity = if zigbee.end_device_capacity {
                        diag::DIAG_TRUE
                    } else {
                        diag::DIAG_FALSE
                    };
                    r.last_update_id = zigbee.update_id;
                }
            });
        }
        RxOutcome::InvalidLength => {
            diag::update(|r| r.invalid_length_count = r.invalid_length_count.wrapping_add(1));
        }
        RxOutcome::InvalidCrc => {
            diag::update(|r| r.invalid_crc_count = r.invalid_crc_count.wrapping_add(1));
        }
        RxOutcome::NotABeacon { len, lqi, rssi } => {
            diag::update(|r| {
                r.last_frame_len = len;
                r.last_frame_lqi = lqi;
                r.last_rssi = rssi;
                r.last_valid_beacon = diag::DIAG_FALSE;
            });
        }
        RxOutcome::Ack { .. } | RxOutcome::AssociationResponse(_) => {}
    }
}
