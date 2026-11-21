//! `NWK_ENERGY_SCAN` and `NWK_ACTIVE_SCAN` (spec 7.2): scans the NCP runs
//! one channel at a time between stack steps, so that the host link and the
//! running network are served while they last. The energy scan's result
//! follows its response as `EVT_ENERGY_SCAN_RESULT`; the active scan answers
//! when it is complete.

use heapless::Vec;
use zigbee_zrhp::msg::network::{EnergyResult, NwkActiveScan, NwkEnergyScan, ScanResult};
use zigbee_zrhp::types::Status;

use crate::port::{Beacon, OpError};

/// Zigbee 2.4 GHz channels 11..=26.
const CHANNELS: u32 = 0x07FF_F800;

/// aBaseSuperframeDuration (960 symbols × 16 µs).
const SUPERFRAME_US: u64 = 15_360;

/// Longest scan the NCP accepts. Every measurement takes the radio off
/// the operating channel, so the bound keeps a host from stalling the
/// network for minutes (hub default: 16 channels at `duration_exp` 4,
/// ≈ 4.2 s).
pub const MAX_SCAN_US: u64 = 60_000_000;

/// An accepted energy scan.
#[derive(Debug, Clone)]
pub struct EnergyScan {
    pub tid: u16,
    mask: u32,
    remaining: u32,
    duration_exp: u8,
    passes_left: u8,
    /// Channels measured at least once.
    measured: u32,
    /// Measurements done (across passes) and planned.
    done: u8,
    total: u8,
    /// Highest energy per channel (index `channel - 11`).
    energy: [u8; 16],
    status: Status,
    /// The last pass measured a channel: let the stack run before the next.
    pub yielded: bool,
}

impl EnergyScan {
    /// Validate a request: a non-empty mask of channels 11..=26,
    /// `duration_exp` ≤ 14 (IEEE 802.15.4 ScanDuration), `count` ≥ 1, and
    /// at most [`MAX_SCAN_US`] in total.
    pub fn new(tid: u16, request: &NwkEnergyScan) -> Result<Self, Status> {
        let mask = request.channel_mask;
        if mask == 0 || mask & !CHANNELS != 0 || request.duration_exp > 14 || request.count == 0 {
            return Err(Status::InvalidParameter);
        }
        let per_channel = ((1u64 << request.duration_exp) + 1) * SUPERFRAME_US;
        let total = per_channel * u64::from(mask.count_ones()) * u64::from(request.count);
        if total > MAX_SCAN_US {
            return Err(Status::InvalidParameter);
        }
        Ok(Self {
            tid,
            mask,
            remaining: mask,
            duration_exp: request.duration_exp,
            passes_left: request.count,
            measured: 0,
            done: 0,
            total: (mask.count_ones() * u32::from(request.count)).min(255) as u8,
            energy: [0; 16],
            status: Status::Success,
            yielded: false,
        })
    }

    /// The next channel to measure, `None` when the scan is complete.
    pub fn next_channel(&self) -> Option<u8> {
        (self.remaining != 0).then(|| self.remaining.trailing_zeros() as u8)
    }

    pub fn duration_exp(&self) -> u8 {
        self.duration_exp
    }

    /// Record the measurement of `channel`. A failed measurement ends the
    /// scan with its status; channels already measured are reported.
    pub fn record(&mut self, channel: u8, result: Result<u8, OpError>) {
        self.remaining &= !(1 << channel);
        self.done = self.done.saturating_add(1);
        match result {
            Ok(energy) => {
                self.measured |= 1 << channel;
                let slot = &mut self.energy[usize::from(channel - 11)];
                *slot = (*slot).max(energy);
            }
            Err(error) => {
                self.status = match error {
                    OpError::Unsupported => Status::UnsupportedCommand,
                    OpError::InvalidState => Status::InvalidState,
                    _ => Status::Failure,
                };
                self.remaining = 0;
                self.passes_left = 1;
            }
        }
        if self.remaining == 0 {
            self.passes_left = self.passes_left.saturating_sub(1);
            if self.passes_left != 0 {
                self.remaining = self.mask;
            }
        }
    }

    /// `EVT_SCAN_PROGRESS` counts: measurements done and planned.
    pub fn progress(&self) -> (u8, u8) {
        (self.done, self.total)
    }

    /// `EVT_ENERGY_SCAN_RESULT` fields once complete: status and the
    /// per-channel maxima in channel order, `max_rssi_dbm` from
    /// `to_dbm`.
    pub fn results(&self, to_dbm: impl Fn(u8) -> i8) -> (Status, Vec<EnergyResult, 16>) {
        let mut out = Vec::new();
        // A failed scan reports only what was measured.
        let mut mask = self.mask & self.measured;
        while mask != 0 {
            let channel = mask.trailing_zeros() as u8;
            mask &= mask - 1;
            let energy = self.energy[usize::from(channel - 11)];
            let _ = out.push(EnergyResult {
                channel,
                max_rssi_dbm: to_dbm(energy),
                energy,
            });
        }
        (self.status, out)
    }
}

/// Largest `NWK_ACTIVE_SCAN.duration_exp`: 16 channels × 65 × 15.36 ms
/// ≈ 16 s.
pub const MAX_ACTIVE_SCAN_DURATION_EXP: u8 = 6;

/// Beacons one `NWK_ACTIVE_SCAN` response carries (18 B each); further
/// beacons are dropped.
pub const MAX_ACTIVE_SCAN_RESULTS: usize = 25;

/// An accepted active scan.
#[derive(Debug, Clone)]
pub struct ActiveScan {
    pub tid: u16,
    remaining: u32,
    duration_exp: u8,
    done: u8,
    total: u8,
    results: Vec<ScanResult, MAX_ACTIVE_SCAN_RESULTS>,
    status: Status,
    /// The last pass scanned a channel: let the stack run before the next.
    pub yielded: bool,
}

impl ActiveScan {
    /// Validate a request: a non-empty mask of channels 11..=26 and
    /// `duration_exp` ≤ [`MAX_ACTIVE_SCAN_DURATION_EXP`].
    pub fn new(tid: u16, request: &NwkActiveScan) -> Result<Self, Status> {
        let mask = request.channel_mask;
        if mask == 0 || mask & !CHANNELS != 0 || request.duration_exp > MAX_ACTIVE_SCAN_DURATION_EXP
        {
            return Err(Status::InvalidParameter);
        }
        Ok(Self {
            tid,
            remaining: mask,
            duration_exp: request.duration_exp,
            done: 0,
            total: mask.count_ones() as u8,
            results: Vec::new(),
            status: Status::Success,
            yielded: false,
        })
    }

    /// The next channel to scan, `None` when the scan is complete.
    pub fn next_channel(&self) -> Option<u8> {
        (self.remaining != 0).then(|| self.remaining.trailing_zeros() as u8)
    }

    pub fn duration_exp(&self) -> u8 {
        self.duration_exp
    }

    /// A beacon heard, with the received power `rssi` (dBm at the antenna).
    pub fn record(&mut self, beacon: &Beacon, rssi: i8) {
        let _ = self.results.push(ScanResult {
            channel: beacon.channel,
            pan: beacon.pan,
            epid: beacon.epid,
            src: beacon.src,
            lqi: beacon.lqi,
            rssi,
            permit: u8::from(beacon.permit),
            stack_profile: beacon.stack_profile,
            update_id: beacon.update_id,
        });
    }

    /// `channel` is done. A failed scan ends the whole scan with its status;
    /// the beacons heard so far are reported.
    pub fn finish_channel(&mut self, channel: u8, result: Result<(), OpError>) {
        self.remaining &= !(1 << channel);
        self.done = self.done.saturating_add(1);
        if let Err(error) = result {
            self.status = match error {
                OpError::Unsupported => Status::UnsupportedCommand,
                OpError::InvalidState => Status::InvalidState,
                _ => Status::Failure,
            };
            self.remaining = 0;
        }
    }

    /// `EVT_SCAN_PROGRESS` counts: channels done, planned, beacons kept.
    pub fn progress(&self) -> (u8, u8, u8) {
        (self.done, self.total, self.results.len() as u8)
    }

    /// The response: status and the beacons in the order heard.
    pub fn response(&self) -> (Status, &[ScanResult]) {
        (self.status, &self.results)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(channel_mask: u32, duration_exp: u8, count: u8) -> NwkEnergyScan {
        NwkEnergyScan {
            channel_mask,
            duration_exp,
            count,
        }
    }

    #[test]
    fn requests_are_bounded() {
        assert!(EnergyScan::new(1, &request(CHANNELS, 4, 1)).is_ok());
        for bad in [
            request(0, 4, 1),
            request(1 << 10, 4, 1),
            request(1 << 27, 4, 1),
            request(CHANNELS, 15, 1),
            request(CHANNELS, 4, 0),
            // One pass of 16 × 17 × 15.36 ms ≈ 4.2 s: 15 passes exceed 60 s.
            request(CHANNELS, 4, 15),
        ] {
            assert_eq!(
                EnergyScan::new(1, &bad).map(|_| ()),
                Err(Status::InvalidParameter),
                "{bad:?}"
            );
        }
        assert!(EnergyScan::new(1, &request(CHANNELS, 4, 4)).is_ok());
    }

    #[test]
    fn passes_keep_the_maximum_per_channel() {
        let mut scan = EnergyScan::new(7, &request((1 << 15) | (1 << 20), 2, 2)).unwrap();
        let mut seen = std::vec::Vec::new();
        let mut energy = [10u8, 200, 50, 30].into_iter();
        while let Some(channel) = scan.next_channel() {
            seen.push(channel);
            scan.record(channel, Ok(energy.next().unwrap()));
        }
        assert_eq!(seen, [15, 20, 15, 20]);
        let (status, results) = scan.results(|e| (e / 2) as i8 - 100);
        assert_eq!(status, Status::Success);
        assert_eq!(
            results
                .iter()
                .map(|r| (r.channel, r.energy, r.max_rssi_dbm))
                .collect::<std::vec::Vec<_>>(),
            [(15, 50, -75), (20, 200, 0)]
        );
    }

    #[test]
    fn a_failed_measurement_ends_the_scan_with_what_was_measured() {
        let mut scan =
            EnergyScan::new(7, &request((1 << 11) | (1 << 12) | (1 << 13), 0, 3)).unwrap();
        scan.record(11, Ok(5));
        scan.record(12, Err(OpError::Zigbee(0xC2)));
        assert_eq!(scan.next_channel(), None);
        let (status, results) = scan.results(|_| 0);
        assert_eq!(status, Status::Failure);
        assert_eq!(
            results
                .iter()
                .map(|r| r.channel)
                .collect::<std::vec::Vec<_>>(),
            [11]
        );
    }

    fn beacon(channel: u8, src: u16) -> Beacon {
        Beacon {
            channel,
            pan: 0x1A2B,
            epid: [1, 2, 3, 4, 5, 6, 7, 8],
            src,
            lqi: 200,
            permit: true,
            stack_profile: 2,
            update_id: 3,
        }
    }

    #[test]
    fn active_scan_requests_are_bounded() {
        let ok = |mask, duration_exp| {
            ActiveScan::new(
                1,
                &NwkActiveScan {
                    channel_mask: mask,
                    duration_exp,
                },
            )
            .map(|_| ())
        };
        assert_eq!(ok(CHANNELS, 3), Ok(()));
        assert_eq!(ok(CHANNELS, 6), Ok(()));
        for (mask, exp) in [(CHANNELS, 7), (0, 3), (1 << 10, 3), (1 << 27, 3)] {
            assert_eq!(
                ok(mask, exp),
                Err(Status::InvalidParameter),
                "{mask:#x} {exp}"
            );
        }
    }

    #[test]
    fn active_scan_keeps_every_beacon_up_to_the_cap_and_stops_on_failure() {
        let mut scan = ActiveScan::new(
            9,
            &NwkActiveScan {
                channel_mask: (1 << 11) | (1 << 15) | (1 << 20),
                duration_exp: 3,
            },
        )
        .unwrap();
        assert_eq!(scan.next_channel(), Some(11));
        for src in 0..30 {
            scan.record(&beacon(11, src), -60);
        }
        scan.finish_channel(11, Ok(()));
        assert_eq!(scan.next_channel(), Some(15));
        scan.finish_channel(15, Err(OpError::Zigbee(0xEA)));
        assert_eq!(scan.next_channel(), None);
        let (status, results) = scan.response();
        assert_eq!(status, Status::Failure);
        assert_eq!(results.len(), MAX_ACTIVE_SCAN_RESULTS);
        assert_eq!(
            (
                results[0].src,
                results[0].rssi,
                results[0].permit,
                results[0].epid
            ),
            (0, -60, 1, [1, 2, 3, 4, 5, 6, 7, 8])
        );
    }
}
