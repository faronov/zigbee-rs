//! Host tests of the GPD lifecycle against a scripted PHY; frames are
//! checked from the sink side with the shared GP parser and crypto.

extern crate std;

use std::vec::Vec;

use embassy_futures::block_on;
use zigbee_crypto::SoftwareAesProvider;
use zigbee_gp::commissioning::CommissioningPayload;
use zigbee_gp::frame::{parse, parse_mac};
use zigbee_gp::security::{self, DEFAULT_TC_LINK_KEY, Direction, WrappedKey};
use zigbee_mac::{MacError, PhyCapabilities, PhyRxFrame, SoftwareTx};

use super::*;
use crate::journal::tests::{Journal, MockFlash, journal};

const EUI: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
const KEY: [u8; 16] = [
    0x3C, 0x91, 0x07, 0xE2, 0x5D, 0xA8, 0x14, 0x6F, 0xB0, 0x2B, 0xC7, 0x99, 0x40, 0x0D, 0x7E, 0x51,
];

fn sensor_profile() -> GpdProfile {
    GpdProfile {
        kind: GpdKind::Sensor,
        device_id: 0x33,
        manufacturer_id: 0x6D6B,
        model_id: 0xF300,
        gpd_commands: &[],
        security_level: SecurityLevel::Encrypted,
        key_type: KeyType::IndividualKey,
        key_wrap_key: DEFAULT_TC_LINK_KEY,
        counter_block: 4,
        channel_mask: ALL_CHANNELS,
        scan_window_us: 100_000,
        commissioning_frames: 3,
        commissioning_gap_ms: 50,
        tx_repeats: 2,
        tx_repeat_gap_ms: 10,
        fixed_location: true,
    }
}

fn switch_profile() -> GpdProfile {
    GpdProfile {
        kind: GpdKind::Switch,
        device_id: 0x02,
        model_id: 0xF400,
        gpd_commands: &[cmd::OFF, cmd::ON, cmd::TOGGLE],
        ..sensor_profile()
    }
}

#[derive(Default)]
struct MockPhy {
    channel: u8,
    now_us: u32,
    /// `(channel, frame)` of every successful transmission.
    sent: Vec<(u8, Vec<u8>)>,
    /// Beacons answered after a Beacon Request: `(channel, lqi, frame)`.
    beacons: Vec<(u8, u8, Vec<u8>)>,
    pending_rx: Vec<(u8, Vec<u8>)>,
    /// CCA attempts that report busy before the channel clears.
    busy: u32,
    delays: Vec<u32>,
    /// Answer to each transmitted GPDF with `rxAfterTx`, in order:
    /// `(start after the end of the GPDF in µs, frame)`; `None` or an
    /// exhausted list leaves the window empty.
    replies: Vec<Option<(u32, Vec<u8>)>>,
    /// Unrelated frames heard first in every rxAfterTx window.
    rx_noise: Vec<Vec<u8>>,
    /// The scheduled reply: `(start time, frame)`.
    timed_rx: Option<(u32, Vec<u8>)>,
    /// End time of every transmitted GPDF with `rxAfterTx`.
    rx_after_tx_ends: Vec<u32>,
    /// Every receive window: `(open, close)`.
    rx_windows: Vec<(u32, u32)>,
}

/// Whether `frame` is a GPDF with `rxAfterTx`.
fn requests_rx_after_tx(frame: &[u8]) -> bool {
    parse_mac(frame)
        .ok()
        .and_then(|mac| parse(&frame[mac.payload_offset..], mac.seq, mac.src_ieee).ok())
        .is_some_and(|g| g.header.rx_after_tx)
}

impl RadioPhy for MockPhy {
    type Tx = SoftwareTx;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities::default()
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        if self.busy > 0 {
            self.busy -= 1;
            return Err(PhyError::ChannelBusy);
        }
        self.now_us += 1_000;
        self.sent.push((self.channel, frame.to_vec()));
        if requests_rx_after_tx(frame) {
            self.rx_after_tx_ends.push(self.now_us);
            self.pending_rx = self.rx_noise.iter().map(|f| (0xC0, f.clone())).collect();
            self.timed_rx = if self.replies.is_empty() {
                None
            } else {
                self.replies
                    .remove(0)
                    .map(|(offset, f)| (self.now_us + offset, f))
            };
        }
        if frame.last() == Some(&0x07) && frame.len() == 8 {
            self.pending_rx = self
                .beacons
                .iter()
                .filter(|(c, _, _)| *c == self.channel)
                .map(|(_, lqi, f)| (*lqi, f.clone()))
                .collect();
        }
        Ok(())
    }

    async fn send_ack(&mut self, _: u8, _: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        self.rx_windows
            .push((self.now_us, self.now_us + timeout_us));
        if self.pending_rx.is_empty() {
            if let Some((at, f)) = self.timed_rx.take() {
                // The preamble must start inside the window.
                if (self.now_us..self.now_us + timeout_us).contains(&at) {
                    // SHR + PHR + PSDU (with FCS) at 32 µs per byte.
                    self.now_us = at + (6 + f.len() as u32 + 2) * 32;
                    frame.set(&f, 0xC0)?;
                    return Ok(true);
                }
            }
            self.now_us += timeout_us;
            return Ok(false);
        }
        let (lqi, f) = self.pending_rx.remove(0);
        self.now_us += 500;
        frame.set(&f, lqi)?;
        Ok(true)
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.channel = channel;
        Ok(())
    }

    fn set_tx_power(&mut self, _: i8) -> Result<(), PhyError> {
        Ok(())
    }

    async fn energy_detect(&mut self, _: u32) -> Result<u8, PhyError> {
        Ok(0)
    }

    fn set_address_filter(
        &mut self,
        _: Option<zigbee_mac::PhyAddressFilter>,
    ) -> Result<(), PhyError> {
        Ok(())
    }
}

impl zigbee_crypto::ForwardAesProvider for MockPhy {}

impl PlatformServices for MockPhy {
    fn monotonic_micros(&self) -> u32 {
        self.now_us
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.delays.push(duration_us);
        self.now_us = self.now_us.wrapping_add(duration_us);
    }

    fn fill_random(&mut self, _: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

/// Zigbee PRO beacon from short address `src` on PAN 0x1A62.
fn beacon(permit: bool, epid: u8) -> Vec<u8> {
    let superframe: u16 = 0x4FFF | if permit { 0x8000 } else { 0 };
    let mut f = std::vec![0x00, 0x80, 0x42, 0x62, 0x1A, 0x00, 0x00];
    f.extend_from_slice(&superframe.to_le_bytes());
    f.extend_from_slice(&[0x00, 0x00]); // GTS, pending addresses
    f.extend_from_slice(&[0x00, 0x22, 0x84]); // protocol 0, PRO v2, caps
    f.extend_from_slice(&[epid; 8]);
    f.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0x00]);
    f
}

type Device = GpdDevice<MockPhy, Journal>;

fn open(phy: MockPhy, flash: MockFlash, profile: GpdProfile) -> (Device, Provisioning) {
    GpdDevice::open(phy, journal(flash), profile, EUI, |out: &mut [u8]| {
        out.copy_from_slice(&KEY);
        Ok::<(), ()>(())
    })
    .unwrap()
}

fn into_parts(device: Device) -> (MockPhy, MockFlash) {
    (device.phy, device.store.into_storage())
}

/// Sink-side decode of a GPDF: `(frame counter, clear command + payload)`.
fn sink_open(frame: &[u8], src_id: u32, key: &AesKey) -> (u32, Vec<u8>) {
    let mac = parse_mac(frame).unwrap();
    let gpdf = parse(&frame[mac.payload_offset..], mac.seq, mac.src_ieee).unwrap();
    assert_eq!(gpdf.header.gpd_id, Some(GpdId::SrcId(src_id)));
    let clear = security::open(&mut SoftwareAesProvider, key, &gpdf, Direction::FromGpd).unwrap();
    (gpdf.header.frame_counter, clear.to_vec())
}

#[test]
fn src_id_derivation_avoids_reserved_and_separates_kinds() {
    let s = derive_src_id(EUI, GpdKind::Sensor);
    let w = derive_src_id(EUI, GpdKind::Switch);
    assert_ne!(s, w);
    assert!(!GpdId::SrcId(s).is_reserved());
    // EUI-64 that folds to 0 and to 0xFFFFFFFF.
    assert!(!GpdId::SrcId(derive_src_id([0; 8], GpdKind::Sensor)).is_reserved());
    let ff = derive_src_id([0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0], GpdKind::Sensor);
    assert!(!GpdId::SrcId(ff).is_reserved());
}

#[test]
fn kinds_on_one_chip_get_distinct_nwk_aliases() {
    // Includes the BRD4183A whose first switch shared the sensor's alias.
    let euis = [
        EUI,
        [0x12, 0x90, 0xC3, 0xFE, 0xFF, 0xD7, 0x6B, 0x08],
        [0; 8],
        [0xFF; 8],
        [0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0],
    ];
    for eui in euis {
        let sensor = GpdId::SrcId(derive_src_id(eui, GpdKind::Sensor));
        let switch = GpdId::SrcId(derive_src_id(eui, GpdKind::Switch));
        assert_ne!(
            zigbee_gp::types::derive_alias(&sensor),
            zigbee_gp::types::derive_alias(&switch),
            "{eui:02X?}"
        );
    }
}

#[test]
fn provisioning_is_persisted_and_resumed() {
    let (d, p) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
    assert_eq!(p, Provisioning::Provisioned);
    let record = *d.record();
    assert_eq!(record.key, KEY);
    assert_eq!(record.src_id, derive_src_id(EUI, GpdKind::Sensor));
    assert_eq!(record.channel, None);
    let (phy, flash) = into_parts(d);
    let (d, p) = GpdDevice::open(
        phy,
        journal(flash),
        sensor_profile(),
        EUI,
        |_: &mut [u8]| Err::<(), ()>(()),
    )
    .unwrap();
    assert_eq!(p, Provisioning::Resumed);
    assert_eq!(*d.record(), record);
}

#[test]
fn kind_change_reprovisions_and_keeps_counter_bound() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
    block_on(d.commission_on(15)).unwrap();
    let limit = d.record().counter_limit;
    assert!(limit > 0);
    let (phy, flash) = into_parts(d);
    let (d, p) = open(phy, flash, switch_profile());
    assert_eq!(p, Provisioning::Reprovisioned);
    assert_eq!(d.record().src_id, derive_src_id(EUI, GpdKind::Switch));
    assert_eq!(d.record().counter_limit, limit);
    assert_eq!(d.record().channel, None);
}

#[test]
fn degenerate_or_failed_entropy_is_rejected() {
    let r = GpdDevice::open(
        MockPhy::default(),
        journal(MockFlash::new()),
        sensor_profile(),
        EUI,
        |out: &mut [u8]| {
            out.fill(0);
            Ok::<(), ()>(())
        },
    );
    assert!(matches!(r, Err(GpdError::Entropy)));
    let r = GpdDevice::open(
        MockPhy::default(),
        journal(MockFlash::new()),
        sensor_profile(),
        EUI,
        |_: &mut [u8]| Err::<(), ()>(()),
    );
    assert!(matches!(r, Err(GpdError::Entropy)));
}

#[test]
fn invalid_profile_is_rejected() {
    let mut p = sensor_profile();
    p.channel_mask = 0x7FF;
    let r = GpdDevice::open(
        MockPhy::default(),
        journal(MockFlash::new()),
        p,
        EUI,
        |_: &mut [u8]| Ok::<(), ()>(()),
    );
    assert!(matches!(r, Err(GpdError::InvalidProfile)));
    let mut p = sensor_profile();
    p.key_type = KeyType::NwkKey;
    let r = GpdDevice::open(
        MockPhy::default(),
        journal(MockFlash::new()),
        p,
        EUI,
        |_: &mut [u8]| Ok::<(), ()>(()),
    );
    assert!(matches!(r, Err(GpdError::InvalidProfile)));
}

#[test]
fn scan_picks_strongest_permitting_network() {
    let phy = MockPhy {
        beacons: std::vec![
            (11, 200, beacon(false, 0xAA)),
            (15, 90, beacon(true, 0xBB)),
            (20, 150, beacon(true, 0xCC)),
            (25, 250, beacon(false, 0xDD)),
        ],
        ..MockPhy::default()
    };
    let (mut d, _) = open(phy, MockFlash::new(), sensor_profile());
    let n = block_on(d.scan_for_joinable_network()).unwrap().unwrap();
    assert_eq!(n.channel, 20);
    assert_eq!(n.lqi, 150);
    assert_eq!(n.extended_pan_id, [0xCC; 8]);
    // One Beacon Request per channel.
    assert_eq!(d.phy.sent.len(), 16);
}

#[test]
fn no_joinable_network_is_reported_and_nothing_is_commissioned() {
    let phy = MockPhy {
        beacons: std::vec![(15, 90, beacon(false, 0xBB))],
        ..MockPhy::default()
    };
    let (mut d, _) = open(phy, MockFlash::new(), sensor_profile());
    assert_eq!(block_on(d.commission()), Err(GpdError::NoPermitJoinNetwork));
    assert_eq!(d.record().channel, None);
}

#[test]
fn commissioning_wraps_key_and_advertises_reserved_counters() {
    let phy = MockPhy {
        beacons: std::vec![(15, 90, beacon(true, 0xBB))],
        ..MockPhy::default()
    };
    let (mut d, _) = open(phy, MockFlash::new(), switch_profile());
    let src_id = d.record().src_id;
    let n = block_on(d.commission()).unwrap();
    assert_eq!(n.channel, 15);
    assert_eq!(d.record().channel, Some(15));

    let frames: Vec<_> = d.phy.sent.iter().filter(|(_, f)| f.len() > 8).collect();
    assert_eq!(frames.len(), 3);
    let mut seqs = Vec::new();
    for (i, (channel, f)) in frames.iter().enumerate() {
        assert_eq!(*channel, 15);
        let mac = parse_mac(f).unwrap();
        seqs.push(mac.seq);
        let g = parse(&f[mac.payload_offset..], mac.seq, None).unwrap();
        assert!(!g.header.security_level.is_protected());
        assert!(!g.header.rx_after_tx);
        assert_eq!(g.payload[0], cmd::COMMISSIONING);
        let p = CommissioningPayload::decode(&g.payload[1..]).unwrap();
        assert_eq!(p.device_id, 0x02);
        let app = p.app_info.unwrap();
        assert_eq!(app.manufacturer_id, Some(0x6D6B));
        assert_eq!(app.model_id, Some(0xF400));
        assert_eq!(
            app.gpd_commands.unwrap().as_slice(),
            &[cmd::OFF, cmd::ON, cmd::TOGGLE]
        );
        let sec = p.security.unwrap();
        assert_eq!(sec.level, SecurityLevel::Encrypted);
        assert_eq!(sec.key_type, KeyType::IndividualKey);
        assert_eq!(sec.outgoing_counter, Some(i as u32));
        let (k, mic) = sec.key.unwrap();
        let key = security::unwrap_key(
            &mut SoftwareAesProvider,
            &DEFAULT_TC_LINK_KEY,
            &GpdId::SrcId(src_id),
            Direction::FromGpd,
            0,
            &WrappedKey {
                key: k,
                mic: mic.unwrap().to_le_bytes(),
            },
        )
        .unwrap();
        assert_eq!(key, KEY);
    }
    // zigpy deduplicates unprotected commissioning frames by MAC sequence.
    seqs.dedup();
    assert_eq!(seqs.len(), 3);
}

#[test]
fn reports_are_encrypted_with_fresh_counters_and_repeated() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
    let src_id = d.record().src_id;
    assert_eq!(
        block_on(d.report_environment(EnvironmentReport {
            temperature_centi_celsius: 2150,
            humidity_centi_percent: None,
            battery_voltage_100mv: None,
            battery_half_percent: None,
        })),
        Err(GpdError::NotCommissioned)
    );
    block_on(d.commission_on(25)).unwrap();
    let before = d.phy.sent.len();
    let used = block_on(d.report_environment(EnvironmentReport {
        temperature_centi_celsius: -125,
        humidity_centi_percent: Some(4875),
        battery_voltage_100mv: Some(29),
        battery_half_percent: Some(30),
    }))
    .unwrap()
    .unwrap();
    // Commissioning advertised 0, 1, 2; the first report uses 3.
    assert_eq!(used, 3);
    let sent = &d.phy.sent[before..];
    assert_eq!(sent.len(), 2);
    assert_eq!(sent[0], sent[1]);
    assert_eq!(sent[0].0, 25);
    let (fc, clear) = sink_open(&sent[0].1, src_id, &KEY);
    assert_eq!(fc, 3);
    let mut expected = std::vec![cmd::MULTI_CLUSTER_REPORTING];
    expected.extend_from_slice(&[
        0x02, 0x04, 0x00, 0x00, 0x29, 0x83, 0xFF, // temp -1.25
        0x05, 0x04, 0x00, 0x00, 0x21, 0x0B, 0x13, // hum 48.75
        0x01, 0x00, 0x20, 0x00, 0x20, 0x1D, // battery 2.9 V
        0x01, 0x00, 0x21, 0x00, 0x20, 0x1E, // battery 15 %
    ]);
    assert_eq!(clear, expected);
    // The protected frame carries the counter LSB as MAC sequence.
    assert_eq!(parse_mac(&sent[0].1).unwrap().seq, 3);

    let used = block_on(d.switch(SwitchCommand::Toggle)).unwrap().unwrap();
    assert_eq!(used, 4);
    let (_, clear) = sink_open(&d.phy.sent.last().unwrap().1, src_id, &KEY);
    assert_eq!(clear, [cmd::TOGGLE]);
}

#[test]
fn counter_is_reserved_before_use_and_never_reused_after_reboot() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), switch_profile());
    block_on(d.commission_on(11)).unwrap();
    let mut used = Vec::new();
    for _ in 0..6 {
        let fc = block_on(d.switch(SwitchCommand::On)).unwrap().unwrap();
        // Every used counter is below the persisted bound.
        assert!(fc < d.record().counter_limit);
        used.push(fc);
    }
    let last = *used.last().unwrap();
    // Reboot (RAM state lost) and continue.
    let (phy, flash) = into_parts(d);
    let (mut d, p) = open(phy, flash, switch_profile());
    assert_eq!(p, Provisioning::Resumed);
    assert_eq!(d.record().channel, Some(11));
    let fc = block_on(d.switch(SwitchCommand::Off)).unwrap().unwrap();
    assert!(fc > last);
    assert!(fc < last + 1 + 2 * switch_profile().counter_block);
}

#[test]
fn failed_reservation_transmits_nothing() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), switch_profile());
    block_on(d.commission_on(11)).unwrap();
    // Use up the current reservation.
    while d.next_frame_counter() < d.record().counter_limit {
        block_on(d.switch(SwitchCommand::On)).unwrap();
    }
    let (phy, mut flash) = into_parts(d);
    let (mut d, _) = {
        flash.programs_left = Some(0);
        open(phy, flash, switch_profile())
    };
    let before = d.phy.sent.len();
    assert_eq!(
        block_on(d.switch(SwitchCommand::On)),
        Err(GpdError::Store(JournalError::Hardware))
    );
    assert_eq!(d.phy.sent.len(), before);
}

#[test]
fn busy_channel_backs_off_then_reports_busy() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), switch_profile());
    block_on(d.commission_on(11)).unwrap();
    d.phy.busy = 2;
    d.phy.delays.clear();
    block_on(d.switch(SwitchCommand::Toggle)).unwrap();
    // Two CSMA backoffs plus one repeat gap.
    assert_eq!(d.phy.delays.len(), 3);
    assert!(d.phy.delays[..2].iter().all(|us| us % UNIT_BACKOFF_US == 0));

    d.phy.busy = u32::MAX;
    let before = d.phy.sent.len();
    assert_eq!(
        block_on(d.switch(SwitchCommand::Toggle)),
        Err(GpdError::ChannelBusy)
    );
    assert_eq!(d.phy.sent.len(), before);
}

#[test]
fn forget_channel_requires_recommissioning() {
    let (mut d, _) = open(MockPhy::default(), MockFlash::new(), switch_profile());
    block_on(d.commission_on(11)).unwrap();
    d.forget_channel().unwrap();
    assert_eq!(
        block_on(d.switch(SwitchCommand::Toggle)),
        Err(GpdError::NotCommissioned)
    );
    let (phy, flash) = into_parts(d);
    let (d, _) = open(phy, flash, switch_profile());
    assert_eq!(d.record().channel, None);
}

#[test]
fn battery_curve_maps_voltage_and_clamped_percentage() {
    let curve = BatteryCurve {
        empty_mv: 1_800,
        full_mv: 3_000,
    };
    assert_eq!(curve.voltage_100mv(3_300), 33);
    assert_eq!(curve.voltage_100mv(2_949), 29);
    assert_eq!(curve.voltage_100mv(2_950), 30);
    assert_eq!(curve.voltage_100mv(u16::MAX), 0xFE);
    assert_eq!(curve.half_percent(1_700), 0);
    assert_eq!(curve.half_percent(1_800), 0);
    assert_eq!(curve.half_percent(2_400), 100);
    assert_eq!(curve.half_percent(2_403), 101);
    assert_eq!(curve.half_percent(3_000), 200);
    assert_eq!(curve.half_percent(3_300), 200);
    let degenerate = BatteryCurve {
        empty_mv: 3_000,
        full_mv: 3_000,
    };
    assert_eq!(degenerate.half_percent(3_300), 0);
    let base = EnvironmentReport {
        temperature_centi_celsius: 2_000,
        humidity_centi_percent: Some(5_000),
        battery_voltage_100mv: None,
        battery_half_percent: None,
    };
    let report = curve.apply(base, 2_700);
    assert_eq!(report.battery_voltage_100mv, Some(27));
    assert_eq!(report.battery_half_percent, Some(150));
    assert_eq!(report.temperature_centi_celsius, 2_000);
    assert_eq!(report.humidity_centi_percent, Some(5_000));
}

#[cfg(feature = "bidirectional")]
mod bidirectional {
    use zigbee_gp::commissioning::CommissioningReply;
    use zigbee_gp::frame::{FrameType, GpdfParams};

    use super::*;

    /// Product-like policy: listen 1 ms early, 4 ms long.
    const POLICY: BidirectionalPolicy = BidirectionalPolicy {
        attempts: 3,
        attempt_gap_ms: 400,
        rx_open_us: 19_000,
        rx_window_us: 4_000,
    };

    /// zigpy's reply: echoes the commissioned level and key type, no key.
    fn zigpy_reply() -> CommissioningReply {
        CommissioningReply {
            pan_id: None,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: None,
        }
    }

    /// Unprotected GPP → GPD Commissioning Reply as a TempMaster proxy
    /// sends it (zigbee-gp `Proxy::take_downlink`).
    fn reply_frame(src_id: u32, reply: &CommissioningReply) -> Vec<u8> {
        let mut payload = [0u8; 40];
        let n = reply.encode(&mut payload).unwrap();
        let params = GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(src_id),
            security_level: SecurityLevel::None,
            individual_key: false,
            rx_after_tx: false,
            to_gpd: true,
            frame_counter: 0,
        };
        let mut out = [0u8; 64];
        let len = security::build_gpdf(
            &mut SoftwareAesProvider,
            None,
            &params,
            0x5A,
            cmd::COMMISSIONING_REPLY,
            &payload[..n],
            &mut out,
        )
        .unwrap();
        out[..len].to_vec()
    }

    fn gpdfs(phy: &MockPhy) -> Vec<Vec<u8>> {
        phy.sent
            .iter()
            .filter(|(_, f)| f.len() > 8)
            .map(|(_, f)| f.clone())
            .collect()
    }

    fn advertised_counter(frame: &[u8]) -> u32 {
        let mac = parse_mac(frame).unwrap();
        let g = parse(&frame[mac.payload_offset..], mac.seq, None).unwrap();
        assert!(g.header.rx_after_tx);
        assert_eq!(g.payload[0], cmd::COMMISSIONING);
        let p = CommissioningPayload::decode(&g.payload[1..]).unwrap();
        assert!(p.rx_on_capability);
        p.security.unwrap().outgoing_counter.unwrap()
    }

    #[test]
    fn reply_in_the_window_is_confirmed_with_success() {
        let (d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
        let src_id = d.record().src_id;
        let (mut phy, flash) = into_parts(d);
        // First GPDF unanswered (the proxy has no GP Response yet), the
        // second one answered at gpdRxOffset after other channel traffic.
        phy.replies = std::vec![None, Some((20_000, reply_frame(src_id, &zigpy_reply())))];
        let mut nwk = std::vec![0x41, 0x88, 0x10, 0x62, 0x1A, 0xFF, 0xFF, 0x00, 0x00];
        nwk.extend_from_slice(&[0x08, 0x00, 0xFD, 0xFF, 0x00, 0x00, 0x1E, 0x10]);
        phy.rx_noise = std::vec![nwk, reply_frame(src_id ^ 1, &zigpy_reply())];
        let (mut d, _) = open(phy, flash, sensor_profile());

        block_on(d.commission_bidirectional_on(15, POLICY)).unwrap();
        assert_eq!(d.record().channel, Some(15));

        let frames = gpdfs(&d.phy);
        // Two Commissioning GPDFs, then Success repeated tx_repeats times.
        assert_eq!(frames.len(), 2 + 2);
        let c0 = advertised_counter(&frames[0]);
        let c1 = advertised_counter(&frames[1]);
        assert!(c1 > c0);
        for f in &frames[2..] {
            let (fc, clear) = sink_open(f, src_id, &KEY);
            // zigpy drops a Success whose counter is not above the last
            // commissioning counter.
            assert_eq!(fc, c1 + 1);
            assert_eq!(clear, [cmd::SUCCESS]);
        }
        assert_eq!(d.next_frame_counter(), c1 + 2);

        // Every window covers the TempMaster's start time; the second
        // window was entered only after the noise frames.
        let ends = d.phy.rx_after_tx_ends.clone();
        assert_eq!(ends.len(), 2);
        let first = d.phy.rx_windows[0];
        assert_eq!(first.0, ends[0] + POLICY.rx_open_us);
        assert!(first.0 < ends[0] + 20_000 && ends[0] + 20_000 < first.1);
        assert!(
            d.phy
                .rx_windows
                .iter()
                .all(|w| w.1 <= ends[0] + POLICY.rx_open_us + POLICY.rx_window_us
                    || w.0 >= ends[1])
        );
    }

    #[test]
    fn no_reply_leaves_the_gpd_uncommissioned_without_counter_reuse() {
        let (d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
        let src_id = d.record().src_id;
        let (mut phy, flash) = into_parts(d);
        // A reply after the window closed is not seen.
        phy.replies = std::vec![
            None,
            Some((
                POLICY.rx_open_us + POLICY.rx_window_us,
                reply_frame(src_id, &zigpy_reply())
            )),
        ];
        let (mut d, _) = open(phy, flash, sensor_profile());
        assert_eq!(
            block_on(d.commission_bidirectional_on(15, POLICY)),
            Err(GpdError::NoCommissioningReply)
        );
        assert_eq!(d.record().channel, None);
        let frames = gpdfs(&d.phy);
        assert_eq!(frames.len(), POLICY.attempts as usize);
        let counters: Vec<u32> = frames.iter().map(|f| advertised_counter(f)).collect();
        assert!(counters.windows(2).all(|w| w[1] > w[0]));
        // Attempts are spaced by the policy gap.
        let ends = &d.phy.rx_after_tx_ends;
        assert!(
            ends.windows(2)
                .all(|w| w[1] - w[0] >= POLICY.attempt_gap_ms * 1000)
        );

        // A reboot never reuses an advertised counter.
        let last = *counters.last().unwrap();
        let (_, flash) = into_parts(d);
        let (d, provisioning) = open(MockPhy::default(), flash, sensor_profile());
        assert_eq!(provisioning, Provisioning::Resumed);
        assert!(d.next_frame_counter() > last);
    }

    #[test]
    fn reply_with_another_security_level_is_rejected() {
        let (d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
        let src_id = d.record().src_id;
        let (mut phy, flash) = into_parts(d);
        let reply = CommissioningReply {
            security_level: SecurityLevel::FullCounterMic,
            ..zigpy_reply()
        };
        phy.replies = std::vec![Some((20_000, reply_frame(src_id, &reply)))];
        let (mut d, _) = open(phy, flash, sensor_profile());
        assert_eq!(
            block_on(d.commission_bidirectional_on(15, POLICY)),
            Err(GpdError::CommissioningRejected)
        );
        assert_eq!(d.record().channel, None);
        // No Success was sent.
        assert_eq!(gpdfs(&d.phy).len(), 1);
    }

    #[test]
    fn delivered_key_replaces_the_gpd_key() {
        const NEW_KEY: AesKey = [0xA5; 16];
        let (d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
        let src_id = d.record().src_id;
        let (mut phy, flash) = into_parts(d);
        let fc = 0x0102_0304;
        let w = security::wrap_key(
            &mut SoftwareAesProvider,
            &DEFAULT_TC_LINK_KEY,
            &GpdId::SrcId(src_id),
            Direction::ToGpd,
            fc,
            &NEW_KEY,
        )
        .unwrap();
        let reply = CommissioningReply {
            key: Some((w.key, Some((u32::from_le_bytes(w.mic), fc)))),
            ..zigpy_reply()
        };
        phy.replies = std::vec![Some((20_000, reply_frame(src_id, &reply)))];
        let (mut d, _) = open(phy, flash, sensor_profile());
        block_on(d.commission_bidirectional_on(15, POLICY)).unwrap();
        assert_eq!(d.record().key, NEW_KEY);
        let frames = gpdfs(&d.phy);
        let (_, clear) = sink_open(&frames[1], src_id, &NEW_KEY);
        assert_eq!(clear, [cmd::SUCCESS]);

        // The key survives a reboot and protects later reports.
        let (_, flash) = into_parts(d);
        let (mut d, provisioning) = open(MockPhy::default(), flash, sensor_profile());
        assert_eq!(provisioning, Provisioning::Resumed);
        assert_eq!(d.record().key, NEW_KEY);
        block_on(d.report_environment(EnvironmentReport {
            temperature_centi_celsius: 2_150,
            humidity_centi_percent: None,
            battery_voltage_100mv: None,
            battery_half_percent: None,
        }))
        .unwrap();
        let frames = gpdfs(&d.phy);
        sink_open(&frames[0], src_id, &NEW_KEY);
    }

    #[test]
    fn invalid_policy_is_rejected() {
        let (mut d, _) = open(MockPhy::default(), MockFlash::new(), sensor_profile());
        for policy in [
            BidirectionalPolicy {
                attempts: 0,
                ..POLICY
            },
            BidirectionalPolicy {
                rx_open_us: 20_001,
                ..POLICY
            },
            BidirectionalPolicy {
                rx_open_us: 15_000,
                rx_window_us: 5_000,
                ..POLICY
            },
        ] {
            assert_eq!(
                block_on(d.commission_bidirectional_on(15, policy)),
                Err(GpdError::InvalidProfile)
            );
        }
        assert!(d.phy.sent.is_empty());
    }
}
