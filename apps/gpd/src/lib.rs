//! Platform-independent Zigbee Green Power Device (GPD) lifecycle.
//!
//! A GPD never joins a network: it owns a SrcID, a GPD key and an outgoing
//! security frame counter, and transmits GP Data Frames (NWK protocol
//! version 3) directly on the radio. This crate provides:
//!
//! - [`journal`]: crash-safe persistence of SrcID, key, counter reservation
//!   and commissioned channel;
//! - [`GpdDevice`]: provisioning (random per-device key from a
//!   product-supplied entropy source), unidirectional commissioning on the
//!   channel of a network that permits joining, and protected reports and
//!   switch commands.
//!
//! Commissioning is unidirectional (no `rxAfterTx`): the sink accepts it
//! while its commissioning window is open (zigpy finalizes on the first
//! Commissioning GPDF without `rxAfterTx`). The GPD key travels in the
//! Commissioning command wrapped with the product's TC link key, as
//! GP 1.1 A.3.7.1.2 specifies for an individual OOB key.
//!
//! The crate depends on the shared radio boundary (`zigbee_mac::RadioPhy`
//! and `PlatformServices` for delays) and the GP frame builders
//! (`zigbee_gp::gpd`) only; product crates own identity, flash layout and the
//! entropy source.

#![no_std]

pub mod journal;

use zigbee_crypto::AesKey;
use zigbee_gp::commissioning::{ApplicationInfo, CommissioningPayload, MultiClusterReport};
use zigbee_gp::gpd::{CounterReservation, Gpd, GpdConfig, GpdSecurity, MAX_GPD_FRAME, TxOptions};
use zigbee_gp::types::{GpdId, KeyType, SecurityLevel, cmd};
use zigbee_mac::frames::{build_beacon_request, parse_beacon};
use zigbee_mac::{PhyError, PhyRxFrame, PlatformServices, RadioPhy};

pub use journal::{GpdJournal, GpdRecord, GpdStore, JournalError};

/// Device kind; part of the SrcID derivation and of the persisted record, so
/// re-flashing another kind on the same chip never reuses an identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpdKind {
    /// Environment sensor (Multi-Cluster Reporting).
    Sensor = 1,
    /// On/Off switch (Off / On / Toggle).
    Switch = 2,
}

impl GpdKind {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::Sensor),
            2 => Some(Self::Switch),
            _ => None,
        }
    }

    /// Differs in both halves: the low 16 bits are the GPD's derived NWK
    /// alias (GP A.3.6.3.3.1), so two kinds on one chip must not share them.
    /// Only new identities use it; a persisted SrcID is never re-derived.
    const fn src_id_salt(self) -> u32 {
        match self {
            Self::Sensor => 0,
            Self::Switch => 0x5357_5357,
        }
    }
}

/// Product-owned GPD identity and radio policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpdProfile {
    /// Device kind.
    pub kind: GpdKind,
    /// GPD DeviceID advertised in the Commissioning command.
    pub device_id: u8,
    /// Manufacturer ID (Application Information).
    pub manufacturer_id: u16,
    /// Model ID (Application Information).
    pub model_id: u16,
    /// GPD commands advertised in the Application Information; empty to
    /// omit the list.
    pub gpd_commands: &'static [u8],
    /// Security level of operational frames.
    pub security_level: SecurityLevel,
    /// Key type of the per-device key.
    pub key_type: KeyType,
    /// Key used to wrap the GPD key in the Commissioning command (the
    /// default TC link key `ZigBeeAlliance09` for a Zigbee 3.0 sink).
    pub key_wrap_key: AesKey,
    /// Frames per persisted counter reservation (flash wear bound).
    pub counter_block: u32,
    /// IEEE 802.15.4 channel mask (bits 11..=26) to search for a network
    /// that permits joining.
    pub channel_mask: u32,
    /// Listen window after each Beacon Request, in microseconds.
    pub scan_window_us: u32,
    /// Commissioning GPDFs sent per commissioning attempt.
    pub commissioning_frames: u8,
    /// Gap between commissioning GPDFs, in milliseconds.
    pub commissioning_gap_ms: u32,
    /// Transmissions of each operational GPDF (same counter; sinks and
    /// proxies drop the duplicates).
    pub tx_repeats: u8,
    /// Gap between repeated transmissions, in milliseconds.
    pub tx_repeat_gap_ms: u32,
    /// The GPD does not move (Commissioning option).
    pub fixed_location: bool,
}

/// All 2.4 GHz channels 11..=26.
pub const ALL_CHANNELS: u32 = 0x07FF_F800;

/// Failure of a GPD operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpdError {
    /// The radio failed.
    Radio(PhyError),
    /// Every CCA attempt found the channel busy.
    ChannelBusy,
    /// GP frame construction failed.
    Frame(zigbee_gp::Error),
    /// Persistence failed; nothing was transmitted with an unpersisted
    /// counter.
    Store(JournalError),
    /// The entropy source failed or returned a degenerate key.
    Entropy,
    /// No joinable network answered the beacon scan.
    NoPermitJoinNetwork,
    /// The GPD is not commissioned (no operational channel).
    NotCommissioned,
    /// The profile is invalid (channel mask, security level, key type).
    InvalidProfile,
}

impl From<PhyError> for GpdError {
    fn from(e: PhyError) -> Self {
        Self::Radio(e)
    }
}

impl From<zigbee_gp::Error> for GpdError {
    fn from(e: zigbee_gp::Error) -> Self {
        Self::Frame(e)
    }
}

impl From<JournalError> for GpdError {
    fn from(e: JournalError) -> Self {
        Self::Store(e)
    }
}

/// How [`GpdDevice::open`] obtained its identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Provisioning {
    /// A persisted record of this kind was resumed.
    Resumed,
    /// No (valid) record: a new key was generated.
    Provisioned,
    /// The record belonged to another kind: a new identity was generated;
    /// the counter bound carries over.
    Reprovisioned,
}

/// Beacon of a network that permits joining.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JoinableNetwork {
    /// Channel.
    pub channel: u8,
    /// Best LQI seen on that channel.
    pub lqi: u8,
    /// Extended PAN ID.
    pub extended_pan_id: [u8; 8],
}

/// Environment measurement for a Multi-Cluster report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnvironmentReport {
    /// Temperature in 0.01 °C (ZCL 0x0402/0x0000).
    pub temperature_centi_celsius: i16,
    /// Relative humidity in 0.01 % (ZCL 0x0405/0x0000).
    pub humidity_centi_percent: Option<u16>,
    /// Battery remaining in 0.5 % (ZCL 0x0001/0x0021).
    pub battery_half_percent: Option<u8>,
}

/// Switch command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwitchCommand {
    /// GPD Off (0x20).
    Off,
    /// GPD On (0x21).
    On,
    /// GPD Toggle (0x22).
    Toggle,
}

impl SwitchCommand {
    /// GPD CommandID.
    pub const fn command_id(self) -> u8 {
        match self {
            Self::Off => cmd::OFF,
            Self::On => cmd::ON,
            Self::Toggle => cmd::TOGGLE,
        }
    }
}

/// CCA attempts per transmission.
const CCA_ATTEMPTS: u32 = 5;
/// IEEE 802.15.4 unit backoff period (20 symbols).
const UNIT_BACKOFF_US: u32 = 320;

/// Derive a SrcID from the EUI-64 and the device kind, avoiding the
/// reserved values 0x00000000 and 0xFFFFFFF9..=0xFFFFFFFF.
pub fn derive_src_id(eui64: [u8; 8], kind: GpdKind) -> u32 {
    let raw = u64::from_le_bytes(eui64);
    let mut id = (raw as u32) ^ ((raw >> 32) as u32).rotate_left(16) ^ kind.src_id_salt();
    while GpdId::SrcId(id).is_reserved() {
        id = id.wrapping_add(0x0100_0007);
    }
    id
}

/// One Green Power Device.
pub struct GpdDevice<P, S> {
    phy: P,
    store: S,
    profile: GpdProfile,
    record: GpdRecord,
    gpd: Gpd,
    counter: CounterReservation,
    rng: u32,
}

impl<P: RadioPhy + PlatformServices, S: GpdStore> GpdDevice<P, S> {
    /// Load or provision the GPD identity.
    ///
    /// `entropy` is called only when a new key is needed; it must fill the
    /// buffer from a cryptographic source.
    pub fn open<E>(
        mut phy: P,
        mut store: S,
        profile: GpdProfile,
        eui64: [u8; 8],
        entropy: impl FnOnce(&mut [u8]) -> Result<(), E>,
    ) -> Result<(Self, Provisioning), GpdError> {
        if profile.channel_mask & ALL_CHANNELS == 0
            || (profile.security_level.is_protected() && !profile.key_type.is_individual())
            || profile.security_level == SecurityLevel::Reserved
        {
            return Err(GpdError::InvalidProfile);
        }
        let loaded = store.load()?;
        let (record, provisioning) = match loaded {
            Some(r) if r.kind == profile.kind => (r, Provisioning::Resumed),
            other => {
                let mut key = [0u8; 16];
                entropy(&mut key).map_err(|_| GpdError::Entropy)?;
                if key.iter().all(|b| *b == key[0]) {
                    return Err(GpdError::Entropy);
                }
                let r = GpdRecord {
                    kind: profile.kind,
                    src_id: derive_src_id(eui64, profile.kind),
                    key,
                    counter_limit: other.map_or(0, |o| o.counter_limit),
                    channel: None,
                };
                store.store(&r)?;
                (
                    r,
                    if other.is_some() {
                        Provisioning::Reprovisioned
                    } else {
                        Provisioning::Provisioned
                    },
                )
            }
        };
        let security = profile
            .security_level
            .is_protected()
            .then_some(GpdSecurity {
                level: profile.security_level,
                key_type: profile.key_type,
                key: record.key,
            });
        let gpd = Gpd::new(
            GpdConfig {
                src_id: record.src_id,
                security,
            },
            record.counter_limit as u8,
        )?;
        phy.set_rx_on_when_idle(false);
        let rng = record.src_id ^ record.counter_limit.rotate_left(13) ^ 0x9E37_79B9;
        Ok((
            Self {
                phy,
                store,
                counter: CounterReservation::resume(record.counter_limit, profile.counter_block),
                profile,
                record,
                gpd,
                rng,
            },
            provisioning,
        ))
    }

    /// Persisted state (never prints the key through `Debug`).
    pub fn record(&self) -> &GpdRecord {
        &self.record
    }

    /// Next security frame counter.
    pub fn next_frame_counter(&self) -> u32 {
        self.counter.peek()
    }

    /// The radio, for platform sleep/wake.
    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    fn next_random(&mut self) -> u32 {
        // xorshift32: CSMA backoff jitter and MAC sequence only.
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    /// Persist a new counter reservation if the next frame needs one.
    fn ensure_counter(&mut self) -> Result<(), GpdError> {
        if !self.profile.security_level.is_protected() {
            return Ok(());
        }
        if let Some(limit) = self.counter.pending_reservation()? {
            let mut r = self.record;
            r.counter_limit = limit;
            self.store.store(&r)?;
            self.record = r;
            self.counter.reserved(limit);
        }
        Ok(())
    }

    async fn transmit(&mut self, frame: &[u8]) -> Result<(), GpdError> {
        for attempt in 0..CCA_ATTEMPTS {
            match self.phy.try_transmit(frame).await {
                Ok(()) => return Ok(()),
                Err(PhyError::ChannelBusy) => {
                    let be = (3 + attempt).min(5);
                    let periods = self.next_random() & ((1 << be) - 1);
                    self.phy.delay_micros(periods * UNIT_BACKOFF_US).await;
                }
                Err(e) => return Err(GpdError::Radio(e)),
            }
        }
        Err(GpdError::ChannelBusy)
    }

    async fn transmit_repeated(&mut self, frame: &[u8]) -> Result<(), GpdError> {
        let n = self.profile.tx_repeats.max(1);
        let mut sent = false;
        let mut last = Err(GpdError::ChannelBusy);
        for i in 0..n {
            match self.transmit(frame).await {
                Ok(()) => sent = true,
                Err(e) => last = Err(e),
            }
            if i + 1 < n {
                self.phy
                    .delay_micros(self.profile.tx_repeat_gap_ms.saturating_mul(1000))
                    .await;
            }
        }
        if sent { Ok(()) } else { last }
    }

    /// Active scan: send a Beacon Request on every channel of the profile's
    /// mask and return the strongest Zigbee PRO network whose beacon
    /// permits association (the sink's permit-join window is open).
    pub async fn scan_for_joinable_network(&mut self) -> Result<Option<JoinableNetwork>, GpdError> {
        let mut best: Option<JoinableNetwork> = None;
        let mut rx = PhyRxFrame::EMPTY;
        for channel in 11u8..=26 {
            if self.profile.channel_mask & (1 << channel) == 0 {
                continue;
            }
            self.phy.set_channel(channel)?;
            let request = build_beacon_request(self.next_random() as u8);
            match self.transmit(&request).await {
                Ok(()) => {}
                Err(GpdError::ChannelBusy) => continue,
                Err(e) => return Err(e),
            }
            let start = self.phy.monotonic_micros();
            loop {
                let elapsed = self.phy.monotonic_micros().wrapping_sub(start);
                if elapsed >= self.profile.scan_window_us {
                    break;
                }
                if !self
                    .phy
                    .receive(self.profile.scan_window_us - elapsed, &mut rx)
                    .await?
                {
                    break;
                }
                let Some(pan) = parse_beacon(channel, rx.as_slice(), rx.lqi) else {
                    continue;
                };
                let zb = pan.zigbee_beacon;
                if zb.protocol_id != 0
                    || zb.stack_profile != 2
                    || !pan.superframe_spec.association_permit
                {
                    continue;
                }
                if best.is_none_or(|b| pan.lqi > b.lqi) {
                    best = Some(JoinableNetwork {
                        channel,
                        lqi: pan.lqi,
                        extended_pan_id: zb.extended_pan_id,
                    });
                }
            }
        }
        Ok(best)
    }

    fn commissioning_payload(&self) -> CommissioningPayload {
        let mut gpd_commands = heapless::Vec::new();
        // The profile's list is bounded by MAX_COMMISSIONING_COMMANDS.
        let _ = gpd_commands.extend_from_slice(self.profile.gpd_commands);
        CommissioningPayload {
            device_id: self.profile.device_id,
            mac_seq_num_capability: true,
            rx_on_capability: false,
            pan_id_request: false,
            security_key_request: false,
            fixed_location: self.profile.fixed_location,
            // Filled from the GpdConfig by Gpd::commissioning_frame.
            security: None,
            app_info: Some(ApplicationInfo {
                manufacturer_id: Some(self.profile.manufacturer_id),
                model_id: Some(self.profile.model_id),
                gpd_commands: (!gpd_commands.is_empty()).then_some(gpd_commands),
                ..ApplicationInfo::default()
            }),
        }
    }

    /// Unidirectional commissioning on `channel`: send the profile's number
    /// of Commissioning GPDFs, then persist `channel` as operational.
    pub async fn commission_on(&mut self, channel: u8) -> Result<(), GpdError> {
        if !(11..=26).contains(&channel) {
            return Err(GpdError::InvalidProfile);
        }
        if self.profile.gpd_commands.len() > zigbee_gp::commissioning::MAX_COMMISSIONING_COMMANDS {
            return Err(GpdError::InvalidProfile);
        }
        self.phy.set_channel(channel)?;
        let n = self.profile.commissioning_frames.max(1);
        let mut sent = false;
        let mut last = Err(GpdError::ChannelBusy);
        for i in 0..n {
            self.ensure_counter()?;
            let payload = self.commissioning_payload();
            let mut out = [0u8; MAX_GPD_FRAME];
            let len = self.gpd.commissioning_frame(
                &mut self.phy,
                &mut self.counter,
                &self.profile.key_wrap_key,
                payload,
                false,
                &mut out,
            )?;
            match self.transmit(&out[..len]).await {
                Ok(()) => sent = true,
                Err(GpdError::ChannelBusy) => last = Err(GpdError::ChannelBusy),
                Err(e) => return Err(e),
            }
            if i + 1 < n {
                self.phy
                    .delay_micros(self.profile.commissioning_gap_ms.saturating_mul(1000))
                    .await;
            }
        }
        if !sent {
            return last;
        }
        let mut r = self.record;
        r.channel = Some(channel);
        self.store.store(&r)?;
        self.record = r;
        Ok(())
    }

    /// Scan for a joinable network and commission on its channel.
    pub async fn commission(&mut self) -> Result<JoinableNetwork, GpdError> {
        let network = self
            .scan_for_joinable_network()
            .await?
            .ok_or(GpdError::NoPermitJoinNetwork)?;
        self.commission_on(network.channel).await?;
        Ok(network)
    }

    /// Send one operational command on the commissioned channel. Returns the
    /// security frame counter used (`None` for an unprotected profile).
    pub async fn send_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Option<u32>, GpdError> {
        let channel = self.record.channel.ok_or(GpdError::NotCommissioned)?;
        self.phy.set_channel(channel)?;
        self.ensure_counter()?;
        let used = self
            .profile
            .security_level
            .is_protected()
            .then_some(self.counter.peek());
        let mut out = [0u8; MAX_GPD_FRAME];
        let len = self.gpd.data_frame(
            &mut self.phy,
            &mut self.counter,
            TxOptions::default(),
            command_id,
            payload,
            &mut out,
        )?;
        self.transmit_repeated(&out[..len]).await?;
        Ok(used)
    }

    /// Send a Multi-Cluster Reporting (0xA2) command.
    pub async fn report_environment(
        &mut self,
        report: EnvironmentReport,
    ) -> Result<Option<u32>, GpdError> {
        let mut buf = [0u8; 32];
        let mut r = MultiClusterReport::new(&mut buf).int16(
            0x0402,
            0x0000,
            report.temperature_centi_celsius,
        )?;
        if let Some(h) = report.humidity_centi_percent {
            r = r.uint16(0x0405, 0x0000, h)?;
        }
        if let Some(b) = report.battery_half_percent {
            r = r.uint8(0x0001, 0x0021, b)?;
        }
        let n = r.finish();
        self.send_command(cmd::MULTI_CLUSTER_REPORTING, &buf[..n])
            .await
    }

    /// Send a switch command.
    pub async fn switch(&mut self, command: SwitchCommand) -> Result<Option<u32>, GpdError> {
        self.send_command(command.command_id(), &[]).await
    }

    /// Forget the commissioned channel (identity, key and counter are kept;
    /// recommissioning with the same key keeps the sink's counter floor).
    pub fn forget_channel(&mut self) -> Result<(), GpdError> {
        if self.record.channel.is_some() {
            let mut r = self.record;
            r.channel = None;
            self.store.store(&r)?;
            self.record = r;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
