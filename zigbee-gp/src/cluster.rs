//! Green Power cluster (0x0021) command payload codecs.
//!
//! Only the ZCL payload (after the ZCL header) is handled here. Directions
//! follow the GP spec: a proxy implements the GP *client* side, so it
//! receives server→client commands ([`client_cmd`]) from sinks and sends
//! client→server commands ([`server_cmd`]) to sinks.
//!
//! Decoders are strict (trailing bytes are rejected) to match zigpy.

use zigbee_crypto::AesKey;

use crate::codec::{Reader, Writer};
use crate::types::{ApplicationId, CommunicationMode, GpdId, IeeeAddress, KeyType, SecurityLevel};
use crate::{Error, Result};

/// Commands received by the GP server (sent by proxies, client → server).
pub mod server_cmd {
    /// GP Notification.
    pub const NOTIFICATION: u8 = 0x00;
    /// GP Pairing Search.
    pub const PAIRING_SEARCH: u8 = 0x01;
    /// GP Tunneling Stop.
    pub const TUNNELING_STOP: u8 = 0x03;
    /// GP Commissioning Notification.
    pub const COMMISSIONING_NOTIFICATION: u8 = 0x04;
    /// GP Sink Commissioning Mode.
    pub const SINK_COMMISSIONING_MODE: u8 = 0x05;
    /// GP Translation Table Update.
    pub const TRANSLATION_TABLE_UPDATE: u8 = 0x07;
    /// GP Translation Table Request.
    pub const TRANSLATION_TABLE_REQUEST: u8 = 0x08;
    /// GP Pairing Configuration.
    pub const PAIRING_CONFIGURATION: u8 = 0x09;
    /// GP Sink Table Request.
    pub const SINK_TABLE_REQUEST: u8 = 0x0A;
    /// GP Proxy Table Response.
    pub const PROXY_TABLE_RESPONSE: u8 = 0x0B;
}

/// Commands received by the GP client (sent by sinks, server → client).
pub mod client_cmd {
    /// GP Notification Response.
    pub const NOTIFICATION_RESPONSE: u8 = 0x00;
    /// GP Pairing.
    pub const PAIRING: u8 = 0x01;
    /// GP Proxy Commissioning Mode.
    pub const PROXY_COMMISSIONING_MODE: u8 = 0x02;
    /// GP Response.
    pub const RESPONSE: u8 = 0x06;
    /// GP Translation Table Response.
    pub const TRANSLATION_TABLE_RESPONSE: u8 = 0x08;
    /// GP Sink Table Response.
    pub const SINK_TABLE_RESPONSE: u8 = 0x0A;
    /// GP Proxy Table Request.
    pub const PROXY_TABLE_REQUEST: u8 = 0x0B;
}

/// Green Power cluster attribute identifiers.
pub mod attr {
    /// gpsMaxSinkTableEntries.
    pub const GPS_MAX_SINK_TABLE_ENTRIES: u16 = 0x0000;
    /// Sink Table.
    pub const SINK_TABLE: u16 = 0x0001;
    /// gpsCommunicationMode.
    pub const GPS_COMMUNICATION_MODE: u16 = 0x0002;
    /// gpsCommissioningExitMode.
    pub const GPS_COMMISSIONING_EXIT_MODE: u16 = 0x0003;
    /// gpsCommissioningWindow.
    pub const GPS_COMMISSIONING_WINDOW: u16 = 0x0004;
    /// gpsSecurityLevel.
    pub const GPS_SECURITY_LEVEL: u16 = 0x0005;
    /// gpsFunctionality.
    pub const GPS_FUNCTIONALITY: u16 = 0x0006;
    /// gpsActiveFunctionality.
    pub const GPS_ACTIVE_FUNCTIONALITY: u16 = 0x0007;
    /// gppMaxProxyTableEntries.
    pub const GPP_MAX_PROXY_TABLE_ENTRIES: u16 = 0x0010;
    /// Proxy Table.
    pub const PROXY_TABLE: u16 = 0x0011;
    /// gppNotificationRetryNumber.
    pub const GPP_NOTIFICATION_RETRY_NUMBER: u16 = 0x0012;
    /// gppNotificationRetryTimer.
    pub const GPP_NOTIFICATION_RETRY_TIMER: u16 = 0x0013;
    /// gppMaxSearchCounter.
    pub const GPP_MAX_SEARCH_COUNTER: u16 = 0x0014;
    /// gppBlockedGPDID.
    pub const GPP_BLOCKED_GPD_ID: u16 = 0x0015;
    /// gppFunctionality.
    pub const GPP_FUNCTIONALITY: u16 = 0x0016;
    /// gppActiveFunctionality.
    pub const GPP_ACTIVE_FUNCTIONALITY: u16 = 0x0017;
    /// gpSharedSecurityKeyType.
    pub const GP_SHARED_SECURITY_KEY_TYPE: u16 = 0x0020;
    /// gpSharedSecurityKey.
    pub const GP_SHARED_SECURITY_KEY: u16 = 0x0021;
    /// gpLinkKey.
    pub const GP_LINK_KEY: u16 = 0x0022;
    /// ClusterRevision (global attribute).
    pub const CLUSTER_REVISION: u16 = 0xFFFD;
}

/// Green Power cluster revision implemented by this crate (GP 1.1).
pub const GP_CLUSTER_REVISION: u16 = 1;

/// gppFunctionality / gpsFunctionality bits (GP spec Table 41).
pub mod functionality {
    /// GP feature.
    pub const GP_FEATURE: u32 = 1 << 0;
    /// Direct communication (reception of GPDF).
    pub const DIRECT_COMMUNICATION: u32 = 1 << 1;
    /// Derived groupcast communication.
    pub const DERIVED_GROUPCAST: u32 = 1 << 2;
    /// Pre-commissioned groupcast communication.
    pub const PRECOMMISSIONED_GROUPCAST: u32 = 1 << 3;
    /// Full unicast communication.
    pub const FULL_UNICAST: u32 = 1 << 4;
    /// Lightweight unicast communication.
    pub const LIGHTWEIGHT_UNICAST: u32 = 1 << 5;
    /// Bidirectional operation.
    pub const BIDIRECTIONAL: u32 = 1 << 7;
    /// Proxy Table maintenance.
    pub const PROXY_TABLE_MAINTENANCE: u32 = 1 << 8;
    /// GP commissioning.
    pub const GP_COMMISSIONING: u32 = 1 << 10;
    /// CT-based commissioning.
    pub const CT_BASED_COMMISSIONING: u32 = 1 << 11;
    /// Maintenance of GPD.
    pub const GPD_MAINTENANCE: u32 = 1 << 12;
    /// Security level 0b00.
    pub const SECURITY_LEVEL_0: u32 = 1 << 13;
    /// Security level 0b10.
    pub const SECURITY_LEVEL_2: u32 = 1 << 15;
    /// Security level 0b11.
    pub const SECURITY_LEVEL_3: u32 = 1 << 16;
    /// GPD IEEE address (ApplicationID 0b010).
    pub const IEEE_ADDRESS: u32 = 1 << 19;

    /// GP Basic proxy functionality set (GP 1.1 Basic proxy).
    pub const BASIC_PROXY: u32 = GP_FEATURE
        | DIRECT_COMMUNICATION
        | DERIVED_GROUPCAST
        | PRECOMMISSIONED_GROUPCAST
        | LIGHTWEIGHT_UNICAST
        | GP_COMMISSIONING
        | CT_BASED_COMMISSIONING
        | SECURITY_LEVEL_0
        | SECURITY_LEVEL_2
        | SECURITY_LEVEL_3
        | IEEE_ADDRESS;
}

fn read_gpd_id(r: &mut Reader<'_>, app: ApplicationId) -> Result<GpdId> {
    match app {
        ApplicationId::SrcId => Ok(GpdId::SrcId(r.u32()?)),
        ApplicationId::Ieee => {
            let ieee = r.array::<8>()?;
            let endpoint = r.u8()?;
            Ok(GpdId::Ieee { ieee, endpoint })
        }
        ApplicationId::Lped => Err(Error::Unsupported),
    }
}

fn write_gpd_id(w: &mut Writer<'_>, id: &GpdId) -> Result<()> {
    match id {
        GpdId::SrcId(s) => w.u32(*s),
        GpdId::Ieee { ieee, endpoint } => {
            w.bytes(ieee)?;
            w.u8(*endpoint)
        }
    }
}

/// Read a GPD ID that is always 4 bytes on the wire, regardless of app id
/// (zigpy's Response / Notification Response / Pairing Search use SrcID only
/// for ApplicationID 0b000; IEEE GPDs carry IEEE + endpoint).
fn read_gpd_id_basic(r: &mut Reader<'_>, app: ApplicationId) -> Result<GpdId> {
    read_gpd_id(r, app)
}

fn read_payload<'a>(r: &mut Reader<'a>) -> Result<Option<&'a [u8]>> {
    let len = r.u8()?;
    if len == 0xFF {
        return Ok(None);
    }
    Ok(Some(r.bytes(len as usize)?))
}

fn write_payload(w: &mut Writer<'_>, payload: Option<&[u8]>) -> Result<()> {
    match payload {
        None => w.u8(0xFF),
        Some(p) => {
            if p.len() >= 0xFF {
                return Err(Error::BufferFull);
            }
            w.u8(p.len() as u8)?;
            w.bytes(p)
        }
    }
}

/// GPP → GPD link field (RSSI and link quality, GP spec Figure 27).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GppGpdLink(pub u8);

impl GppGpdLink {
    /// Build the link byte from an RSSI in dBm and an 802.15.4 LQI (0-255).
    ///
    /// RSSI is capped to −109..=+8 dBm, offset by 110 and halved; the
    /// 2-bit link quality is the LQI quartile.
    pub fn from_rssi_lqi(rssi_dbm: i8, lqi: u8) -> Self {
        let capped = rssi_dbm.clamp(-109, 8) as i16;
        let rssi = (((capped + 110) / 2) as u8) & 0x3F;
        Self(rssi | ((lqi >> 6) << 6))
    }

    /// Encoded RSSI part converted back to dBm.
    pub fn rssi_dbm(self) -> i16 {
        (self.0 & 0x3F) as i16 * 2 - 110
    }

    /// Link quality (0 poor .. 3 excellent).
    pub fn link_quality(self) -> u8 {
        self.0 >> 6
    }
}

/// Optional proxy information attached to (Commissioning) Notifications.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProxyInfo {
    /// No proxy information (rxAfterTx must be false).
    None,
    /// Proxy info present: GPP short address and GPP-GPD link.
    Link {
        /// Short address of the forwarding proxy.
        gpp_short: u16,
        /// Link quality of the received GPDF.
        link: GppGpdLink,
    },
    /// rxAfterTx without proxy info: GPP short address and distance.
    Distance {
        /// Short address of the forwarding proxy.
        gpp_short: u16,
        /// GPP distance to the GPD.
        distance: u8,
    },
}

impl ProxyInfo {
    fn proxy_info_present(&self) -> bool {
        matches!(self, Self::Link { .. })
    }

    fn validate(&self, rx_after_tx: bool) -> Result<()> {
        match (self, rx_after_tx) {
            (Self::None, true) | (Self::Distance { .. }, false) => Err(Error::Unsupported),
            _ => Ok(()),
        }
    }

    fn write(&self, w: &mut Writer<'_>) -> Result<()> {
        match *self {
            Self::None => Ok(()),
            Self::Link { gpp_short, link } => {
                w.u16(gpp_short)?;
                w.u8(link.0)
            }
            Self::Distance {
                gpp_short,
                distance,
            } => {
                w.u16(gpp_short)?;
                w.u8(distance)
            }
        }
    }

    fn read(r: &mut Reader<'_>, proxy_info_present: bool, rx_after_tx: bool) -> Result<Self> {
        if proxy_info_present {
            let gpp_short = r.u16()?;
            Ok(Self::Link {
                gpp_short,
                link: GppGpdLink(r.u8()?),
            })
        } else if rx_after_tx {
            let gpp_short = r.u16()?;
            Ok(Self::Distance {
                gpp_short,
                distance: r.u8()?,
            })
        } else {
            Ok(Self::None)
        }
    }
}

/// GP Notification (client → server 0x00).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Notification<'a> {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// Also sent in unicast.
    pub also_unicast: bool,
    /// Also sent to the derived group.
    pub also_derived_group: bool,
    /// Also sent to a commissioned group.
    pub also_commissioned_group: bool,
    /// Security level of the GPDF.
    pub security_level: SecurityLevel,
    /// Key type of the GPDF.
    pub key_type: KeyType,
    /// RxAfterTx of the GPDF.
    pub rx_after_tx: bool,
    /// gpTxQueue full for this GPD.
    pub tx_queue_full: bool,
    /// Proxy is bidirectional-capable.
    pub bidirectional_cap: bool,
    /// Security frame counter, or MAC sequence number for level 0.
    pub frame_counter: u32,
    /// GPD CommandID.
    pub command_id: u8,
    /// GPD command payload (`None` = unspecified, 0xFF length).
    pub payload: Option<&'a [u8]>,
    /// Proxy information.
    pub proxy_info: ProxyInfo,
}

impl<'a> Notification<'a> {
    /// Encode the options word.
    pub fn options(&self) -> u16 {
        (self.gpd_id.application_id() as u16)
            | (self.also_unicast as u16) << 3
            | (self.also_derived_group as u16) << 4
            | (self.also_commissioned_group as u16) << 5
            | (self.security_level as u16) << 6
            | (self.key_type as u16) << 8
            | (self.rx_after_tx as u16) << 11
            | (self.tx_queue_full as u16) << 12
            | (self.bidirectional_cap as u16) << 13
            | (self.proxy_info.proxy_info_present() as u16) << 14
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        self.proxy_info.validate(self.rx_after_tx)?;
        let mut w = Writer::new(out);
        w.u16(self.options())?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        w.u32(self.frame_counter)?;
        w.u8(self.command_id)?;
        write_payload(&mut w, self.payload)?;
        self.proxy_info.write(&mut w)?;
        Ok(w.len())
    }

    /// Strictly decode a GP Notification payload.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let app = ApplicationId::from_bits(o as u8)?;
        let gpd_id = read_gpd_id(&mut r, app)?;
        let rx_after_tx = o & (1 << 11) != 0;
        let frame_counter = r.u32()?;
        let command_id = r.u8()?;
        let payload = read_payload(&mut r)?;
        let proxy_info = ProxyInfo::read(&mut r, o & (1 << 14) != 0, rx_after_tx)?;
        r.finish()?;
        Ok(Self {
            gpd_id,
            also_unicast: o & (1 << 3) != 0,
            also_derived_group: o & (1 << 4) != 0,
            also_commissioned_group: o & (1 << 5) != 0,
            security_level: SecurityLevel::from_bits((o >> 6) as u8),
            key_type: KeyType::from_bits((o >> 8) as u8)?,
            rx_after_tx,
            tx_queue_full: o & (1 << 12) != 0,
            bidirectional_cap: o & (1 << 13) != 0,
            frame_counter,
            command_id,
            payload,
            proxy_info,
        })
    }
}

/// GP Commissioning Notification (client → server 0x04).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissioningNotification<'a> {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// RxAfterTx of the GPDF.
    pub rx_after_tx: bool,
    /// Security level of the GPDF.
    pub security_level: SecurityLevel,
    /// Key type the proxy used / would use.
    pub key_type: KeyType,
    /// Security processing failed; `mic` carries the GPDF MIC and the
    /// command id / payload are forwarded still protected.
    pub security_failed: Option<u32>,
    /// Proxy is bidirectional-capable.
    pub bidirectional_cap: bool,
    /// Security frame counter, or MAC sequence number for level 0.
    pub frame_counter: u32,
    /// GPD CommandID.
    pub command_id: u8,
    /// GPD command payload (`None` = unspecified).
    pub payload: Option<&'a [u8]>,
    /// Proxy information.
    pub proxy_info: ProxyInfo,
}

impl<'a> CommissioningNotification<'a> {
    /// Encode the options word.
    pub fn options(&self) -> u16 {
        (self.gpd_id.application_id() as u16)
            | (self.rx_after_tx as u16) << 3
            | (self.security_level as u16) << 4
            | (self.key_type as u16) << 6
            | (self.security_failed.is_some() as u16) << 9
            | (self.bidirectional_cap as u16) << 10
            | (self.proxy_info.proxy_info_present() as u16) << 11
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        self.proxy_info.validate(self.rx_after_tx)?;
        let mut w = Writer::new(out);
        w.u16(self.options())?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        w.u32(self.frame_counter)?;
        w.u8(self.command_id)?;
        write_payload(&mut w, self.payload)?;
        self.proxy_info.write(&mut w)?;
        if let Some(mic) = self.security_failed {
            w.u32(mic)?;
        }
        Ok(w.len())
    }

    /// Strictly decode a GP Commissioning Notification payload.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let app = ApplicationId::from_bits(o as u8)?;
        let gpd_id = read_gpd_id(&mut r, app)?;
        let rx_after_tx = o & (1 << 3) != 0;
        let frame_counter = r.u32()?;
        let command_id = r.u8()?;
        let payload = read_payload(&mut r)?;
        let proxy_info = ProxyInfo::read(&mut r, o & (1 << 11) != 0, rx_after_tx)?;
        let security_failed = if o & (1 << 9) != 0 {
            Some(r.u32()?)
        } else {
            None
        };
        r.finish()?;
        Ok(Self {
            gpd_id,
            rx_after_tx,
            security_level: SecurityLevel::from_bits((o >> 4) as u8),
            key_type: KeyType::from_bits((o >> 6) as u8)?,
            security_failed,
            bidirectional_cap: o & (1 << 10) != 0,
            frame_counter,
            command_id,
            payload,
            proxy_info,
        })
    }
}

/// Sink addressing carried by a GP Pairing command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairingSink {
    /// No sink fields (RemoveGPD).
    None,
    /// Unicast sink (full or lightweight unicast).
    Unicast {
        /// Sink IEEE address.
        ieee: IeeeAddress,
        /// Sink NWK address.
        nwk: u16,
    },
    /// Groupcast sink group (derived or commissioned).
    Group(u16),
}

/// GP Pairing (server → client 0x01).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pairing {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// AddSink.
    pub add_sink: bool,
    /// RemoveGPD.
    pub remove_gpd: bool,
    /// Communication mode.
    pub communication_mode: CommunicationMode,
    /// GPDfixed.
    pub gpd_fixed: bool,
    /// GPD MAC sequence number capability (incremental).
    pub mac_seq_cap: bool,
    /// Security level.
    pub security_level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// Sink addressing.
    pub sink: PairingSink,
    /// DeviceID (present iff AddSink).
    pub device_id: Option<u8>,
    /// GPD security frame counter.
    pub frame_counter: Option<u32>,
    /// GPD key.
    pub key: Option<AesKey>,
    /// Assigned alias.
    pub assigned_alias: Option<u16>,
    /// Groupcast radius.
    pub groupcast_radius: Option<u8>,
}

impl Pairing {
    /// Encode the 24-bit options field.
    pub fn options(&self) -> u32 {
        (self.gpd_id.application_id() as u32)
            | (self.add_sink as u32) << 3
            | (self.remove_gpd as u32) << 4
            | (self.communication_mode as u32) << 5
            | (self.gpd_fixed as u32) << 7
            | (self.mac_seq_cap as u32) << 8
            | (self.security_level as u32) << 9
            | (self.key_type as u32) << 11
            | (self.frame_counter.is_some() as u32) << 14
            | (self.key.is_some() as u32) << 15
            | (self.assigned_alias.is_some() as u32) << 16
            | (self.groupcast_radius.is_some() as u32) << 17
    }

    fn sink_expected(remove_gpd: bool, mode: CommunicationMode) -> u8 {
        if remove_gpd {
            0
        } else if mode.is_unicast() {
            1
        } else {
            2
        }
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let expected = Self::sink_expected(self.remove_gpd, self.communication_mode);
        let actual = match self.sink {
            PairingSink::None => 0,
            PairingSink::Unicast { .. } => 1,
            PairingSink::Group(_) => 2,
        };
        if expected != actual || self.add_sink != self.device_id.is_some() {
            return Err(Error::Unsupported);
        }
        if !self.add_sink
            && (self.frame_counter.is_some()
                || self.key.is_some()
                || self.assigned_alias.is_some()
                || self.groupcast_radius.is_some())
        {
            return Err(Error::Unsupported);
        }
        let mut w = Writer::new(out);
        w.u24(self.options())?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        match self.sink {
            PairingSink::None => {}
            PairingSink::Unicast { ieee, nwk } => {
                w.bytes(&ieee)?;
                w.u16(nwk)?;
            }
            PairingSink::Group(g) => w.u16(g)?,
        }
        if let Some(d) = self.device_id {
            w.u8(d)?;
        }
        if let Some(fc) = self.frame_counter {
            w.u32(fc)?;
        }
        if let Some(k) = self.key {
            w.bytes(&k)?;
        }
        if let Some(a) = self.assigned_alias {
            w.u16(a)?;
        }
        if let Some(r) = self.groupcast_radius {
            w.u8(r)?;
        }
        Ok(w.len())
    }

    /// Strictly decode a GP Pairing payload.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u24()?;
        let app = ApplicationId::from_bits(o as u8)?;
        let gpd_id = read_gpd_id(&mut r, app)?;
        let add_sink = o & (1 << 3) != 0;
        let remove_gpd = o & (1 << 4) != 0;
        let mode = CommunicationMode::from_bits((o >> 5) as u8);
        let sink = match Self::sink_expected(remove_gpd, mode) {
            0 => PairingSink::None,
            1 => {
                let ieee = r.array::<8>()?;
                PairingSink::Unicast {
                    ieee,
                    nwk: r.u16()?,
                }
            }
            _ => PairingSink::Group(r.u16()?),
        };
        let device_id = if add_sink { Some(r.u8()?) } else { None };
        let fc = if add_sink && o & (1 << 14) != 0 {
            Some(r.u32()?)
        } else {
            None
        };
        let key = if add_sink && o & (1 << 15) != 0 {
            Some(r.array::<16>()?)
        } else {
            None
        };
        let alias = if add_sink && o & (1 << 16) != 0 {
            Some(r.u16()?)
        } else {
            None
        };
        let radius = if add_sink && o & (1 << 17) != 0 {
            Some(r.u8()?)
        } else {
            None
        };
        r.finish()?;
        Ok(Self {
            gpd_id,
            add_sink,
            remove_gpd,
            communication_mode: mode,
            gpd_fixed: o & (1 << 7) != 0,
            mac_seq_cap: o & (1 << 8) != 0,
            security_level: SecurityLevel::from_bits((o >> 9) as u8),
            key_type: KeyType::from_bits((o >> 11) as u8)?,
            sink,
            device_id,
            frame_counter: fc,
            key,
            assigned_alias: alias,
            groupcast_radius: radius,
        })
    }
}

/// Exit-mode bits of GP Proxy Commissioning Mode.
pub mod exit_mode {
    /// Exit on the first GP Pairing received.
    pub const ON_FIRST_PAIRING: u8 = 0b01;
    /// Exit on an explicit exit command.
    pub const ON_EXPLICIT_EXIT: u8 = 0b10;
}

/// GP Proxy Commissioning Mode (server → client 0x02).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProxyCommissioningMode {
    /// Enter (true) or exit (false) commissioning mode.
    pub enter: bool,
    /// Exit mode bits ([`exit_mode`]).
    pub exit_mode: u8,
    /// Commissioning window in seconds.
    pub window_s: Option<u16>,
    /// Channel (low nibble = channel − 11).
    pub channel: Option<u8>,
    /// Unicast communication requested.
    pub unicast: bool,
}

impl ProxyCommissioningMode {
    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        let o = (self.enter as u8)
            | (self.window_s.is_some() as u8) << 1
            | (self.exit_mode & 0x03) << 2
            | (self.channel.is_some() as u8) << 4
            | (self.unicast as u8) << 5;
        w.u8(o)?;
        if let Some(win) = self.window_s {
            w.u16(win)?;
        }
        if let Some(ch) = self.channel {
            w.u8(ch)?;
        }
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let window_s = if o & 0x02 != 0 { Some(r.u16()?) } else { None };
        let channel = if o & 0x10 != 0 { Some(r.u8()?) } else { None };
        r.finish()?;
        Ok(Self {
            enter: o & 0x01 != 0,
            exit_mode: (o >> 2) & 0x03,
            window_s,
            channel,
            unicast: o & 0x20 != 0,
        })
    }
}

/// GP Response (server → client 0x06).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Response<'a> {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// Transmit on endpoint match (IEEE GPDs).
    pub transmit_on_endpoint_match: bool,
    /// Appointed TempMaster short address.
    pub temp_master_short: u16,
    /// TempMaster Tx channel (low nibble = channel − 11).
    pub temp_master_tx_channel: u8,
    /// GPD CommandID to send.
    pub command_id: u8,
    /// GPD command payload.
    pub payload: &'a [u8],
}

impl<'a> Response<'a> {
    /// IEEE 802.15.4 channel the TempMaster transmits on.
    pub fn tx_channel(&self) -> u8 {
        (self.temp_master_tx_channel & 0x0F) + 11
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.gpd_id.application_id() as u8 | (self.transmit_on_endpoint_match as u8) << 3)?;
        w.u16(self.temp_master_short)?;
        w.u8(self.temp_master_tx_channel)?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        w.u8(self.command_id)?;
        write_payload(&mut w, Some(self.payload))?;
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let app = ApplicationId::from_bits(o)?;
        let temp_master_short = r.u16()?;
        let temp_master_tx_channel = r.u8()?;
        let gpd_id = read_gpd_id_basic(&mut r, app)?;
        let command_id = r.u8()?;
        let payload = read_payload(&mut r)?.unwrap_or(&[]);
        r.finish()?;
        Ok(Self {
            gpd_id,
            transmit_on_endpoint_match: o & 0x08 != 0,
            temp_master_short,
            temp_master_tx_channel,
            command_id,
            payload,
        })
    }
}

/// GP Notification Response (server → client 0x00).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotificationResponse {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// FirstToForward.
    pub first_to_forward: bool,
    /// NoPairing.
    pub no_pairing: bool,
    /// Security frame counter being acknowledged.
    pub frame_counter: u32,
}

impl NotificationResponse {
    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.gpd_id.application_id() as u8
            | (self.first_to_forward as u8) << 3
            | (self.no_pairing as u8) << 4)?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        w.u32(self.frame_counter)?;
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let gpd_id = read_gpd_id(&mut r, ApplicationId::from_bits(o)?)?;
        let frame_counter = r.u32()?;
        r.finish()?;
        Ok(Self {
            gpd_id,
            first_to_forward: o & 0x08 != 0,
            no_pairing: o & 0x10 != 0,
            frame_counter,
        })
    }
}

/// GP Pairing Search (client → server 0x01).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PairingSearch {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// Request unicast sinks.
    pub request_unicast: bool,
    /// Request derived groupcast sinks.
    pub request_derived_groupcast: bool,
    /// Request commissioned groupcast sinks.
    pub request_commissioned_groupcast: bool,
    /// Request the frame counter.
    pub request_frame_counter: bool,
    /// Request the security key.
    pub request_key: bool,
}

impl PairingSearch {
    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(
            self.gpd_id.application_id() as u16
                | (self.request_unicast as u16) << 3
                | (self.request_derived_groupcast as u16) << 4
                | (self.request_commissioned_groupcast as u16) << 5
                | (self.request_frame_counter as u16) << 6
                | (self.request_key as u16) << 7,
        )?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let gpd_id = read_gpd_id(&mut r, ApplicationId::from_bits(o as u8)?)?;
        r.finish()?;
        Ok(Self {
            gpd_id,
            request_unicast: o & (1 << 3) != 0,
            request_derived_groupcast: o & (1 << 4) != 0,
            request_commissioned_groupcast: o & (1 << 5) != 0,
            request_frame_counter: o & (1 << 6) != 0,
            request_key: o & (1 << 7) != 0,
        })
    }
}

/// GP Proxy Table Request (server → client 0x0B).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProxyTableRequest {
    /// Request the entry for a GPD.
    ByGpdId(GpdId),
    /// Request entries starting at an index.
    ByIndex(u8),
}

impl ProxyTableRequest {
    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let req = match (o >> 3) & 0x03 {
            0 => Self::ByGpdId(read_gpd_id(&mut r, ApplicationId::from_bits(o)?)?),
            1 => Self::ByIndex(r.u8()?),
            _ => return Err(Error::Unsupported),
        };
        r.finish()?;
        Ok(req)
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        match self {
            Self::ByGpdId(id) => {
                w.u8(id.application_id() as u8)?;
                write_gpd_id(&mut w, id)?;
            }
            Self::ByIndex(i) => {
                w.u8(1 << 3)?;
                w.u8(*i)?;
            }
        }
        Ok(w.len())
    }
}

/// ZCL status values used in table responses.
pub mod status {
    /// Success.
    pub const SUCCESS: u8 = 0x00;
    /// Not found.
    pub const NOT_FOUND: u8 = 0x8B;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_byte_matches_zigpy_formula() {
        let l = GppGpdLink::from_rssi_lqi(-60, 200);
        assert_eq!(l.rssi_dbm(), -60);
        assert_eq!(l.link_quality(), 3);
        let l = GppGpdLink::from_rssi_lqi(-120, 10);
        assert_eq!(l.rssi_dbm(), -110);
        assert_eq!(l.link_quality(), 0);
    }

    #[test]
    fn notification_round_trip_and_strictness() {
        let n = Notification {
            gpd_id: GpdId::SrcId(0x0102_0304),
            also_unicast: false,
            also_derived_group: false,
            also_commissioned_group: true,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            rx_after_tx: false,
            tx_queue_full: false,
            bidirectional_cap: false,
            frame_counter: 7,
            command_id: 0x20,
            payload: Some(&[]),
            proxy_info: ProxyInfo::Link {
                gpp_short: 0x045B,
                link: GppGpdLink(0xC0 | 20),
            },
        };
        let mut buf = [0u8; 32];
        let len = n.encode(&mut buf).unwrap();
        assert_eq!(Notification::decode(&buf[..len]).unwrap(), n);
        assert_eq!(
            Notification::decode(&buf[..len + 1]),
            Err(Error::TrailingBytes)
        );
    }
}
