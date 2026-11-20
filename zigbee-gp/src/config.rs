//! GP sink configuration commands of the Green Power cluster: GP Pairing
//! Configuration (client → server 0x09), GP Translation Table Update (0x07)
//! and Request (0x08), GP Translation Table Response (server → client 0x08),
//! and decoders for the GP Sink Table Response (server → client 0x0A) and
//! GP Proxy Table Response (client → server 0x0B) a commissioning tool
//! receives.
//!
//! A commissioning tool (a hub, a ZRHP network co-processor) encodes the
//! requests and decodes the responses; a sink with the translation table
//! ([`crate::sink`], feature `translation-table`) decodes the requests and
//! encodes the response.
//!
//! No GP specification text is available to this project. The field order,
//! presence conditions and value semantics follow the certified Silicon Labs
//! GP 1.1 implementation: gecko_sdk 4.5 `app/zcl/green-power.xml`
//! (`GpPairingConfiguration`, `GpTranslationTableUpdate`,
//! `GpTranslationTableResponse`), the generated parser
//! `zcl_decode_green_power_cluster_gp_pairing_configuration_command`, and
//! `green-power-translation-table.c` (`copyTranslationTableEntryToBuffer`,
//! `emberAfGreenPowerClusterGpTranslationTableUpdateCallback`).

use zigbee_crypto::AesKey;

use crate::codec::{Reader, Writer};
use crate::types::{ApplicationId, CommunicationMode, GpdId, KeyType, SecurityLevel};
use crate::{Error, Result};

/// GP Pairing Configuration `Actions` field.
pub mod pairing_action {
    /// Action sub-field mask (bits 0-2).
    pub const ACTION_MASK: u8 = 0x07;
    /// No action (with [`SEND_GP_PAIRING`]: re-send the GP Pairing of the
    /// existing entry).
    pub const NO_ACTION: u8 = 0b000;
    /// Extend the Sink Table entry (create it when missing).
    pub const EXTEND: u8 = 0b001;
    /// Replace the Sink Table entry.
    pub const REPLACE: u8 = 0b010;
    /// Remove this sink's pairing of the GPD.
    pub const REMOVE_PAIRING: u8 = 0b011;
    /// Remove the GPD.
    pub const REMOVE_GPD: u8 = 0b100;
    /// Application Description (report descriptors of the GPD).
    pub const APPLICATION_DESCRIPTION: u8 = 0b101;
    /// Bit 3: the sink sends GP Pairing to the proxies.
    pub const SEND_GP_PAIRING: u8 = 0x08;
}

/// GP Pairing Configuration `ApplicationInformation` bits.
pub mod app_info {
    /// Manufacturer ID present.
    pub const MANUFACTURER_ID: u8 = 0x01;
    /// Model ID present.
    pub const MODEL_ID: u8 = 0x02;
    /// GPD command list present.
    pub const GPD_COMMANDS: u8 = 0x04;
    /// Cluster list present.
    pub const CLUSTER_LIST: u8 = 0x08;
    /// Switch information present.
    pub const SWITCH_INFORMATION: u8 = 0x10;
    /// Application description (report descriptors) present / follows.
    pub const APPLICATION_DESCRIPTION: u8 = 0x20;
}

/// Special `NumberOfPairedEndpoints` values (gecko_sdk
/// `green-power-common.h`).
pub mod paired_endpoints {
    /// No paired endpoint: no translation for the GPD.
    pub const NONE: u8 = 0x00;
    /// Reserved.
    pub const RESERVED: u8 = 0xFD;
    /// The sink derives the endpoints.
    pub const SINK_DERIVES: u8 = 0xFE;
    /// All matching sink endpoints.
    pub const ALL: u8 = 0xFF;
}

/// GP Translation Table Update action (options bits 3-4).
pub mod tt_action {
    /// Add translations.
    pub const ADD: u8 = 0b00;
    /// Replace the translations of the same (GPD ID, GPD command,
    /// endpoint, profile, cluster).
    pub const REPLACE: u8 = 0b01;
    /// Remove the translations of the same (GPD ID, GPD command, endpoint,
    /// profile, cluster).
    pub const REMOVE: u8 = 0b10;
}

/// Translation endpoint that hands the GPD command to the application
/// untranslated (gecko_sdk
/// `EMBER_AF_GP_TRANSLATION_TABLE_ZB_ENDPOINT_PASS_FRAME_TO_APLLICATION`).
pub const PASS_TO_APPLICATION: u8 = 0xFC;
/// Translations in one GP Translation Table Update (3-bit count).
pub const MAX_TRANSLATIONS_PER_UPDATE: usize = 7;
/// `ZigbeePayloadLength` of a payload derived from the GPD command
/// (gecko_sdk `GREEN_POWER_SERVER_GPD_COMMAND_PAYLOAD_TT_DERIVED_LENGTH`).
pub const PAYLOAD_DERIVED: u8 = 0xFE;
/// `ZigbeePayloadLength` of a payload copied from the GPD command (gecko_sdk
/// `GREEN_POWER_SERVER_GPD_COMMAND_PAYLOAD_UNSPECIFIED_LENGTH`).
pub const PAYLOAD_COPY_GPD: u8 = 0xFF;

fn read_gpd_id(r: &mut Reader<'_>, app: ApplicationId) -> Result<GpdId> {
    match app {
        ApplicationId::SrcId => Ok(GpdId::SrcId(r.u32()?)),
        ApplicationId::Ieee => {
            let ieee = r.array::<8>()?;
            Ok(GpdId::Ieee {
                ieee,
                endpoint: r.u8()?,
            })
        }
        ApplicationId::Lped => Err(Error::Unsupported),
    }
}

fn write_gpd_id(w: &mut Writer<'_>, id: &GpdId) -> Result<()> {
    match *id {
        GpdId::SrcId(s) => w.u32(s),
        GpdId::Ieee { ieee, endpoint } => {
            w.bytes(&ieee)?;
            w.u8(endpoint)
        }
    }
}

/// Security of a GP Pairing Configuration (`SecurityUse`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PairingSecurity {
    /// Security level (the reserved level is rejected by the sink).
    pub level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// GPD key.
    pub key: AesKey,
}

/// GP Pairing Configuration application information.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ApplicationInformation<'a> {
    /// [`app_info`] bits.
    pub bits: u8,
    /// Manufacturer ID ([`app_info::MANUFACTURER_ID`]).
    pub manufacturer_id: Option<u16>,
    /// Model ID ([`app_info::MODEL_ID`]).
    pub model_id: Option<u16>,
    /// GPD command list ([`app_info::GPD_COMMANDS`]).
    pub gpd_commands: &'a [u8],
    /// Server cluster IDs, little-endian `u16` each
    /// ([`app_info::CLUSTER_LIST`]).
    pub server_clusters: &'a [u8],
    /// Client cluster IDs, little-endian `u16` each.
    pub client_clusters: &'a [u8],
    /// Switch information after its length byte
    /// ([`app_info::SWITCH_INFORMATION`]): configuration, current contacts.
    pub switch_information: &'a [u8],
    /// Total number of reports, number of reports and the report
    /// descriptors ([`app_info::APPLICATION_DESCRIPTION`]); the descriptors
    /// run to the end of the frame.
    pub application_description: Option<(u8, u8, &'a [u8])>,
}

/// GP Pairing Configuration (client → server 0x09).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PairingConfiguration<'a> {
    /// `Actions` ([`pairing_action`]).
    pub actions: u8,
    /// GPD identity.
    pub gpd_id: GpdId,
    /// Communication mode.
    pub communication_mode: CommunicationMode,
    /// GPD MAC sequence number capability.
    pub mac_seq_cap: bool,
    /// GPD RxOnCapability.
    pub rx_on_cap: bool,
    /// GPD fixed location.
    pub fixed_location: bool,
    /// GPD DeviceID.
    pub device_id: u8,
    /// Group list of [`CommunicationMode::CommissionedGroupcast`]: (group,
    /// alias) pairs, 4 bytes each, little-endian.
    pub groups: &'a [u8],
    /// Assigned alias.
    pub assigned_alias: Option<u16>,
    /// Groupcast radius.
    pub groupcast_radius: u8,
    /// `SecurityUse` fields.
    pub security: Option<PairingSecurity>,
    /// GPD security frame counter (present with security or the sequence
    /// number capability).
    pub frame_counter: Option<u32>,
    /// `NumberOfPairedEndpoints` ([`paired_endpoints`] or a count).
    pub number_of_paired_endpoints: u8,
    /// Paired endpoints (when the count is 1..=0xFC).
    pub paired_endpoints: &'a [u8],
    /// Application information.
    pub application_information: Option<ApplicationInformation<'a>>,
}

impl<'a> PairingConfiguration<'a> {
    /// The action sub-field.
    pub fn action(&self) -> u8 {
        self.actions & pairing_action::ACTION_MASK
    }

    /// Whether the sink must send GP Pairing.
    pub fn send_gp_pairing(&self) -> bool {
        self.actions & pairing_action::SEND_GP_PAIRING != 0
    }

    /// The commissioned (group, alias) pairs; alias `0xFFFF` means derived.
    pub fn group_list(&self) -> impl Iterator<Item = (u16, u16)> + 'a {
        self.groups.chunks_exact(4).map(|c| {
            (
                u16::from_le_bytes([c[0], c[1]]),
                u16::from_le_bytes([c[2], c[3]]),
            )
        })
    }

    /// The 16-bit options field.
    pub fn options(&self) -> u16 {
        (self.gpd_id.application_id() as u16)
            | (self.communication_mode as u16) << 3
            | (self.mac_seq_cap as u16) << 5
            | (self.rx_on_cap as u16) << 6
            | (self.fixed_location as u16) << 7
            | (self.assigned_alias.is_some() as u16) << 8
            | (self.security.is_some() as u16) << 9
            | (self.application_information.is_some() as u16) << 10
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let groupcast = self.communication_mode == CommunicationMode::CommissionedGroupcast;
        let counted = (1..paired_endpoints::RESERVED).contains(&self.number_of_paired_endpoints);
        if !self.groups.len().is_multiple_of(4)
            || (!groupcast && !self.groups.is_empty())
            || self.groups.len() / 4 > u8::MAX as usize
            || self.frame_counter.is_some() != (self.security.is_some() || self.mac_seq_cap)
            || (counted && self.paired_endpoints.len() != self.number_of_paired_endpoints as usize)
            || (!counted && !self.paired_endpoints.is_empty())
        {
            return Err(Error::Unsupported);
        }
        let mut w = Writer::new(out);
        w.u8(self.actions)?;
        w.u16(self.options())?;
        write_gpd_id(&mut w, &self.gpd_id)?;
        w.u8(self.device_id)?;
        if groupcast {
            w.u8((self.groups.len() / 4) as u8)?;
            w.bytes(self.groups)?;
        }
        if let Some(alias) = self.assigned_alias {
            w.u16(alias)?;
        }
        w.u8(self.groupcast_radius)?;
        if let Some(s) = self.security {
            w.u8(s.level as u8 | (s.key_type as u8) << 2)?;
        }
        if let Some(fc) = self.frame_counter {
            w.u32(fc)?;
        }
        if let Some(s) = self.security {
            w.bytes(&s.key)?;
        }
        w.u8(self.number_of_paired_endpoints)?;
        w.bytes(self.paired_endpoints)?;
        if let Some(info) = &self.application_information {
            encode_app_info(&mut w, info)?;
        }
        Ok(w.len())
    }

    /// Decode. Every field up to the application information is checked
    /// strictly; trailing bytes are an error except after an application
    /// description, whose report descriptors run to the end.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let actions = r.u8()?;
        let o = r.u16()?;
        let gpd_id = read_gpd_id(&mut r, ApplicationId::from_bits(o as u8)?)?;
        let communication_mode = CommunicationMode::from_bits((o >> 3) as u8);
        let device_id = r.u8()?;
        let groups = if communication_mode == CommunicationMode::CommissionedGroupcast {
            let n = r.u8()? as usize;
            r.bytes(n * 4)?
        } else {
            &[]
        };
        let assigned_alias = if o & (1 << 8) != 0 {
            Some(r.u16()?)
        } else {
            None
        };
        let groupcast_radius = r.u8()?;
        let security_options = if o & (1 << 9) != 0 {
            Some(r.u8()?)
        } else {
            None
        };
        let mac_seq_cap = o & (1 << 5) != 0;
        let frame_counter = if security_options.is_some() || mac_seq_cap {
            Some(r.u32()?)
        } else {
            None
        };
        let security = match security_options {
            Some(so) => Some(PairingSecurity {
                level: SecurityLevel::from_bits(so),
                key_type: KeyType::from_bits(so >> 2)?,
                key: r.array::<16>()?,
            }),
            None => None,
        };
        let number_of_paired_endpoints = r.u8()?;
        let paired_endpoints =
            if (1..paired_endpoints::RESERVED).contains(&number_of_paired_endpoints) {
                r.bytes(number_of_paired_endpoints as usize)?
            } else {
                &[]
            };
        let application_information = if o & (1 << 10) != 0 {
            Some(decode_app_info(&mut r)?)
        } else {
            None
        };
        r.finish()?;
        Ok(Self {
            actions,
            gpd_id,
            communication_mode,
            mac_seq_cap,
            rx_on_cap: o & (1 << 6) != 0,
            fixed_location: o & (1 << 7) != 0,
            device_id,
            groups,
            assigned_alias,
            groupcast_radius,
            security,
            frame_counter,
            number_of_paired_endpoints,
            paired_endpoints,
            application_information,
        })
    }
}

fn decode_app_info<'a>(r: &mut Reader<'a>) -> Result<ApplicationInformation<'a>> {
    let bits = r.u8()?;
    let mut info = ApplicationInformation {
        bits,
        ..ApplicationInformation::default()
    };
    if bits & app_info::MANUFACTURER_ID != 0 {
        info.manufacturer_id = Some(r.u16()?);
    }
    if bits & app_info::MODEL_ID != 0 {
        info.model_id = Some(r.u16()?);
    }
    if bits & app_info::GPD_COMMANDS != 0 {
        let n = r.u8()? as usize;
        info.gpd_commands = r.bytes(n)?;
    }
    if bits & app_info::CLUSTER_LIST != 0 {
        let counts = r.u8()?;
        info.server_clusters = r.bytes(usize::from(counts & 0x0F) * 2)?;
        info.client_clusters = r.bytes(usize::from(counts >> 4) * 2)?;
    }
    if bits & app_info::SWITCH_INFORMATION != 0 {
        let n = r.u8()? as usize;
        info.switch_information = r.bytes(n)?;
    }
    if bits & app_info::APPLICATION_DESCRIPTION != 0 {
        let total = r.u8()?;
        let count = r.u8()?;
        info.application_description = Some((total, count, r.rest()));
    }
    Ok(info)
}

fn encode_app_info(w: &mut Writer<'_>, info: &ApplicationInformation<'_>) -> Result<()> {
    use app_info::*;
    let present = |bit: u8, has: bool| (info.bits & bit != 0) == has;
    if !present(MANUFACTURER_ID, info.manufacturer_id.is_some())
        || !present(MODEL_ID, info.model_id.is_some())
        || !present(
            APPLICATION_DESCRIPTION,
            info.application_description.is_some(),
        )
        || (info.bits & GPD_COMMANDS == 0 && !info.gpd_commands.is_empty())
        || (info.bits & CLUSTER_LIST == 0
            && !(info.server_clusters.is_empty() && info.client_clusters.is_empty()))
        || (info.bits & SWITCH_INFORMATION == 0 && !info.switch_information.is_empty())
        || !info.server_clusters.len().is_multiple_of(2)
        || !info.client_clusters.len().is_multiple_of(2)
        || info.server_clusters.len() / 2 > 0x0F
        || info.client_clusters.len() / 2 > 0x0F
        || info.gpd_commands.len() > u8::MAX as usize
        || info.switch_information.len() > u8::MAX as usize
    {
        return Err(Error::Unsupported);
    }
    w.u8(info.bits)?;
    if let Some(m) = info.manufacturer_id {
        w.u16(m)?;
    }
    if let Some(m) = info.model_id {
        w.u16(m)?;
    }
    if info.bits & GPD_COMMANDS != 0 {
        w.u8(info.gpd_commands.len() as u8)?;
        w.bytes(info.gpd_commands)?;
    }
    if info.bits & CLUSTER_LIST != 0 {
        w.u8(
            (info.server_clusters.len() / 2) as u8 | ((info.client_clusters.len() / 2) as u8) << 4
        )?;
        w.bytes(info.server_clusters)?;
        w.bytes(info.client_clusters)?;
    }
    if info.bits & SWITCH_INFORMATION != 0 {
        w.u8(info.switch_information.len() as u8)?;
        w.bytes(info.switch_information)?;
    }
    if let Some((total, count, descriptors)) = info.application_description {
        w.u8(total)?;
        w.u8(count)?;
        w.bytes(descriptors)?;
    }
    Ok(())
}

/// The Zigbee payload of a translation (`ZigbeePayloadLength` semantics).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TtPayload<'a> {
    /// A fixed payload (length 0..=0xFD; empty: no payload).
    Fixed(&'a [u8]),
    /// Derived from the GPD command payload as in the sink's default
    /// translation ([`PAYLOAD_DERIVED`]).
    Derived,
    /// The GPD command payload, copied ([`PAYLOAD_COPY_GPD`]).
    CopyGpd,
}

/// One translation of a GP Translation Table Update or Response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TranslationRecord<'a> {
    /// Translation Table index (Update only; GP 1.1 sends 0xFF and the sink
    /// ignores it).
    pub index: u8,
    /// GPD CommandID.
    pub gpd_command: u8,
    /// Target endpoint on the sink ([`PASS_TO_APPLICATION`]: untranslated).
    pub endpoint: u8,
    /// Zigbee profile.
    pub profile: u16,
    /// Zigbee cluster.
    pub cluster: u16,
    /// Zigbee (ZCL cluster-specific) command.
    pub zcl_command: u8,
    /// Zigbee payload.
    pub payload: TtPayload<'a>,
    /// Additional information block contents (after its length byte).
    pub additional_info: Option<&'a [u8]>,
}

impl<'a> TranslationRecord<'a> {
    fn read(r: &mut Reader<'a>, index: bool, additional: bool) -> Result<Self> {
        let index = if index { r.u8()? } else { 0xFF };
        let gpd_command = r.u8()?;
        let endpoint = r.u8()?;
        let profile = r.u16()?;
        let cluster = r.u16()?;
        let zcl_command = r.u8()?;
        let payload = match r.u8()? {
            PAYLOAD_DERIVED => TtPayload::Derived,
            PAYLOAD_COPY_GPD => TtPayload::CopyGpd,
            n => TtPayload::Fixed(r.bytes(n as usize)?),
        };
        let additional_info = if additional {
            let n = r.u8()? as usize;
            Some(r.bytes(n)?)
        } else {
            None
        };
        Ok(Self {
            index,
            gpd_command,
            endpoint,
            profile,
            cluster,
            zcl_command,
            payload,
            additional_info,
        })
    }

    /// Write the record (`index`: the Update layout with the index byte).
    pub(crate) fn write(&self, w: &mut Writer<'_>, index: bool) -> Result<()> {
        if index {
            w.u8(self.index)?;
        }
        w.u8(self.gpd_command)?;
        w.u8(self.endpoint)?;
        w.u16(self.profile)?;
        w.u16(self.cluster)?;
        w.u8(self.zcl_command)?;
        match self.payload {
            TtPayload::Derived => w.u8(PAYLOAD_DERIVED)?,
            TtPayload::CopyGpd => w.u8(PAYLOAD_COPY_GPD)?,
            TtPayload::Fixed(p) => {
                if p.len() >= PAYLOAD_DERIVED as usize {
                    return Err(Error::BufferFull);
                }
                w.u8(p.len() as u8)?;
                w.bytes(p)?;
            }
        }
        if let Some(info) = self.additional_info {
            if info.len() > u8::MAX as usize {
                return Err(Error::BufferFull);
            }
            w.u8(info.len() as u8)?;
            w.bytes(info)?;
        }
        Ok(())
    }

    /// The (GPD command, endpoint, profile, cluster) part of the quintuple
    /// that identifies translations for Replace and Remove.
    pub fn key(&self) -> (u8, u8, u16, u16) {
        (self.gpd_command, self.endpoint, self.profile, self.cluster)
    }
}

/// Iterator over translations validated by a decoder.
pub struct Translations<'a> {
    reader: Reader<'a>,
    left: u8,
    index: bool,
    additional: bool,
}

impl<'a> Iterator for Translations<'a> {
    type Item = TranslationRecord<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.left == 0 {
            return None;
        }
        self.left -= 1;
        TranslationRecord::read(&mut self.reader, self.index, self.additional).ok()
    }
}

/// GP Translation Table Update (client → server 0x07).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TranslationTableUpdate<'a> {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// [`tt_action`].
    pub action: u8,
    /// The translations carry an additional information block.
    pub additional_info: bool,
    count: u8,
    translations: &'a [u8],
}

impl<'a> TranslationTableUpdate<'a> {
    /// Number of translations.
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Whether the update carries no translation.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// The translations.
    pub fn translations(&self) -> Translations<'a> {
        Translations {
            reader: Reader::new(self.translations),
            left: self.count,
            index: true,
            additional: self.additional_info,
        }
    }

    /// Strictly decode.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        if o & 0xFE00 != 0 {
            return Err(Error::Unsupported);
        }
        let gpd_id = read_gpd_id(&mut r, ApplicationId::from_bits(o as u8)?)?;
        let action = ((o >> 3) & 0x03) as u8;
        let count = ((o >> 5) & 0x07) as u8;
        let additional_info = o & (1 << 8) != 0;
        let translations = r.rest();
        let mut check = Reader::new(translations);
        for _ in 0..count {
            TranslationRecord::read(&mut check, true, additional_info)?;
        }
        check.finish()?;
        Ok(Self {
            gpd_id,
            action,
            additional_info,
            count,
            translations,
        })
    }

    /// Encode an update of `records` (at most
    /// [`MAX_TRANSLATIONS_PER_UPDATE`]; all with or all without an
    /// additional information block) into `out`.
    pub fn encode(
        gpd_id: &GpdId,
        action: u8,
        records: &[TranslationRecord<'_>],
        out: &mut [u8],
    ) -> Result<usize> {
        let additional = records.first().is_some_and(|t| t.additional_info.is_some());
        if records.len() > MAX_TRANSLATIONS_PER_UPDATE
            || action > tt_action::REMOVE
            || records
                .iter()
                .any(|t| t.additional_info.is_some() != additional)
        {
            return Err(Error::Unsupported);
        }
        let mut w = Writer::new(out);
        w.u16(
            gpd_id.application_id() as u16
                | u16::from(action) << 3
                | (records.len() as u16) << 5
                | u16::from(additional) << 8,
        )?;
        write_gpd_id(&mut w, gpd_id)?;
        for t in records {
            t.write(&mut w, true)?;
        }
        Ok(w.len())
    }
}

/// GP Translation Table Request (client → server 0x08).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TranslationTableRequest {
    /// First entry index.
    pub start_index: u8,
}

impl TranslationTableRequest {
    /// Encode into `out`.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.start_index)?;
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let start_index = r.u8()?;
        r.finish()?;
        Ok(Self { start_index })
    }
}

/// GP Translation Table Response (server → client 0x08): status, options
/// (ApplicationID, additional information block present), total entries,
/// start index, entry count, then the entries, each the GPD ID followed by
/// a [`TranslationRecord`] without the index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TranslationTableResponse<'a> {
    /// ZCL status (SUCCESS, NOT_FOUND, INSUFFICIENT_SPACE).
    pub status: u8,
    /// ApplicationID of every entry.
    pub application_id: ApplicationId,
    /// The entries carry an additional information block.
    pub additional_info: bool,
    /// Entries in the sink's table.
    pub total: u8,
    /// Index of the first entry.
    pub start_index: u8,
    count: u8,
    entries: &'a [u8],
}

impl<'a> TranslationTableResponse<'a> {
    /// Number of entries in this response.
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Whether the response carries no entry.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// The entries.
    pub fn entries(&self) -> impl Iterator<Item = (GpdId, TranslationRecord<'a>)> + 'a {
        let mut r = Reader::new(self.entries);
        let app = self.application_id;
        let additional = self.additional_info;
        (0..self.count).map_while(move |_| {
            let id = read_gpd_id(&mut r, app).ok()?;
            Some((id, TranslationRecord::read(&mut r, false, additional).ok()?))
        })
    }

    /// Strictly decode.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let status = r.u8()?;
        let options = r.u8()?;
        if options & 0xF0 != 0 {
            return Err(Error::Unsupported);
        }
        let application_id = ApplicationId::from_bits(options)?;
        let additional_info = options & 0x08 != 0;
        let total = r.u8()?;
        let start_index = r.u8()?;
        let count = r.u8()?;
        let entries = r.rest();
        let mut check = Reader::new(entries);
        for _ in 0..count {
            read_gpd_id(&mut check, application_id)?;
            TranslationRecord::read(&mut check, false, additional_info)?;
        }
        check.finish()?;
        Ok(Self {
            status,
            application_id,
            additional_info,
            total,
            start_index,
            count,
            entries,
        })
    }

    /// Encode the fixed header; entries are appended with
    /// [`encode_entry`](Self::encode_entry) and the count patched at
    /// offset 4.
    pub fn encode_header(
        out: &mut [u8],
        status: u8,
        application_id: ApplicationId,
        total: u8,
        start_index: u8,
        count: u8,
    ) -> Result<usize> {
        let mut w = Writer::new(out);
        w.bytes(&[status, application_id as u8, total, start_index, count])?;
        Ok(w.len())
    }

    /// Encode one entry (GPD ID and the translation) into `out`.
    pub fn encode_entry(
        gpd_id: &GpdId,
        record: &TranslationRecord<'_>,
        out: &mut [u8],
    ) -> Result<usize> {
        let mut w = Writer::new(out);
        write_gpd_id(&mut w, gpd_id)?;
        record.write(&mut w, false)?;
        Ok(w.len())
    }
}

/// GP Sink Table Response (server → client 0x0A) header and entries in the
/// Sink Table attribute format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SinkTableResponse<'a> {
    /// ZCL status (SUCCESS, NOT_FOUND).
    pub status: u8,
    /// Non-empty Sink Table entries of the sink.
    pub total: u8,
    /// Index of the first entry (0xFF for a request by GPD ID).
    pub start_index: u8,
    /// Entries in this response.
    pub count: u8,
    /// The encoded entries.
    pub entries: &'a [u8],
}

impl<'a> SinkTableResponse<'a> {
    /// Decode the header; [`entries`](Self::entries) decodes the entries.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        Ok(Self {
            status: r.u8()?,
            total: r.u8()?,
            start_index: r.u8()?,
            count: r.u8()?,
            entries: r.rest(),
        })
    }

    /// The entries; an entry that does not decode ends the iteration with
    /// an error.
    pub fn entries(&self) -> impl Iterator<Item = Result<crate::sink::SinkEntry>> + 'a {
        let mut rest = self.entries;
        let mut left = self.count;
        core::iter::from_fn(move || {
            if left == 0 {
                return None;
            }
            left -= 1;
            match crate::sink::SinkEntry::decode(rest) {
                Ok((entry, used)) => {
                    rest = &rest[used..];
                    Some(Ok(entry))
                }
                Err(e) => {
                    left = 0;
                    Some(Err(e))
                }
            }
        })
    }
}

/// GP Proxy Table Response (client → server 0x0B): the header and the
/// entries in the Proxy Table attribute format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProxyTableResponse<'a> {
    /// ZCL status (SUCCESS, NOT_FOUND).
    pub status: u8,
    /// Non-empty Proxy Table entries of the proxy.
    pub total: u8,
    /// Index of the first entry (0xFF for a request by GPD ID).
    pub start_index: u8,
    /// Entries in this response.
    pub count: u8,
    /// The encoded entries.
    pub entries: &'a [u8],
}

impl<'a> ProxyTableResponse<'a> {
    /// Decode the header; [`entries`](Self::entries) splits the entries.
    pub fn decode(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        Ok(Self {
            status: r.u8()?,
            total: r.u8()?,
            start_index: r.u8()?,
            count: r.u8()?,
            entries: r.rest(),
        })
    }

    /// The encoded bytes of each entry with the decoded entry; an entry
    /// that does not decode ends the iteration with an error.
    pub fn entries(
        &self,
    ) -> impl Iterator<Item = Result<(&'a [u8], crate::proxy::ProxyEntry)>> + 'a {
        let mut rest = self.entries;
        let mut left = self.count;
        core::iter::from_fn(move || {
            if left == 0 {
                return None;
            }
            left -= 1;
            match crate::proxy::ProxyEntry::decode(rest) {
                Ok((entry, used)) => {
                    let bytes = &rest[..used];
                    rest = &rest[used..];
                    Some(Ok((bytes, entry)))
                }
                Err(e) => {
                    left = 0;
                    Some(Err(e))
                }
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// gecko_sdk-shaped GP Pairing Configuration: Replace + send GP Pairing,
    /// SrcID 0x7A6B9879, pre-commissioned groupcast to 0x0B84 (derived
    /// alias), radius 0, level 3 individual key, counter 0x10, paired
    /// endpoint 1, application information with manufacturer and model.
    const PAIRING_CONFIG: [u8; 46] = [
        0x0A, // actions: replace | send GP Pairing
        0x10, 0x06, // options: SrcID, pre-commissioned groupcast, security, app info
        0x79, 0x98, 0x6B, 0x7A, // SrcID
        0x02, // DeviceID: on/off switch
        0x01, 0x84, 0x0B, 0xFF, 0xFF, // 1 group: 0x0B84, alias 0xFFFF
        0x00, // groupcast radius
        0x13, // security options: level 3, key type individual (0b100)
        0x10, 0x00, 0x00, 0x00, // frame counter
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
        0xFF, // key
        0x01, 0x01, // 1 paired endpoint: 1
        0x07, // app info: manufacturer, model, GPD commands
        0x6B, 0x12, // manufacturer
        0x34, 0x00, // model
        0x03, 0x20, 0x21, 0x22, // commands
    ];

    #[test]
    fn pairing_configuration_round_trips() {
        let pc = PairingConfiguration::decode(&PAIRING_CONFIG).unwrap();
        assert_eq!(pc.action(), pairing_action::REPLACE);
        assert!(pc.send_gp_pairing());
        assert_eq!(pc.gpd_id, GpdId::SrcId(0x7A6B_9879));
        assert_eq!(
            pc.communication_mode,
            CommunicationMode::CommissionedGroupcast
        );
        assert_eq!(
            pc.group_list().collect::<heapless::Vec<_, 2>>(),
            [(0x0B84, 0xFFFF)]
        );
        let s = pc.security.unwrap();
        assert_eq!(
            (s.level, s.key_type),
            (SecurityLevel::Encrypted, KeyType::IndividualKey)
        );
        assert_eq!(pc.frame_counter, Some(0x10));
        assert_eq!(pc.paired_endpoints, &[0x01]);
        let info = pc.application_information.unwrap();
        assert_eq!(
            (info.manufacturer_id, info.model_id),
            (Some(0x126B), Some(0x0034))
        );
        assert_eq!(info.gpd_commands, &[0x20, 0x21, 0x22]);
        let mut out = [0u8; 64];
        let n = pc.encode(&mut out).unwrap();
        assert_eq!(&out[..n], &PAIRING_CONFIG);
    }

    #[test]
    fn pairing_configuration_minimal_and_special_endpoint_counts() {
        // Remove GPD, SrcID, derived groupcast, no security, radius, 0xFE.
        let raw = [0x04, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0xFE];
        let pc = PairingConfiguration::decode(&raw).unwrap();
        assert_eq!(pc.action(), pairing_action::REMOVE_GPD);
        assert!(!pc.send_gp_pairing());
        assert_eq!(pc.communication_mode, CommunicationMode::DerivedGroupcast);
        assert_eq!(
            pc.number_of_paired_endpoints,
            paired_endpoints::SINK_DERIVES
        );
        assert!(pc.paired_endpoints.is_empty());
        let mut out = [0u8; 16];
        let n = pc.encode(&mut out).unwrap();
        assert_eq!(&out[..n], &raw);
        // Truncated, trailing, IEEE application id.
        assert_eq!(
            PairingConfiguration::decode(&raw[..9]),
            Err(Error::Truncated)
        );
        let mut long = [0u8; 11];
        long[..10].copy_from_slice(&raw);
        assert_eq!(
            PairingConfiguration::decode(&long),
            Err(Error::TrailingBytes)
        );
        let ieee = [
            0x01, 0x02, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 0x01, 0x02, 0x00, 0x00,
        ];
        let pc = PairingConfiguration::decode(&ieee).unwrap();
        assert_eq!(
            pc.gpd_id,
            GpdId::Ieee {
                ieee: [1, 2, 3, 4, 5, 6, 7, 8],
                endpoint: 1
            }
        );
        assert_eq!(pc.communication_mode, CommunicationMode::FullUnicast);
    }

    #[test]
    fn pairing_configuration_switch_info_and_application_description() {
        // Extend, sequence number capability (counter without security),
        // app info: cluster list (1 server 0x0402, 1 client 0x0006), switch
        // information (2 bytes), application description (1 of 1 report).
        let raw = [
            0x01, 0x28, 0x04, 0x11, 0x22, 0x33, 0x44, 0x30, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00,
            0x38, 0x11, 0x02, 0x04, 0x06, 0x00, 0x02, 0x41, 0x0F, 0x01, 0x01, 0xAA, 0xBB,
        ];
        let pc = PairingConfiguration::decode(&raw).unwrap();
        assert_eq!(pc.frame_counter, Some(5));
        assert!(pc.mac_seq_cap && pc.security.is_none());
        let info = pc.application_information.unwrap();
        assert_eq!(info.server_clusters, &[0x02, 0x04]);
        assert_eq!(info.client_clusters, &[0x06, 0x00]);
        assert_eq!(info.switch_information, &[0x41, 0x0F]);
        assert_eq!(
            info.application_description,
            Some((1, 1, &[0xAA, 0xBB][..]))
        );
        let mut out = [0u8; 64];
        let n = pc.encode(&mut out).unwrap();
        assert_eq!(&out[..n], &raw);
    }

    /// GP Translation Table Update, Add, SrcID 0x7A6B9879, two translations:
    /// Toggle → endpoint 2 Level Control Move to Level (with On/Off) fixed
    /// payload, Off → endpoint 1 On/Off derived; index 0xFF as GP 1.1 sends.
    const TT_UPDATE: [u8; 33] = [
        0x40, 0x00, // options: SrcID, Add, 2 translations
        0x79, 0x98, 0x6B, 0x7A, // SrcID
        0xFF, 0x22, 0x02, 0x04, 0x01, 0x08, 0x00, 0x04, 0x03, 0x80, 0x0A, 0x00, // T1
        0xFF, 0x20, 0x01, 0x04, 0x01, 0x06, 0x00, 0x00, 0xFE, // T2
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // padding replaced below
    ];

    #[test]
    fn translation_table_update_round_trips() {
        let raw = &TT_UPDATE[..27];
        let u = TranslationTableUpdate::decode(raw).unwrap();
        assert_eq!(u.gpd_id, GpdId::SrcId(0x7A6B_9879));
        assert_eq!(
            (u.action, u.len(), u.additional_info),
            (tt_action::ADD, 2, false)
        );
        let t: heapless::Vec<_, 2> = u.translations().collect();
        assert_eq!(t[0].index, 0xFF);
        assert_eq!(t[0].key(), (0x22, 0x02, 0x0104, 0x0008));
        assert_eq!(t[0].zcl_command, 0x04);
        assert_eq!(t[0].payload, TtPayload::Fixed(&[0x80, 0x0A, 0x00]));
        assert_eq!(t[1].payload, TtPayload::Derived);
        let mut out = [0u8; 64];
        let n = TranslationTableUpdate::encode(&u.gpd_id, u.action, &t, &mut out).unwrap();
        assert_eq!(&out[..n], raw);
        // Count larger than the records, trailing bytes, reserved bits.
        assert_eq!(
            TranslationTableUpdate::decode(&raw[..26]),
            Err(Error::Truncated)
        );
        assert_eq!(
            TranslationTableUpdate::decode(&TT_UPDATE[..28]),
            Err(Error::TrailingBytes)
        );
        let mut reserved = [0u8; 27];
        reserved.copy_from_slice(raw);
        reserved[1] = 0x02;
        assert_eq!(
            TranslationTableUpdate::decode(&reserved),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn translation_table_update_with_additional_information_block() {
        let raw = [
            0x28, 0x01, // SrcID, Replace, 1 translation, additional info
            0x01, 0x00, 0x00, 0x00, 0xFF, 0x69, 0x01, 0x04, 0x01, 0x06, 0x00, 0x02, 0x00, 0x03,
            0x01, 0x01, 0x01,
        ];
        let u = TranslationTableUpdate::decode(&raw).unwrap();
        assert!(u.additional_info);
        let t = u.translations().next().unwrap();
        assert_eq!(t.additional_info, Some(&[0x01, 0x01, 0x01][..]));
        assert_eq!(t.payload, TtPayload::Fixed(&[]));
    }

    #[test]
    fn translation_table_response_round_trips() {
        let mut out = [0u8; 64];
        let rec = TranslationRecord {
            index: 0xFF,
            gpd_command: 0x22,
            endpoint: 0x02,
            profile: 0x0104,
            cluster: 0x0006,
            zcl_command: 0x02,
            payload: TtPayload::CopyGpd,
            additional_info: None,
        };
        let id = GpdId::SrcId(0x0102_0304);
        let mut n =
            TranslationTableResponse::encode_header(&mut out, 0, ApplicationId::SrcId, 3, 1, 1)
                .unwrap();
        n += TranslationTableResponse::encode_entry(&id, &rec, &mut out[n..]).unwrap();
        assert_eq!(
            &out[..n],
            &[
                0x00, 0x00, 0x03, 0x01, 0x01, 0x04, 0x03, 0x02, 0x01, 0x22, 0x02, 0x04, 0x01, 0x06,
                0x00, 0x02, 0xFF
            ]
        );
        let r = TranslationTableResponse::decode(&out[..n]).unwrap();
        assert_eq!((r.status, r.total, r.start_index, r.len()), (0, 3, 1, 1));
        let (gpd, t) = r.entries().next().unwrap();
        assert_eq!(gpd, id);
        assert_eq!(t, rec);
        // NOT_FOUND without entries (gecko_sdk layout).
        let nf = TranslationTableResponse::decode(&[0x8B, 0x00, 0x00, 0x00, 0x00]).unwrap();
        assert_eq!((nf.status, nf.len()), (0x8B, 0));
    }

    #[test]
    fn sink_and_proxy_table_responses_decode() {
        let mut e = crate::sink::SinkEntry::new(GpdId::SrcId(0x1122_3344));
        e.device_id = 0x02;
        let mut out = [0u8; 64];
        out[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x01]);
        let n = e.encode(&mut out[4..]).unwrap();
        let r = SinkTableResponse::decode(&out[..4 + n]).unwrap();
        assert_eq!((r.status, r.total, r.start_index, r.count), (0, 1, 0, 1));
        let got: heapless::Vec<_, 2> = r.entries().collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].as_ref().unwrap().gpd_id, e.gpd_id);
        // A count beyond the entries ends with an error.
        out[3] = 2;
        let r = SinkTableResponse::decode(&out[..4 + n]).unwrap();
        assert!(r.entries().nth(1).unwrap().is_err());

        let p = crate::proxy::ProxyEntry::new(GpdId::SrcId(0x5566_7788));
        let mut out = [0u8; 80];
        out[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x01]);
        let n = p.encode(&mut out[4..]).unwrap();
        let r = ProxyTableResponse::decode(&out[..4 + n]).unwrap();
        let (bytes, entry) = r.entries().next().unwrap().unwrap();
        assert_eq!(bytes.len(), n);
        assert_eq!(entry.gpd_id, p.gpd_id);
    }
}
