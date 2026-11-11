//! GPD commissioning and sensor command payloads (GP spec A.4.2.1, A.4.2.3).
//!
//! Layouts match zigpy's `zigpy.zgp.commands` schemas.

use zigbee_crypto::AesKey;

use crate::codec::{Reader, Writer};
use crate::types::{KeyType, SecurityLevel};
use crate::{Error, Result};

/// Maximum number of GPD command IDs carried in a Commissioning command.
pub const MAX_COMMISSIONING_COMMANDS: usize = 16;
/// Maximum number of server or client clusters (4-bit count).
pub const MAX_COMMISSIONING_CLUSTERS: usize = 15;

/// Security settings advertised by the GPD in a Commissioning command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissioningSecurity {
    /// Security level the GPD will use.
    pub level: SecurityLevel,
    /// Key type of the advertised key.
    pub key_type: KeyType,
    /// The GPD key: `(key, Some(mic))` when TC-LK protected, `(key, None)`
    /// when sent in the clear.
    pub key: Option<(AesKey, Option<u32>)>,
    /// GPD outgoing security frame counter.
    pub outgoing_counter: Option<u32>,
}

/// Generic switch information (Figure 114/115).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SwitchInformation {
    /// Number of contacts (0-15).
    pub num_contacts: u8,
    /// Switch type (0 button, 1 rocker).
    pub switch_type: u8,
    /// Current contact status bitmap.
    pub contact_status: u8,
}

/// Application information of a Commissioning command.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ApplicationInfo {
    /// Manufacturer ID.
    pub manufacturer_id: Option<u16>,
    /// Model ID.
    pub model_id: Option<u16>,
    /// Supported GPD command IDs.
    pub gpd_commands: Option<heapless::Vec<u8, MAX_COMMISSIONING_COMMANDS>>,
    /// Server and client cluster lists.
    pub clusters: Option<(
        heapless::Vec<u16, MAX_COMMISSIONING_CLUSTERS>,
        heapless::Vec<u16, MAX_COMMISSIONING_CLUSTERS>,
    )>,
    /// Switch information.
    pub switch_info: Option<SwitchInformation>,
    /// An Application Description (0xE4) command follows.
    pub app_description_follows: bool,
}

/// GPD Commissioning command (0xE0) payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommissioningPayload {
    /// GPD DeviceID.
    pub device_id: u8,
    /// GPD MAC sequence number is incremental.
    pub mac_seq_num_capability: bool,
    /// GPD can receive (bidirectional).
    pub rx_on_capability: bool,
    /// GPD requests a PAN ID.
    pub pan_id_request: bool,
    /// GPD requests a GP security key.
    pub security_key_request: bool,
    /// GPD is not moving.
    pub fixed_location: bool,
    /// Security information (Extended Options).
    pub security: Option<CommissioningSecurity>,
    /// Application information.
    pub app_info: Option<ApplicationInfo>,
}

impl CommissioningPayload {
    /// Encode (without the 0xE0 command ID) into `out`.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.device_id)?;
        let opts = (self.mac_seq_num_capability as u8)
            | (self.rx_on_capability as u8) << 1
            | (self.app_info.is_some() as u8) << 2
            | (self.pan_id_request as u8) << 4
            | (self.security_key_request as u8) << 5
            | (self.fixed_location as u8) << 6
            | (self.security.is_some() as u8) << 7;
        w.u8(opts)?;
        if let Some(sec) = &self.security {
            if sec.level == SecurityLevel::Reserved {
                return Err(Error::Unsupported);
            }
            let (key_present, key_encrypted) = match &sec.key {
                None => (false, false),
                Some((_, mic)) => (true, mic.is_some()),
            };
            w.u8(sec.level as u8
                | (sec.key_type as u8) << 2
                | (key_present as u8) << 5
                | (key_encrypted as u8) << 6
                | (sec.outgoing_counter.is_some() as u8) << 7)?;
            if let Some((key, mic)) = &sec.key {
                w.bytes(key)?;
                if let Some(mic) = mic {
                    w.u32(*mic)?;
                }
            }
            if let Some(c) = sec.outgoing_counter {
                w.u32(c)?;
            }
        }
        if let Some(ai) = &self.app_info {
            w.u8((ai.manufacturer_id.is_some() as u8)
                | (ai.model_id.is_some() as u8) << 1
                | (ai.gpd_commands.is_some() as u8) << 2
                | (ai.clusters.is_some() as u8) << 3
                | (ai.switch_info.is_some() as u8) << 4
                | (ai.app_description_follows as u8) << 5)?;
            if let Some(m) = ai.manufacturer_id {
                w.u16(m)?;
            }
            if let Some(m) = ai.model_id {
                w.u16(m)?;
            }
            if let Some(cmds) = &ai.gpd_commands {
                w.u8(cmds.len() as u8)?;
                w.bytes(cmds)?;
            }
            if let Some((server, client)) = &ai.clusters {
                w.u8(server.len() as u8 | (client.len() as u8) << 4)?;
                for c in server.iter().chain(client.iter()) {
                    w.u16(*c)?;
                }
            }
            if let Some(si) = &ai.switch_info {
                if si.num_contacts > 0x0F || si.switch_type > 0x03 {
                    return Err(Error::Unsupported);
                }
                w.u8(2)?;
                w.u8(si.num_contacts | si.switch_type << 4)?;
                w.u8(si.contact_status)?;
            }
        }
        Ok(w.len())
    }

    /// Strictly decode a Commissioning command payload (without 0xE0).
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let device_id = r.u8()?;
        let opts = r.u8()?;
        let security = if opts & 0x80 != 0 {
            let ext = r.u8()?;
            let level = SecurityLevel::from_bits(ext);
            if level == SecurityLevel::Reserved {
                return Err(Error::Unsupported);
            }
            let key_type = KeyType::from_bits(ext >> 2)?;
            let key = if ext & 0x20 != 0 {
                let k = r.array::<16>()?;
                let mic = if ext & 0x40 != 0 {
                    Some(r.u32()?)
                } else {
                    None
                };
                Some((k, mic))
            } else {
                None
            };
            let outgoing_counter = if ext & 0x80 != 0 {
                Some(r.u32()?)
            } else {
                None
            };
            Some(CommissioningSecurity {
                level,
                key_type,
                key,
                outgoing_counter,
            })
        } else {
            None
        };
        let app_info = if opts & 0x04 != 0 {
            let ai = r.u8()?;
            let mut info = ApplicationInfo {
                app_description_follows: ai & 0x20 != 0,
                ..ApplicationInfo::default()
            };
            if ai & 0x01 != 0 {
                info.manufacturer_id = Some(r.u16()?);
            }
            if ai & 0x02 != 0 {
                info.model_id = Some(r.u16()?);
            }
            if ai & 0x04 != 0 {
                let n = r.u8()? as usize;
                let cmds = heapless::Vec::from_slice(r.bytes(n)?).map_err(|_| Error::BufferFull)?;
                info.gpd_commands = Some(cmds);
            }
            if ai & 0x08 != 0 {
                let counts = r.u8()?;
                let mut server = heapless::Vec::new();
                let mut client = heapless::Vec::new();
                for _ in 0..(counts & 0x0F) {
                    server.push(r.u16()?).map_err(|_| Error::BufferFull)?;
                }
                for _ in 0..(counts >> 4) {
                    client.push(r.u16()?).map_err(|_| Error::BufferFull)?;
                }
                info.clusters = Some((server, client));
            }
            if ai & 0x10 != 0 {
                let len = r.u8()?;
                if len < 2 {
                    return Err(Error::Truncated);
                }
                let cfg = r.u8()?;
                let contact_status = r.u8()?;
                // Tolerate future extensions of the switch info block.
                r.bytes(len as usize - 2)?;
                info.switch_info = Some(SwitchInformation {
                    num_contacts: cfg & 0x0F,
                    switch_type: (cfg >> 4) & 0x03,
                    contact_status,
                });
            }
            Some(info)
        } else {
            None
        };
        r.finish()?;
        Ok(Self {
            device_id,
            mac_seq_num_capability: opts & 0x01 != 0,
            rx_on_capability: opts & 0x02 != 0,
            pan_id_request: opts & 0x10 != 0,
            security_key_request: opts & 0x20 != 0,
            fixed_location: opts & 0x40 != 0,
            security,
            app_info,
        })
    }
}

/// GPD Commissioning Reply command (0xF0) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissioningReply {
    /// PAN ID.
    pub pan_id: Option<u16>,
    /// Security level to use.
    pub security_level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// Key: `(key, Some((mic, frame_counter)))` when TC-LK protected.
    pub key: Option<(AesKey, Option<(u32, u32)>)>,
}

impl CommissioningReply {
    /// Encode (without the 0xF0 command ID).
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        let (kp, ke) = match &self.key {
            None => (false, false),
            Some((_, e)) => (true, e.is_some()),
        };
        w.u8((self.pan_id.is_some() as u8)
            | (kp as u8) << 1
            | (ke as u8) << 2
            | (self.security_level as u8) << 3
            | (self.key_type as u8) << 5)?;
        if let Some(p) = self.pan_id {
            w.u16(p)?;
        }
        if let Some((k, enc)) = &self.key {
            w.bytes(k)?;
            if let Some((mic, fc)) = enc {
                w.u32(*mic)?;
                if self.security_level.is_protected() {
                    w.u32(*fc)?;
                }
            }
        }
        Ok(w.len())
    }

    /// Strictly decode (without the 0xF0 command ID).
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let security_level = SecurityLevel::from_bits(o >> 3);
        let key_type = KeyType::from_bits(o >> 5)?;
        let pan_id = if o & 0x01 != 0 { Some(r.u16()?) } else { None };
        let key = if o & 0x02 != 0 {
            let k = r.array::<16>()?;
            let enc = if o & 0x04 != 0 {
                let mic = r.u32()?;
                let fc = if security_level.is_protected() {
                    r.u32()?
                } else {
                    0
                };
                Some((mic, fc))
            } else {
                None
            };
            Some((k, enc))
        } else {
            None
        };
        r.finish()?;
        Ok(Self {
            pan_id,
            security_level,
            key_type,
            key,
        })
    }
}

/// GPD Channel Request (0xE3) payload byte.
pub fn channel_request(next_channel: u8, second_next_channel: u8) -> Result<u8> {
    if !(11..=26).contains(&next_channel) || !(11..=26).contains(&second_next_channel) {
        return Err(Error::Unsupported);
    }
    Ok((next_channel - 11) | (second_next_channel - 11) << 4)
}

/// Decode a GPD Channel Configuration (0xF3) payload byte into the
/// IEEE 802.15.4 operational channel and the Basic flag.
pub fn channel_configuration(byte: u8) -> (u8, bool) {
    ((byte & 0x0F) + 11, byte & 0x10 != 0)
}

/// Encode a GPD Channel Configuration (0xF3) payload byte.
pub fn encode_channel_configuration(channel: u8, basic: bool) -> Result<u8> {
    if !(11..=26).contains(&channel) {
        return Err(Error::Unsupported);
    }
    Ok((channel - 11) | (basic as u8) << 4)
}

/// Builder for GPD Attribute Reporting (0xA0) / Manufacturer-specific
/// Attribute Reporting (0xA1) payloads.
pub struct AttributeReport<'a> {
    w: Writer<'a>,
}

impl<'a> AttributeReport<'a> {
    /// Start a 0xA0 payload for `cluster_id`.
    pub fn new(out: &'a mut [u8], cluster_id: u16) -> Result<Self> {
        let mut w = Writer::new(out);
        w.u16(cluster_id)?;
        Ok(Self { w })
    }

    /// Start a 0xA1 payload.
    pub fn new_manufacturer(out: &'a mut [u8], manufacturer: u16, cluster_id: u16) -> Result<Self> {
        let mut w = Writer::new(out);
        w.u16(manufacturer)?;
        w.u16(cluster_id)?;
        Ok(Self { w })
    }

    /// Append a raw attribute record (attribute ID, ZCL data type, value).
    pub fn record(mut self, attr_id: u16, data_type: u8, value: &[u8]) -> Result<Self> {
        self.w.u16(attr_id)?;
        self.w.u8(data_type)?;
        self.w.bytes(value)?;
        Ok(self)
    }

    /// Append an `int16` attribute (ZCL type 0x29).
    pub fn int16(self, attr_id: u16, value: i16) -> Result<Self> {
        self.record(attr_id, 0x29, &value.to_le_bytes())
    }

    /// Append a `uint16` attribute (ZCL type 0x21).
    pub fn uint16(self, attr_id: u16, value: u16) -> Result<Self> {
        self.record(attr_id, 0x21, &value.to_le_bytes())
    }

    /// Append a `uint8` attribute (ZCL type 0x20).
    pub fn uint8(self, attr_id: u16, value: u8) -> Result<Self> {
        self.record(attr_id, 0x20, &[value])
    }

    /// Finish, returning the payload length.
    pub fn finish(self) -> usize {
        self.w.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commissioning_round_trip() {
        let mut cmds = heapless::Vec::new();
        cmds.extend_from_slice(&[0xA0, 0xA1]).unwrap();
        let p = CommissioningPayload {
            device_id: 0xFE,
            mac_seq_num_capability: true,
            rx_on_capability: false,
            pan_id_request: false,
            security_key_request: false,
            fixed_location: true,
            security: Some(CommissioningSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: Some(([0x11; 16], Some(0xDEAD_BEEF))),
                outgoing_counter: Some(5),
            }),
            app_info: Some(ApplicationInfo {
                manufacturer_id: Some(0x6D6B),
                model_id: Some(0xF300),
                gpd_commands: Some(cmds),
                ..ApplicationInfo::default()
            }),
        };
        let mut buf = [0u8; 64];
        let n = p.encode(&mut buf).unwrap();
        assert_eq!(&buf[..3], &[0xFE, 0xC5, 0xF3]);
        assert_eq!(CommissioningPayload::decode(&buf[..n]).unwrap(), p);
    }

    #[test]
    fn reply_round_trip() {
        let r = CommissioningReply {
            pan_id: None,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: Some(([7; 16], Some((1, 2)))),
        };
        let mut buf = [0u8; 32];
        let n = r.encode(&mut buf).unwrap();
        assert_eq!(n, 1 + 16 + 4 + 4);
        assert_eq!(CommissioningReply::decode(&buf[..n]).unwrap(), r);
    }

    #[test]
    fn attribute_report_layout() {
        let mut buf = [0u8; 16];
        let n = AttributeReport::new(&mut buf, 0x0402)
            .unwrap()
            .int16(0x0000, 2150)
            .unwrap()
            .finish();
        assert_eq!(&buf[..n], &[0x02, 0x04, 0x00, 0x00, 0x29, 0x66, 0x08]);
    }

    #[test]
    fn channel_bytes() {
        assert_eq!(channel_request(15, 20).unwrap(), 0x94);
        assert_eq!(channel_configuration(0x14), (15, true));
        assert_eq!(encode_channel_configuration(26, false).unwrap(), 0x0F);
    }
}
