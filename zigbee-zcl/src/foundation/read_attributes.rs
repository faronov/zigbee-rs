//! Read Attributes command (0x00) and Read Attributes Response (0x01).

use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ZclStatus};

/// Maximum number of attributes in a single read request / response.
pub const MAX_READ_ATTRS: usize = 16;

/// Read Attributes request — a list of attribute IDs to read.
#[derive(Debug, Clone)]
pub struct ReadAttributesRequest {
    pub attributes: heapless::Vec<AttributeId, MAX_READ_ATTRS>,
}

/// A single record in the Read Attributes Response.
#[derive(Debug, Clone)]
pub struct ReadAttributeRecord {
    pub id: AttributeId,
    pub status: ZclStatus,
    /// Present only when `status == Success`.
    pub data_type: ZclDataType,
    pub value: Option<ZclValue>,
}

/// Read Attributes Response.
#[derive(Debug, Clone)]
pub struct ReadAttributesResponse {
    pub records: heapless::Vec<ReadAttributeRecord, MAX_READ_ATTRS>,
}

impl ReadAttributesRequest {
    /// Parse from ZCL payload bytes (list of little-endian u16 attribute IDs).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if !data.len().is_multiple_of(2) {
            return None;
        }
        let mut attributes = heapless::Vec::new();
        let mut i = 0;
        while i + 1 < data.len() {
            let id = u16::from_le_bytes([data[i], data[i + 1]]);
            attributes.push(AttributeId(id)).ok()?;
            i += 2;
        }
        Some(Self { attributes })
    }

    /// Serialize to ZCL payload bytes. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for attr in &self.attributes {
            if pos + 2 > buf.len() {
                break;
            }
            let b = attr.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
        }
        pos
    }
}

impl ReadAttributesResponse {
    /// Serialize the response to ZCL payload bytes. Returns bytes written.
    /// Stops serializing records if the buffer would overflow.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rec in &self.records {
            // Minimum record: 2 (attr id) + 1 (status) = 3 bytes
            if pos + 3 > buf.len() {
                break;
            }
            // Attribute ID
            let b = rec.id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
            // Status
            buf[pos] = rec.status as u8;
            pos += 1;
            // On success: data type + value
            if rec.status == ZclStatus::Success
                && let Some(ref val) = rec.value
            {
                // Need at least 1 byte for data type
                if pos + 1 > buf.len() {
                    break;
                }
                buf[pos] = rec.data_type as u8;
                pos += 1;
                let remaining = &mut buf[pos..];
                if remaining.is_empty() {
                    break;
                }
                pos += val.serialize(remaining);
            }
        }
        pos
    }

    /// Parse from ZCL payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        let mut records = heapless::Vec::new();
        let mut i = 0;
        while i + 2 < data.len() {
            let id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
            i += 2;
            if i >= data.len() {
                break;
            }
            let status = ZclStatus::from_u8(data[i]);
            i += 1;
            let (data_type, value) = if status == ZclStatus::Success && i < data.len() {
                let dt = ZclDataType::from_u8(data[i])?;
                i += 1;
                let (val, consumed) = ZclValue::deserialize(dt, &data[i..])?;
                i += consumed;
                (dt, Some(val))
            } else {
                (ZclDataType::NoData, None)
            };
            records
                .push(ReadAttributeRecord {
                    id,
                    status,
                    data_type,
                    value,
                })
                .ok()?;
        }
        Some(Self { records })
    }
}

/// Process a Read Attributes request using a type-erased attribute store.
pub fn process_read_dyn(
    store: &dyn crate::clusters::AttributeStoreAccess,
    request: &ReadAttributesRequest,
) -> ReadAttributesResponse {
    let mut records = heapless::Vec::new();
    for &attr_id in &request.attributes {
        let rec = match store.find(attr_id) {
            Some(def) => {
                if !def.access.is_readable() {
                    ReadAttributeRecord {
                        id: attr_id,
                        status: ZclStatus::WriteOnly,
                        data_type: ZclDataType::NoData,
                        value: None,
                    }
                } else if let Some(val) = store.get(attr_id) {
                    ReadAttributeRecord {
                        id: attr_id,
                        status: ZclStatus::Success,
                        data_type: def.data_type,
                        value: Some(val.clone()),
                    }
                } else {
                    ReadAttributeRecord {
                        id: attr_id,
                        status: ZclStatus::Failure,
                        data_type: ZclDataType::NoData,
                        value: None,
                    }
                }
            }
            None => ReadAttributeRecord {
                id: attr_id,
                status: ZclStatus::UnsupportedAttribute,
                data_type: ZclDataType::NoData,
                value: None,
            },
        };
        let _ = records.push(rec);
    }
    ReadAttributesResponse { records }
}
