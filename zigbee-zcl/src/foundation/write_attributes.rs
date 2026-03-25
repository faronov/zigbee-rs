//! Write Attributes command (0x02) and Write Attributes Response (0x03).

use crate::attribute::AttributeStore;
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ZclStatus};

/// Maximum number of attributes in a single write request / response.
pub const MAX_WRITE_ATTRS: usize = 16;

/// A single write record: attribute to write, its data type, and the value.
#[derive(Debug, Clone)]
pub struct WriteAttributeRecord {
    pub id: AttributeId,
    pub data_type: ZclDataType,
    pub value: ZclValue,
}

/// Write Attributes request.
#[derive(Debug, Clone)]
pub struct WriteAttributesRequest {
    pub records: heapless::Vec<WriteAttributeRecord, MAX_WRITE_ATTRS>,
}

/// A single status record in the Write Attributes Response.
#[derive(Debug, Clone)]
pub struct WriteAttributeStatusRecord {
    pub status: ZclStatus,
    pub id: AttributeId,
}

/// Write Attributes Response.
#[derive(Debug, Clone)]
pub struct WriteAttributesResponse {
    pub records: heapless::Vec<WriteAttributeStatusRecord, MAX_WRITE_ATTRS>,
}

impl WriteAttributesRequest {
    /// Parse from ZCL payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        let mut records = heapless::Vec::new();
        let mut i = 0;
        while i + 3 < data.len() {
            let id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
            i += 2;
            let dt = ZclDataType::from_u8(data[i])?;
            i += 1;
            let (value, consumed) = ZclValue::deserialize(dt, &data[i..])?;
            i += consumed;
            records
                .push(WriteAttributeRecord {
                    id,
                    data_type: dt,
                    value,
                })
                .ok()?;
        }
        Some(Self { records })
    }

    /// Serialize to ZCL payload bytes.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rec in &self.records {
            let b = rec.id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
            buf[pos] = rec.data_type as u8;
            pos += 1;
            pos += rec.value.serialize(&mut buf[pos..]);
        }
        pos
    }
}

impl WriteAttributesResponse {
    /// Serialize the response to ZCL payload bytes.
    ///
    /// Per the spec, if all writes succeed a single Success status with no
    /// attribute ID is returned.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let all_success = self.records.iter().all(|r| r.status == ZclStatus::Success);
        if all_success {
            buf[0] = ZclStatus::Success as u8;
            return 1;
        }
        let mut pos = 0;
        for rec in &self.records {
            if rec.status != ZclStatus::Success {
                buf[pos] = rec.status as u8;
                pos += 1;
                let b = rec.id.0.to_le_bytes();
                buf[pos] = b[0];
                buf[pos + 1] = b[1];
                pos += 2;
            }
        }
        pos
    }
}

/// Process a Write Attributes request against an attribute store.
pub fn process_write<const N: usize>(
    store: &mut AttributeStore<N>,
    request: &WriteAttributesRequest,
) -> WriteAttributesResponse {
    let mut records = heapless::Vec::new();
    for rec in &request.records {
        let status = match store.set(rec.id, rec.value.clone()) {
            Ok(()) => ZclStatus::Success,
            Err(e) => e,
        };
        let _ = records.push(WriteAttributeStatusRecord { status, id: rec.id });
    }
    WriteAttributesResponse { records }
}

/// Process a Write Attributes Undivided request (command 0x03).
///
/// All attributes are validated first; if any single write would fail,
/// none are applied ("all or nothing" semantics).
pub fn process_write_undivided<const N: usize>(
    store: &mut AttributeStore<N>,
    request: &WriteAttributesRequest,
) -> WriteAttributesResponse {
    // First pass: validate all writes
    let mut records = heapless::Vec::new();
    let mut all_ok = true;
    for rec in &request.records {
        let status = match store.find(rec.id) {
            Some(def) => {
                if !def.access.is_writable() {
                    ZclStatus::ReadOnly
                } else if rec.data_type != def.data_type {
                    ZclStatus::InvalidDataType
                } else {
                    ZclStatus::Success
                }
            }
            None => ZclStatus::UnsupportedAttribute,
        };
        if status != ZclStatus::Success {
            all_ok = false;
        }
        let _ = records.push(WriteAttributeStatusRecord { status, id: rec.id });
    }

    // Second pass: apply if all valid
    if all_ok {
        for rec in &request.records {
            let _ = store.set(rec.id, rec.value.clone());
        }
        // All success → single success status
        let mut ok_records = heapless::Vec::new();
        for rec in &request.records {
            let _ = ok_records.push(WriteAttributeStatusRecord {
                status: ZclStatus::Success,
                id: rec.id,
            });
        }
        return WriteAttributesResponse {
            records: ok_records,
        };
    }

    WriteAttributesResponse { records }
}

/// Process a Write Attributes request using a type-erased attribute store.
pub fn process_write_dyn(
    store: &mut dyn crate::clusters::AttributeStoreMutAccess,
    request: &WriteAttributesRequest,
) -> WriteAttributesResponse {
    let mut records = heapless::Vec::new();
    for rec in &request.records {
        let status = match store.set(rec.id, rec.value.clone()) {
            Ok(()) => ZclStatus::Success,
            Err(e) => e,
        };
        let _ = records.push(WriteAttributeStatusRecord { status, id: rec.id });
    }
    WriteAttributesResponse { records }
}

/// Process a Write Attributes No Response request (command 0x05) using type-erased store.
pub fn process_write_no_response_dyn(
    store: &mut dyn crate::clusters::AttributeStoreMutAccess,
    request: &WriteAttributesRequest,
) {
    for rec in &request.records {
        let _ = store.set(rec.id, rec.value.clone());
    }
}
