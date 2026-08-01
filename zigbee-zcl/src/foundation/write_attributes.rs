//! Write Attributes command (0x02) and Write Attributes Response (0x03).

use crate::attribute::AttributeStore;
use crate::data_types::{ZclDataType, ZclValue, data_type_size, is_data_type_enabled};
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
#[derive(Debug, Clone, Copy)]
pub struct WriteAttributeStatusRecord {
    pub status: ZclStatus,
    pub id: AttributeId,
}

/// Write Attributes Response.
#[derive(Debug, Clone)]
pub struct WriteAttributesResponse {
    pub records: heapless::Vec<WriteAttributeStatusRecord, MAX_WRITE_ATTRS>,
}

/// A per-attribute Write failure discovered while parsing (a disabled /
/// unsupported data type), tagged with the record's zero-based position in the
/// request.
///
/// The ordinal lets [`WriteAttributesParseOutcome::merge_in_request_order`]
/// splice these failures back among the applied records' statuses in exact wire
/// order, even when attribute IDs repeat — ordering never depends on matching
/// IDs.
#[derive(Debug, Clone, Copy)]
pub struct WriteAttributeParseFailure {
    /// Zero-based index of this record within the original request.
    pub ordinal: u8,
    /// Status to report for this record.
    pub record: WriteAttributeStatusRecord,
}

/// Decoded records plus attribute-specific failures that do not invalidate
/// other writes in the same command.
#[derive(Debug, Clone)]
pub struct WriteAttributesParseOutcome {
    pub request: WriteAttributesRequest,
    /// Disabled-data-type failures, each carrying its original request ordinal
    /// so the response can be reassembled in wire order.
    pub invalid_data_types: heapless::Vec<WriteAttributeParseFailure, MAX_WRITE_ATTRS>,
}

impl WriteAttributesParseOutcome {
    /// Merge the statuses produced for the *supported* records (`applied`, in
    /// the order of [`Self::request`]`.records`) with the disabled-data-type
    /// failures captured during parsing, restoring the exact order of the
    /// original request.
    ///
    /// The merge is purely positional: every failure carries the ordinal of the
    /// record it replaced, and the applied statuses fill the remaining ordinals
    /// in order. Duplicate attribute IDs therefore stay unambiguous.
    ///
    /// `applied` is expected to hold exactly one status per supported record
    /// (as produced by [`process_write_dyn`] / [`validate_write_undivided_dyn`]).
    #[inline(never)]
    pub fn merge_in_request_order(
        &self,
        applied: &WriteAttributesResponse,
    ) -> WriteAttributesResponse {
        let mut records = heapless::Vec::new();
        let total = self.request.records.len() + self.invalid_data_types.len();
        let mut fi = 0;
        let mut ai = 0;
        for ordinal in 0..total {
            if fi < self.invalid_data_types.len()
                && self.invalid_data_types[fi].ordinal as usize == ordinal
            {
                let _ = records.push(self.invalid_data_types[fi].record);
                fi += 1;
            } else if ai < applied.records.len() {
                let _ = records.push(applied.records[ai]);
                ai += 1;
            }
        }
        WriteAttributesResponse { records }
    }
}

/// Reason a Write Attributes payload could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WriteAttributesParseError {
    Malformed,
}

impl WriteAttributesRequest {
    /// Parse from ZCL payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        let outcome = Self::parse_checked(data).ok()?;
        if outcome.invalid_data_types.is_empty() {
            Some(outcome.request)
        } else {
            None
        }
    }

    /// Parse while preserving disabled data types for per-attribute ZCL errors.
    pub fn parse_checked(
        data: &[u8],
    ) -> Result<WriteAttributesParseOutcome, WriteAttributesParseError> {
        // A ZCL Write Attributes command must carry at least one write record.
        // An empty payload is malformed; accepting it would serialize a
        // vacuous "all writes succeeded" response for a request that wrote
        // nothing.
        if data.is_empty() {
            return Err(WriteAttributesParseError::Malformed);
        }
        let mut records = heapless::Vec::new();
        let mut invalid_data_types = heapless::Vec::new();
        let mut record_count = 0usize;
        let mut i = 0;
        while i < data.len() {
            if data.len() - i < 3 {
                return Err(WriteAttributesParseError::Malformed);
            }
            if record_count == MAX_WRITE_ATTRS {
                return Err(WriteAttributesParseError::Malformed);
            }
            record_count += 1;

            let id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
            i += 2;
            let dt = ZclDataType::from_u8(data[i]).ok_or(WriteAttributesParseError::Malformed)?;
            i += 1;
            if !is_data_type_enabled(dt) {
                let value_size = data_type_size(dt).ok_or(WriteAttributesParseError::Malformed)?;
                if i + value_size > data.len() {
                    return Err(WriteAttributesParseError::Malformed);
                }
                i += value_size;
                invalid_data_types
                    .push(WriteAttributeParseFailure {
                        ordinal: (record_count - 1) as u8,
                        record: WriteAttributeStatusRecord {
                            status: ZclStatus::InvalidDataType,
                            id,
                        },
                    })
                    .map_err(|_| WriteAttributesParseError::Malformed)?;
                continue;
            }
            let (value, consumed) = ZclValue::deserialize(dt, &data[i..])
                .ok_or(WriteAttributesParseError::Malformed)?;
            i += consumed;
            records
                .push(WriteAttributeRecord {
                    id,
                    data_type: dt,
                    value,
                })
                .map_err(|_| WriteAttributesParseError::Malformed)?;
        }
        Ok(WriteAttributesParseOutcome {
            request: Self { records },
            invalid_data_types,
        })
    }

    /// Serialize to ZCL payload bytes. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rec in &self.records {
            // Need at least 2 (id) + 1 (type) = 3 bytes
            if pos + 3 > buf.len() {
                break;
            }
            let b = rec.id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
            buf[pos] = rec.data_type as u8;
            pos += 1;
            let remaining = &mut buf[pos..];
            if remaining.is_empty() {
                break;
            }
            pos += rec.value.serialize(remaining);
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
            if buf.is_empty() {
                return 0;
            }
            buf[0] = ZclStatus::Success as u8;
            return 1;
        }
        let mut pos = 0;
        for rec in &self.records {
            if rec.status != ZclStatus::Success {
                // Need 1 (status) + 2 (id) = 3 bytes
                if pos + 3 > buf.len() {
                    break;
                }
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
        // Preflight through the same resolver `set` uses so validation covers
        // *every* way the apply below could fail (unsupported / read-only /
        // value-type mismatch), keeping the second pass partial-commit-free.
        let status = match store.validate_set(rec.id, &rec.value) {
            Ok(()) => ZclStatus::Success,
            Err(e) => e,
        };
        if status != ZclStatus::Success {
            all_ok = false;
        }
        let _ = records.push(WriteAttributeStatusRecord { status, id: rec.id });
    }

    // Second pass: apply if all valid. Every record already passed
    // `validate_set`, so none of these `set` calls can fail.
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
///
/// This is the shared "apply every record and collect its status" engine for
/// all three type-erased write paths. `#[inline(never)]` keeps the
/// `ZclValue`-clone + `store.set` loop compiled exactly once instead of being
/// duplicated inside each foundation write closure (0x02 / 0x03 commit / 0x05).
#[inline(never)]
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
///
/// Applies every record through the shared [`process_write_dyn`] engine and
/// discards the collected status, since command 0x05 must not emit a response.
pub fn process_write_no_response_dyn(
    store: &mut dyn crate::clusters::AttributeStoreMutAccess,
    request: &WriteAttributesRequest,
) {
    let _ = process_write_dyn(store, request);
}

/// Process a Write Attributes Undivided request (command 0x03) using type-erased store.
///
/// All-or-nothing: validates every write first; if any would fail, none are applied.
pub fn process_write_undivided_dyn(
    store: &mut dyn crate::clusters::AttributeStoreMutAccess,
    request: &WriteAttributesRequest,
) -> WriteAttributesResponse {
    let validation = validate_write_undivided_dyn(store, request);
    if validation
        .records
        .iter()
        .any(|record| record.status != ZclStatus::Success)
    {
        return validation;
    }

    // All records validated, so the command can be applied atomically through
    // the same shared apply engine as the regular write path.
    process_write_dyn(store, request)
}

/// Validate an undivided write without applying any record.
///
/// Preflights each record with
/// [`validate_set`](crate::clusters::AttributeStoreMutAccess::validate_set),
/// which returns the exact status the matching
/// [`set`](crate::clusters::AttributeStoreMutAccess::set) would produce. A
/// caller that only applies the batch when every status is
/// [`ZclStatus::Success`] is therefore guaranteed the apply phase cannot fail
/// part-way.
pub fn validate_write_undivided_dyn(
    store: &dyn crate::clusters::AttributeStoreMutAccess,
    request: &WriteAttributesRequest,
) -> WriteAttributesResponse {
    let mut records = heapless::Vec::new();
    for rec in &request.records {
        let status = match store.validate_set(rec.id, &rec.value) {
            Ok(()) => ZclStatus::Success,
            Err(e) => e,
        };
        let _ = records.push(WriteAttributeStatusRecord { status, id: rec.id });
    }
    WriteAttributesResponse { records }
}

#[cfg(all(test, any(not(feature = "float32"), not(feature = "float64"))))]
mod tests {
    use super::WriteAttributesRequest;
    #[cfg(not(feature = "float32"))]
    use super::{
        WriteAttributeRecord, process_write_dyn, process_write_no_response_dyn,
        process_write_undivided_dyn, validate_write_undivided_dyn,
    };
    use crate::AttributeId;
    #[cfg(not(feature = "float32"))]
    use crate::ZclStatus;
    #[cfg(not(feature = "float32"))]
    use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
    use crate::data_types::ZclDataType;
    #[cfg(not(feature = "float32"))]
    use crate::data_types::ZclValue;

    #[cfg(not(feature = "float32"))]
    const MIXED_FLOAT32_PAYLOAD: [u8; 16] = [
        0x01,
        0x00,
        ZclDataType::U8 as u8,
        0x2A,
        0x34,
        0x12,
        ZclDataType::Float32 as u8,
        0,
        0,
        0,
        0,
        0x02,
        0x00,
        ZclDataType::U16 as u8,
        0x78,
        0x56,
    ];

    #[cfg(not(feature = "float32"))]
    fn writable_store() -> AttributeStore<2> {
        let mut store = AttributeStore::new();
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(0x0001),
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadWrite,
                    name: "first",
                },
                ZclValue::U8(0),
            )
            .unwrap();
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(0x0002),
                    data_type: ZclDataType::U16,
                    access: AttributeAccess::ReadWrite,
                    name: "second",
                },
                ZclValue::U16(0),
            )
            .unwrap();
        store
    }

    /// Store fixture with a writable `0x0001` (U8) and a **read-only** `0x0003`
    /// (U8), used to exercise interleaved processed failures next to a
    /// parse-time disabled-type failure.
    #[cfg(not(feature = "float32"))]
    fn store_with_readonly() -> AttributeStore<4> {
        let mut store = AttributeStore::new();
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(0x0001),
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadWrite,
                    name: "writable",
                },
                ZclValue::U8(0),
            )
            .unwrap();
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(0x0003),
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "read-only",
                },
                ZclValue::U8(0x55),
            )
            .unwrap();
        store
    }

    /// Wire-order regression: a request that interleaves an *unsupported*
    /// record, a *disabled-data-type* record, a *read-only* record and a
    /// *duplicate unsupported* record must serialize its failure statuses in
    /// exact request order. Before the ordinal-preserving merge, the disabled
    /// type (parsed aside) was appended last, so it surfaced out of order.
    #[cfg(not(feature = "float32"))]
    #[test]
    fn interleaved_failures_serialize_in_request_order() {
        // ord0: 0x0009 unsupported | ord1: 0x1234 Float32 (disabled)
        // ord2: 0x0003 read-only   | ord3: 0x0009 unsupported (duplicate id)
        let payload = [
            0x09,
            0x00,
            ZclDataType::U8 as u8,
            0x11,
            0x34,
            0x12,
            ZclDataType::Float32 as u8,
            0,
            0,
            0,
            0,
            0x03,
            0x00,
            ZclDataType::U8 as u8,
            0x22,
            0x09,
            0x00,
            ZclDataType::U8 as u8,
            0x33,
        ];

        let outcome = WriteAttributesRequest::parse_checked(&payload).unwrap();
        // Three supported records decoded; the disabled Float32 kept at ord1.
        assert_eq!(outcome.request.records.len(), 3);
        assert_eq!(outcome.invalid_data_types.len(), 1);
        assert_eq!(outcome.invalid_data_types[0].ordinal, 1);

        let mut store = store_with_readonly();
        let applied = process_write_dyn(&mut store, &outcome.request);
        let response = outcome.merge_in_request_order(&applied);

        let mut payload_out = [0u8; 16];
        let len = response.serialize(&mut payload_out);
        assert_eq!(
            &payload_out[..len],
            &[
                ZclStatus::UnsupportedAttribute as u8,
                0x09,
                0x00,
                // The disabled type (ord1) lands *between* the processed
                // failures, not appended after them.
                ZclStatus::InvalidDataType as u8,
                0x34,
                0x12,
                ZclStatus::ReadOnly as u8,
                0x03,
                0x00,
                ZclStatus::UnsupportedAttribute as u8,
                0x09,
                0x00,
            ]
        );
    }

    /// Undivided atomicity (finding 2): the second record's *declared*
    /// `data_type` matches the attribute (U16), but the carried `ZclValue` is
    /// U8 — the exact case old prevalidation (comparing only `record.data_type`)
    /// admitted, letting `set` fail after the first record had already
    /// committed. `validate_set` now predicts the `set` failure, so nothing
    /// applies.
    #[cfg(not(feature = "float32"))]
    #[test]
    fn undivided_declared_type_ok_but_value_type_mismatch_applies_nothing() {
        let request = WriteAttributesRequest {
            records: heapless::Vec::from_slice(&[
                WriteAttributeRecord {
                    id: AttributeId(0x0001),
                    data_type: ZclDataType::U8,
                    value: ZclValue::U8(0x2A),
                },
                WriteAttributeRecord {
                    id: AttributeId(0x0002),
                    data_type: ZclDataType::U16, // declared U16 (matches def) …
                    value: ZclValue::U8(0x99),   // … but value is U8 → set fails
                },
            ])
            .unwrap(),
        };

        let mut store = writable_store();
        let response = process_write_undivided_dyn(&mut store, &request);

        // Atomic: neither the valid first record nor the mismatched second wrote.
        assert_eq!(store.get(AttributeId(0x0001)), Some(&ZclValue::U8(0)));
        assert_eq!(store.get(AttributeId(0x0002)), Some(&ZclValue::U16(0)));
        assert_eq!(response.records.len(), 2);
        assert_eq!(response.records[0].status, ZclStatus::Success);
        assert_eq!(response.records[1].status, ZclStatus::InvalidDataType);
    }

    #[cfg(not(feature = "float32"))]
    #[test]
    fn disabled_float32_is_reported_as_an_invalid_data_type() {
        let outcome = WriteAttributesRequest::parse_checked(&MIXED_FLOAT32_PAYLOAD).unwrap();
        assert_eq!(outcome.request.records.len(), 2);
        assert_eq!(outcome.request.records[0].id, AttributeId(0x0001));
        assert_eq!(outcome.request.records[1].id, AttributeId(0x0002));
        assert_eq!(outcome.invalid_data_types.len(), 1);
        // The disabled Float32 record sits *between* the two supported records,
        // so its recorded ordinal must be 1 (not appended at the end).
        assert_eq!(outcome.invalid_data_types[0].ordinal, 1);
        assert_eq!(outcome.invalid_data_types[0].record.id, AttributeId(0x1234));
        assert_eq!(
            outcome.invalid_data_types[0].record.status,
            crate::ZclStatus::InvalidDataType
        );
    }

    #[cfg(not(feature = "float32"))]
    #[test]
    fn regular_mixed_write_applies_valid_records_and_reports_disabled_float() {
        let outcome = WriteAttributesRequest::parse_checked(&MIXED_FLOAT32_PAYLOAD).unwrap();
        let mut store = writable_store();
        let applied = process_write_dyn(&mut store, &outcome.request);
        let response = outcome.merge_in_request_order(&applied);

        assert_eq!(store.get(AttributeId(0x0001)), Some(&ZclValue::U8(0x2A)));
        assert_eq!(store.get(AttributeId(0x0002)), Some(&ZclValue::U16(0x5678)));
        let mut payload = [0u8; 8];
        let len = response.serialize(&mut payload);
        assert_eq!(
            &payload[..len],
            &[ZclStatus::InvalidDataType as u8, 0x34, 0x12]
        );
    }

    #[cfg(not(feature = "float32"))]
    #[test]
    fn undivided_mixed_write_validates_all_records_without_applying_any() {
        let outcome = WriteAttributesRequest::parse_checked(&MIXED_FLOAT32_PAYLOAD).unwrap();
        let store = writable_store();
        let validated = validate_write_undivided_dyn(&store, &outcome.request);
        let response = outcome.merge_in_request_order(&validated);

        assert_eq!(store.get(AttributeId(0x0001)), Some(&ZclValue::U8(0)));
        assert_eq!(store.get(AttributeId(0x0002)), Some(&ZclValue::U16(0)));
        let mut payload = [0u8; 8];
        let len = response.serialize(&mut payload);
        assert_eq!(
            &payload[..len],
            &[ZclStatus::InvalidDataType as u8, 0x34, 0x12]
        );
    }

    #[cfg(not(feature = "float32"))]
    #[test]
    fn no_response_mixed_write_applies_valid_records() {
        let outcome = WriteAttributesRequest::parse_checked(&MIXED_FLOAT32_PAYLOAD).unwrap();
        let mut store = writable_store();
        process_write_no_response_dyn(&mut store, &outcome.request);

        assert_eq!(store.get(AttributeId(0x0001)), Some(&ZclValue::U8(0x2A)));
        assert_eq!(store.get(AttributeId(0x0002)), Some(&ZclValue::U16(0x5678)));
    }

    /// The refactor routes 0x02 (regular) and the 0x03 (undivided) commit phase
    /// through the same `process_write_dyn` engine, so an all-valid undivided
    /// write must apply every record and report all-success — byte-identical to
    /// the regular write for the same records.
    #[cfg(not(feature = "float32"))]
    #[test]
    fn undivided_all_valid_write_applies_every_record() {
        let request = WriteAttributesRequest {
            records: heapless::Vec::from_slice(&[
                WriteAttributeRecord {
                    id: AttributeId(0x0001),
                    data_type: ZclDataType::U8,
                    value: ZclValue::U8(0x2A),
                },
                WriteAttributeRecord {
                    id: AttributeId(0x0002),
                    data_type: ZclDataType::U16,
                    value: ZclValue::U16(0x5678),
                },
            ])
            .unwrap(),
        };

        let mut undivided_store = writable_store();
        let undivided = process_write_undivided_dyn(&mut undivided_store, &request);
        let mut regular_store = writable_store();
        let regular = process_write_dyn(&mut regular_store, &request);

        // Both engines apply the same writes and produce the same status records.
        assert_eq!(
            undivided_store.get(AttributeId(0x0001)),
            Some(&ZclValue::U8(0x2A))
        );
        assert_eq!(
            undivided_store.get(AttributeId(0x0002)),
            Some(&ZclValue::U16(0x5678))
        );
        assert!(
            undivided
                .records
                .iter()
                .all(|r| r.status == ZclStatus::Success)
        );
        let mut a = [0u8; 8];
        let mut b = [0u8; 8];
        let na = undivided.serialize(&mut a);
        let nb = regular.serialize(&mut b);
        assert_eq!(&a[..na], &b[..nb]);
    }

    /// Undivided atomicity: if any record would fail, `process_write_undivided_dyn`
    /// must apply *nothing* and return the validation statuses (not the apply
    /// statuses). This guards the "all or nothing" contract preserved by the
    /// refactor.
    #[cfg(not(feature = "float32"))]
    #[test]
    fn undivided_write_with_one_failing_record_applies_nothing() {
        let request = WriteAttributesRequest {
            records: heapless::Vec::from_slice(&[
                WriteAttributeRecord {
                    id: AttributeId(0x0001),
                    data_type: ZclDataType::U8,
                    value: ZclValue::U8(0x2A),
                },
                // 0x0003 is not registered → UnsupportedAttribute during validation.
                WriteAttributeRecord {
                    id: AttributeId(0x0003),
                    data_type: ZclDataType::U8,
                    value: ZclValue::U8(0x55),
                },
            ])
            .unwrap(),
        };

        let mut store = writable_store();
        let response = process_write_undivided_dyn(&mut store, &request);

        // No record applied — the valid first record must remain untouched.
        assert_eq!(store.get(AttributeId(0x0001)), Some(&ZclValue::U8(0)));
        // Response carries per-record validation statuses.
        assert_eq!(response.records.len(), 2);
        assert_eq!(response.records[0].status, ZclStatus::Success);
        assert_eq!(response.records[1].status, ZclStatus::UnsupportedAttribute);
    }

    #[cfg(not(feature = "float32"))]
    #[test]
    fn no_response_write_applies_same_records_as_regular_write() {
        // The no-response path delegates to `process_write_dyn`; its observable
        // effect on the store must match the regular write for the same request.
        let outcome = WriteAttributesRequest::parse_checked(&MIXED_FLOAT32_PAYLOAD).unwrap();
        let mut regular_store = writable_store();
        let _ = process_write_dyn(&mut regular_store, &outcome.request);
        let mut no_resp_store = writable_store();
        process_write_no_response_dyn(&mut no_resp_store, &outcome.request);

        assert_eq!(
            no_resp_store.get(AttributeId(0x0001)),
            regular_store.get(AttributeId(0x0001))
        );
        assert_eq!(
            no_resp_store.get(AttributeId(0x0002)),
            regular_store.get(AttributeId(0x0002))
        );
    }

    #[cfg(not(feature = "float64"))]
    #[test]
    fn disabled_float64_is_reported_as_an_invalid_data_type() {
        let payload = [
            0x78,
            0x56,
            ZclDataType::Float64 as u8,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        let outcome = WriteAttributesRequest::parse_checked(&payload).unwrap();
        assert!(outcome.request.records.is_empty());
        assert_eq!(outcome.invalid_data_types.len(), 1);
        assert_eq!(outcome.invalid_data_types[0].ordinal, 0);
        assert_eq!(outcome.invalid_data_types[0].record.id, AttributeId(0x5678));
    }

    #[test]
    fn trailing_partial_record_is_malformed() {
        let payload = [0x01, 0x00];
        assert!(WriteAttributesRequest::parse_checked(&payload).is_err());
    }

    #[cfg(not(feature = "float64"))]
    #[test]
    fn disabled_records_count_toward_max_write_attrs() {
        // Disabled data types still occupy a record slot, so they count toward
        // the MAX_WRITE_ATTRS cap: MAX_WRITE_ATTRS - 1 valid records plus one
        // disabled Float64 record fill the request exactly.
        let mut payload = heapless::Vec::<u8, 256>::new();
        for n in 0..(super::MAX_WRITE_ATTRS as u16 - 1) {
            payload.extend_from_slice(&n.to_le_bytes()).unwrap();
            payload.push(ZclDataType::U8 as u8).unwrap();
            payload.push(n as u8).unwrap();
        }
        // Disabled Float64 record: id(2) + type(1) + value(8).
        payload.extend_from_slice(&0x00FFu16.to_le_bytes()).unwrap();
        payload.push(ZclDataType::Float64 as u8).unwrap();
        payload.extend_from_slice(&[0u8; 8]).unwrap();

        let outcome = WriteAttributesRequest::parse_checked(&payload).unwrap();
        assert_eq!(outcome.request.records.len(), super::MAX_WRITE_ATTRS - 1);
        assert_eq!(outcome.invalid_data_types.len(), 1);

        // One more record (the 17th) overflows the cap regardless of its type.
        payload.extend_from_slice(&0x0EEEu16.to_le_bytes()).unwrap();
        payload.push(ZclDataType::U8 as u8).unwrap();
        payload.push(0).unwrap();
        assert!(WriteAttributesRequest::parse_checked(&payload).is_err());
    }
}

/// Record-count / empty-payload bounds are independent of which data-type
/// features are enabled, so they live in their own always-compiled module
/// (the module above only compiles when a float type is disabled).
#[cfg(test)]
mod bounds_tests {
    use super::{MAX_WRITE_ATTRS, WriteAttributesRequest};
    use crate::data_types::ZclDataType;

    #[test]
    fn empty_payload_is_malformed() {
        // A Write Attributes command with no records is malformed; it must not
        // decode to an empty request that serializes a vacuous Success.
        assert!(WriteAttributesRequest::parse_checked(&[]).is_err());
        assert!(WriteAttributesRequest::parse(&[]).is_none());
    }

    #[test]
    fn max_write_attrs_records_accepted_seventeenth_rejected() {
        // Each record is id(2 LE) + type(U8) + value(1) = 4 bytes.
        let mut payload = heapless::Vec::<u8, 256>::new();
        for n in 0..MAX_WRITE_ATTRS as u16 {
            payload.extend_from_slice(&n.to_le_bytes()).unwrap();
            payload.push(ZclDataType::U8 as u8).unwrap();
            payload.push(n as u8).unwrap();
        }

        // Exactly MAX_WRITE_ATTRS records decode successfully.
        let outcome = WriteAttributesRequest::parse_checked(&payload).unwrap();
        assert_eq!(outcome.request.records.len(), MAX_WRITE_ATTRS);
        assert!(outcome.invalid_data_types.is_empty());

        // A 17th record exceeds the cap and is rejected as Malformed.
        let seventeenth = MAX_WRITE_ATTRS as u16;
        payload
            .extend_from_slice(&seventeenth.to_le_bytes())
            .unwrap();
        payload.push(ZclDataType::U8 as u8).unwrap();
        payload.push(0).unwrap();
        assert!(WriteAttributesRequest::parse_checked(&payload).is_err());
    }
}
