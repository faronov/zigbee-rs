//! ZCL data type system (ZCL Rev 8, Chapter 2.6).
//!
//! Defines every standard data type ID and a value enum for storing typed
//! attribute values in a `no_std` context.

/// Standard ZCL data type identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ZclDataType {
    NoData = 0x00,
    Bool = 0x10,
    Bitmap8 = 0x18,
    Bitmap16 = 0x19,
    Bitmap32 = 0x1B,
    Bitmap64 = 0x1F,
    U8 = 0x20,
    U16 = 0x21,
    U24 = 0x22,
    U32 = 0x23,
    U48 = 0x25,
    U64 = 0x27,
    I8 = 0x28,
    I16 = 0x29,
    I32 = 0x2B,
    I64 = 0x2F,
    Enum8 = 0x30,
    Enum16 = 0x31,
    Float16 = 0x38,
    Float32 = 0x39,
    Float64 = 0x3A,
    OctetString = 0x41,
    CharString = 0x42,
    OctetString16 = 0x43,
    CharString16 = 0x44,
    Array = 0x48,
    Struct = 0x4C,
    Set = 0x50,
    Bag = 0x51,
    TimeOfDay = 0xE0,
    Date = 0xE1,
    UtcTime = 0xE2,
    ClusterId = 0xE8,
    AttributeId = 0xE9,
    BacNetOid = 0xEA,
    IeeeAddr = 0xF0,
    SecurityKey128 = 0xF1,
}

impl ZclDataType {
    /// Try to create a `ZclDataType` from its wire byte.
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::NoData),
            0x10 => Some(Self::Bool),
            0x18 => Some(Self::Bitmap8),
            0x19 => Some(Self::Bitmap16),
            0x1B => Some(Self::Bitmap32),
            0x1F => Some(Self::Bitmap64),
            0x20 => Some(Self::U8),
            0x21 => Some(Self::U16),
            0x22 => Some(Self::U24),
            0x23 => Some(Self::U32),
            0x25 => Some(Self::U48),
            0x27 => Some(Self::U64),
            0x28 => Some(Self::I8),
            0x29 => Some(Self::I16),
            0x2B => Some(Self::I32),
            0x2F => Some(Self::I64),
            0x30 => Some(Self::Enum8),
            0x31 => Some(Self::Enum16),
            0x38 => Some(Self::Float16),
            0x39 => Some(Self::Float32),
            0x3A => Some(Self::Float64),
            0x41 => Some(Self::OctetString),
            0x42 => Some(Self::CharString),
            0x43 => Some(Self::OctetString16),
            0x44 => Some(Self::CharString16),
            0x48 => Some(Self::Array),
            0x4C => Some(Self::Struct),
            0x50 => Some(Self::Set),
            0x51 => Some(Self::Bag),
            0xE0 => Some(Self::TimeOfDay),
            0xE1 => Some(Self::Date),
            0xE2 => Some(Self::UtcTime),
            0xE8 => Some(Self::ClusterId),
            0xE9 => Some(Self::AttributeId),
            0xEA => Some(Self::BacNetOid),
            0xF0 => Some(Self::IeeeAddr),
            0xF1 => Some(Self::SecurityKey128),
            _ => None,
        }
    }
}

/// Return the fixed wire size (in bytes) for a data type, or `None` for
/// variable-length types.
pub fn data_type_size(dt: ZclDataType) -> Option<usize> {
    match dt {
        ZclDataType::NoData => Some(0),
        ZclDataType::Bool
        | ZclDataType::U8
        | ZclDataType::I8
        | ZclDataType::Enum8
        | ZclDataType::Bitmap8 => Some(1),
        ZclDataType::U16
        | ZclDataType::I16
        | ZclDataType::Enum16
        | ZclDataType::Bitmap16
        | ZclDataType::Float16
        | ZclDataType::ClusterId
        | ZclDataType::AttributeId => Some(2),
        ZclDataType::U24 => Some(3),
        ZclDataType::U32
        | ZclDataType::I32
        | ZclDataType::Bitmap32
        | ZclDataType::Float32
        | ZclDataType::UtcTime
        | ZclDataType::BacNetOid => Some(4),
        ZclDataType::TimeOfDay | ZclDataType::Date => Some(4),
        ZclDataType::U48 => Some(6),
        ZclDataType::U64
        | ZclDataType::I64
        | ZclDataType::Bitmap64
        | ZclDataType::IeeeAddr
        | ZclDataType::Float64 => Some(8),
        ZclDataType::SecurityKey128 => Some(16),
        // Variable-length types
        ZclDataType::OctetString
        | ZclDataType::CharString
        | ZclDataType::OctetString16
        | ZclDataType::CharString16
        | ZclDataType::Array
        | ZclDataType::Struct
        | ZclDataType::Set
        | ZclDataType::Bag => None,
    }
}

/// Maximum size (bytes) of an inline string/octet-string value.
pub const MAX_STRING_LEN: usize = 32;

/// Typed ZCL value matching the data type system.
#[derive(Debug, Clone, PartialEq)]
pub enum ZclValue {
    NoData,
    Bool(bool),
    Bitmap8(u8),
    Bitmap16(u16),
    Bitmap32(u32),
    Bitmap64(u64),
    U8(u8),
    U16(u16),
    U24(u32),
    U32(u32),
    U48(u64),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    Enum8(u8),
    Enum16(u16),
    Float32(f32),
    Float64(f64),
    /// Octet/char string stored inline (length-prefixed on the wire).
    OctetString(heapless::Vec<u8, MAX_STRING_LEN>),
    CharString(heapless::Vec<u8, MAX_STRING_LEN>),
    UtcTime(u32),
    IeeeAddr(u64),
    SecurityKey128([u8; 16]),
}

impl ZclValue {
    /// Return the ZCL data type tag for this value.
    pub fn data_type(&self) -> ZclDataType {
        match self {
            Self::NoData => ZclDataType::NoData,
            Self::Bool(_) => ZclDataType::Bool,
            Self::Bitmap8(_) => ZclDataType::Bitmap8,
            Self::Bitmap16(_) => ZclDataType::Bitmap16,
            Self::Bitmap32(_) => ZclDataType::Bitmap32,
            Self::Bitmap64(_) => ZclDataType::Bitmap64,
            Self::U8(_) => ZclDataType::U8,
            Self::U16(_) => ZclDataType::U16,
            Self::U24(_) => ZclDataType::U24,
            Self::U32(_) => ZclDataType::U32,
            Self::U48(_) => ZclDataType::U48,
            Self::U64(_) => ZclDataType::U64,
            Self::I8(_) => ZclDataType::I8,
            Self::I16(_) => ZclDataType::I16,
            Self::I32(_) => ZclDataType::I32,
            Self::I64(_) => ZclDataType::I64,
            Self::Enum8(_) => ZclDataType::Enum8,
            Self::Enum16(_) => ZclDataType::Enum16,
            Self::Float32(_) => ZclDataType::Float32,
            Self::Float64(_) => ZclDataType::Float64,
            Self::OctetString(_) => ZclDataType::OctetString,
            Self::CharString(_) => ZclDataType::CharString,
            Self::UtcTime(_) => ZclDataType::UtcTime,
            Self::IeeeAddr(_) => ZclDataType::IeeeAddr,
            Self::SecurityKey128(_) => ZclDataType::SecurityKey128,
        }
    }

    /// Serialize this value to a buffer (little-endian, ZCL wire format).
    /// Returns the number of bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::NoData => 0,
            Self::Bool(v) => {
                buf[0] = if *v { 1 } else { 0 };
                1
            }
            Self::U8(v) | Self::Enum8(v) | Self::Bitmap8(v) => {
                buf[0] = *v;
                1
            }
            Self::U16(v) | Self::Enum16(v) | Self::Bitmap16(v) => {
                let b = v.to_le_bytes();
                buf[..2].copy_from_slice(&b);
                2
            }
            Self::U24(v) => {
                let b = v.to_le_bytes();
                buf[..3].copy_from_slice(&b[..3]);
                3
            }
            Self::U32(v) | Self::UtcTime(v) | Self::Bitmap32(v) => {
                let b = v.to_le_bytes();
                buf[..4].copy_from_slice(&b);
                4
            }
            Self::U48(v) => {
                let b = v.to_le_bytes();
                buf[..6].copy_from_slice(&b[..6]);
                6
            }
            Self::U64(v) | Self::IeeeAddr(v) | Self::Bitmap64(v) => {
                let b = v.to_le_bytes();
                buf[..8].copy_from_slice(&b);
                8
            }
            Self::I8(v) => {
                buf[0] = *v as u8;
                1
            }
            Self::I16(v) => {
                let b = v.to_le_bytes();
                buf[..2].copy_from_slice(&b);
                2
            }
            Self::I32(v) => {
                let b = v.to_le_bytes();
                buf[..4].copy_from_slice(&b);
                4
            }
            Self::I64(v) => {
                let b = v.to_le_bytes();
                buf[..8].copy_from_slice(&b);
                8
            }
            Self::Float32(v) => {
                let b = v.to_le_bytes();
                buf[..4].copy_from_slice(&b);
                4
            }
            Self::Float64(v) => {
                let b = v.to_le_bytes();
                buf[..8].copy_from_slice(&b);
                8
            }
            Self::OctetString(v) | Self::CharString(v) => {
                buf[0] = v.len() as u8;
                buf[1..1 + v.len()].copy_from_slice(v);
                1 + v.len()
            }
            Self::SecurityKey128(v) => {
                buf[..16].copy_from_slice(v);
                16
            }
        }
    }

    /// Deserialize a ZCL value of a known type from a buffer.
    /// Returns `(value, bytes_consumed)`.
    pub fn deserialize(dt: ZclDataType, data: &[u8]) -> Option<(Self, usize)> {
        match dt {
            ZclDataType::NoData => Some((Self::NoData, 0)),
            ZclDataType::Bool => {
                let v = *data.first()?;
                Some((Self::Bool(v != 0), 1))
            }
            ZclDataType::U8 => Some((Self::U8(*data.first()?), 1)),
            ZclDataType::Enum8 => Some((Self::Enum8(*data.first()?), 1)),
            ZclDataType::Bitmap8 => Some((Self::Bitmap8(*data.first()?), 1)),
            ZclDataType::U16 => {
                if data.len() < 2 {
                    return None;
                }
                Some((Self::U16(u16::from_le_bytes([data[0], data[1]])), 2))
            }
            ZclDataType::Enum16 => {
                if data.len() < 2 {
                    return None;
                }
                Some((Self::Enum16(u16::from_le_bytes([data[0], data[1]])), 2))
            }
            ZclDataType::Bitmap16 => {
                if data.len() < 2 {
                    return None;
                }
                Some((Self::Bitmap16(u16::from_le_bytes([data[0], data[1]])), 2))
            }
            ZclDataType::U24 => {
                if data.len() < 3 {
                    return None;
                }
                let v = u32::from_le_bytes([data[0], data[1], data[2], 0]);
                Some((Self::U24(v), 3))
            }
            ZclDataType::U32 => {
                if data.len() < 4 {
                    return None;
                }
                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                Some((Self::U32(v), 4))
            }
            ZclDataType::Bitmap32 => {
                if data.len() < 4 {
                    return None;
                }
                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                Some((Self::Bitmap32(v), 4))
            }
            ZclDataType::U48 => {
                if data.len() < 6 {
                    return None;
                }
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], 0, 0,
                ]);
                Some((Self::U48(v), 6))
            }
            ZclDataType::U64 => {
                if data.len() < 8 {
                    return None;
                }
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                ]);
                Some((Self::U64(v), 8))
            }
            ZclDataType::Bitmap64 => {
                if data.len() < 8 {
                    return None;
                }
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                ]);
                Some((Self::Bitmap64(v), 8))
            }
            ZclDataType::I8 => Some((Self::I8(*data.first()? as i8), 1)),
            ZclDataType::I16 => {
                if data.len() < 2 {
                    return None;
                }
                Some((Self::I16(i16::from_le_bytes([data[0], data[1]])), 2))
            }
            ZclDataType::I32 => {
                if data.len() < 4 {
                    return None;
                }
                let v = i32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                Some((Self::I32(v), 4))
            }
            ZclDataType::I64 => {
                if data.len() < 8 {
                    return None;
                }
                let v = i64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                ]);
                Some((Self::I64(v), 8))
            }
            ZclDataType::Float32 => {
                if data.len() < 4 {
                    return None;
                }
                let v = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                Some((Self::Float32(v), 4))
            }
            ZclDataType::Float64 => {
                if data.len() < 8 {
                    return None;
                }
                let v = f64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                ]);
                Some((Self::Float64(v), 8))
            }
            ZclDataType::UtcTime => {
                if data.len() < 4 {
                    return None;
                }
                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                Some((Self::UtcTime(v), 4))
            }
            ZclDataType::IeeeAddr => {
                if data.len() < 8 {
                    return None;
                }
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                ]);
                Some((Self::IeeeAddr(v), 8))
            }
            ZclDataType::SecurityKey128 => {
                if data.len() < 16 {
                    return None;
                }
                let mut key = [0u8; 16];
                key.copy_from_slice(&data[..16]);
                Some((Self::SecurityKey128(key), 16))
            }
            ZclDataType::OctetString => {
                let len = *data.first()? as usize;
                if data.len() < 1 + len {
                    return None;
                }
                let mut v = heapless::Vec::new();
                for &b in &data[1..1 + len] {
                    v.push(b).ok()?;
                }
                Some((Self::OctetString(v), 1 + len))
            }
            ZclDataType::CharString => {
                let len = *data.first()? as usize;
                if data.len() < 1 + len {
                    return None;
                }
                let mut v = heapless::Vec::new();
                for &b in &data[1..1 + len] {
                    v.push(b).ok()?;
                }
                Some((Self::CharString(v), 1 + len))
            }
            // Complex/variable types not yet supported for inline deserialization.
            _ => None,
        }
    }
}

impl ZclValue {
    /// Check if the absolute difference between `self` and `other` exceeds a threshold.
    /// Used by the reporting engine for reportable_change comparison.
    /// Returns `true` if |self - other| >= threshold, `false` otherwise.
    /// For non-numeric types, always returns `true` if values differ.
    pub fn exceeds_threshold(&self, other: &ZclValue, threshold: &ZclValue) -> bool {
        match (self, other, threshold) {
            (ZclValue::U8(a), ZclValue::U8(b), ZclValue::U8(t)) => a.abs_diff(*b) >= *t,
            (ZclValue::U16(a), ZclValue::U16(b), ZclValue::U16(t)) => a.abs_diff(*b) >= *t,
            (ZclValue::U32(a), ZclValue::U32(b), ZclValue::U32(t)) => a.abs_diff(*b) >= *t,
            (ZclValue::U64(a), ZclValue::U64(b), ZclValue::U64(t)) => a.abs_diff(*b) >= *t,
            (ZclValue::I8(a), ZclValue::I8(b), ZclValue::I8(t)) => {
                ((*a as i16) - (*b as i16)).unsigned_abs() >= *t as u16
            }
            (ZclValue::I16(a), ZclValue::I16(b), ZclValue::I16(t)) => {
                ((*a as i32) - (*b as i32)).unsigned_abs() >= *t as u32
            }
            (ZclValue::I32(a), ZclValue::I32(b), ZclValue::I32(t)) => {
                ((*a as i64) - (*b as i64)).unsigned_abs() >= *t as u64
            }
            // For non-numeric or mismatched types, any difference triggers
            _ => self != other,
        }
    }
}

/// Convenience free function: serialize a value into `buf`.
/// Returns the number of bytes written.
pub fn serialize_value(val: &ZclValue, buf: &mut [u8]) -> usize {
    val.serialize(buf)
}

/// Convenience free function: parse a value of a known type from `data`.
/// Returns `(value, bytes_consumed)`.
pub fn parse_value(data_type: ZclDataType, data: &[u8]) -> Option<(ZclValue, usize)> {
    ZclValue::deserialize(data_type, data)
}

/// Whether a data type is "analog" (supports reportable change thresholds).
pub fn is_analog_type(dt: ZclDataType) -> bool {
    matches!(
        dt,
        ZclDataType::U8
            | ZclDataType::U16
            | ZclDataType::U24
            | ZclDataType::U32
            | ZclDataType::U48
            | ZclDataType::U64
            | ZclDataType::I8
            | ZclDataType::I16
            | ZclDataType::I32
            | ZclDataType::I64
            | ZclDataType::Float16
            | ZclDataType::Float32
            | ZclDataType::Float64
            | ZclDataType::UtcTime
            | ZclDataType::TimeOfDay
            | ZclDataType::Date
    )
}

/// Whether a data type is "discrete" (does not support reportable change).
pub fn is_discrete_type(dt: ZclDataType) -> bool {
    !is_analog_type(dt)
}
