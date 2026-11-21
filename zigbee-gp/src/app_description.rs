//! GPD Application Description command (GPDF `0xE4`) codec.
//!
//! A multi-sensor or Compact Attribute Reporting GPD announces "application
//! description follows" in its Commissioning command and then describes its
//! reports in one or more Application Description commands:
//!
//! ```text
//! u8 total number of reports, u8 number of reports in this frame,
//! report descriptor × number of reports
//!
//! report descriptor:
//!   u8 report ID, u8 options (bit 0 timeout period present),
//!   [u16 timeout period], u8 remaining length (of the data points),
//!   data point descriptor × n
//!
//! data point descriptor:
//!   u8 options (bits 0-2 attribute records - 1, bit 3 server cluster,
//!               bit 4 manufacturer ID present),
//!   u16 cluster ID, [u16 manufacturer ID], attribute record × (n + 1)
//!
//! attribute record:
//!   u16 attribute ID, u8 attribute data type,
//!   u8 options (bits 0-3 remaining length, bit 4 reported,
//!               bit 5 attribute value present),
//!   [u8 offset within the report, if reported], [value, if present]
//! ```
//!
//! The same report descriptors follow the Application Information of a GP
//! Pairing Configuration with action 0b101 (gecko_sdk
//! `emberAfFillCommandGreenPowerClusterGpPairingConfigurationSmart`).
//! [`ApplicationDescription::decode`] validates every descriptor, so the
//! iterators never meet malformed data; a descriptor is at least
//! [`MIN_REPORT_LEN`] octets (gecko_sdk `GREEN_POWER_SERVER_MIN_REPORT_LENGTH`).

use crate::codec::Reader;
use crate::{Error, Result};

/// Shortest report descriptor: report ID, options, remaining length and one
/// data point with one attribute record without offset or value.
pub const MIN_REPORT_LEN: usize = 10;

/// Report descriptor option: timeout period present.
const TIMEOUT_PRESENT: u8 = 0x01;
/// Data point option: attribute records − 1.
const RECORDS_MASK: u8 = 0x07;
/// Data point option: server cluster.
const SERVER_CLUSTER: u8 = 0x08;
/// Data point option: manufacturer ID present.
const MANUFACTURER_PRESENT: u8 = 0x10;
/// Attribute option: remaining length.
const REMAINING_MASK: u8 = 0x0F;
/// Attribute option: reported (offset within the report present).
const REPORTED: u8 = 0x10;
/// Attribute option: attribute value present.
const VALUE_PRESENT: u8 = 0x20;

/// A validated Application Description command payload (after the command
/// ID) or the report part of a GP Pairing Configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApplicationDescription<'a> {
    /// Reports the GPD describes in total.
    pub total_reports: u8,
    /// Report descriptors in this frame.
    pub report_count: u8,
    /// The report descriptors, back to back.
    pub reports: &'a [u8],
}

impl<'a> ApplicationDescription<'a> {
    /// Decode and validate an Application Description command payload.
    pub fn decode(payload: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(payload);
        let total_reports = r.u8()?;
        let report_count = r.u8()?;
        Self::from_parts(total_reports, report_count, r.rest())
    }

    /// Validate `report_count` descriptors that fill `reports` exactly
    /// (GP Pairing Configuration application description).
    pub fn from_parts(total_reports: u8, report_count: u8, reports: &'a [u8]) -> Result<Self> {
        if report_count == 0 || total_reports < report_count {
            return Err(Error::Unsupported);
        }
        let mut rest = reports;
        for _ in 0..report_count {
            rest = report(rest)?.1;
        }
        if !rest.is_empty() {
            return Err(Error::TrailingBytes);
        }
        Ok(Self {
            total_reports,
            report_count,
            reports,
        })
    }

    /// The report descriptors.
    pub fn iter(&self) -> Reports<'a> {
        Reports(self.reports)
    }
}

/// Iterator over validated report descriptors.
pub struct Reports<'a>(&'a [u8]);

impl<'a> Iterator for Reports<'a> {
    type Item = ReportDescriptor<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let (item, rest) = report(self.0).ok()?;
        self.0 = rest;
        Some(item)
    }
}

/// One report descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReportDescriptor<'a> {
    /// Report identifier (`0..total_reports`).
    pub report_id: u8,
    /// Timeout period, when present.
    pub timeout: Option<u16>,
    /// The whole descriptor as received.
    pub raw: &'a [u8],
    /// The data point descriptors.
    pub data_points: &'a [u8],
}

impl<'a> ReportDescriptor<'a> {
    /// The data point descriptors.
    pub fn iter(&self) -> DataPoints<'a> {
        DataPoints(self.data_points)
    }
}

/// Iterator over the data point descriptors of a report.
pub struct DataPoints<'a>(&'a [u8]);

impl<'a> Iterator for DataPoints<'a> {
    type Item = DataPoint<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let (item, rest) = data_point(self.0).ok()?;
        self.0 = rest;
        Some(item)
    }
}

/// One data point descriptor (a cluster of the report).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataPoint<'a> {
    /// Server (`true`) or client cluster.
    pub server: bool,
    /// Cluster identifier.
    pub cluster_id: u16,
    /// Manufacturer code, when present.
    pub manufacturer_id: Option<u16>,
    /// The attribute records.
    pub records: &'a [u8],
}

impl<'a> DataPoint<'a> {
    /// The attribute records.
    pub fn iter(&self) -> AttributeRecords<'a> {
        AttributeRecords(self.records)
    }
}

/// Iterator over the attribute records of a data point.
pub struct AttributeRecords<'a>(&'a [u8]);

impl<'a> Iterator for AttributeRecords<'a> {
    type Item = AttributeRecord<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let (item, rest) = attribute(self.0).ok()?;
        self.0 = rest;
        Some(item)
    }
}

/// One attribute record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttributeRecord<'a> {
    /// Attribute identifier.
    pub attribute_id: u16,
    /// ZCL data type.
    pub data_type: u8,
    /// Offset of the value within the Compact Attribute Report payload
    /// (after the report ID), when the attribute is reported.
    pub offset: Option<u8>,
    /// Fixed attribute value, when present.
    pub value: Option<&'a [u8]>,
}

/// Split `buf` after its first `len` octets.
fn split(buf: &[u8], len: usize) -> Result<(&[u8], &[u8])> {
    buf.split_at_checked(len).ok_or(Error::Truncated)
}

fn report(buf: &[u8]) -> Result<(ReportDescriptor<'_>, &[u8])> {
    let mut r = Reader::new(buf);
    let report_id = r.u8()?;
    let options = r.u8()?;
    let timeout = if options & TIMEOUT_PRESENT != 0 {
        Some(r.u16()?)
    } else {
        None
    };
    let len = usize::from(r.u8()?);
    let data_points = r.bytes(len)?;
    let (raw, rest) = split(buf, buf.len() - r.remaining())?;
    if raw.len() < MIN_REPORT_LEN {
        return Err(Error::Unsupported);
    }
    let mut dp = data_points;
    while !dp.is_empty() {
        dp = data_point(dp)?.1;
    }
    let report = ReportDescriptor {
        report_id,
        timeout,
        raw,
        data_points,
    };
    Ok((report, rest))
}

fn data_point(buf: &[u8]) -> Result<(DataPoint<'_>, &[u8])> {
    let mut r = Reader::new(buf);
    let options = r.u8()?;
    let cluster_id = r.u16()?;
    let manufacturer_id = if options & MANUFACTURER_PRESENT != 0 {
        Some(r.u16()?)
    } else {
        None
    };
    let header = buf.len() - r.remaining();
    let mut rest = r.rest();
    for _ in 0..=(options & RECORDS_MASK) {
        rest = attribute(rest)?.1;
    }
    let (_, tail) = split(buf, header)?;
    let (records, _) = split(tail, tail.len() - rest.len())?;
    let dp = DataPoint {
        server: options & SERVER_CLUSTER != 0,
        cluster_id,
        manufacturer_id,
        records,
    };
    Ok((dp, rest))
}

fn attribute(buf: &[u8]) -> Result<(AttributeRecord<'_>, &[u8])> {
    let mut r = Reader::new(buf);
    let attribute_id = r.u16()?;
    let data_type = r.u8()?;
    let options = r.u8()?;
    let mut tail = Reader::new(r.bytes(usize::from(options & REMAINING_MASK))?);
    let offset = if options & REPORTED != 0 {
        Some(tail.u8()?)
    } else {
        None
    };
    let value = tail.rest();
    if (options & VALUE_PRESENT != 0) == value.is_empty() {
        return Err(Error::Unsupported);
    }
    let record = AttributeRecord {
        attribute_id,
        data_type,
        offset,
        value: (!value.is_empty()).then_some(value),
    };
    Ok((record, r.rest()))
}

#[cfg(test)]
#[path = "app_description/tests.rs"]
pub(crate) mod tests;
