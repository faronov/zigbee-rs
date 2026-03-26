//! Discover Attributes command (0x0C) and Discover Attributes Response (0x0D).

use crate::AttributeId;
use crate::data_types::ZclDataType;

/// Maximum attributes returned in a single discover response.
pub const MAX_DISCOVER: usize = 16;

/// Discover Attributes request.
#[derive(Debug, Clone)]
pub struct DiscoverAttributesRequest {
    /// Start attribute identifier.
    pub start_id: AttributeId,
    /// Maximum number of attribute IDs to return.
    pub max_results: u8,
}

/// A single entry in the Discover Attributes Response.
#[derive(Debug, Clone)]
pub struct DiscoverAttributeInfo {
    pub id: AttributeId,
    pub data_type: ZclDataType,
}

/// Discover Attributes Response.
#[derive(Debug, Clone)]
pub struct DiscoverAttributesResponse {
    /// `true` when the entire attribute list has been returned.
    pub complete: bool,
    pub attributes: heapless::Vec<DiscoverAttributeInfo, MAX_DISCOVER>,
}

impl DiscoverAttributesRequest {
    /// Parse from ZCL payload (2 bytes start_id + 1 byte max).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 3 {
            return None;
        }
        Some(Self {
            start_id: AttributeId(u16::from_le_bytes([data[0], data[1]])),
            max_results: data[2],
        })
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 3 {
            return 0;
        }
        let b = self.start_id.0.to_le_bytes();
        buf[0] = b[0];
        buf[1] = b[1];
        buf[2] = self.max_results;
        3
    }
}

impl DiscoverAttributesResponse {
    /// Serialize the response to ZCL payload bytes.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        buf[0] = if self.complete { 1 } else { 0 };
        let mut pos = 1;
        for info in &self.attributes {
            // Need 2 (id) + 1 (type) = 3 bytes
            if pos + 3 > buf.len() {
                break;
            }
            let b = info.id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
            buf[pos] = info.data_type as u8;
            pos += 1;
        }
        pos
    }
}

/// Process a discover request using a type-erased attribute store.
pub fn process_discover_dyn(
    store: &dyn crate::clusters::AttributeStoreAccess,
    request: &DiscoverAttributesRequest,
) -> DiscoverAttributesResponse {
    let ids = store.all_ids();
    let mut attributes = heapless::Vec::new();
    let max = request.max_results as usize;
    let mut count = 0;
    let mut complete = true;

    for id in &ids {
        if id.0 >= request.start_id.0 {
            if count >= max {
                complete = false;
                break;
            }
            if let Some(def) = store.find(*id) {
                let _ = attributes.push(DiscoverAttributeInfo {
                    id: *id,
                    data_type: def.data_type,
                });
                count += 1;
            }
        }
    }

    DiscoverAttributesResponse {
        complete,
        attributes,
    }
}
