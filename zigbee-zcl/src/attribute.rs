//! ZCL attribute definitions, access control, and a fixed-capacity attribute store.

use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ZclStatus};

/// Attribute access mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttributeAccess {
    ReadOnly,
    WriteOnly,
    ReadWrite,
    /// Read-only but can be included in attribute reports.
    Reportable,
}

impl AttributeAccess {
    /// Whether reads are allowed.
    pub fn is_readable(&self) -> bool {
        matches!(self, Self::ReadOnly | Self::ReadWrite | Self::Reportable)
    }

    /// Whether writes are allowed.
    pub fn is_writable(&self) -> bool {
        matches!(self, Self::WriteOnly | Self::ReadWrite)
    }

    /// Whether the attribute is reportable.
    pub fn is_reportable(&self) -> bool {
        matches!(self, Self::Reportable | Self::ReadWrite | Self::ReadOnly)
    }
}

/// Static metadata about an attribute (constant, lives in flash).
#[derive(Debug, Clone, Copy)]
pub struct AttributeDefinition {
    pub id: AttributeId,
    pub data_type: ZclDataType,
    pub access: AttributeAccess,
    pub name: &'static str,
}

/// An attribute entry holding definition metadata and a current value.
#[derive(Debug, Clone)]
pub struct AttributeValue {
    pub definition: AttributeDefinition,
    pub value: ZclValue,
}

/// Resolve the slot index of a writable attribute in `attrs` that will accept
/// `value`, returning the exact [`ZclStatus`] that [`AttributeStore::set`]
/// would produce on failure (`UnsupportedAttribute` / `ReadOnly` /
/// `InvalidDataType`).
///
/// Free function over the attribute *slice* (not the capacity-generic store),
/// so this resolver — shared by both `set` and `validate_set` — is compiled
/// exactly once instead of once per `AttributeStore<N>`. Keeping preflight and
/// write funnelled through the same resolver is what lets Write Attributes
/// Undivided guarantee its all-or-nothing contract.
#[inline(never)]
fn resolve_writable_slot(
    attrs: &[AttributeValue],
    id: AttributeId,
    value: &ZclValue,
) -> Result<usize, ZclStatus> {
    let idx = attrs
        .iter()
        .position(|a| a.definition.id == id)
        .ok_or(ZclStatus::UnsupportedAttribute)?;
    let def = &attrs[idx].definition;
    if !def.access.is_writable() {
        return Err(ZclStatus::ReadOnly);
    }
    if value.data_type() != def.data_type {
        return Err(ZclStatus::InvalidDataType);
    }
    Ok(idx)
}

/// Fixed-capacity store of attribute values using `heapless::Vec`.
///
/// The const generic `N` determines the maximum number of attributes.
#[derive(Debug, Clone)]
pub struct AttributeStore<const N: usize> {
    attrs: heapless::Vec<AttributeValue, N>,
}

impl<const N: usize> Default for AttributeStore<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> AttributeStore<N> {
    /// Create an empty store.
    pub const fn new() -> Self {
        Self {
            attrs: heapless::Vec::new(),
        }
    }

    /// Register an attribute with its initial value.
    pub fn register(
        &mut self,
        def: AttributeDefinition,
        initial: ZclValue,
    ) -> Result<(), ZclStatus> {
        self.attrs
            .push(AttributeValue {
                definition: def,
                value: initial,
            })
            .map_err(|_| ZclStatus::InsufficientSpace)
    }

    /// Look up the current value by attribute ID.
    pub fn get(&self, id: AttributeId) -> Option<&ZclValue> {
        self.attrs
            .iter()
            .find(|a| a.definition.id == id)
            .map(|a| &a.value)
    }

    /// Write a value to an attribute, respecting access control and type.
    pub fn set(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus> {
        // `set` and `validate_set` share the capacity-independent resolver so a
        // preflight validation can never disagree with the write it guards —
        // the guarantee Write Attributes Undivided relies on for atomicity.
        let idx = resolve_writable_slot(&self.attrs, id, &value)?;
        self.attrs[idx].value = value;
        Ok(())
    }

    /// Report whether [`Self::set`] would succeed for `value` without mutating
    /// the store. Returns the identical [`ZclStatus`] error `set` would return
    /// (`UnsupportedAttribute`, `ReadOnly`, or `InvalidDataType`), covering
    /// every way `set` can fail. Undivided writes preflight with this so a
    /// validated batch can be applied without any record failing part-way.
    pub fn validate_set(&self, id: AttributeId, value: &ZclValue) -> Result<(), ZclStatus> {
        resolve_writable_slot(&self.attrs, id, value).map(|_| ())
    }

    /// Unconditionally set a value (for internal / server-side updates that
    /// bypass access control, e.g. updating a MeasuredValue sensor reading).
    pub fn set_raw(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus> {
        let entry = self
            .attrs
            .iter_mut()
            .find(|a| a.definition.id == id)
            .ok_or(ZclStatus::UnsupportedAttribute)?;
        entry.value = value;
        Ok(())
    }

    /// Find the definition for an attribute.
    pub fn find(&self, id: AttributeId) -> Option<&AttributeDefinition> {
        self.attrs
            .iter()
            .find(|a| a.definition.id == id)
            .map(|a| &a.definition)
    }

    /// Iterate over all stored attribute values.
    pub fn iter(&self) -> impl Iterator<Item = &AttributeValue> {
        self.attrs.iter()
    }

    /// Number of registered attributes.
    pub fn len(&self) -> usize {
        self.attrs.len()
    }

    /// Whether the store has no attributes.
    pub fn is_empty(&self) -> bool {
        self.attrs.is_empty()
    }
}
