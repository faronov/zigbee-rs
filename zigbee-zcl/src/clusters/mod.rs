//! Standard ZCL cluster implementations.
//!
//! Each sub-module implements one cluster, exposing attribute constants,
//! command IDs, and a struct that implements the [`Cluster`] trait.

pub mod alarms;
#[cfg(feature = "float32")]
pub mod analog_input;
#[cfg(feature = "float32")]
pub mod analog_output;
#[cfg(feature = "float32")]
pub mod analog_value;
pub mod ballast_config;
pub mod basic;
pub mod binary_input;
pub mod binary_output;
pub mod binary_value;
#[cfg(feature = "float32")]
pub mod carbon_dioxide;
pub mod color_control;
pub mod device_temp_config;
pub mod diagnostics;
pub mod door_lock;
pub mod electrical;
pub mod fan_control;
pub mod flow_measurement;
pub mod green_power;
pub mod groups;
pub mod humidity;
pub mod ias_ace;
pub mod ias_wd;
pub mod ias_zone;
pub mod identify;
pub mod illuminance;
pub mod illuminance_level;
pub mod level_control;
pub mod metering;
pub mod multistate_input;
pub mod occupancy;
pub mod on_off;
pub mod on_off_switch_config;
pub mod ota;
pub mod ota_image;
#[cfg(feature = "float32")]
pub mod pm25;
pub mod poll_control;
pub mod power_config;
pub mod pressure;
pub mod scenes;
pub mod soil_moisture;
pub mod temperature;
pub mod thermostat;
pub mod thermostat_ui;
pub mod time;
pub mod touchlink;
pub mod window_covering;

use crate::attribute::AttributeStore;
use crate::{ClusterId, CommandId, ZclStatus};

/// Trait that all cluster implementations must satisfy.
pub trait Cluster {
    /// The cluster identifier for this cluster.
    fn cluster_id(&self) -> ClusterId;

    /// Handle a cluster-specific command.
    ///
    /// Returns the response payload on success, or a ZCL status on failure.
    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus>;

    /// Immutable access to the cluster's attribute store.
    fn attributes(&self) -> &dyn AttributeStoreAccess;

    /// Mutable access to the cluster's attribute store.
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess;

    /// Command IDs this cluster can receive (client→server).
    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::new()
    }

    /// Command IDs this cluster can generate (server→client).
    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::new()
    }

    /// Restore this cluster's writable application attributes and transient
    /// command/effect state to their constructor (factory) defaults, as part
    /// of BDB 3.0.1 / ZCL Basic cluster `ResetToFactoryDefaults` handling.
    ///
    /// Implementations MUST NOT:
    /// - touch APS group or binding tables, network/security state, or any
    ///   commissioning/enrollment relationship (e.g. IAS CIE binding) —
    ///   those are reset only by the full BDB factory-reset procedure;
    /// - erase security-sensitive credential stores (e.g. Door Lock PIN
    ///   codes) or durable security frame counters/keys;
    /// - discard cumulative metering/energy registers or other durable
    ///   billing-relevant data;
    /// - clobber read-only identity, physical capability, or calibration
    ///   attributes supplied by the product at construction time (e.g.
    ///   manufacturer/model strings, sensor min/max range, zone/switch
    ///   type).
    ///
    /// Every in-tree implementation provides an explicit body — including a
    /// deliberate no-op for clusters with no resettable state — so that
    /// adding a new writable attribute or piece of transient command state
    /// forces an explicit decision here rather than a silent default.
    fn reset_to_factory_defaults(&mut self);
}

/// Type-erased read access to an attribute store.
pub trait AttributeStoreAccess {
    fn get(&self, id: crate::AttributeId) -> Option<&crate::data_types::ZclValue>;
    fn find(&self, id: crate::AttributeId) -> Option<&crate::attribute::AttributeDefinition>;
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool;
    /// Return all attribute IDs in the store (for discover attributes).
    fn all_ids(&self) -> heapless::Vec<crate::AttributeId, 32> {
        heapless::Vec::new() // default empty, overridden by real stores
    }
}

/// Type-erased write access to an attribute store.
pub trait AttributeStoreMutAccess {
    fn set(
        &mut self,
        id: crate::AttributeId,
        value: crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus>;
    fn set_raw(
        &mut self,
        id: crate::AttributeId,
        value: crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus>;
    /// Look up an attribute definition for validation (needed by Write Undivided).
    fn find(&self, id: crate::AttributeId) -> Option<&crate::attribute::AttributeDefinition>;
    /// Report whether a [`Self::set`] of `value` into `id` would succeed,
    /// without mutating the store.
    ///
    /// Write Attributes Undivided relies on this to honour its all-or-nothing
    /// contract: once `validate_set` returns `Ok(())` for every record, the
    /// apply phase's `set` calls must not fail, so no record can partially
    /// commit.
    ///
    /// The default reproduces the documented store contract enforced by
    /// [`AttributeStore::set`]: the attribute must exist
    /// (`UnsupportedAttribute`), be writable (`ReadOnly`), and the supplied
    /// value's data type must match the attribute's declared type
    /// (`InvalidDataType`). A store whose `set` can fail for any additional
    /// reason MUST override this method so it stays exactly consistent with
    /// `set`; otherwise an undivided write it validates may still commit part
    /// of a batch.
    fn validate_set(
        &self,
        id: crate::AttributeId,
        value: &crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus> {
        match self.find(id) {
            Some(def) => {
                if !def.access.is_writable() {
                    Err(ZclStatus::ReadOnly)
                } else if value.data_type() != def.data_type {
                    Err(ZclStatus::InvalidDataType)
                } else {
                    Ok(())
                }
            }
            None => Err(ZclStatus::UnsupportedAttribute),
        }
    }
}

impl<const N: usize> AttributeStoreAccess for AttributeStore<N> {
    fn get(&self, id: crate::AttributeId) -> Option<&crate::data_types::ZclValue> {
        self.get(id)
    }
    fn find(&self, id: crate::AttributeId) -> Option<&crate::attribute::AttributeDefinition> {
        self.find(id)
    }
    fn len(&self) -> usize {
        self.len()
    }
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn all_ids(&self) -> heapless::Vec<crate::AttributeId, 32> {
        let mut ids = heapless::Vec::new();
        for attr in self.iter() {
            let _ = ids.push(attr.definition.id);
        }
        ids
    }
}

impl<const N: usize> AttributeStoreMutAccess for AttributeStore<N> {
    fn set(
        &mut self,
        id: crate::AttributeId,
        value: crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus> {
        self.set(id, value)
    }
    fn set_raw(
        &mut self,
        id: crate::AttributeId,
        value: crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus> {
        self.set_raw(id, value)
    }
    fn find(&self, id: crate::AttributeId) -> Option<&crate::attribute::AttributeDefinition> {
        self.find(id)
    }
    fn validate_set(
        &self,
        id: crate::AttributeId,
        value: &crate::data_types::ZclValue,
    ) -> Result<(), ZclStatus> {
        // Exact preflight for the in-tree store: delegates to the same
        // `writable_slot` resolver used by `set`, so validation and the write
        // it guards can never diverge.
        self.validate_set(id, value)
    }
}
