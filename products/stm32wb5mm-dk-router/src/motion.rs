//! IMU motion as ZCL Occupancy Sensing (0x0406) with a writable, durable
//! hold time.
//!
//! The DK reports *physical contact* occupancy: the board is moved or
//! touched (ISM330DHCX), not a person in the room. The hold after the last
//! motion is `PIROccupiedToUnoccupiedDelay` (0x0010), the attribute ZHA and
//! Zigbee2MQTT expose as the occupancy timeout. Nothing in this sensor
//! delays the unoccupied → occupied transition, so the PIR/ultrasonic
//! delay and threshold attributes (0x0011, 0x0012, 0x0020, 0x0021) are not
//! declared and read as unsupported.
//!
//! [`MotionHoldPersistence`] keeps the written hold across resets in a
//! [`SettingsStore`], bound to the network's extended PAN ID like the
//! runtime's reporting persistence, so another network starts from
//! [`DEFAULT_HOLD_SECS`].

use zigbee_runtime::settings_journal::{SettingsError, SettingsRecord, SettingsStore};
use zigbee_zcl::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use zigbee_zcl::clusters::occupancy::{
    ATTR_OCCUPANCY, ATTR_OCCUPANCY_SENSOR_TYPE, ATTR_OCCUPANCY_SENSOR_TYPE_BITMAP,
    ATTR_PIR_O_TO_U_DELAY, SENSOR_TYPE_PHYSICAL_CONTACT,
};
use zigbee_zcl::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::{ClusterId, CommandId, ZclStatus};

/// Hold after the last motion until a factory reset or a client write.
pub const DEFAULT_HOLD_SECS: u16 = 60;

/// Occupancy Sensing server of the DK: occupancy, sensor type and the
/// occupied → unoccupied delay.
pub struct MotionOccupancyCluster {
    store: AttributeStore<4>,
}

impl Default for MotionOccupancyCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionOccupancyCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::Reportable,
                name: "Occupancy",
            },
            ZclValue::Bitmap8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY_SENSOR_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "OccupancySensorType",
            },
            ZclValue::Enum8(SENSOR_TYPE_PHYSICAL_CONTACT),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY_SENSOR_TYPE_BITMAP,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "OccupancySensorTypeBitmap",
            },
            ZclValue::Bitmap8(1 << SENSOR_TYPE_PHYSICAL_CONTACT),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PIR_O_TO_U_DELAY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "PIROccupiedToUnoccupiedDelay",
            },
            ZclValue::U16(DEFAULT_HOLD_SECS),
        );
        Self { store }
    }

    /// Publish the occupancy reading (bit 0).
    pub fn set_occupied(&mut self, occupied: bool) {
        let _ = self
            .store
            .set_raw(ATTR_OCCUPANCY, ZclValue::Bitmap8(u8::from(occupied)));
    }

    pub fn occupied(&self) -> bool {
        matches!(self.store.get(ATTR_OCCUPANCY), Some(ZclValue::Bitmap8(b)) if b & 1 != 0)
    }

    /// Current hold after the last motion, seconds.
    pub fn hold_secs(&self) -> u16 {
        match self.store.get(ATTR_PIR_O_TO_U_DELAY) {
            Some(ZclValue::U16(secs)) => *secs,
            _ => DEFAULT_HOLD_SECS,
        }
    }

    /// Set the hold locally (restore from flash).
    pub fn set_hold_secs(&mut self, secs: u16) {
        let _ = self
            .store
            .set_raw(ATTR_PIR_O_TO_U_DELAY, ZclValue::U16(secs));
    }
}

impl Cluster for MotionOccupancyCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::OCCUPANCY
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    fn reset_to_factory_defaults(&mut self) {
        // Occupancy is a live reading; the sensor type is physical.
        self.set_hold_secs(DEFAULT_HOLD_SECS);
    }
}

/// A persisted hold, bound to the network it was written in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StoredHold {
    pub extended_pan_id: [u8; 8],
    pub hold_secs: u16,
}

/// Payload format of [`StoredHold`] in the settings record.
const RECORD_FORMAT: u8 = 1;
const RECORD_LEN: usize = 1 + 8 + 2;

impl StoredHold {
    /// No network: written by a factory reset so nothing is restored.
    pub const FACTORY: Self = Self {
        extended_pan_id: [0; 8],
        hold_secs: DEFAULT_HOLD_SECS,
    };

    fn encode(&self) -> [u8; RECORD_LEN] {
        let mut out = [0u8; RECORD_LEN];
        out[0] = RECORD_FORMAT;
        out[1..9].copy_from_slice(&self.extended_pan_id);
        out[9..11].copy_from_slice(&self.hold_secs.to_le_bytes());
        out
    }

    fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != RECORD_LEN || payload[0] != RECORD_FORMAT {
            return None;
        }
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(&payload[1..9]);
        Some(Self {
            extended_pan_id,
            hold_secs: u16::from_le_bytes([payload[9], payload[10]]),
        })
    }
}

/// What [`MotionHoldPersistence::sync`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldSync {
    Unchanged,
    /// The stored hold of this network was applied to the cluster.
    Restored(u16),
    /// The cluster's hold was written to flash.
    Stored(u16),
}

/// Restore-once / write-on-change policy for the motion hold.
pub struct MotionHoldPersistence<S> {
    store: S,
    /// Last record known to be in flash; `None` before the first load or
    /// when nothing valid is stored.
    stored: Option<StoredHold>,
    loaded: bool,
    /// Network the hold was last restored for (cleared while not joined).
    restored_for: Option<[u8; 8]>,
}

impl<S: SettingsStore> MotionHoldPersistence<S> {
    pub const fn new(store: S) -> Self {
        Self {
            store,
            stored: None,
            loaded: false,
            restored_for: None,
        }
    }

    fn load(&mut self) -> Result<(), SettingsError> {
        if !self.loaded {
            self.stored = self
                .store
                .load()?
                .and_then(|record| StoredHold::decode(record.as_slice()));
            self.loaded = true;
        }
        Ok(())
    }

    fn write(&mut self, hold: StoredHold) -> Result<(), SettingsError> {
        self.store.store(&SettingsRecord::new(&hold.encode())?)?;
        self.stored = Some(hold);
        Ok(())
    }

    /// Call periodically with the joined network's extended PAN ID (`None`
    /// while not joined). After joining, applies the hold stored for that
    /// network once; afterwards writes the cluster's hold when it changed.
    pub fn sync(
        &mut self,
        extended_pan_id: Option<[u8; 8]>,
        cluster: &mut MotionOccupancyCluster,
    ) -> Result<HoldSync, SettingsError> {
        let Some(extended_pan_id) = extended_pan_id else {
            self.restored_for = None;
            return Ok(HoldSync::Unchanged);
        };
        self.load()?;
        if self.restored_for != Some(extended_pan_id) {
            self.restored_for = Some(extended_pan_id);
            if let Some(stored) = self.stored
                && stored.extended_pan_id == extended_pan_id
            {
                cluster.set_hold_secs(stored.hold_secs);
                return Ok(HoldSync::Restored(stored.hold_secs));
            }
        }
        let current = StoredHold {
            extended_pan_id,
            hold_secs: cluster.hold_secs(),
        };
        // What a reboot would restore for this network: its own record, or
        // the default. Only a difference needs a flash write.
        let restorable = match self.stored {
            Some(stored) if stored.extended_pan_id == extended_pan_id => stored.hold_secs,
            _ => DEFAULT_HOLD_SECS,
        };
        if current.hold_secs == restorable {
            return Ok(HoldSync::Unchanged);
        }
        self.write(current)?;
        Ok(HoldSync::Stored(current.hold_secs))
    }

    /// Forget the stored hold (factory reset): the next network starts from
    /// [`DEFAULT_HOLD_SECS`].
    pub fn forget(&mut self) -> Result<(), SettingsError> {
        self.load()?;
        if self.stored.is_none() || self.stored == Some(StoredHold::FACTORY) {
            return Ok(());
        }
        self.write(StoredHold::FACTORY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::settings_journal::RamSettingsStore;

    const NET_A: [u8; 8] = [0xA1; 8];
    const NET_B: [u8; 8] = [0xB2; 8];

    #[test]
    fn cluster_declares_only_the_implemented_attributes() {
        let mut cluster = MotionOccupancyCluster::new();
        let attributes = cluster.attributes();
        let ids: heapless::Vec<u16, 8> = attributes.all_ids().iter().map(|id| id.0).collect();
        assert_eq!(ids.as_slice(), &[0x0000, 0x0001, 0x0002, 0x0010]);
        assert_eq!(
            attributes.get(ATTR_OCCUPANCY_SENSOR_TYPE),
            Some(&ZclValue::Enum8(3))
        );
        assert_eq!(cluster.hold_secs(), DEFAULT_HOLD_SECS);
        let attributes = cluster.attributes_mut();
        assert_eq!(
            attributes.validate_set(ATTR_PIR_O_TO_U_DELAY, &ZclValue::U16(5)),
            Ok(())
        );
        assert_eq!(
            attributes.validate_set(ATTR_OCCUPANCY, &ZclValue::Bitmap8(1)),
            Err(ZclStatus::ReadOnly)
        );
        assert_eq!(
            attributes.validate_set(ATTR_PIR_O_TO_U_DELAY, &ZclValue::U8(5)),
            Err(ZclStatus::InvalidDataType)
        );
    }

    #[test]
    fn factory_defaults_restore_the_hold_and_keep_the_reading() {
        let mut cluster = MotionOccupancyCluster::new();
        cluster.set_occupied(true);
        cluster
            .attributes_mut()
            .set(ATTR_PIR_O_TO_U_DELAY, ZclValue::U16(5))
            .unwrap();
        assert_eq!(cluster.hold_secs(), 5);
        cluster.reset_to_factory_defaults();
        assert_eq!(cluster.hold_secs(), DEFAULT_HOLD_SECS);
        assert!(cluster.occupied());
    }

    #[test]
    fn written_hold_is_stored_and_restored_for_the_same_network() {
        let mut persistence = MotionHoldPersistence::new(RamSettingsStore::new());
        let mut cluster = MotionOccupancyCluster::new();
        // Not joined: nothing happens.
        assert_eq!(
            persistence.sync(None, &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        // Joined with the default: no write.
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        cluster.set_hold_secs(15);
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Stored(15))
        );
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Unchanged)
        );

        // Reboot: same flash, fresh cluster.
        let store = persistence.store;
        assert_eq!(store.writes(), 1);
        let mut persistence = MotionHoldPersistence::new(store);
        let mut cluster = MotionOccupancyCluster::new();
        assert_eq!(
            persistence.sync(None, &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Restored(15))
        );
        assert_eq!(cluster.hold_secs(), 15);
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        assert_eq!(persistence.store.writes(), 1);
    }

    #[test]
    fn another_network_starts_from_the_default() {
        let mut persistence = MotionHoldPersistence::new(RamSettingsStore::new());
        let mut cluster = MotionOccupancyCluster::new();
        cluster.set_hold_secs(15);
        persistence.sync(Some(NET_A), &mut cluster).unwrap();

        let mut cluster = MotionOccupancyCluster::new();
        persistence.sync(None, &mut cluster).unwrap();
        // NET_B: NET_A's hold is not applied, and its default needs no write.
        assert_eq!(
            persistence.sync(Some(NET_B), &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        assert_eq!(cluster.hold_secs(), DEFAULT_HOLD_SECS);
        cluster.set_hold_secs(30);
        assert_eq!(
            persistence.sync(Some(NET_B), &mut cluster),
            Ok(HoldSync::Stored(30))
        );
        assert_eq!(persistence.store.writes(), 2);
    }

    #[test]
    fn factory_reset_forgets_the_hold() {
        let mut persistence = MotionHoldPersistence::new(RamSettingsStore::new());
        // Nothing stored: nothing to forget.
        persistence.forget().unwrap();
        assert_eq!(persistence.store.writes(), 0);

        let mut cluster = MotionOccupancyCluster::new();
        cluster.set_hold_secs(15);
        persistence.sync(Some(NET_A), &mut cluster).unwrap();
        persistence.forget().unwrap();
        persistence.forget().unwrap();
        assert_eq!(persistence.store.writes(), 2);

        // Rejoining the same network does not bring the old hold back.
        let store = persistence.store;
        let mut persistence = MotionHoldPersistence::new(store);
        let mut cluster = MotionOccupancyCluster::new();
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        assert_eq!(cluster.hold_secs(), DEFAULT_HOLD_SECS);
    }

    #[test]
    fn foreign_or_malformed_records_are_ignored() {
        let mut store = RamSettingsStore::new();
        store
            .store(&SettingsRecord::new(&[9, 1, 2, 3]).unwrap())
            .unwrap();
        let mut persistence = MotionHoldPersistence::new(store);
        let mut cluster = MotionOccupancyCluster::new();
        assert_eq!(
            persistence.sync(Some(NET_A), &mut cluster),
            Ok(HoldSync::Unchanged)
        );
        assert_eq!(cluster.hold_secs(), DEFAULT_HOLD_SECS);
    }

    #[test]
    fn record_encoding_round_trips() {
        let hold = StoredHold {
            extended_pan_id: [1, 2, 3, 4, 5, 6, 7, 8],
            hold_secs: 0xBEEF,
        };
        assert_eq!(StoredHold::decode(&hold.encode()), Some(hold));
        assert_eq!(StoredHold::decode(&hold.encode()[..10]), None);
    }
}
