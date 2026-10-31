# Writing Custom Clusters

When the standard ZCL clusters don't cover your needs, you can implement custom clusters using the same traits the built-in clusters use. This chapter walks through creating a custom sensor cluster from scratch.

---

## The Cluster Trait

Every cluster in zigbee-rs implements the `Cluster` trait:

```rust
pub trait Cluster {
    /// The cluster identifier.
    fn cluster_id(&self) -> ClusterId;

    /// Handle a cluster-specific command.
    /// Returns response payload on success, or a ZCL status on failure.
    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus>;

    /// Like `handle_command`, but with the sender's IEEE address (if known).
    /// The runtime always calls this; the default ignores the source.
    fn handle_command_from(
        &mut self,
        src_ieee: Option<u64>,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        let _ = src_ieee;
        self.handle_command(cmd_id, payload)
    }

    /// Immutable access to the attribute store.
    fn attributes(&self) -> &dyn AttributeStoreAccess;

    /// Mutable access to the attribute store.
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess;

    /// Command IDs this cluster can receive (client→server).
    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::new()
    }

    /// Scene table / scene-capable hooks (default: none).
    fn cluster_role(&mut self) -> ClusterRole<'_> {
        ClusterRole::None
    }

    /// Command IDs this cluster can generate (server→client).
    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::new()
    }

    /// Restore every attribute to its default (Basic ResetToFactoryDefaults).
    fn reset_to_factory_defaults(&mut self);
}
```

`reset_to_factory_defaults()` has no default body, so every cluster must implement it.

---

## The Attribute Store

`AttributeStore<N>` is a fixed-capacity, `#![no_std]`-friendly container for attribute values. The const generic `N` determines how many attributes the cluster can hold.

### Attribute Definition

Each attribute needs a definition with metadata:

```rust
use zigbee_zcl::attribute::{AttributeAccess, AttributeDefinition};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::AttributeId;

let def = AttributeDefinition {
    id: AttributeId(0x0000),
    data_type: ZclDataType::U16,
    access: AttributeAccess::Reportable,
    name: "MyMeasuredValue",
};
```

### Access Modes

| Mode | Reads | Writes | Reporting |
|------|-------|--------|-----------|
| `ReadOnly` | ✓ | ✗ | ✓ |
| `WriteOnly` | ✗ | ✓ | ✗ |
| `ReadWrite` | ✓ | ✓ | ✓ |
| `Reportable` | ✓ | ✗ | ✓ |

### Store Operations

```rust
use zigbee_zcl::attribute::AttributeStore;

let mut store = AttributeStore::<8>::new();

// Register attribute with initial value:
store.register(def, ZclValue::U16(0))?;

// Read (returns Option<&ZclValue>):
let val = store.get(AttributeId(0x0000));

// Write (respects access control + type checking):
store.set(AttributeId(0x0000), ZclValue::U16(42))?;

// Check whether set() would succeed, without writing:
store.validate_set(AttributeId(0x0000), &ZclValue::U16(42))?;

// Write bypassing access control and type checks (for server-side updates):
store.set_raw(AttributeId(0x0000), ZclValue::U16(42))?;
```

The runtime calls `set()` for remote Write Attributes commands (which checks `is_writable()`). Your application code should use `set_raw()` to update values that are read-only to the network but set by the firmware.

---

## Type-Erased Access Traits

The `Cluster` trait returns attribute stores through two type-erased traits:

```rust
/// Read access
pub trait AttributeStoreAccess {
    fn get(&self, id: AttributeId) -> Option<&ZclValue>;
    fn find(&self, id: AttributeId) -> Option<&AttributeDefinition>;
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool;
    fn all_ids(&self) -> heapless::Vec<AttributeId, 32>;
}

/// Write access
pub trait AttributeStoreMutAccess {
    fn set(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus>;
    fn set_raw(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus>;
    fn find(&self, id: AttributeId) -> Option<&AttributeDefinition>;
    /// Dry-run of `set()`; used to preflight Write Attributes Undivided.
    fn validate_set(&self, id: AttributeId, value: &ZclValue) -> Result<(), ZclStatus>;
}
```

Both traits are automatically implemented for any `AttributeStore<N>`, so you just return `&self.store` and `&mut self.store` from your cluster.

---

## Example: Custom UV Index Sensor

Here's a complete custom cluster for a UV index sensor:

```rust
use zigbee_zcl::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use zigbee_zcl::clusters::{
    AttributeStoreAccess, AttributeStoreMutAccess, Cluster,
};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::{AttributeId, ClusterId, CommandId, ZclStatus};

// Cluster ID — use manufacturer-specific range (0xFC00–0xFFFF)
pub const CLUSTER_UV_INDEX: ClusterId = ClusterId(0xFC01);

// Attribute IDs
pub const ATTR_UV_INDEX: AttributeId = AttributeId(0x0000);
pub const ATTR_UV_INDEX_MIN: AttributeId = AttributeId(0x0001);
pub const ATTR_UV_INDEX_MAX: AttributeId = AttributeId(0x0002);

// Command IDs
pub const CMD_RESET_MAX: CommandId = CommandId(0x00);

pub struct UvIndexCluster {
    store: AttributeStore<4>,
}

impl UvIndexCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UV_INDEX,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "UVIndex",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UV_INDEX_MIN,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MinUVIndex",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UV_INDEX_MAX,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MaxUVIndex",
            },
            ZclValue::U8(0),
        );
        Self { store }
    }

    /// Update the UV index reading.
    pub fn set_uv_index(&mut self, index: u8) {
        let _ = self.store.set_raw(ATTR_UV_INDEX, ZclValue::U8(index));

        // Track maximum
        if let Some(ZclValue::U8(max)) = self.store.get(ATTR_UV_INDEX_MAX) {
            if index > *max {
                let _ = self
                    .store
                    .set_raw(ATTR_UV_INDEX_MAX, ZclValue::U8(index));
            }
        }
    }
}

impl Cluster for UvIndexCluster {
    fn cluster_id(&self) -> ClusterId {
        CLUSTER_UV_INDEX
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_RESET_MAX => {
                let _ = self
                    .store
                    .set_raw(ATTR_UV_INDEX_MAX, ZclValue::U8(0));
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[CMD_RESET_MAX.0]).unwrap_or_default()
    }

    fn reset_to_factory_defaults(&mut self) {
        let _ = self.store.set_raw(ATTR_UV_INDEX, ZclValue::U8(0));
        let _ = self.store.set_raw(ATTR_UV_INDEX_MIN, ZclValue::U8(0));
        let _ = self.store.set_raw(ATTR_UV_INDEX_MAX, ZclValue::U8(0));
    }
}
```

---

## Registering with the Device Builder

Declare the cluster ID on an endpoint in the device builder (this drives the
Simple Descriptor), then hand the cluster instance to the runtime through a
`ClusterRef` slice:

```rust
use zigbee_runtime::ClusterRef;
use zigbee_zcl::DeviceId;

let builder = builder.endpoint(1, 0x0104, DeviceId(0x0302), |ep| {
    ep.cluster_server(CLUSTER_UV_INDEX) // or your custom device ID above
});

let mut uv_sensor = UvIndexCluster::new();

// In the main loop:
let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut uv_sensor }];
device.process_incoming(&indication, &mut clusters).await;
device.tick(elapsed_secs, &mut clusters).await;
```

---

## Handling Commands

The `handle_command` method receives:
- `cmd_id` — the cluster-specific command ID (0x00, 0x01, etc.)
- `payload` — raw bytes after the ZCL header

The runtime actually calls `handle_command_from()`, passing the sender's IEEE
address when known; override it if a command must be authenticated by source
(as IAS Zone does for Zone Enroll Response).

Return values:
- `Ok(Vec::new())` — success, runtime sends a Default Response (unicast only,
  unless the request set Disable Default Response)
- `Ok(vec_with_data)` — success, runtime sends a server→client cluster-specific
  frame carrying the payload, **using the same command ID as the request**
  (only Identify Query is mapped to a different response ID)
- `Err(ZclStatus)` — failure, runtime sends a Default Response with that status

### Parsing Payloads

Parse command payloads manually from the `&[u8]` slice:

```rust
fn handle_command(
    &mut self,
    cmd_id: CommandId,
    payload: &[u8],
) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
    match cmd_id {
        CommandId(0x00) => {
            if payload.len() < 3 {
                return Err(ZclStatus::MalformedCommand);
            }
            let param1 = payload[0];
            let param2 = u16::from_le_bytes([payload[1], payload[2]]);
            // Process...
            Ok(heapless::Vec::new())
        }
        _ => Err(ZclStatus::UnsupClusterCommand),
    }
}
```

---

## What the Runtime Handles Automatically

When you implement `Cluster`, the runtime provides these features for free:

| Feature | How It Works |
|---------|--------------|
| **Read Attributes** | Calls `attributes().get()` for each requested ID |
| **Write Attributes** | Calls `attributes_mut().set()` with access control |
| **Write Undivided** | Validates all writes first, then applies atomically |
| **Configure Reporting** | Stores config in `ReportingEngine` |
| **Report Attributes** | Checks values via `attributes()` on each tick |
| **Discover Attributes** | Enumerates from `attributes().all_ids()` |
| **Discover Commands** | Calls `received_commands()` / `generated_commands()` |
| **Default Response** | Generated for commands without a specific response |

You only need to implement `handle_command()` for **cluster-specific** commands. Everything else is automatic.
