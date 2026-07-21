# BDB Commissioning

**Base Device Behavior (BDB)** is the Zigbee 3.0 specification that
standardises how devices join networks, form networks, and find each other.
Before BDB, every manufacturer invented its own commissioning process — some
used button sequences, others relied on proprietary apps.  BDB defines four
universal methods so that any Zigbee 3.0 device can join any Zigbee 3.0
network.

```text
┌──────────────────────────────────────┐
│  Application                         │
└──────────────┬───────────────────────┘
               │ BDB commissioning API
┌──────────────┴───────────────────────┐
│  BDB Layer (zigbee-bdb)              │
│  ├── state_machine: top-level FSM    │
│  ├── steering:      join network     │
│  ├── formation:     create network   │
│  ├── finding_binding: EZ-Mode F&B    │
│  ├── touchlink:     proximity comm.  │
│  └── attributes:    BDB attributes   │
└──────────────┬───────────────────────┘
               │ ZDP services / NLME-*
┌──────────────┴───────────────────────┐
│  ZDO Layer (zigbee-zdo)              │
└──────────────────────────────────────┘
```

## The Four Commissioning Modes

| Mode | What it does | Who uses it |
|---|---|---|
| **Network Steering** | Join an existing network (or open it for others) | End Devices, Routers |
| **Network Formation** | Create a new PAN from scratch | Coordinators |
| **Finding & Binding** | Automatically create bindings between compatible endpoints | All device types |
| **Touchlink** | Join via physical proximity (Inter-PAN) | Lights, remotes |

Each mode can be enabled or disabled independently through a bitmask:

```rust
use zigbee_bdb::CommissioningMode;

// Enable only steering (most common for end devices)
let mode = CommissioningMode::STEERING;

// Enable steering + finding & binding
let mode = CommissioningMode::STEERING.or(CommissioningMode::FINDING_BINDING);

// Enable everything
let mode = CommissioningMode::ALL; // 0x0F

// Check what's enabled
if mode.contains(CommissioningMode::FORMATION) {
    println!("Formation is enabled");
}
```

The four bits:

| Constant | Value | Method |
|---|---|---|
| `CommissioningMode::TOUCHLINK` | `0x01` | Touchlink |
| `CommissioningMode::STEERING` | `0x02` | Network Steering |
| `CommissioningMode::FORMATION` | `0x04` | Network Formation |
| `CommissioningMode::FINDING_BINDING` | `0x08` | Finding & Binding |
| `CommissioningMode::ALL` | `0x0F` | All of the above |

## The `BdbLayer` Struct

`BdbLayer<M: MacDriver>` is the top-level type in zigbee-rs.  It owns the
ZDO layer (which owns APS, which owns NWK, which owns MAC) and drives the
commissioning state machine.

```rust
use zigbee_bdb::BdbLayer;
use zigbee_zdo::ZdoLayer;

let bdb = BdbLayer::new(zdo_layer);

// Access lower layers
let zdo = bdb.zdo();
let aps = bdb.zdo().aps();
let nwk = bdb.zdo().aps().nwk();

// Check state
println!("On network: {}", bdb.is_on_network());
println!("State: {:?}", bdb.state());
```

### Key Accessors

| Method | Returns | Purpose |
|---|---|---|
| `bdb.zdo()` | `&ZdoLayer<M>` | Access ZDO and below |
| `bdb.zdo_mut()` | `&mut ZdoLayer<M>` | Mutable ZDO access |
| `bdb.attributes()` | `&BdbAttributes` | Read BDB attributes |
| `bdb.attributes_mut()` | `&mut BdbAttributes` | Configure BDB behavior |
| `bdb.state()` | `&BdbState` | Current state machine state |
| `bdb.is_on_network()` | `bool` | Whether device has joined |

## The State Machine

BDB commissioning follows a strict state machine.  When you call
`bdb.commission()`, it runs each enabled method in order, skipping any that
aren't available for the device type:

```text
                    ┌──────────┐
         ┌─────────►│   Idle   │◄────────────────┐
         │          └────┬─────┘                  │
         │               │ commission()           │
         │          ┌────▼──────────┐             │
         │          │ Initializing  │             │
         │          └────┬──────────┘             │
         │               │                        │
         │       ┌───────▼────────┐               │
         │  TL?  │   Touchlink    │──► fail ──┐   │
         │       └───────┬────────┘           │   │
         │               │ skip/done          │   │
         │       ┌───────▼────────┐           │   │
         │  NS?  │ NetworkSteering│──► fail ──┤   │
         │       └───────┬────────┘           │   │
         │               │ skip/done          │   │
         │       ┌───────▼────────┐           │   │
         │  NF?  │NetworkFormation│──► fail ──┤   │
         │       └───────┬────────┘           │   │
         │               │ skip/done          │   │
         │       ┌───────▼────────┐           │   │
         │  FB?  │FindingBinding  │──► fail ──┘   │
         │       └───────┬────────┘               │
         │               │                        │
         └───────────────┴────────────────────────┘
```

### `BdbState`

```rust
pub enum BdbState {
    Idle,              // No commissioning in progress
    Initializing,      // Running BDB initialization
    NetworkSteering,   // Scanning / joining a network
    NetworkFormation,  // Creating a new PAN
    FindingBinding,    // EZ-Mode automatic binding
    Touchlink,         // Proximity commissioning
}
```

### Device Type Capabilities

Not every device can use every mode.  The `initialize()` method sets the
capability mask automatically:

| Device Type | Available Modes |
|---|---|
| **Coordinator** | Steering + Formation + Finding & Binding |
| **Router** | Steering + Finding & Binding + Touchlink |
| **End Device** | Steering + Finding & Binding + Touchlink |

The requested mode is intersected with the capability mask to produce the
*effective* commissioning mode.  If you request Formation on an End Device,
it is silently skipped.

## Initialization

Before any commissioning, you must call `initialize()` once after power-on:

```rust
// Initialize BDB — sets capabilities, syncs on-network state
bdb.initialize()?;
```

This performs:
1. Reset of lower layers (`NLME-RESET`)
2. Detection of device type (Coordinator / Router / End Device)
3. Setting `node_commissioning_capability` based on device type
4. Syncing `node_is_on_a_network` with NWK layer state

## Network Steering

Network Steering is the most common commissioning method.  Its behavior
depends on whether the device is already on a network.

### Not on a Network — Join

When `node_is_on_a_network` is `false`, steering performs a full join sequence:

```text
1. Scan primary channels (11, 15, 20, 25) for open networks
   └── NLME-NETWORK-DISCOVERY
2. Filter by extended PAN ID (if configured)
3. Join the best-LQI network with permit-joining enabled
   └── NLME-JOIN
4. Broadcast Device_annce
5. Wait for Transport-Key from Trust Center
   └── Poll parent via MAC Data Request
6. Send APSME-REQUEST-KEY for unique TC link key
7. Re-broadcast Device_annce (now secured)
```

If primary channels yield no results, secondary channels (all other 2.4 GHz
channels) are scanned.

```rust
// Configure and run steering
bdb.attributes_mut().commissioning_mode = CommissioningMode::STEERING;
bdb.attributes_mut().primary_channel_set = BDB_PRIMARY_CHANNEL_SET;
bdb.attributes_mut().secondary_channel_set = BDB_SECONDARY_CHANNEL_SET;

// Option A: Run the full state machine (recommended)
bdb.commission().await?;

// Option B: Call steering directly
bdb.network_steering().await?;
```

The steering retry budget is controlled by `steering_attempts_remaining`
(default: 5).  Each call to `steer_off_network` decrements this counter.

### Already on a Network — Open for Joining

When `node_is_on_a_network` is `true`, steering opens the network so *other*
devices can join:

```text
1. Open local permit joining (180 seconds)
   └── NLME-PERMIT-JOINING
2. Broadcast Mgmt_Permit_Joining_req to all routers
```

For End Devices (which can't accept joins themselves), steering sends a
`Mgmt_Permit_Joining_req` to the coordinator.

## Network Formation

Network Formation creates a brand-new Zigbee PAN.  Only **coordinators** can
form networks.

```text
1. Verify this device is coordinator-capable
2. Form network on primary channels
   └── NLME-NETWORK-FORMATION (energy scan + selection)
3. If primary channels fail, try secondary channels
4. Configure Trust Center policies
5. Install NWK key
6. Open permit joining for 180 seconds
```

```rust
// Configure as coordinator
bdb.attributes_mut().commissioning_mode = CommissioningMode::FORMATION;

// Form the network
bdb.network_formation().await?;
```

After formation:
- `aps.aib().aps_designated_coordinator` is set to `true`
- `aps.aib().aps_trust_center_address` is set to the coordinator's IEEE address
- The NWK key is installed by the NWK layer
- Permit joining is opened for `BDB_MIN_COMMISSIONING_TIME` (180 seconds)

### Security Modes

Formation supports two security models:
- **Centralized** (default): The coordinator acts as the Trust Center and
  distributes the NWK key to all joining devices.
- **Distributed**: Routers form their own trust domain with no central TC
  (used in some ZLL/Touchlink scenarios).

## Finding & Binding (EZ-Mode)

Finding & Binding (F&B) automatically creates bindings between compatible
endpoints on different devices.  It uses the **Identify cluster** (0x0003) to
discover targets.

There are two roles:

### Initiator — The Device That Creates Bindings

The initiator broadcasts an **Identify Query** and waits for responses from
devices that are currently in Identify mode (e.g., LED blinking after a button
press).

```text
1. Broadcast Identify Query to 0xFFFD
2. Collect responses for 180 seconds (bdbcMinCommissioningTime)
3. For each responding target:
   a. Send Active_EP_req to get endpoint list
   b. Send Simple_Desc_req for each endpoint
   c. Match clusters:
      • Our output clusters ↔ their input clusters
      • Our input clusters ↔ their output clusters
   d. Create local binding + send ZDP Bind_req to remote
```

```rust
// Start F&B initiator on endpoint 1
bdb.finding_binding_initiator(1).await?;

// In your event loop, tick every second:
loop {
    // ... process incoming frames ...
    let completed = bdb.tick_finding_binding(1).await;
    if completed {
        println!("F&B finished!");
        break;
    }
    sleep(1_second).await;
}
```

The cluster matching algorithm:
- A binding is created when the initiator's **output** cluster matches the
  target's **input** cluster (client → server).
- And when the initiator's **input** cluster matches the target's **output**
  cluster (server → client).
- Both endpoints must share the same profile ID (or one must be the wildcard
  `0xFFFF`).
- If `commissioning_group_id` is not `0xFFFF`, group bindings are also created.

### Target — The Device That Gets Bound To

The target enters Identify mode and waits for an initiator to discover it:

```rust
// Enter F&B target mode on endpoint 1 (LED blinks for 180 seconds)
bdb.finding_binding_target(1).await?;
```

This sets `fb_target_request` to `Some((endpoint, 180))`.  Your runtime reads
this and writes the `IdentifyTime` attribute on the Identify cluster, which
makes the device respond to Identify Query broadcasts.

The target's normal APS/ZCL processing handles incoming `Simple_Desc_req`
and `Bind_req` from the initiator automatically.

## Touchlink

Touchlink (formerly ZLL commissioning) is a proximity-based method.  Devices
must be brought physically close together (RSSI threshold: **-40 dBm**).

```text
1. Initiator sends Scan Request via Inter-PAN on each primary channel
   └── Channels: 11, 15, 20, 25
2. Target responds if RSSI > -40 dBm
3. Initiator sends Network Start/Join Request
4. Target applies network parameters and joins
```

### Current Status

> **⚠️ Touchlink is currently a stub implementation.**
>
> Full Touchlink requires Inter-PAN frame support in the MAC layer, which is
> not yet implemented.  Calling `touchlink_commissioning()` returns
> `Err(BdbStatus::TouchlinkFailure)`.

Key types and constants are defined for future implementation:

```rust
use zigbee_bdb::touchlink::*;

// Touchlink primary channels
const TOUCHLINK_PRIMARY_CHANNELS: [u8; 4] = [11, 15, 20, 25];

// RSSI threshold for proximity detection
const TOUCHLINK_RSSI_THRESHOLD: i8 = -40; // dBm

// Pre-configured link key for key transport
const TOUCHLINK_PRECONFIGURED_LINK_KEY: [u8; 16] = [0xD0, ..., 0xDF];

// Command IDs (cluster 0x1000)
touchlink::command_id::SCAN_REQUEST          // 0x00
touchlink::command_id::SCAN_RESPONSE         // 0x01
touchlink::command_id::NETWORK_START_REQUEST // 0x10
touchlink::command_id::FACTORY_NEW_RESET     // 0x07
// ... and more
```

## BDB Attributes

The `BdbAttributes` struct controls all BDB behavior.  You configure these
before calling `commission()`:

```rust
pub struct BdbAttributes {
    /// Group ID for F&B group bindings (0xFFFF = disabled)
    pub commissioning_group_id: u16,

    /// Which commissioning modes to run
    pub commissioning_mode: CommissioningMode,

    /// Result of the last commissioning attempt
    pub commissioning_status: BdbCommissioningStatus,

    /// Whether this node is currently on a network
    pub node_is_on_a_network: bool,

    /// How this node's link key was obtained
    pub node_join_link_key_type: NodeJoinLinkKeyType,

    /// Primary channels to scan first (default: 11, 15, 20, 25)
    pub primary_channel_set: ChannelMask,

    /// Secondary channels if primary fails (default: all others)
    pub secondary_channel_set: ChannelMask,

    /// TC join timeout in seconds (default: 10)
    pub trust_center_node_join_timeout: u16,

    /// Whether TC requires link key exchange (default: true)
    pub trust_center_require_key_exchange: bool,

    /// Steering retry budget (default: 5)
    pub steering_attempts_remaining: u8,

    // ... internal fields
}
```

### Channel Sets

BDB defines two channel sets for scanning:

```rust
use zigbee_bdb::attributes::*;

// Primary: channels 11, 15, 20, 25 (fastest discovery)
BDB_PRIMARY_CHANNEL_SET   // ChannelMask(0x0210_8800)

// Secondary: all other 2.4 GHz channels
BDB_SECONDARY_CHANNEL_SET // ChannelMask(0x05EF_7000)

// Minimum commissioning time for F&B (180 seconds)
BDB_MIN_COMMISSIONING_TIME // 180
```

### Link Key Types

```rust
pub enum NodeJoinLinkKeyType {
    DefaultGlobalTrustCenterLinkKey = 0x00, // "ZigBeeAlliance09"
    IcDerivedTrustCenterLinkKey    = 0x01, // install code
    AppTrustCenterLinkKey          = 0x02, // pre-configured
    TouchlinkPreconfiguredLinkKey  = 0x03, // ZLL key
}
```

## `BdbStatus` — All Variants

| Variant | Value | Meaning |
|---|---|---|
| `Success` | `0x00` | Commissioning completed successfully |
| `InProgress` | `0x01` | Commissioning is currently running |
| `NotOnNetwork` | `0x02` | Operation requires network membership |
| `NotPermitted` | `0x03` | Not supported by this device type |
| `NoScanResponse` | `0x04` | No beacons received during steering |
| `FormationFailure` | `0x05` | Network formation failed |
| `SteeringFailure` | `0x06` | Steering failed after all retries |
| `NoIdentifyResponse` | `0x07` | No Identify Query response during F&B |
| `BindingTableFull` | `0x08` | Binding table full during F&B |
| `TouchlinkFailure` | `0x09` | Touchlink failed or not supported |
| `TargetFailure` | `0x0A` | Target not in identifying mode |
| `Timeout` | `0x0B` | Operation timed out |

## `BdbCommissioningStatus`

The `commissioning_status` attribute records the outcome of the last
commissioning attempt with finer granularity:

```rust
pub enum BdbCommissioningStatus {
    Success                   = 0x00,
    InProgress                = 0x01,
    NoNetwork                 = 0x02,
    TlTargetFailure           = 0x03,
    TlNotAddressAssignment    = 0x04,
    TlNoScanResponse          = 0x05,
    NotPermitted              = 0x06,
    SteeringFormationFailure  = 0x07,
    NoIdentifyQueryResponse   = 0x08,
    BindingTableFull          = 0x09,
    NoScanResponse            = 0x0A,
}
```

## Factory Reset

BDB provides a standardized factory reset procedure:

```rust
bdb.factory_reset().await?;
```

This performs:
1. Leave the current network (if joined)
2. Reset NWK + MAC layers (clears neighbor table, security, routing)
3. Clear APS binding table and group table
4. Reset all BDB attributes to defaults

After factory reset, the device is in a "fresh out of box" state and must be
commissioned again.

## Rejoin

When a device loses its parent or detects network problems, it can attempt a
rejoin:

```rust
// Attempt rejoin using stored NWK key
bdb.rejoin().await?;

// Or leave and rejoin (clean restart)
bdb.leave_and_rejoin().await?;
```

The rejoin procedure:
1. Scan the last-known channel for the previous network
2. Attempt `NLME-JOIN` with Rejoin method (uses stored NWK key)
3. Broadcast `Device_annce`
4. If rejoin fails, fall back to full Network Steering

## Complete Example: End Device Commissioning

Here's how a typical temperature sensor joins a network:

```rust
use zigbee_bdb::{BdbLayer, BdbStatus, CommissioningMode};
use zigbee_bdb::attributes::BDB_PRIMARY_CHANNEL_SET;

// 1. Create the stack (MAC → NWK → APS → ZDO → BDB)
let mac = MyMacDriver::new();
let nwk = NwkLayer::new(mac, DeviceType::EndDevice);
let aps = ApsLayer::new(nwk);
let zdo = ZdoLayer::new(aps);
let mut bdb = BdbLayer::new(zdo);

// 2. Register our endpoint (temperature sensor)
bdb.zdo_mut().register_endpoint(SimpleDescriptor {
    endpoint: 1,
    profile_id: 0x0104,               // Home Automation
    device_id: 0x0302,                // Temperature Sensor
    device_version: 1,
    input_clusters: vec![
        0x0000,   // Basic
        0x0003,   // Identify
        0x0402,   // Temperature Measurement
    ].into(),
    output_clusters: heapless::Vec::new(),
});

// 3. Initialize BDB
bdb.initialize()?;

// 4. Configure commissioning
bdb.attributes_mut().commissioning_mode =
    CommissioningMode::STEERING.or(CommissioningMode::FINDING_BINDING);

// 5. Commission! This will:
//    - Skip Touchlink (not requested)
//    - Run Network Steering (scan, join, announce, key exchange)
//    - Run Finding & Binding (Identify Query, match clusters, bind)
//    - Skip Formation (we're an End Device, not supported)
bdb.commission().await?;

// 6. We're on the network!
assert!(bdb.is_on_network());

// 7. Now send periodic temperature reports via indirect addressing
//    (bindings were created by F&B)
loop {
    let temp = read_temperature_sensor();
    send_zcl_report(bdb.zdo_mut().aps_mut(), 0x0402, temp).await;
    sleep(60_seconds).await;
}
```

## Complete Example: Coordinator Formation

And here's how a coordinator creates and manages a network:

```rust
// 1. Create the stack as Coordinator
let nwk = NwkLayer::new(mac, DeviceType::Coordinator);
let aps = ApsLayer::new(nwk);
let zdo = ZdoLayer::new(aps);
let mut bdb = BdbLayer::new(zdo);

// 2. Initialize
bdb.initialize()?;

// 3. Form the network
bdb.attributes_mut().commissioning_mode = CommissioningMode::FORMATION;
bdb.commission().await?;

// The network is now active:
// - NWK key is installed
// - Permit joining is open for 180 seconds
// - We are the Trust Center

// 4. Later, open joining for more devices
bdb.attributes_mut().commissioning_mode = CommissioningMode::STEERING;
bdb.commission().await?;
// This calls steer_on_network() which broadcasts Mgmt_Permit_Joining_req
```

## Summary

BDB commissioning provides a standardized, interoperable way to get Zigbee 3.0
devices onto the network:

- **Network Steering** handles the common case of joining an existing network
  (scanning channels, joining the best PAN, TC key exchange).
- **Network Formation** lets coordinators create new PANs with proper Trust
  Center configuration.
- **Finding & Binding** automates the tedious process of creating bindings
  between compatible devices using the Identify cluster.
- **Touchlink** enables proximity-based commissioning for consumer-friendly
  experiences (stub implementation — Inter-PAN MAC support needed).
- The **state machine** runs methods in priority order and handles fallbacks.
- **Factory reset** and **rejoin** provide recovery paths when things go wrong.

All of this is driven by the `BdbLayer` struct and its `BdbAttributes`
configuration.  Set the attributes, call `commission()`, and zigbee-rs handles
the rest.
