# BDB Commissioning

**Base Device Behavior (BDB)** is the Zigbee 3.0 specification that
standardises how devices join networks, form networks, and find each other.
Before BDB, every manufacturer invented its own commissioning process — some
used button sequences, others relied on proprietary apps.  BDB defines four
universal methods so that any Zigbee 3.0 device can join any Zigbee 3.0
network.

> zigbee-rs uses **BDB 3.0.1 (`16-02828-012`) with Zigbee PRO R22** as
> its production baseline. Touchlink remains experimental, is compiled out by
> default, and is never advertised as an available commissioning method unless
> the `zigbee-bdb/touchlink` feature and the application capability bit are
> both enabled.

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

// Request every compiled and device-supported method
let mode = CommissioningMode::ALL; // 0x0F; still gated by capabilities

// Check what's enabled
if mode.contains(CommissioningMode::FORMATION) {
    println!("Formation is enabled");
}
```

The four bits:

| Constant | Value | Method |
|---|---|---|
| `CommissioningMode::STEERING` | `0x01` | Network Steering |
| `CommissioningMode::FORMATION` | `0x02` | Network Formation |
| `CommissioningMode::FINDING_BINDING` | `0x04` | Finding & Binding |
| `CommissioningMode::TOUCHLINK` | `0x08` | Touchlink |
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
| **Coordinator** | Steering + Formation; optional Finding & Binding / Touchlink |
| **Router** | Steering; optional Finding & Binding / Touchlink |
| **End Device** | Steering; optional Finding & Binding / Touchlink |

The requested mode is intersected with the capability mask to produce the
*effective* commissioning mode. Finding & Binding must be enabled explicitly
in `node_commissioning_capability`. Touchlink additionally requires the
off-by-default Cargo feature. If you request Formation on an End Device, it is
silently skipped.

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
Pre-network (awaited inside `network_steering`)
1. Scan primary channels (11, 15, 20, 25) for open networks
   └── NLME-NETWORK-DISCOVERY
2. Filter by extended PAN ID (if configured)
3. Join the best-LQI network with permit-joining enabled
   └── NLME-JOIN
4. Wait for Transport-Key from Trust Center (5 s)
   └── Poll parent via MAC Data Request
5. Reserve network security counters, broadcast Device_annce
   └── returns: the network is up

Post-network (one bounded step per tick — `advance_tclk_exchange`)
6. Node_Desc_req to the Trust Center → stack compliance revision
   └── pre-R21 Trust Center: no unique key is required, done
7. APSME-REQUEST-KEY → Transport-Key installs the unique TC link key
8. APS Verify-Key → Confirm-Key proves possession
9. Re-broadcast Device_annce (now authenticated)
```

If primary channels yield no results, secondary channels (all other 2.4 GHz
channels) are scanned.

#### Unique Trust Center link-key exchange (steps 6–8)

Steps 6–8 mirror the Silicon Labs GSDK `update-tc-link-key` plugin, which the
stack advances through scheduled events *after* the network is up. Retries are
budgeted **per message type** (GSDK `emberUpdateTcLinkKey(maxAttempts)`), not
per whole procedure:

| Message           | Transmissions | Response window |
| ----------------- | ------------- | --------------- |
| Node_Desc_req     | 3             | 1.5 s           |
| APS Request-Key   | 3             | 3 s             |
| APS Verify-Key    | 3             | 5 s             |

The first probe starts 300 ms after `Device_annce`, and the whole handshake has
a wrapping-safe 15 s deadline. One full pass through every stage takes at most
9.8 s, so a slow-but-answering Trust Center is never cut off. Both an expired
deadline and an exhausted message budget fail the exchange strictly. An APS
acknowledgement is transport feedback only and never substitutes for
`Confirm-Key`.

Synchronous transmit failures and rejected Confirm-Key responses use a
dedicated **250 ms retry backoff**. This mirrors GSDK's scheduled-event pacing:
it prevents the 50 ms runtime tick from spending all three transmissions in
about 200 ms, but does not restore the old 5 s whole-procedure cooldown. Even
all two inter-attempt backoffs for all three message types plus one full
response window per stage total 11.3 s, inside the strict 15 s deadline.

Because the budgets are independent:

* a lost `Node_Desc_rsp` retransmits only `Node_Desc_req`;
* a missing Transport-Key retransmits only Request-Key;
* a lost Confirm-Key retransmits **Verify-Key**, keeping the unique key the
  Trust Center already installed.

A unique key that the Trust Center pushed unsolicited is never erased: the
exchange reserves it and goes straight to Verify-Key. An unconfirmed unique key
is dropped only when a replacement Request-Key is issued.

`Confirm-Key` is a Trust Center *verdict*, and this state machine treats it as
a security predicate: an authenticated rejection is a hard failure that leaves
the network. So a `Confirm-Key` only reaches the exchange counters
(`confirm_key_received` / `_successes` / `_rejections`) when it authenticates:
APS-secured with a data key identifier under the *unique* Trust Center link
key, NWK-secured, from the centralized Trust Center at `0x0000` with the
configured Trust Center IEEE address as its APS security source, naming the
Trust Center link-key type, and addressed to this device's own IEEE address.

Anything else — unsecured, malformed, addressed elsewhere, or secured with the
globally known `ZigBeeAlliance09` key — is counted only in
`ApsSecurityHandshakeStats::confirm_key_ignored` and has no effect on
commissioning. Without that split, a single forged unicast that requires no key
material at all could drive a commissioning device straight into its hard
failure, i.e. kick it off the network on demand.

When the exchange fails during **initial** steering the device sends a secured
NWK Leave (falling back to a local `NLME-RESET` if the Leave cannot be sent),
clears the Trust Center key and reports
`CommissioningComplete { success: false }` — a failed R21+ initial join never
stays commissioned.

#### Verify-Key security and completion

R22 Table 4-7 and §4.4.7.1.3 require `Verify-Key` to be sent without APS
encryption. The command still contains the keyed hash calculated from the
installed unique TCLK, and its enclosing NWK frame remains secured, but it has
no APS auxiliary header or MIC and consumes no TCLK outgoing security counter.

An APS acknowledgement of that command confirms reception only. It does not
carry a status or prove that the Trust Center validated the hash, so it cannot
mark the key verified, defer failure, or commit the commissioned network. The
exchange completes only after an authenticated successful `Confirm-Key`.
Missing `Confirm-Key` follows the bounded Verify-Key retry and replacement-key
policy above; exhausted budgets or the overall deadline fail strictly.

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
1. Send a NWK-secured Identify Query:
   • NWK destination 0xFFFF
   • APS destination endpoint 0xFF
   • actual initiator source endpoint and profile
2. Collect responses for 180 seconds (bdbcMinCommissioningTime)
3. Deduplicate each responding (NWK address, endpoint)
4. For each target:
   a. In unicast mode, resolve its IEEE address from cache or IEEE_addr_req
   b. Send an asynchronous Simple_Desc_req for the responding endpoint
   c. Match clusters:
      • Our output clusters ↔ their input clusters
      • Our input clusters ↔ their output clusters
   d. If commissioning_group_id == 0xFFFF:
      • create local unicast bindings to (IEEE address, endpoint)
   e. Otherwise:
      • create group bindings instead of unicast bindings
      • send APS-acknowledged Groups Add Group to the target
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
- Unicast and group mode are mutually exclusive. Group mode does not require
  the target's IEEE address.
- IEEE and Simple Descriptor responses are handled by bounded, asynchronous
  pending-response slots, so normal APS/ZDO/ZCL processing continues while
  each five-second response window is open.
- Bindings are created only in the initiator's local APS binding table; F&B
  does not send a remote ZDP `Bind_req`.

### Target — The Device That Gets Bound To

The target enters Identify mode and waits for an initiator to discover it:

```rust
// Enter F&B target mode on endpoint 1 (LED blinks for 180 seconds)
bdb.finding_binding_target(1).await?;
```

This sets `fb_target_request` to `Some((endpoint, 180))`.  Your runtime reads
this and writes the `IdentifyTime` attribute on the Identify cluster, which
makes the device respond to Identify Query broadcasts.

The target's normal APS/ZCL processing handles the Identify Query and
`Simple_Desc_req`. In group mode it also handles the Groups `Add Group`
command.

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

BDB provides a full factory-new procedure:

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

This is distinct from the Basic cluster **Reset to Factory Defaults** command.
That command produces `StackEvent::BasicResetToFactoryDefaults` and resets
writable application-cluster attributes to their defaults while preserving
network membership, NWK/APS keys, outgoing counter floors, groups, and
bindings. Applications must not translate the Basic command into Leave or the
full `bdb.factory_reset()` operation.

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
2. Filter and rank the beacons with the R22 rejoin parent rules (see below) and
   reject everything that is not a suitable parent
3. Attempt `NLME-JOIN` with Rejoin method (uses stored NWK key), minimum-depth
   candidate first, moving on to the next suitable parent on failure
4. Broadcast `Device_annce`
5. If rejoin fails, fall back to full Network Steering

### Rejoin parent selection (R22 §3.6.1.4.2)

§3.6.1.4.2 defines NWK rejoin as identical to association (§3.6.1.4.1) except
that MAC association is replaced by the NWK Rejoin Request/Response exchange
and that joining into a **closed** network is allowed.

`NwkLayer::select_rejoin_parents` (built from
`zigbee_nwk::nlme::RejoinParentCriteria`) both filters and orders the discovery
result.

**Base eligibility** — decidable from a single beacon
(`RejoinParentCriteria::is_base_eligible`). All of the following must hold:

- its extended PAN ID equals `nwkExtendedPANID`;
- it advertises capacity **for the device type being requested**:
  `router_capacity` when rejoining as a router, `end_device_capacity` when
  rejoining as an end device. R22 requires capacity even though it drops the
  permit-joining requirement;
- its `nwkUpdateId` is equal to or newer than ours, compared as an **8-bit
  serial number** so a network that wrapped `0xFF → 0x00` still counts as
  fresher — a stale parent is rejected even if its signal is the strongest,
  and the ambiguous half-window distance (128) is treated as stale in both
  directions. This gate applies **only when our own `nwkUpdateId` is
  known-good**: `RejoinParentCriteria::nwk_update_id` is an `Option<u8>` fed
  from `Nib::nwk_update_id()`, and a device that holds no authoritative update
  state (factory-new, or restored from a record that carries no update state)
  rejects nothing as stale — see
  [NWK → `nwkUpdateId` validity](nwk.md);
- its link cost (derived from beacon LQI, R22 §3.6.3.1) is valid and at most
  `MAX_PARENT_LINK_COST` (3). This is a hard gate, not a ranking penalty.

**Suitability** — decidable only over the whole scan result. Among the
base-eligible candidates, only those carrying the **single most recent**
`nwkUpdateId` are suitable; base-eligible candidates advertising an older (but
not stale) update id are dropped. "Most recent" is computed as the maximum
wrap-aware distance ahead of our own `nwkUpdateId`, which is bounded by 127 for
base-eligible candidates and is therefore deterministic across a wrap
(`RejoinParentCriteria::most_recent_update_id`).

With an **unknown** local update state there is no reference point, and
serial-number comparison over an arbitrary set is not a total order. The most
recent id is then the fixed point of a left-to-right, forward-only fold: the
first base-eligible candidate in discovery order seeds the answer and a later
one replaces it only when it is *strictly newer*, so older and ambiguous values
never pull it back and discovery order breaks every tie. The selection is still
narrowed to exactly one update id, and the rejoin that succeeds makes that id
authoritative.

**Ordering** of the suitable candidates:

1. **minimum depth** — the only preference R22 states, and therefore the only
   normative ranking rule;
2. *implementation tie-break*: lower link cost at equal depth;
3. *implementation tie-break*: the previous parent;
4. *implementation tie-break*: discovery order (the sort is stable).

Rules 2–4 are chosen purely to make the attempt order deterministic; the
specification does not rank them.

`NLME-JOIN` with the Rejoin method re-checks **base eligibility** only, so a
hand-picked descriptor cannot bypass the per-candidate rules; such a candidate
fails with `NwkStatus::InvalidRequest` before any frame is transmitted. It
carries the same known-vs-unknown update-state semantics as whole-scan
selection, because it is built from the same `RejoinParentCriteria`. The
guard deliberately does not apply the most-recent-update-id rule: without the
scan set it cannot know which update id was the most recent, and that filter
stays with the policy layer (`select_rejoin_parents`).

Permit-joining is deliberately *not* required: §3.6.1.4.2 allows rejoin into a
closed network. It continues to apply to association-based Network Steering.

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
