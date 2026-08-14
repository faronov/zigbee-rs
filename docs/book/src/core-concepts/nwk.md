# Network Layer

The NWK (Network) layer sits between the MAC and APS layers and is responsible
for everything that makes Zigbee a *mesh network*: discovering PANs, joining,
routing frames across multiple hops, managing neighbors, and encrypting all
routed traffic.

```text
┌──────────────────────────────────────┐
│  APS Layer (zigbee-aps)              │
└──────────────┬───────────────────────┘
               │ NLDE-DATA / NLME-*
┌──────────────┴───────────────────────┐
│  NWK Layer (zigbee-nwk)              │
│  ├── nlme: management primitives     │
│  ├── nlde: data service              │
│  ├── nib: network information base   │
│  ├── frames: NWK frame codec         │
│  ├── neighbor: neighbor table        │
│  ├── routing: tree + AODV routing    │
│  └── security: NWK encryption        │
└──────────────┬───────────────────────┘
               │ MacDriver trait
┌──────────────┴───────────────────────┐
│  MAC Layer (zigbee-mac)              │
└──────────────────────────────────────┘
```

In zigbee-rs the NWK layer is implemented as `NwkLayer<M>`, generic over the
MAC driver.  You normally don't interact with it directly — the
`ZigbeeDevice` runtime drives it through BDB → ZDO → APS.  But understanding
how it works is essential for debugging and advanced use.

## `NwkLayer` — The Core Struct

```rust,ignore
pub struct NwkLayer<M: MacDriver> {
    mac: M,                          // The MAC driver
    nib: Nib,                        // Network Information Base
    neighbors: NeighborTable,        // Known neighbors
    routing: RoutingTable,           // Routing + route discovery
    security: NwkSecurity,           // Encryption keys & frame counters
    device_type: DeviceType,         // Coordinator / Router / EndDevice
    joined: bool,                    // Whether we're on a network
    rx_on_when_idle: bool,           // false = sleepy end device
}
```

Key accessors:

```rust,ignore
nwk.nib()              // &Nib — read network state
nwk.nib_mut()          // &mut Nib — modify network state
nwk.neighbor_table()   // &NeighborTable
nwk.routing_table()    // &RoutingTable
nwk.security()         // &NwkSecurity — read keys
nwk.security_mut()     // &mut NwkSecurity — install keys
nwk.is_joined()        // bool
nwk.device_type()      // DeviceType
nwk.mac() / mac_mut()  // Access the underlying MAC driver
```

## Network Discovery

Before joining, a device must find available networks.  This is done with
`nlme_network_discovery()`:

```rust,ignore
let networks = nwk.nlme_network_discovery(
    ChannelMask::ALL_2_4GHZ,  // Scan all 2.4 GHz channels
    3,                         // Scan duration exponent
).await?;
```

**What happens internally:**

1. Sets `macAutoRequest = false` (don't auto-request data during scan)
2. Sends an **Active Scan** via MAC — beacon requests on each channel
3. Collects beacon responses as `PanDescriptor` structs
4. Filters for Zigbee PRO beacons (`protocol_id == 0`, `stack_profile == 2`)
5. Converts to `NetworkDescriptor` structs
6. Sorts by LQI (best signal first)
7. Restores `macAutoRequest = true`

The returned `NetworkDescriptor` contains everything needed to join:

```rust,ignore
pub struct NetworkDescriptor {
    pub extended_pan_id: IeeeAddress,  // 64-bit network ID
    pub pan_id: PanId,                 // 16-bit PAN ID
    pub logical_channel: u8,           // Channel (11-26)
    pub stack_profile: u8,             // 2 = Zigbee PRO
    pub permit_joining: bool,          // Network is open for joining
    pub router_capacity: bool,         // Can accept router children
    pub end_device_capacity: bool,     // Can accept end device children
    pub lqi: u8,                       // Signal quality (0-255)
    pub router_address: ShortAddress,  // Beacon sender's address
    pub depth: u8,                     // Sender's depth in tree
    // ... more fields
}
```

## Joining a Network

After discovery, the NWK layer joins the best network via MAC association:

```rust,ignore
nwk.nlme_join(&best_network).await?;
```

**The join sequence:**

1. Select the best network (highest LQI, open for joining, has capacity)
2. Configure MAC: set channel, PAN ID, coordinator address
3. Send `MLME-ASSOCIATE.request` to the chosen router/coordinator
4. Receive `MLME-ASSOCIATE.confirm` with our assigned short address
5. Update NIB: PAN ID, channel, short address, parent address
6. Add parent to neighbor table with `Relationship::Parent`
7. Set `joined = true`

### Join Methods

```rust,ignore
pub enum JoinMethod {
    /// Normal first join — MAC-level association
    Association,
    /// Rejoin using existing network key (after losing parent)
    Rejoin,
    /// Direct join — coordinator adds device without association
    Direct,
}
```

- **Association** is the normal path for a fresh device.
- **Rejoin** is used after power loss when the device has saved network state
  (NV storage).  It's faster because it skips the full BDB commissioning.
  Candidate parents are filtered and ranked by
  `NwkLayer::select_rejoin_parents` (R22 §3.6.1.4.2: matching extended PAN ID,
  capacity for the requested device type, only the most recent wrap-aware
  `nwkUpdateId`, link cost ≤ 3, then minimum depth — see
  [BDB → Rejoin](bdb.md)); `nlme_join` itself refuses a foreign, stale,
  capacity-less or unusable-link candidate with `NwkStatus::InvalidRequest`
  without transmitting.
- **Direct** is used by coordinators to pre-authorize devices.

### `nwkUpdateId` validity

`nwkUpdateId` is a *serial number*, so `0` is an ordinary live value, not an
"unset" marker. The NIB therefore carries an explicit `update_id_valid` flag
and exposes `Nib::nwk_update_id() -> Option<u8>`; every decision that *rejects*
a peer for holding an older update state must go through that accessor.

The value becomes known-good at exactly these points:

- network formation (the coordinator defines the network's update state);
- a successful association;
- a successful rejoin — including one that started from an unknown state;
- a security-journal restore whose record marks the update state valid —
  record version 4 stores that validity explicitly in flags bit 7, and records
  written by versions 1..=3 (which predate the bit) always count as valid
  because their stored byte was authoritative in the firmware that wrote it;
- a legacy NV record that actually contains a valid `NwkUpdateId` item;
- an accepted `Mgmt_NWK_Update` channel/manager change.

It is *unknown* on a factory-new device, after a full leave, after a legacy NV
restore whose record predates the `NwkUpdateId` item, and after restoring a
security-journal record that was itself migrated from a persistence format
which never stored the value (the legacy ESP32 log-structured NV region, for
instance). Legacy `save_state` writes the item only when the value is known and
**deletes** it otherwise, and the journal encodes "unknown" rather than a known
`0`, so a reboot cannot turn an unknown state into an authoritative `0`. Once
the update state *is* learned, `refresh_security_state` adopts it durably.

While unknown, rejoin does not reject any candidate as stale — a fabricated
reference of `0` would make every advertised id in `0x81..=0xFF` look stale and
strand the device off its own network. The scan is still narrowed to a single
update id, chosen by a forward-only wrap-aware pairwise comparison with
discovery order breaking ambiguous ties, and the rejoin that succeeds makes
that id authoritative.

`Mgmt_NWK_Update` (ZDO §3.4.12) applies the same distinction before it touches
the NIB or PIB: an unknown local state accepts the incoming id, a known state
advances only on a strictly newer id (`nwk_update_id_is_newer`), an equal id is
idempotent only when the requested channel/manager is already in effect, and
equal-but-conflicting, older or ambiguous ids are answered with
`INV_REQUESTTYPE` without changing anything.

## Network Formation (Coordinator)

A coordinator *creates* a new network instead of joining one:

```rust,ignore
nwk.nlme_network_formation(
    ChannelMask::ALL_2_4GHZ,  // Channels to evaluate
    3,                         // Scan duration
).await?;
```

**What happens:**

1. **ED Scan** — measures energy (noise) on each channel
2. **Pick quietest channel** — lowest energy = least interference
3. **Generate PAN ID** — random 16-bit ID, avoiding 0xFFFF
4. **Configure MAC** — set short address to 0x0000 (coordinator), set PAN ID
5. **Start PAN** — `MLME-START.request` begins beacon transmission
6. **Update NIB** — record channel, PAN ID, address, depth = 0

After formation, the coordinator opens permit joining so other devices can
associate.

## Routing

The NWK layer supports two routing algorithms:

### AODV Mesh Routing

AODV (Ad-hoc On-demand Distance Vector) is the primary routing mechanism in
Zigbee PRO.  Routes are discovered on-demand when a frame needs to reach a
destination with no known route.

**Route discovery flow:**

1. Router needs to send to destination `D` but has no route
2. Broadcasts a **Route Request (RREQ)** with destination `D`
3. Each receiving router re-broadcasts the RREQ, recording path cost
4. When RREQ reaches `D` (or a router with a route to `D`), a **Route Reply
   (RREP)** is unicast back along the best path
5. Each router along the path installs a route entry

### Tree Routing

Tree routing uses the hierarchical network address space to forward frames
without a route table.  It's a fallback when mesh routing isn't available:

```rust,ignore
// CSkip algorithm determines next hop based on address ranges
routing.tree_route(
    our_addr,     // Our NWK address
    dst_addr,     // Destination address
    depth,        // Our depth in the tree
    max_routers,  // nib.max_routers
    max_depth,    // nib.max_depth
) -> Option<ShortAddress>
```

If the destination is within our child address range, forward to the appropriate
child.  Otherwise, forward to our parent.

### The Route Table

```rust,ignore
pub struct RoutingTable {
    routes: [RouteEntry; MAX_ROUTES],          // 32 entries
    discoveries: [RouteDiscovery; MAX_ROUTE_DISCOVERIES],  // 8 pending
}
```

Each `RouteEntry` tracks:

```rust,ignore
pub struct RouteEntry {
    pub destination: ShortAddress,   // Target NWK address
    pub next_hop: ShortAddress,      // Where to forward
    pub status: RouteStatus,         // Active, DiscoveryUnderway, etc.
    pub many_to_one: bool,           // Concentrator route
    pub route_record_required: bool,
    pub route_record_every_frame: bool, // Low-RAM concentrator
    pub group_id: bool,              // Multicast route
    pub path_cost: u8,               // Sum of link costs
    pub age: u16,                    // Ticks since last use
    pub active: bool,
}
```

**Route status values:**

| Status | Meaning |
|--------|---------|
| `Active` | Route is valid and ready for forwarding |
| `DiscoveryUnderway` | Route request broadcast, awaiting reply |
| `DiscoveryFailed` | No route reply received within timeout |
| `Inactive` | Route expired or was removed |
| `ValidationUnderway` | Route is being validated |

Key operations:

```rust,ignore
routing.next_hop(destination)                    // Look up next hop
routing.update_route(destination, next_hop, cost) // Add/update route
routing.remove(destination)                       // Delete a route
routing.age_tick()                                // Age all entries
routing.mark_discovery(destination)               // Mark as discovering
```

When the route table is full, the oldest inactive or highest-cost route is
evicted.

## Neighbor Table

The neighbor table tracks all known nearby devices:

```rust,ignore
pub struct NeighborTable {
    entries: [NeighborEntry; MAX_NEIGHBORS],  // 32 entries
    count: usize,
}
```

Each `NeighborEntry` contains:

```rust,ignore
pub struct NeighborEntry {
    pub ieee_address: IeeeAddress,      // 64-bit address
    pub network_address: ShortAddress,  // 16-bit NWK address
    pub device_type: NeighborDeviceType, // Coordinator/Router/EndDevice/Unknown
    pub rx_on_when_idle: bool,          // false = sleepy
    pub relationship: Relationship,      // Parent/Child/Sibling/etc.
    pub lqi: u8,                        // Link Quality (rolling average)
    pub outgoing_cost: u8,              // 1-7, derived from LQI
    pub depth: u8,                      // Network depth
    pub permit_joining: bool,           // For routers/coordinators
    pub security_capable: bool,          // Child can authenticate with NWK security
    pub age: u16,                       // Ticks since last heard from
    pub extended_pan_id: IeeeAddress,
    pub active: bool,
}
```

### Relationship Types

```rust,ignore
pub enum Relationship {
    Parent,              // Device we joined through
    Child,               // Device that joined through us
    Sibling,             // Same parent (used for routing)
    PreviousChild,       // Was our child, rejoined elsewhere
    UnauthenticatedChild, // Joined but not yet authenticated
}
```

### Link Cost Calculation

LQI (Link Quality Indicator, 0–255) is converted to an outgoing cost (1–7)
used by the routing algorithm:

| LQI Range | Cost | Quality |
|-----------|------|---------|
| 201–255 | 1 | Excellent |
| 151–200 | 2 | Good |
| 101–150 | 3 | Fair |
| 51–100 | 5 | Poor |
| 0–50 | 7 | Very poor |

### Table Operations

```rust,ignore
neighbors.find_by_short(addr)     // Look up by NWK address
neighbors.find_by_ieee(&ieee)     // Look up by IEEE address
neighbors.parent()                // Get our parent entry
neighbors.children()              // Iterate over child entries
neighbors.add_or_update(entry)    // Insert or update
neighbors.remove(addr)            // Remove by NWK address
neighbors.age_tick()              // Increment all age counters
neighbors.iter()                  // Iterate active entries
```

**Eviction policy:** When the table is full, the oldest non-parent, non-child
entry is evicted.  Parents and children are never evicted automatically — this
ensures the device never loses track of its parent or its children.

## NIB — Network Information Base

The NIB holds all NWK-layer configuration and state.  It's the NWK equivalent
of the MAC PIB.

### Key Fields

#### Network Identity

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `extended_pan_id` | `IeeeAddress` | 64-bit network identifier | `[0; 8]` |
| `pan_id` | `PanId` | 16-bit PAN ID | 0xFFFF |
| `network_address` | `ShortAddress` | Our 16-bit address | 0xFFFF |
| `logical_channel` | `u8` | Operating channel (11-26) | 0 |

#### Network Parameters

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `stack_profile` | `u8` | 0x02 = Zigbee PRO | 0x02 |
| `depth` | `u8` | Our depth in network tree | 0 |
| `max_depth` | `u8` | Maximum network depth | 15 |
| `max_routers` | `u8` | Max child routers | 5 |
| `max_children` | `u8` | Max child end devices | 20 |
| `update_id` | `u8` | Network update counter (`nwkUpdateId`) | 0 |
| `update_id_valid` | `bool` | Whether `update_id` is known-good | `false` |

#### Addressing

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `ieee_address` | `IeeeAddress` | Our 64-bit IEEE address | `[0; 8]` |
| `parent_address` | `ShortAddress` | Parent's NWK address | 0xFFFF |
| `address_assign` | `AddressAssignMethod` | `TreeBased` or `Stochastic` | `Stochastic` |

#### Routing

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `use_tree_routing` | `bool` | Enable tree routing fallback | false |
| `source_routing` | `bool` | Enable source routing | false |
| `route_discovery_retries` | `u8` | Max RREQ retries | 3 |

#### Security

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `security_level` | `u8` | 5 = ENC-MIC-32 | 5 |
| `security_enabled` | `bool` | NWK encryption on/off | true |
| `active_key_seq_number` | `u8` | Active key index | 0 |
| `outgoing_frame_counter` | `u32` | Outgoing frame counter | 0 |

#### Permit Joining

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `permit_joining` | `bool` | Accept new join requests | false |
| `permit_joining_duration` | `u8` | Time remaining (seconds) | 0 |

#### R22 End Device Timeout (client side)

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `parent_information` | `u8` | `nwkParentInformation`: bit0 MAC Data Poll keepalive, bit1 End Device Timeout Request keepalive | 0 |
| `parent_information_valid` | `bool` | Whether `parent_information` describes the *current* parent | false |
| `end_device_timeout` | `u8` | `nwkEndDeviceTimeout` enumeration in effect (0..=14) | 8 |
| `requested_end_device_timeout` | `u8` | Enumeration carried by the next request | 8 |

### Helper Methods

```rust,ignore
nib.next_seq()            // Get next NWK sequence number (wrapping)
nib.next_route_request_id() // Get next route request ID
nib.next_frame_counter()  // Increment frame counter (returns None if exhausted)

nib.reset_end_device_timeout_negotiation() // New parent: renegotiate from 14
nib.restore_end_device_timeout(info, valid, timeout) // Install a persisted result
nib.end_device_timeout_seconds()           // Enumeration → seconds
```

> **Frame counter exhaustion:** The outgoing frame counter is a 32-bit value.
> If it reaches `u32::MAX`, the device **cannot send any more secured frames**
> and must perform a key update or factory reset.  In practice this takes
> billions of frames and is unlikely, but `next_frame_counter()` returns `None`
> to protect against it.

## End Device Timeout (NWK 0x0B / 0x0C)

A R22 parent ages a child out of its child table unless the child keeps the
relationship alive. An end device negotiates that lifetime with an **End Device
Timeout Request** and the parent answers with an **End Device Timeout
Response**.

### Wire format

`EdTimeoutRequest` (command 0x0B), two bytes:

| Byte | Field | Notes |
|------|-------|-------|
| 0 | Requested Timeout Enumeration | 0..=14 |
| 1 | End Device Configuration | **Reserved in R22 — always 0** |

`EdTimeoutResponse` (command 0x0C), two bytes; trailing bytes from a future
revision are ignored:

| Byte | Field | Notes |
|------|-------|-------|
| 0 | Status | 0x00 success, 0x01 incorrect value |
| 1 | `nwkParentInformation` | bit0 MAC Data Poll keepalive, bit1 End Device Timeout Request keepalive |

The enumeration is arithmetic, not a lookup table:

```text
0      => 10 s
n >= 1 => 120 << (n - 1) seconds   (2 minutes doubling per step)
8      => 256 min  (the R22 default a parent applies without negotiation)
14     => 16384 min (~11 days, the maximum)
```

`ed_timeout_enum_to_seconds()` rejects anything above 14 rather than clamping,
and `EdTimeoutRequest::new()` is a checked constructor, so an undefined
enumeration can never reach the air or a keepalive deadline.

### Client behaviour

- A fresh join or secured rejoin requests enumeration **14** for battery life
  and interoperability. Enumeration **8** is used only as the recurring
  fallback when no negotiation succeeded.
- The negotiation is reset at the authoritative NWK parent-assignment and
  parent-loss points — association success, secured rejoin success, a Leave
  addressed to this device, a parent that announced its own leave, and a full
  `nlme_leave` — never from a runtime wrapper.
- A 0x0C response is accepted only when the device is an end device, the frame
  came from the current valid unicast parent, it was addressed to this device's
  own network address, and the shared secured-NWK-command guard passed.
- `SUCCESS` stores `parent_info & 0x03` and marks it valid. `INCORRECT_VALUE`
  lowers the requested enumeration one step, never below 8, and never touches
  previously validated parent information. An unknown status changes nothing.
- No `NwkCommandOutcome` is produced: the effect is NIB state, which the
  runtime detects by comparing the negotiation fields around
  `process_incoming_nwk_frame`.

The recurring keepalive, response timeout, retransmissions and persistence live
in `zigbee-runtime` — see
[Event Loop → End Device Timeout keepalive](./event-loop.md).

### Parent (router/coordinator) behaviour

> The entire parent/router server side described below — End Device Timeout
> child aging, MAC parent-command servicing, the rejoin/Update-Device path and
> Parent Announce — is gated on the `router` capability feature and its
> periodic work runs from a dedicated `run_parent_nwk_maintenance` step. A
> non-routing end-device (sensor) build compiles it out of the image entirely;
> only the role-independent common NWK maintenance (neighbour-cache aging) and
> the End Device Timeout **client** remain.

A routing device implements the **server** side of the same exchange:

- A 0x0B request is accepted only when it comes from a directly-attached,
  **authenticated** child (`Relationship::Child`) whose stored IEEE matches the
  authenticated NWK source, and only when addressed to this parent's own short
  address (and matching IEEE, if the child included one). The reserved End
  Device Configuration byte must be zero and the enumeration must be defined.
  This keeps an arbitrary network member — or a still-provisional child — from
  rewriting a child's timeout state.
- The supported-enumeration policy accepts every defined R22 value 0..=14
  (`respond_to_end_device_timeout_request`); the parent imposes no tighter
  bound, so the answer is `SUCCESS` with the requested enumeration. An
  out-of-range value takes the deterministic `INCORRECT_VALUE` branch and falls
  back to enumeration 8. `nwkParentInformation` advertises both keepalive
  methods (`PARENT_INFO_MASK`).
- The 0x0C response is delivered directly to an rx-on child and **indirectly**
  (queued, MAC Frame Pending armed) to a sleepy child, reusing the same parent
  queue / Data Request path as the Rejoin Response.

### Child aging and keepalive (parent side)

Each authenticated end-device child carries an accepted timeout enumeration and
a `keepalive_remaining_secs` deadline (a `u32`, because enumeration 14 exceeds
the `u16` neighbour age counter). The deadline is:

- **armed** to the full window of the accepted enumeration when the child
  authenticates (association + key proof, secured rejoin, or restore);
- **reset** on every advertised keepalive — a MAC Data Poll, an End Device
  Timeout Request — and on any CCM*-authenticated frame whose auxiliary-header
  source IEEE matches the child (the *secured-traffic keepalive*, keyed on the
  authenticated identity so it cannot be spoofed);
- **aged down** each shared tick with saturating subtraction; at zero the child
  is evicted, cleaning the coupled indirect queue, routing entry, replay
  counters, MAC Frame Pending flag and the runtime's deferred Update-Device.

Router children age via Link Status, not this deadline; unauthenticated
(provisional) children keep the existing short provisional-child timer.

### Durable child table and Parent Announce

A router persists its authenticated children through a **separate** crash-safe
store (`child_store`), independent of the security journal — see
[NV storage → Child table](../advanced/nv-storage.md). After the restored child
state is authoritative, `announce_parent()` broadcasts a R22 **Parent Announce**
(ZDP `Parent_annce`, 0x001F) so a former parent of a child that has since moved
prunes its stale record; see [ZDO → Parent Announce](./zdo.md).

## `NwkStatus` — Error Codes

NWK operations return `NwkStatus` on failure:

```rust,ignore
pub enum NwkStatus {
    Success              = 0x00,
    InvalidParameter     = 0xC1,
    InvalidRequest       = 0xC2,  // e.g., formation on non-coordinator
    NotPermitted         = 0xC3,
    StartupFailure       = 0xC4,  // MAC start failed
    AlreadyPresent       = 0xC5,
    SyncFailure          = 0xC6,
    NeighborTableFull    = 0xC7,
    UnknownDevice        = 0xC8,
    UnsupportedAttribute = 0xC9,
    NoNetworks           = 0xCA,  // Scan found nothing
    MaxFrmCounterReached = 0xCC,  // Frame counter exhausted
    NoKey                = 0xCD,  // No network key available
    BadCcmOutput         = 0xCE,  // AES-CCM* decryption failed
    RouteDiscoveryFailed = 0xD0,  // No route found
    RouteError           = 0xD1,  // Route broke during use
    BtTableFull          = 0xD2,  // Broadcast transaction table full
    FrameNotBuffered     = 0xD3,
    FrameTooLong         = 0xD4,  // NWK frame exceeds max size
}
```

## Network Security

All NWK-layer frames in Zigbee 3.0 are encrypted.  zigbee-rs implements
standard Zigbee PRO NWK security:

### How It Works

- **Algorithm:** AES-128-CCM* with a 4-byte Message Integrity Code (MIC)
- **Security Level:** 5 (ENC-MIC-32) — standard for Zigbee PRO
- **Key type:** A single **network key** shared by all devices on the network
- **Frame counter:** 32-bit counter for replay protection (each sender
  maintains their own)
- **Key distribution:** The coordinator distributes the network key during
  joining via the APS Transport Key command (itself protected by the well-known
  Trust Center Link Key)

### NWK Security Header

Every secured NWK frame includes an auxiliary security header:

```rust,ignore
pub struct NwkSecurityHeader {
    pub security_control: u8,      // Security level + key identifier + flags
    pub frame_counter: u32,        // Replay protection
    pub source_address: IeeeAddress, // 64-bit sender IEEE address
    pub key_seq_number: u8,        // Which network key was used
}
```

The security control field for standard Zigbee is always `0x2D`:
- Security Level = 5 (ENC-MIC-32)
- Key Identifier = 1 (Network Key)
- Extended Nonce = 1 (source address present)

### Key Management

```rust,ignore
// Install a network key
nwk.security_mut().set_network_key(key, seq_number);

// Read the active key
if let Some(key_entry) = nwk.security().active_key() {
    // key_entry.key: [u8; 16]
    // key_entry.seq_number: u8
}

// Look up key by sequence number (for key rotation)
let key = nwk.security().key_by_seq(1);
```

The security module stores up to 2 keys (current + previous) to support
seamless key rotation.

### Replay Protection

The NWK security module maintains a **frame counter table** that maps each
sender's IEEE address to the last seen frame counter.  When a secured frame
arrives:

1. `check_frame_counter(source, counter)` — verifies the counter is strictly
   greater than the last seen value
2. If the frame decrypts and verifies successfully:
   `commit_frame_counter(source, counter)` — updates the table

This two-phase approach prevents attackers from advancing the counter with
forged frames that fail MIC verification.

## Summary

The NWK layer handles the "mesh" in Zigbee mesh networking:

| Capability | How |
|------------|-----|
| Find networks | Active scan + beacon parsing |
| Join | MAC association + short address assignment |
| Form (coordinator) | ED scan + PAN creation |
| Route (mesh) | AODV on-demand route discovery |
| Route (tree) | CSkip hierarchical forwarding |
| Track neighbors | Neighbor table with LQI-based costs |
| Encrypt | AES-128-CCM* with network key + frame counter |
| Prevent replay | Per-sender frame counter tracking |

Most of this happens transparently when you call `device.start()` and run the
event loop.  The NWK layer's internal state (NIB, neighbor table, routing table,
security keys) can be inspected for debugging and is automatically persisted
when you call `device.save_state()`.
