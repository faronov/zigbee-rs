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
- **Direct** is used by coordinators to pre-authorize devices.

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
| `update_id` | `u8` | Network update counter | 0 |

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

### Helper Methods

```rust,ignore
nib.next_seq()            // Get next NWK sequence number (wrapping)
nib.next_route_request_id() // Get next route request ID
nib.next_frame_counter()  // Increment frame counter (returns None if exhausted)
```

> **Frame counter exhaustion:** The outgoing frame counter is a 32-bit value.
> If it reaches `u32::MAX`, the device **cannot send any more secured frames**
> and must perform a key update or factory reset.  In practice this takes
> billions of frames and is unlikely, but `next_frame_counter()` returns `None`
> to protect against it.

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
