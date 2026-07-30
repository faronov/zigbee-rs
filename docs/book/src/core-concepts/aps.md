# The APS Layer

The **Application Support Sub-layer (APS)** is the bridge between your
application and the Zigbee network layer.  Every time you send a ZCL command,
report an attribute, or receive a message from another device, the data passes
through the APS layer.  Think of it as the *postal service* of a Zigbee
network — it handles addresses, tracks deliveries, encrypts letters, and
reassembles oversized packages.

```text
┌──────────────────────────────────────┐
│  ZDO / ZCL / Application             │
└──────────────┬───────────────────────┘
               │ APSDE-DATA / APSME-*
┌──────────────┴───────────────────────┐
│  APS Layer (zigbee-aps)              │
│  ├── apsde:     data service         │
│  ├── apsme:     management entity    │
│  ├── aib:       APS information base │
│  ├── binding:   binding table        │
│  ├── group:     group table          │
│  ├── fragment:  reassembly           │
│  └── security:  APS encryption       │
└──────────────┬───────────────────────┘
               │ NLDE-DATA / NLME-*
┌──────────────┴───────────────────────┐
│  NWK Layer (zigbee-nwk)              │
└──────────────────────────────────────┘
```

The `zigbee-aps` crate is a `#![no_std]` library — it compiles for bare-metal
MCUs just like the rest of zigbee-rs.

## The `ApsLayer` Struct

`ApsLayer<M: MacDriver>` is the central type.  It owns the NWK layer and all
APS-level state: the binding table, the group table, security material,
duplicate-rejection, ACK tracking, and fragment reassembly.

```rust
use zigbee_aps::ApsLayer;
use zigbee_nwk::NwkLayer;

// Create the APS layer by wrapping an existing NWK layer.
let aps = ApsLayer::new(nwk_layer);
```

You rarely construct `ApsLayer` directly — the higher-level `ZdoLayer` and
`BdbLayer` wrap it for you.  But you can always reach down:

```rust
// From a BdbLayer, reach the APS layer
let aps: &ApsLayer<_> = bdb.zdo().aps();

// Or get a mutable reference
let aps_mut = bdb.zdo_mut().aps_mut();
```

### Important Accessors

| Method | Returns | Purpose |
|---|---|---|
| `aps.nwk()` | `&NwkLayer<M>` | Read NWK state (NIB, neighbor table, …) |
| `aps.nwk_mut()` | `&mut NwkLayer<M>` | Send NWK frames, join/leave |
| `aps.aib()` | `&Aib` | Read APS Information Base attributes |
| `aps.aib_mut()` | `&mut Aib` | Write AIB attributes |
| `aps.binding_table()` | `&BindingTable` | Inspect binding entries |
| `aps.binding_table_mut()` | `&mut BindingTable` | Add/remove bindings |
| `aps.group_table()` | `&GroupTable` | Inspect group memberships |
| `aps.group_table_mut()` | `&mut GroupTable` | Add/remove groups |
| `aps.security()` | `&ApsSecurity` | Inspect link keys |
| `aps.security_mut()` | `&mut ApsSecurity` | Add/remove link keys |
| `aps.fragment_rx()` | `&FragmentReassembly` | Inspect reassembly state |

## Addressing Modes

When you send data through the APS layer, you choose an *addressing mode* that
tells the layer how to find the destination.  The `ApsAddressMode` enum
captures the four modes defined by the Zigbee specification:

```rust
#[repr(u8)]
pub enum ApsAddressMode {
    /// Indirect — look up destinations in the binding table
    Indirect  = 0x00,
    /// Group — deliver to all members of a 16-bit group
    Group     = 0x01,
    /// Direct (short) — 16-bit NWK address + endpoint
    Short     = 0x02,
    /// Direct (extended) — 64-bit IEEE address + endpoint
    Extended  = 0x03,
}
```

And `ApsAddress` carries the actual address value:

```rust
pub enum ApsAddress {
    Short(ShortAddress),    // e.g. ShortAddress(0x1A2B)
    Extended(IeeeAddress),  // e.g. [0x00, 0x12, …, 0xFF]
    Group(u16),             // e.g. 0x0001
}
```

### Direct Addressing (Unicast)

The most common mode.  You specify the recipient's 16-bit short address (or
64-bit IEEE address) and endpoint number.  The message is delivered to exactly
one device, one endpoint.

```rust
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};
use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_types::ShortAddress;

let payload = [0x01, 0x00]; // ZCL frame bytes

let req = ApsdeDataRequest {
    dst_addr_mode: ApsAddressMode::Short,
    dst_address: ApsAddress::Short(ShortAddress(0x1A2B)),
    dst_endpoint: 1,
    profile_id: 0x0104,        // Home Automation
    cluster_id: 0x0006,        // On/Off cluster
    src_endpoint: 1,
    payload: &payload,
    tx_options: ApsTxOptions {
        ack_request: true,     // request APS-level ACK
        ..ApsTxOptions::default()
    },
    radius: 0,                 // 0 = use default NWK radius
    alias_src_addr: None,
    alias_seq: None,
};

// Send — returns Ok(()) on success
aps.apsde_data_request(&req).await?;
```

### Indirect Addressing (via Binding Table)

With indirect addressing you don't specify a destination at all.  Instead the
APS layer looks up matching entries in the **binding table** and delivers the
frame to every matching destination.  This is the mode used by Finding &
Binding (EZ-Mode).

```rust
let req = ApsdeDataRequest {
    dst_addr_mode: ApsAddressMode::Indirect,
    dst_address: ApsAddress::Short(ShortAddress(0x0000)), // ignored
    dst_endpoint: 0,  // ignored — determined by binding table
    profile_id: 0x0104,
    cluster_id: 0x0006,
    src_endpoint: 1,   // looked up in binding table
    payload: &payload,
    tx_options: ApsTxOptions::default(),
    radius: 0,
    alias_src_addr: None,
    alias_seq: None,
};
```

When this request is processed, the APS layer calls
`binding_table.find_by_source(our_ieee, src_endpoint, cluster_id)` and sends
the frame to each destination returned by the iterator.

### Group Addressing (Multicast)

Group addressing delivers the message to every device that has registered the
given group address in its group table.  This is how Zigbee "rooms" and
"scenes" work — a single frame reaches all the lights in the living room.

```rust
let req = ApsdeDataRequest {
    dst_addr_mode: ApsAddressMode::Group,
    dst_address: ApsAddress::Group(0x0001), // group 1
    dst_endpoint: 0xFF,                     // broadcast endpoint
    // ...
    tx_options: ApsTxOptions::default(),    // no ACK for groups
    ..
};
```

### Broadcast

Broadcast is not a separate `ApsAddressMode` variant — you use
`ApsAddressMode::Short` with one of the well-known broadcast NWK addresses:

| Address | Meaning |
|---|---|
| `0xFFFF` | All devices |
| `0xFFFD` | All rx-on-when-idle devices (routers + mains-powered EDs) |
| `0xFFFC` | All routers (+ coordinator) |

## Well-Known Endpoints and Profiles

The APS layer defines several constants you'll encounter frequently:

```rust
pub const ZDO_ENDPOINT: u8      = 0x00;  // Zigbee Device Object
pub const MIN_APP_ENDPOINT: u8  = 0x01;  // First application endpoint
pub const MAX_APP_ENDPOINT: u8  = 0xF0;  // Last application endpoint
pub const BROADCAST_ENDPOINT: u8 = 0xFF; // Deliver to all endpoints

pub const PROFILE_ZDP: u16              = 0x0000; // Zigbee Device Profile
pub const PROFILE_HOME_AUTOMATION: u16  = 0x0104; // HA profile
pub const PROFILE_SMART_ENERGY: u16     = 0x0109; // SE profile
pub const PROFILE_ZLL: u16             = 0xC05E; // Zigbee Light Link
pub const PROFILE_WILDCARD: u16        = 0xFFFF; // matches any profile
```

Endpoint 0 is always reserved for ZDO (the Zigbee Device Object that handles
discovery, binding, and management).  Your application clusters live on
endpoints 1–240.

## TX Options

The `ApsTxOptions` bitfield controls per-frame behavior:

```rust
pub struct ApsTxOptions {
    pub security_enabled: bool,       // APS link-key encryption
    pub use_nwk_key: bool,            // Use NWK key instead of link key
    pub ack_request: bool,            // Request an APS ACK
    pub fragmentation_permitted: bool, // Allow automatic fragmentation
    pub include_extended_nonce: bool,  // Include IEEE addr in security header
}
```

If `ack_request` is `true`, the APS layer registers the frame in an internal
ACK-tracking table (up to 8 concurrent pending ACKs) and will retransmit up
to 3 times if the ACK doesn't arrive.

## The Binding Table

The binding table maps *(source address, source endpoint, cluster)* to one or
more destinations.  It is the backbone of **indirect addressing** — when you
send a frame with `ApsAddressMode::Indirect`, the APS layer looks up matching
entries here.

### Data Model

```rust
/// A single binding table entry.
pub struct BindingEntry {
    pub src_addr: IeeeAddress,     // our IEEE address
    pub src_endpoint: u8,          // our endpoint (1-240)
    pub cluster_id: u16,           // e.g. 0x0006 (On/Off)
    pub dst_addr_mode: BindingDstMode,
    pub dst: BindingDst,           // where to send
}

/// Destination can be unicast or group.
pub enum BindingDst {
    Group(u16),
    Unicast { dst_addr: IeeeAddress, dst_endpoint: u8 },
}
```

The table holds up to **`MAX_BINDING_ENTRIES` (32)** entries in a fixed-capacity
`heapless::Vec` — no heap allocation.

### Creating Bindings

There are two convenient constructors:

```rust
use zigbee_aps::binding::BindingEntry;

// Unicast binding: our ep 1, On/Off cluster → remote device ep 1
let entry = BindingEntry::unicast(
    our_ieee,           // source IEEE address
    1,                  // source endpoint
    0x0006,             // On/Off cluster
    remote_ieee,        // destination IEEE address
    1,                  // destination endpoint
);

// Group binding: our ep 1, On/Off cluster → group 0x0001
let entry = BindingEntry::group(
    our_ieee,
    1,
    0x0006,
    0x0001,             // group address
);
```

### Managing Bindings

Through the `BindingTable`:

```rust
let table = aps.binding_table_mut();

// Add — returns Err if table full or duplicate
table.add(entry)?;

// Remove — returns true if found
table.remove(&src_addr, src_endpoint, cluster_id, &dst);

// Query
for entry in table.find_by_source(&our_ieee, 1, 0x0006) {
    // each matching destination for indirect sends
}

// Iterate all entries
for entry in table.entries() {
    println!("{:?}", entry);
}

table.len();       // number of entries
table.is_full();   // true if 32 entries
table.clear();     // remove all
```

### APSME Bind / Unbind

The formal Zigbee management primitives go through `ApsLayer` methods:

```rust
use zigbee_aps::apsme::{ApsmeBindRequest, ApsmeBindConfirm};
use zigbee_aps::binding::BindingDstMode;

let req = ApsmeBindRequest {
    src_addr: our_ieee,
    src_endpoint: 1,
    cluster_id: 0x0006,
    dst_addr_mode: BindingDstMode::Extended,
    dst_addr: remote_ieee,
    dst_endpoint: 1,
    group_address: 0,
};

let confirm: ApsmeBindConfirm = aps.apsme_bind(&req);
assert_eq!(confirm.status, ApsStatus::Success);

// To unbind:
let confirm = aps.apsme_unbind(&req);
```

## The Group Table

The group table maps 16-bit group addresses to local endpoints.  When a frame
arrives addressed to a group, the APS layer delivers it to each endpoint that
is a member of that group.

```rust
let gt = aps.group_table_mut();

// Add endpoint 1 to group 0x0001
assert!(gt.add_group(0x0001, 1));

// Add endpoint 2 to the same group
assert!(gt.add_group(0x0001, 2));

// Check membership
assert!(gt.is_member(0x0001, 1));  // true
assert!(!gt.is_member(0x0001, 3)); // false

// Remove endpoint 1 from the group
gt.remove_group(0x0001, 1);

// Remove endpoint 2 from ALL groups at once
gt.remove_all_groups(2);

// Inspect
for group in gt.groups() {
    println!("Group 0x{:04X}: endpoints {:?}",
        group.group_address,
        group.endpoint_list);
}
```

Capacity: up to **`MAX_GROUPS` (16)** groups, each with up to
**`MAX_ENDPOINTS_PER_GROUP` (8)** endpoints.

### APSME Group Management

The formal management primitives:

```rust
use zigbee_aps::apsme::{ApsmeAddGroupRequest, ApsmeRemoveGroupRequest};

let confirm = aps.apsme_add_group(&ApsmeAddGroupRequest {
    group_address: 0x0001,
    endpoint: 1,
});
assert_eq!(confirm.status, ApsStatus::Success);

let confirm = aps.apsme_remove_group(&ApsmeRemoveGroupRequest {
    group_address: 0x0001,
    endpoint: 1,
});
```

## Fragmentation

The APS layer automatically splits large payloads into fragments and
reassembles them at the receiver.  The `FragmentReassembly` context manages
up to **4 concurrent reassembly sessions**, each tracking up to 8 blocks via a
bitmask.

You don't call fragmentation directly — it works behind the scenes:

1. **Sender side**: When `ApsTxOptions::fragmentation_permitted` is `true` and
   the payload exceeds the NWK maximum transfer unit, the APS layer splits it
   into numbered blocks.

2. **Receiver side**: The `FragmentReassembly` module collects blocks identified
   by `(src_addr, aps_counter)`.  The first fragment (`block_num == 0`)
   creates a reassembly slot; subsequent fragments fill in the bitmask.  When
   all blocks arrive, the complete payload is returned.

3. **Timeout**: Call `fragment_rx_mut().age_entries()` once per second from your
   event loop.  Incomplete reassemblies are expired after **10 seconds** of
   inactivity.

```rust
// In your 1-second tick handler:
aps.fragment_rx_mut().age_entries();
aps.age_dup_table();
```

### Fragment API (Advanced)

If you need to inspect reassembly state:

```rust
let frag = aps.fragment_rx_mut();

// Insert a fragment — returns Some(&[u8]) when complete
if let Some(full_payload) = frag.insert_fragment(
    src_addr,        // sender short addr
    aps_counter,     // APS counter
    block_num,       // 0 for first fragment
    total_blocks,    // total blocks (first frag only)
    &fragment_data,
) {
    // Process the reassembled payload
    process(full_payload);
    // Free the slot
    frag.complete_entry(src_addr, aps_counter);
}
```

## APS Security

APS security provides **end-to-end encryption** between two specific devices,
on top of the network-wide NWK encryption.  While every device on the network
shares the NWK key, APS link keys are known only to the two communicating
parties.

### Key Types

```rust
pub enum ApsKeyType {
    TrustCenterMasterKey    = 0x00, // pre-installed
    TrustCenterLinkKey      = 0x01, // shared with TC
    NetworkKey              = 0x02, // shared by all devices
    ApplicationLinkKey      = 0x03, // between two app devices
    DistributedGlobalLinkKey = 0x04, // for distributed TC networks
}
```

The **well-known default TC link key** is the ASCII string `"ZigBeeAlliance09"`:

```rust
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = [
    0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
    0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39,
];
```

Every Zigbee 3.0 device ships with this key pre-installed.  During joining,
the Trust Center encrypts the actual network key with this well-known key so
it can be delivered securely over the air.

### Link Key Table

The `ApsSecurity` struct maintains a table of up to
**`MAX_KEY_TABLE_ENTRIES` (16)** link keys:

```rust
pub struct ApsLinkKeyEntry {
    pub partner_address: IeeeAddress,
    pub key: [u8; 16],
    pub key_type: ApsKeyType,
    pub outgoing_frame_counter: u32,
    pub incoming_frame_counter: u32,
}
```

Each entry pairs a partner's IEEE address with a 128-bit AES key and
independent frame counters for replay protection.

### Key Management Primitives

```rust
// Distribute a key to another device
aps.apsme_transport_key(&ApsmeTransportKeyRequest {
    dst_address: remote_ieee,
    key_type: ApsKeyType::ApplicationLinkKey,
    key: my_app_key,
    key_seq_number: 0, // used only for NetworkKey
}).await;

// Request a key from the Trust Center
aps.apsme_request_key(&ApsmeRequestKeyRequest {
    dst_address: tc_ieee,
    key_type: ApsKeyType::TrustCenterLinkKey,
    partner_address: None,
}).await;

// Switch all devices to a new network key
aps.apsme_switch_key(&ApsmeSwitchKeyRequest {
    dst_address: broadcast_ieee,
    key_seq_number: 1,
}).await;

// Verify a TC link key
aps.apsme_verify_key(&ApsmeVerifyKeyRequest {
    dst_address: tc_ieee,
    key_type: ApsKeyType::TrustCenterLinkKey,
}).await;
```

### APS Security Header

When APS security is enabled, an auxiliary header is prepended to the payload:

```text
┌───────────────────────────────────────────────────────┐
│ Security Control (1 byte)                              │
│  ├── Security Level    (bits 0-2)                      │
│  ├── Key Identifier    (bits 3-4)                      │
│  └── Extended Nonce    (bit 5)                          │
├───────────────────────────────────────────────────────┤
│ Frame Counter (4 bytes LE)                              │
│ Source Address (8 bytes) — if Extended Nonce bit set    │
│ Key Sequence Number (1 byte) — if Key ID = Network Key │
└───────────────────────────────────────────────────────┘
```

The default security level is **ENC-MIC-32** (AES-CCM encryption + 4-byte
message integrity code).

## The APS Information Base (AIB)

The `Aib` struct holds all APS-layer configuration, analogous to the MAC PIB
and NWK NIB:

```rust
pub struct Aib {
    pub aps_designated_coordinator: bool,  // true if this is the TC
    pub aps_channel_mask: u32,             // 2.4 GHz channel bitmask
    pub aps_use_extended_pan_id: IeeeAddress,
    pub aps_use_insecure_join: bool,       // default: true
    pub aps_interframe_delay: u8,          // ms between frames (default: 10)
    pub aps_max_window_size: u8,           // fragmentation window (default: 8)
    pub aps_max_frame_retries: u8,         // fragment retries (default: 3)
    pub aps_duplicate_rejection_timeout: u16, // ms (default: 3000)
    pub aps_trust_center_address: IeeeAddress,
    pub aps_security_enabled: bool,        // default: true
    pub aps_outgoing_frame_counter: u32,
    // ... channel quality attributes
}
```

Read and write attributes through the APSME-GET / APSME-SET interface:

```rust
use zigbee_aps::aib::AibAttribute;

// Read
let is_tc = aps.apsme_get_bool(AibAttribute::ApsDesignatedCoordinator)?;
let window = aps.apsme_get_u8(AibAttribute::ApsMaxWindowSize)?;
let mask = aps.apsme_get_u32(AibAttribute::ApsChannelMaskList)?;

// Write
aps.apsme_set_bool(AibAttribute::ApsSecurityEnabled, true);
aps.apsme_set_u8(AibAttribute::ApsInterframeDelay, 20);
```

## `ApsStatus` — All Variants

Every APS operation returns an `ApsStatus` code:

| Variant | Value | Meaning |
|---|---|---|
| `Success` | `0x00` | Request executed successfully |
| `AsduTooLong` | `0xA0` | Payload too large and fragmentation not supported |
| `DefragDeferred` | `0xA1` | Received fragment could not be defragmented |
| `DefragUnsupported` | `0xA2` | Device does not support fragmentation |
| `IllegalRequest` | `0xA3` | A parameter value was out of range |
| `InvalidBinding` | `0xA4` | Unbind failed — entry not found |
| `InvalidParameter` | `0xA5` | Unknown AIB attribute identifier |
| `NoAck` | `0xA6` | APS ACK not received (after retries) |
| `NoBoundDevice` | `0xA7` | Indirect send but no matching binding entry |
| `NoShortAddress` | `0xA8` | Group send but no matching group entry |
| `TableFull` | `0xA9` | Binding or group table is full |
| `UnsecuredKey` | `0xAA` | Frame secured with link key but key not found |
| `UnsupportedAttribute` | `0xAB` | Unknown AIB attribute in GET/SET |
| `SecurityFail` | `0xAD` | Unsecured frame received |
| `DecryptionError` | `0xAE` | APS frame decryption or authentication failed |
| `InsufficientSpace` | `0xAF` | Not enough buffers for the operation |
| `NotFound` | `0xB0` | No matching entry in binding table |

## Duplicate Rejection

The APS layer maintains a **duplicate rejection table** (16 entries) that
remembers recently seen `(src_addr, aps_counter)` pairs.  This prevents
delivering the same frame twice when NWK-level retransmission is active.

Call `aps.age_dup_table()` periodically (every ~1 second) to expire stale
entries.  The timeout is controlled by `aib.aps_duplicate_rejection_timeout`
(default: 3000 ms).

## ACK Tracking and Retransmission

When you send with `ack_request: true`, the APS layer:

1. Registers the frame in the ACK table (up to 8 slots)
2. Starts a retry counter (default: 3 retries)
3. If no ACK arrives within one tick, retransmits the original frame
4. After all retries, reports `ApsStatus::NoAck`

```rust
// In your periodic tick handler:
let retransmissions = aps.age_ack_table();
for frame_bytes in retransmissions {
    // The APS layer has already prepared these for retransmission
    aps.nwk_mut().nlde_data_request(/* ... */).await;
}
```

## Putting It Together

Here's a complete example of an APS-layer interaction in a typical Zigbee
application:

```rust
// 1. Set up security
aps.security_mut().install_default_tc_link_key();

// 2. Join a network (handled by BDB steering, but conceptually:)
//    ... network steering happens ...

// 3. Create a binding for attribute reports
let entry = BindingEntry::unicast(
    our_ieee, 1, 0x0402, // Temperature Measurement cluster
    coordinator_ieee, 1,
);
aps.binding_table_mut().add(entry).unwrap();

// 4. Send a temperature report via indirect addressing
let report_payload = build_zcl_report(temperature);
let req = ApsdeDataRequest {
    dst_addr_mode: ApsAddressMode::Indirect,
    dst_address: ApsAddress::Short(ShortAddress(0)),
    dst_endpoint: 0,
    profile_id: 0x0104,
    cluster_id: 0x0402,
    src_endpoint: 1,
    payload: &report_payload,
    tx_options: ApsTxOptions {
        ack_request: true,
        ..Default::default()
    },
    radius: 0,
    alias_src_addr: None,
    alias_seq: None,
};
aps.apsde_data_request(&req).await?;

// 5. Periodic maintenance (call every ~1 second)
aps.age_dup_table();
aps.fragment_rx_mut().age_entries();
let retx = aps.age_ack_table();
```

## Summary

The APS layer is the workhorse of the Zigbee application stack:

- **Four addressing modes** let you send unicast, multicast, indirect, and
  broadcast messages.
- **The binding table** powers indirect addressing and Finding & Binding.
- **The group table** enables room-level multicast.
- **Fragmentation** transparently handles large payloads.
- **APS security** provides end-to-end link-key encryption.
- **ACK tracking** and **duplicate rejection** ensure reliable delivery.

In the next chapter, we'll look at the [ZDO layer](zdo.md), which sits on top
of APS endpoint 0 and provides device discovery and network management.
