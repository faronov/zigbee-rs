# The ZDO Layer (Zigbee Device Objects)

The **Zigbee Device Object (ZDO)** layer is your device's "identity card" and
"phone book" combined.  It answers questions like *"Who is at NWK address
0x1A2B?"*, *"What clusters does endpoint 3 support?"*, and *"Please create a
binding between these two devices."*  All of this traffic flows over **APS
endpoint 0** using the **Zigbee Device Profile (ZDP, profile ID 0x0000)**.

```text
┌─────────────────────────────────────────┐
│  Application / ZCL / BDB                │
└───────────────┬─────────────────────────┘
┌───────────────┴─────────────────────────┐
│  ZDO Layer (zigbee-zdo)                 │
│  ├── descriptors   — node/power/simple  │
│  ├── discovery     — addr/desc/EP/match │
│  ├── binding_mgmt  — bind/unbind        │
│  ├── network_mgmt  — mgmt LQI/RTG/…    │
│  ├── device_announce                    │
│  └── handler       — ZDP dispatcher     │
└───────────────┬─────────────────────────┘
                │  APS endpoint 0
┌───────────────┴─────────────────────────┐
│  APS Layer (zigbee-aps)                 │
└─────────────────────────────────────────┘
```

The `zigbee-zdo` crate is `#![no_std]` and builds for bare-metal targets.

## The `ZdoLayer` Struct

`ZdoLayer<M: MacDriver>` owns the APS layer and all ZDO-local state:
descriptors, the endpoint registry, address caches, and a pending
request-response table for correlating ZDP transactions.

```rust
use zigbee_zdo::ZdoLayer;
use zigbee_aps::ApsLayer;

let zdo = ZdoLayer::new(aps_layer);
```

In practice you access it through `BdbLayer`:

```rust
let zdo: &ZdoLayer<_> = bdb.zdo();
let zdo_mut = bdb.zdo_mut();
```

### Key Accessors

| Method | Returns | Purpose |
|---|---|---|
| `zdo.aps()` | `&ApsLayer<M>` | Read APS / NWK state |
| `zdo.aps_mut()` | `&mut ApsLayer<M>` | Send frames, manage bindings |
| `zdo.nwk()` | `&NwkLayer<M>` | Shortcut to NWK layer |
| `zdo.nwk_mut()` | `&mut NwkLayer<M>` | NWK management |
| `zdo.node_descriptor()` | `&NodeDescriptor` | This device's node descriptor |
| `zdo.power_descriptor()` | `&PowerDescriptor` | This device's power descriptor |
| `zdo.get_local_descriptor(ep)` | `Option<&SimpleDescriptor>` | Simple descriptor for an endpoint |

## ZDP Cluster IDs

Every ZDP command has a request cluster ID and a response cluster ID.  The
response ID is always `request_id | 0x8000`:

| Service | Request | Response |
|---|---|---|
| NWK_addr | `0x0000` | `0x8000` |
| IEEE_addr | `0x0001` | `0x8001` |
| Node_Desc | `0x0002` | `0x8002` |
| Power_Desc | `0x0003` | `0x8003` |
| Simple_Desc | `0x0004` | `0x8004` |
| Active_EP | `0x0005` | `0x8005` |
| Match_Desc | `0x0006` | `0x8006` |
| Device_annce | `0x0013` | — |
| Bind | `0x0021` | `0x8021` |
| Unbind | `0x0022` | `0x8022` |
| Mgmt_Lqi | `0x0031` | `0x8031` |
| Mgmt_Rtg | `0x0032` | `0x8032` |
| Mgmt_Bind | `0x0033` | `0x8033` |
| Mgmt_Leave | `0x0034` | `0x8034` |
| Mgmt_Permit_Joining | `0x0036` | `0x8036` |
| Mgmt_NWK_Update | `0x0038` | `0x8038` |

## Incoming Request Policy

`ZdoLayer::handle_indication` decides what to send back from the indication's
own destination address mode and address, compared against this node's short
address. A frame is treated as unicast only when it was individually
addressed; group deliveries and the four NWK broadcast addresses (`0xFFFF`,
`0xFFFD`, `0xFFFC`, `0xFFFB`) are not.

| Incoming frame | Response |
|---|---|
| Supported request, unicast | Normal response cluster |
| Supported discovery broadcast that matches this node | Normal response cluster |
| Broadcast address discovery miss | Silent |
| Broadcast `Mgmt_Permit_Joining_req` | Apply the request, no response |
| Unsupported request cluster, unicast | `cluster \| 0x8000` with `[TSN, NOT_SUPPORTED(0x84)]` |
| Unsupported request cluster, broadcast or group | Dropped |
| Malformed known request, unicast | Rejected without emitting a malformed response |
| Malformed known request, broadcast or group | Dropped |
| Response cluster (bit 15 set) with no pending request | Dropped — never answered |
| `Match_Desc_req` broadcast with no match | Silent |
| `Match_Desc_req` unicast with no match | `NO_MATCH` |

A "not supported" or "no match" reply from every receiver of a broadcast is a
broadcast storm, so those cases stay silent; a unicast requester always gets a
status for unsupported request clusters. Malformed known requests are not
answered with a status-only body because most defined response clusters require
additional mandatory fields even on failure.

## Stack Compliance Revision

Node Descriptor server-mask bits 9..=14 carry the Stack Compliance Revision.
This stack advertises **revision 22** (Zigbee Core R22) via
`zigbee_zdo::descriptors::STACK_COMPLIANCE_REVISION`; a device that leaves the
field at 0 is treated as pre-R21 by certified coordinators. `zigbee-runtime`
sets it in both builder paths (`build()` and `build_into()`).

Service bits are only claimed where the stack provides the service: a
coordinator sets Primary Trust Center (bit 0) because it forms a centralized
network and transports the network key; routers and end devices claim no
services. Network Manager and the binding/discovery cache bits stay clear.

```rust
use zigbee_zdo::descriptors::{NodeDescriptor, STACK_COMPLIANCE_REVISION};

let server_mask = NodeDescriptor::server_mask(
    NodeDescriptor::SERVER_PRIMARY_TRUST_CENTER,
    STACK_COMPLIANCE_REVISION,
); // 0x2C01
```

## Device Discovery

Device discovery lets you translate between the two types of addresses in a
Zigbee network: the 16-bit NWK short address (changes when a device rejoins)
and the 64-bit IEEE extended address (permanent, factory-programmed).

### NWK_addr_req — Find a Short Address

*"I know this device's IEEE address.  What is its current NWK short address?"*

```rust
use zigbee_zdo::discovery::{NwkAddrReq, NwkAddrRsp, RequestType};
use zigbee_types::ShortAddress;

let req = NwkAddrReq {
    ieee_addr: target_ieee,
    request_type: RequestType::Single,
    start_index: 0,
};

// Send to the device itself (or broadcast to 0xFFFD)
let rsp: NwkAddrRsp = zdo.nwk_addr_req(
    ShortAddress(0xFFFD), // broadcast
    &req,
).await?;

println!("Device is at NWK 0x{:04X}", rsp.nwk_addr.0);
```

The response includes the status, the IEEE address echo, and the resolved NWK
address.  If `RequestType::Extended` is used, it also lists associated devices
(children).

### IEEE_addr_req — Find an IEEE Address

The inverse operation: *"I see NWK address 0x1A2B on the network.  What is its
permanent IEEE address?"*

```rust
use zigbee_zdo::discovery::{IeeeAddrReq, RequestType};

let req = IeeeAddrReq {
    nwk_addr_of_interest: ShortAddress(0x1A2B),
    request_type: RequestType::Single,
    start_index: 0,
};

let rsp = zdo.ieee_addr_req(ShortAddress(0x1A2B), &req).await?;
println!("IEEE address: {:02X?}", rsp.ieee_addr);
```

The response type `IeeeAddrRsp` is a type alias for `NwkAddrRsp` — both
carry the same fields.

## Service Discovery

Service discovery answers the question: *"What does this device do?"*

### Node_Desc_req — What Kind of Device?

The Node Descriptor tells you the logical type (Coordinator, Router, or End
Device), frequency band, MAC capabilities, manufacturer code, and buffer sizes.

```rust
use zigbee_zdo::discovery::{NodeDescReq, NodeDescRsp};

let req = NodeDescReq {
    nwk_addr_of_interest: ShortAddress(0x1A2B),
};

let rsp: NodeDescRsp = zdo.node_desc_req(
    ShortAddress(0x1A2B),
    &req,
).await?;

if let Some(desc) = rsp.node_descriptor {
    println!("Logical type: {:?}", desc.logical_type);
    println!("Manufacturer: 0x{:04X}", desc.manufacturer_code);
    println!("Max buffer: {} bytes", desc.max_buffer_size);
}
```

The `NodeDescriptor` struct (13 bytes on the wire):

```rust
pub struct NodeDescriptor {
    pub logical_type: LogicalType,         // Coordinator / Router / EndDevice
    pub complex_desc_available: bool,
    pub user_desc_available: bool,
    pub aps_flags: u8,
    pub frequency_band: u8,               // bit 3 = 2.4 GHz
    pub mac_capabilities: u8,
    pub manufacturer_code: u16,
    pub max_buffer_size: u8,
    pub max_incoming_transfer: u16,
    pub server_mask: u16,
    pub max_outgoing_transfer: u16,
    pub descriptor_capabilities: u8,
}
```

### Simple_Desc_req — What Clusters on This Endpoint?

The Simple Descriptor is the most important descriptor for application
interoperability.  It tells you the profile, device type, and the exact list
of input (server) and output (client) clusters on an endpoint.

```rust
use zigbee_zdo::discovery::{SimpleDescReq, SimpleDescRsp};

let req = SimpleDescReq {
    nwk_addr_of_interest: ShortAddress(0x1A2B),
    endpoint: 1,
};

let rsp: SimpleDescRsp = zdo.simple_desc_req(
    ShortAddress(0x1A2B),
    &req,
).await?;

if let Some(desc) = rsp.simple_descriptor {
    println!("Profile: 0x{:04X}", desc.profile_id);
    println!("Device ID: 0x{:04X}", desc.device_id);
    println!("Input clusters: {:04X?}", desc.input_clusters);
    println!("Output clusters: {:04X?}", desc.output_clusters);
}
```

The `SimpleDescriptor` struct:

```rust
pub struct SimpleDescriptor {
    pub endpoint: u8,
    pub profile_id: u16,           // e.g. 0x0104 (HA)
    pub device_id: u16,            // e.g. 0x0302 (Temperature Sensor)
    pub device_version: u8,
    pub input_clusters: Vec<u16, MAX_CLUSTERS>,   // server clusters
    pub output_clusters: Vec<u16, MAX_CLUSTERS>,  // client clusters
}
```

Up to **`MAX_CLUSTERS` (16)** input and 16 output clusters per descriptor.

### Active_EP_req — Which Endpoints Are Active?

Before you can query simple descriptors, you need to know which endpoints exist:

```rust
use zigbee_zdo::discovery::ActiveEpRsp;

let rsp: ActiveEpRsp = zdo.active_ep_req(ShortAddress(0x1A2B)).await?;

for &ep in &rsp.active_ep_list {
    println!("Endpoint {} is active", ep);
    // Now query Simple_Desc for each endpoint
}
```

### Match_Desc_req — Find Compatible Endpoints

*"Who on this device (or the whole network) supports the On/Off cluster?"*

```rust
use zigbee_zdo::discovery::{MatchDescReq, MatchDescRsp};

let req = MatchDescReq {
    nwk_addr_of_interest: ShortAddress(0xFFFD), // broadcast
    profile_id: 0x0104,                         // Home Automation
    input_clusters: vec![0x0006].into(),         // On/Off (server)
    output_clusters: heapless::Vec::new(),
};

let rsp: MatchDescRsp = zdo.match_desc_req(
    ShortAddress(0xFFFD),
    &req,
).await?;

for &ep in &rsp.match_list {
    println!("Matching endpoint: {}", ep);
}
```

## Other Descriptors

### PowerDescriptor

Reports the device's power configuration (2 bytes on the wire):

```rust
pub struct PowerDescriptor {
    pub current_power_mode: u8,         // 0 = on, synced with receiver
    pub available_power_sources: u8,    // bitmask (mains, battery, …)
    pub current_power_source: u8,       // which source is active
    pub current_power_level: u8,        // 0x0C = 100%
}
```

### ComplexDescriptor

A list of compressed XML tags describing additional device capabilities
(rarely used in practice):

```rust
pub struct ComplexDescriptor {
    pub data: Vec<u8, 64>,  // raw bytes
}
```

### UserDescriptor

Up to 16 characters of user-settable text (like a friendly name):

```rust
pub struct UserDescriptor {
    pub data: Vec<u8, 16>,  // ASCII text
}
```

## Binding Management

ZDP provides over-the-air commands to create and remove bindings on remote
devices.  These are different from the local `APSME-BIND` / `APSME-UNBIND` —
here you're asking *another device* to update *its* binding table.

### Bind_req

```rust
use zigbee_zdo::binding_mgmt::{BindReq, BindRsp, BindTarget};

let req = BindReq {
    src_addr: sensor_ieee,       // source device (the one creating the binding)
    src_endpoint: 1,
    cluster_id: 0x0402,          // Temperature Measurement
    dst: BindTarget::Unicast {
        dst_addr: gateway_ieee,
        dst_endpoint: 1,
    },
};

// Send Bind_req to the sensor device
let rsp: BindRsp = zdo.bind_req(sensor_nwk_addr, &req).await?;
assert_eq!(rsp.status, ZdpStatus::Success);
```

The `BindTarget` enum mirrors APS binding destinations:

```rust
pub enum BindTarget {
    Group(u16),
    Unicast { dst_addr: IeeeAddress, dst_endpoint: u8 },
}
```

### Unbind_req

Structurally identical to Bind_req — `UnbindReq` is a type alias for `BindReq`,
and `UnbindRsp` is a type alias for `BindRsp`:

```rust
let rsp = zdo.unbind_req(sensor_nwk_addr, &req).await?;
```

## Network Management

Network management commands let you query and control the Zigbee mesh topology.
These are essential tools for diagnostics and network administration.

### Mgmt_Lqi_req — Neighbor Table

Query a device's neighbor table to map the mesh topology.  Each record
includes the neighbor's address, link quality (LQI), relationship, and
device type.

```rust
use zigbee_zdo::network_mgmt::{MgmtLqiReq, MgmtLqiRsp, NeighborTableRecord};

let req = MgmtLqiReq { start_index: 0 };
let rsp: MgmtLqiRsp = zdo.mgmt_lqi_req(ShortAddress(0x0000), &req).await?;

for neighbor in &rsp.neighbor_table_list {
    println!(
        "  0x{:04X} LQI={} type={} rel={}",
        neighbor.network_addr.0,
        neighbor.lqi,
        neighbor.device_type,     // 0=Coord, 1=Router, 2=ED
        neighbor.relationship,    // 0=parent, 1=child, 2=sibling
    );
}
```

The `NeighborTableRecord` (22 bytes each):

```rust
pub struct NeighborTableRecord {
    pub extended_pan_id: [u8; 8],
    pub extended_addr: IeeeAddress,
    pub network_addr: ShortAddress,
    pub device_type: u8,        // 2-bit: 0=Coord, 1=Router, 2=EndDevice
    pub rx_on_when_idle: u8,    // 2-bit: 0=off, 1=on, 2=unknown
    pub relationship: u8,       // 3-bit: 0=parent, 1=child, 2=sibling
    pub permit_joining: u8,     // 2-bit: 0=no, 1=yes, 2=unknown
    pub depth: u8,
    pub lqi: u8,
}
```

### Mgmt_Rtg_req — Routing Table

Query a router's routing table to understand message paths:

```rust
use zigbee_zdo::network_mgmt::MgmtRtgReq;

let req = MgmtRtgReq { start_index: 0 };
let rsp = zdo.mgmt_rtg_req(ShortAddress(0x0000), &req).await?;

for route in &rsp.routing_table_list {
    println!(
        "  dst=0x{:04X} → next_hop=0x{:04X} status={:?}",
        route.dst_addr.0,
        route.next_hop_addr.0,
        route.status,
    );
}
```

### Mgmt_Bind_req — Remote Binding Table

Query another device's binding table:

```rust
use zigbee_zdo::network_mgmt::MgmtBindReq;

let req = MgmtBindReq { start_index: 0 };
let rsp = zdo.mgmt_bind_req(ShortAddress(0x1A2B), &req).await?;

for entry in &rsp.binding_table_list {
    println!(
        "  ep {} cluster 0x{:04X} → {:?}",
        entry.src_endpoint,
        entry.cluster_id,
        entry.dst,
    );
}
```

### Mgmt_Leave_req — Ask a Device to Leave

Tell a device to leave the network (optionally removing its children too):

```rust
use zigbee_zdo::network_mgmt::MgmtLeaveReq;

let req = MgmtLeaveReq {
    device_address: device_ieee,
    remove_children: false,
    rejoin: false,
};

let rsp = zdo.mgmt_leave_req(ShortAddress(0x1A2B), &req).await?;
```

### Mgmt_Permit_Joining_req — Open/Close the Network

Control whether new devices can join through a particular router (or the whole
network via broadcast):

```rust
// Open the whole network for 180 seconds
zdo.mgmt_permit_joining_req(
    ShortAddress(0xFFFC),  // broadcast to all routers
    180,                   // duration in seconds
    true,                  // TC significance
).await?;

// Close the network
zdo.mgmt_permit_joining_req(
    ShortAddress(0xFFFC),
    0,     // 0 = close
    true,
).await?;
```

## `ZdpStatus` — All Variants

Every ZDP response carries a status code:

| Variant | Value | Meaning |
|---|---|---|
| `Success` | `0x00` | Request completed successfully |
| `InvRequestType` | `0x80` | Invalid request type field |
| `DeviceNotFound` | `0x81` | No device with the requested address |
| `InvalidEp` | `0x82` | Endpoint is not valid (0 or > 240) |
| `NotActive` | `0x83` | Endpoint exists but is not active |
| `NotSupported` | `0x84` | Request not supported on this device |
| `Timeout` | `0x85` | Request timed out |
| `NoMatch` | `0x86` | No matching descriptors found |
| `TableFull` | `0x87` | Binding / neighbor / routing table is full |
| `NoEntry` | `0x88` | No matching entry found (unbind, remove) |
| `NoDescriptor` | `0x89` | Requested descriptor does not exist |

## `ZdoError`

Operations that fail locally (before reaching the network) return `ZdoError`:

```rust
pub enum ZdoError {
    BufferTooSmall,             // serialization buffer too small
    InvalidLength,              // frame shorter than expected
    InvalidData,                // reserved / malformed field
    ApsError(ApsStatus),        // APS layer error
    TableFull,                  // local table capacity exceeded
}
```

## ZDP Transaction Sequence Numbers

Every ZDP exchange is correlated by a **Transaction Sequence Number (TSN)**.
The ZDO layer manages this automatically — you don't need to track TSNs
yourself.  Internally, `ZdoLayer` maintains a pending-response table (up to
4 concurrent requests) that matches incoming response TSNs to outstanding
requests.

```rust
// TSN is allocated and tracked internally
let tsn = zdo.next_seq(); // wrapping u8 counter
```

## Device Announce

When a device joins (or rejoins) a network, it broadcasts a `Device_annce`
(cluster `0x0013`) to inform all routers of its presence:

```rust
// Automatically called by BDB steering on join, but available manually:
zdo.device_annce(my_nwk_addr, my_ieee_addr).await?;
```

This is a one-way broadcast — there is no response.

## Complete Example: Discovering a New Device

Here's how you'd discover everything about a device that just joined your
network:

```rust
// 1. We received a Device_annce — we know the NWK addr and IEEE addr
let device_nwk = ShortAddress(0x5E3F);
let device_ieee = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];

// 2. Get the node descriptor — what type of device is it?
let node_desc = zdo.node_desc_req(device_nwk, &NodeDescReq {
    nwk_addr_of_interest: device_nwk,
}).await?;

if let Some(nd) = node_desc.node_descriptor {
    match nd.logical_type {
        LogicalType::EndDevice => println!("It's an end device"),
        LogicalType::Router => println!("It's a router"),
        LogicalType::Coordinator => println!("It's a coordinator"),
    }
}

// 3. Enumerate active endpoints
let eps = zdo.active_ep_req(device_nwk).await?;

// 4. Get the simple descriptor for each endpoint
for &ep in &eps.active_ep_list {
    let sd = zdo.simple_desc_req(device_nwk, &SimpleDescReq {
        nwk_addr_of_interest: device_nwk,
        endpoint: ep,
    }).await?;

    if let Some(desc) = sd.simple_descriptor {
        println!("Endpoint {}: profile=0x{:04X} device=0x{:04X}",
            ep, desc.profile_id, desc.device_id);
        println!("  Server clusters: {:04X?}", desc.input_clusters);
        println!("  Client clusters: {:04X?}", desc.output_clusters);
    }
}

// 5. Create a binding if we find a matching cluster
//    (BDB Finding & Binding does this automatically)
```

## Parent Announce (`Parent_annce` 0x001F / `Parent_annce_rsp` 0x801F)

`Parent_annce` is the R22 ZDP primitive a router/coordinator uses to reconcile
its child table with the rest of the network after it restores or rebuilds it.
It is a **ZDP** command carried over APS on endpoint 0 (not a NWK-layer
command), restricted to routing devices — end devices have no child table, so
`send_parent_annce()` is a no-op on them and the receive path is gated on
`can_route()`.

| Cluster | Payload |
|---------|---------|
| `Parent_annce` 0x001F | `NumberOfChildren(1)` then that many `IEEE(8)` |
| `Parent_annce_rsp` 0x801F | `Status(1)`, `NumberOfChildren(1)`, then `IEEE(8)` |

Behaviour (`zigbee_zdo::parent_annce`):

- **Send** — after a router's restored child state is authoritative,
  `ZigbeeDevice::announce_parent()` (also fired once automatically by the joined
  tick following `restore_child_table`) broadcasts the IEEE addresses of its
  authenticated children.
- **Receive `Parent_annce`** — for each announced child the receiver also
  holds: if it has **confirmed** the child's liveness this power cycle it keeps
  the child and reports it in a `Parent_annce_rsp`; otherwise (an unconfirmed
  record just restored from flash) it yields the child to the announcer and
  evicts it. This confirmed-liveness gate is what makes simultaneous reboots
  safe — neither router drops a child it is actually keeping alive.
- **Receive `Parent_annce_rsp`** — the announcer relinquishes every child the
  responder is keeping.

The confirmed-liveness bit is set by any keepalive (poll, End Device Timeout
Request, valid secured traffic) or live admission, and is `false` for a child
restored from persistence until it is next heard from.

## Summary

The ZDO layer provides the essential management infrastructure for every
Zigbee network:

- **Device discovery** (`NWK_addr_req`, `IEEE_addr_req`) translates between
  address types.
- **Service discovery** (`Node_Desc`, `Simple_Desc`, `Active_EP`, `Match_Desc`)
  reveals device capabilities.
- **Binding management** (`Bind_req`, `Unbind_req`) creates and removes
  over-the-air bindings.
- **Network management** (`Mgmt_Lqi`, `Mgmt_Rtg`, `Mgmt_Bind`, `Mgmt_Leave`,
  `Mgmt_Permit_Joining`) monitors and controls the mesh.
- **Descriptors** (`NodeDescriptor`, `PowerDescriptor`, `SimpleDescriptor`,
  `ComplexDescriptor`, `UserDescriptor`) describe each device's identity and
  capabilities.

In the next chapter, we'll look at [BDB Commissioning](bdb.md) — the
standardized process of getting devices onto the network and binding them
together.
