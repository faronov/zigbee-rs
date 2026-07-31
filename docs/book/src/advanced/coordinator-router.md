# Coordinator & Router

The `zigbee` top-level crate provides role-specific modules for coordinators
and routers. A **coordinator** forms the network and acts as the Trust Center;
a **router** relays frames and manages child devices. Both are built on top of
the same layered stack.

```text
zigbee/src/
├── coordinator.rs   — CoordinatorConfig, Coordinator
├── router.rs        — RouterConfig, Router, ChildDevice
├── trust_center.rs  — TrustCenter, TcLinkKeyEntry
└── lib.rs           — re-exports all sub-crates
```

---

## Coordinator

### CoordinatorConfig

```rust
pub struct CoordinatorConfig {
    /// Channel mask for formation (ED scan).
    pub channel_mask: ChannelMask,
    /// Extended PAN ID (0 = auto-generate from IEEE address).
    pub extended_pan_id: IeeeAddress,
    /// Whether to use centralized security (Trust Center).
    pub centralized_security: bool,
    /// Whether to use install codes for joining.
    pub require_install_codes: bool,
    /// Maximum number of child devices.
    pub max_children: u8,
    /// Maximum network depth.
    pub max_depth: u8,
    /// Default permit-join duration after formation (seconds, 0 = closed).
    pub initial_permit_join_duration: u8,
}
```

The defaults are sensible for development:

```rust
CoordinatorConfig::default()
// channel_mask:                ChannelMask::ALL_2_4GHZ
// extended_pan_id:             [0; 8]  (auto-generate)
// centralized_security:        true
// require_install_codes:       false
// max_children:                20
// max_depth:                   5
// initial_permit_join_duration: 0  (joining closed until explicitly opened)
```

### Coordinator State

The `Coordinator` struct manages network-level state:

```rust
pub struct Coordinator {
    config: CoordinatorConfig,
    network_key: [u8; 16],
    frame_counter: u32,
    child_count: u8,
    next_address_seed: u16,
    formed: bool,
}
```

Key methods:

```rust
let mut coord = Coordinator::new(CoordinatorConfig::default());

// Generate a network key (should use hardware RNG in production)
coord.generate_network_key();

// Or set a specific key
coord.set_network_key([0xAB; 16]);

// Check if the network has been formed
assert!(!coord.is_formed());
coord.mark_formed();
assert!(coord.is_formed());

// Allocate a short address for a joining device
let addr = coord.allocate_address();  // ShortAddress(1), then 2, 3, ...

// Check capacity
assert!(coord.can_accept_child());

// Get next frame counter for secured frame transmission
let fc = coord.next_frame_counter();
```

### Address Allocation

The coordinator uses **stochastic address assignment** — it assigns sequential
addresses starting from 1, wrapping around before the reserved range
(`0xFFF8`–`0xFFFF`). A production implementation should use random addresses
with collision detection.

---

## Trust Center

The Trust Center (TC) is responsible for all security-related decisions on a
centralized-security network. In zigbee-rs, it's a separate struct that the
coordinator owns.

### TrustCenter State

```rust
pub struct TrustCenter {
    network_key: [u8; 16],
    key_seq_number: u8,
    link_keys: [Option<TcLinkKeyEntry>; 32],  // up to 32 joined devices
    require_install_codes: bool,
    frame_counter: u32,
}

pub struct TcLinkKeyEntry {
    pub ieee_address: IeeeAddress,
    pub key: [u8; 16],
    pub key_type: TcKeyType,
    pub incoming_frame_counter: u32,
    pub verified: bool,
    pub active: bool,
}

pub enum TcKeyType {
    DefaultGlobal,       // ZigBeeAlliance09
    InstallCode,         // derived from device install code
    ApplicationDefined,  // provisioned by the application
}
```

### Well-Known Keys

```rust
/// "ZigBeeAlliance09" — the default TC link key every Zigbee 3.0 device knows.
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = [
    0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
    0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39,
];

/// Distributed security global link key (for TC-less networks).
pub const DISTRIBUTED_SECURITY_KEY: [u8; 16] = [
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
    0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
];
```

### Key Management

```rust
let mut tc = TrustCenter::new(network_key);

// Get the link key for a device (falls back to DEFAULT_TC_LINK_KEY)
let key = tc.link_key_for_device(&device_ieee);

// Pre-provision a unique key for a device
tc.set_link_key(device_ieee, unique_key, TcKeyType::ApplicationDefined)?;

// After APSME-VERIFY-KEY completes successfully
tc.mark_key_verified(&device_ieee);

// Remove a device (e.g., on leave)
tc.remove_link_key(&device_ieee);

// Rotate the network key (increments key_seq_number)
tc.set_network_key(new_key);
println!("New key seq: {}", tc.key_seq_number());
```

### Join Authorization

The `should_accept_join()` method checks whether a device is allowed to join:

```rust
// When require_install_codes is false: accept everyone
assert!(tc.should_accept_join(&any_device));

// When require_install_codes is true: only accept pre-provisioned devices
tc.set_require_install_codes(true);
assert!(!tc.should_accept_join(&unknown_device));
```

### Replay Protection

The TC tracks per-device incoming frame counters:

```rust
// Returns true if the counter is valid (strictly increasing)
let ok = tc.update_frame_counter(&device_ieee, frame_counter);
if !ok {
    // Replay attack detected — drop the frame
}
```

---

## Router

### RouterConfig

```rust
pub struct RouterConfig {
    /// Maximum number of child end devices.
    pub max_children: u8,
    /// Maximum number of child routers.
    pub max_routers: u8,
    /// Whether to accept join requests.
    pub permit_joining: bool,
    /// Link status period (in seconds).
    pub link_status_period: u16,
}
```

Defaults:

```rust
RouterConfig::default()
// max_children:       20
// max_routers:        5
// permit_joining:     false
// link_status_period: 15 seconds
```

### Child Management

The router maintains a table of up to 32 child devices:

```rust
pub struct ChildDevice {
    pub ieee_address: IeeeAddress,
    pub short_address: ShortAddress,
    pub is_ffd: bool,               // full-function device (router-capable)
    pub rx_on_when_idle: bool,       // false for sleepy end devices
    pub timeout: u16,                // seconds before declaring child lost
    pub age: u16,                    // seconds since last communication
    pub active: bool,
}
```

Managing children:

```rust
let mut router = Router::new(RouterConfig::default());

// Add a child
router.add_child(ieee, short_addr, /*is_ffd=*/false, /*rx_on=*/false)?;

// Look up children
let child = router.find_child(short_addr);
let child = router.find_child_by_ieee(&ieee);

// Check if a destination is our child
if router.is_child(dest_addr) {
    // Buffer the frame for the next poll (if sleepy)
}

// Record activity (resets age timer)
router.child_activity(short_addr);

// Age all children (call periodically, e.g., once per second)
router.age_children(1);
// Sleepy children that exceed their timeout are automatically removed.

// Remove a child explicitly
router.remove_child(short_addr);
```

### Capacity Check

```rust
if router.can_accept_child() {
    // Accept the association request
}
println!("Active children: {}", router.child_count());
```

---

## Network Formation Flow

When a coordinator forms a new network, the following sequence occurs:

```text
1. Energy Detection (ED) scan
   └── Scan channels in channel_mask to find the quietest one.

2. Active scan
   └── Check for existing networks to avoid PAN ID conflicts.

3. Select channel + PAN ID
   └── Choose the channel with lowest noise and a unique PAN ID.

4. Generate network key
   └── coord.generate_network_key()  (should use hardware RNG)

5. Start as coordinator
   └── coord.mark_formed()
   └── Begin transmitting beacons.

6. (Optional) Open permit joining
   └── initial_permit_join_duration > 0
```

---

## Permit Joining

Joining is controlled at two levels:

1. **Router/Coordinator level** — `RouterConfig::permit_joining` or the
   coordinator's `initial_permit_join_duration`.
2. **Trust Center level** — `TrustCenter::should_accept_join()` decides
   whether the security credentials are acceptable.

Both must allow the join for a device to successfully associate.

The permit-join window is typically opened for a limited time (e.g., 180
seconds) when the user triggers a "pairing mode" action, then automatically
closes.

---

## Current Implementation Status

The coordinator, router, and Trust Center modules provide the data structures
and core logic for network management. The shared router path is integrated
with `zigbee-runtime`. TLSR8258 join, durable restart, Link Status, and NWK
relay are hardware-verified. Parent/child traffic has been exercised, but a
clean first-attempt join and complete interview with the corrected child image
remain the release gate.

| Feature | Status |
|---------|--------|
| Coordinator config and state | ✅ Implemented |
| Network key generation | ✅ Placeholder (needs hardware RNG) |
| Address allocation (stochastic) | ✅ Hash-based, reserved-address filtering and collision checks |
| Trust Center link key table | ✅ Implemented (32 entries) |
| Install code derivation (MMO hash) | ⚠️ Structural placeholder |
| Router child management | ✅ Provisional admission, authorization and rollback |
| Child aging and timeout | ✅ Implemented, including indirect cleanup |
| Route discovery (AODV) | ✅ Integrated with runtime forwarding |
| Source routing / Route Record | ✅ Integrated with concentrator behavior |
| Link status messages | ✅ Receive, maintenance and periodic sending |
| Trust Center Update/Transport/Switch Key | ✅ APS security and Tunnel forwarding implemented |
| Distributed security (TC-less) | ⚠️ Secured rejoin supported; full coordinator policy remains incomplete |
| TLSR8258 router MAC | ✅ Join, silent restart, Link Status and NWK relay hardware-verified; corrected-image first-attempt child acceptance remains gated |

> **Contributing:** Coordinator policy, install codes, long-duration routing
> tests, and hardware interoperability remain useful production-hardening
> areas.
