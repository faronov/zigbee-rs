# Coordinator and Router Applications

Logical role and hardware parent capability are compile-time types.

## Runtime roles

```rust,ignore
let end_device = ZigbeeDevice::builder(mac).build();
let relay = ZigbeeDevice::builder(mac).build_relay();
let parent = ZigbeeDevice::builder(parent_mac).build_router();
let coordinator = ZigbeeDevice::builder(parent_mac).build_coordinator();
```

| role | routes | admits children | MAC bound |
|---|---:|---:|---|
| `EndDevice` | no | no | `MacDriver` |
| `RelayRouter` | yes | no | `router` feature + `ParentMacDriver` |
| `Router` | yes | yes | `router` feature + `ParentMacDriver` |

`router-app` requires centralized TCLK commissioning even with default
features disabled; its defaults additionally select distributed security and
groups. This does not select a routing role. A relay, parent-router, or
coordinator product must opt into routing explicitly:

```toml
router-app = { path = "../../apps/router", features = ["router"] }
```

An always-on End Device leaves that feature disabled, so the dependency graph
does not carry route, parent, or child-table capacities.

Parent-only APIs are not exposed to an end device or relay. A backend without
association responses, pending transactions, and indirect delivery cannot
construct or advertise any Router descriptor.

Only tc32 `TelinkMac` and host-only `MockMac` implement the sealed
`ParentMacDriver`. Telink advertises router capability but explicitly reports
`coordinator: false`; all other real backends are non-parent End Device
backends. No production backend currently advertises Coordinator or
Trust-Center server support.

## Application frontends

`apps/router` adds role-specific startup, persistence, and finite lifecycle:

| frontend | runtime role | children | startup |
|---|---|---|---|
| `AlwaysOnEndDeviceApp` | `EndDevice` | none | steering or persisted resume |
| `RelayRouterApp` | `RelayRouter` | `NoChildren` | steering or persisted resume |
| `ParentRouterApp` | `Router` | `PersistentChildren<C>` | steering or persisted resume |
| `DistributedRouterApp` | `Router` | `PersistentChildren<C>` | distributed formation or persisted-PAN restart |
| `CoordinatorApp` | `Router` | `PersistentChildren<C>` | formation or persisted-PAN restart |
| `TrustCenterCoordinatorApp` | `Router` | `PersistentChildren<C>` plus durable TC devices | centralized formation/restart and TC transaction recovery |

Relay and parent frontends cannot construct a coordinator formation future.
`DistributedRouterApp` can construct only router-owned distributed formation;
`CoordinatorApp` can construct only centralized coordinator formation. Their
pending Join/Toggle actions use the same statically selected path, so a
distributed product cannot accidentally fall back to centralized steering.

```rust,ignore
let mut app = RelayRouterApp::new(
    ZigbeeNode::new(&mut device, &mut security_store, &mut profile),
    NoChildren,
    &policy,
    RouterParts::new(status, supervisor, diagnostics),
)?;

app.initialize().await?;
loop {
    let events = app.step().await?;
    // Synchronize fitted relay/light hardware after profile command handling.
}
```

`run()` is the infinite convenience wrapper. `StepEvents` returns at most one
incoming event and one tick event from the bounded cycle.

## Relay versus parent

### nRF52840

The Nordic MAC backend is not `ParentMacDriver`. The product uses:

```text
AlwaysOnEndDeviceApp + EndDevice + NoChildren
```

It is a receiver-on End Device. It does not link routing, route maintenance,
Link Status, child admission, pending transactions, or indirect delivery.
The historical `examples/nrf52840-router` directory name does not change its
Zigbee role.

### TLSR8258

The Telink MAC supplies parent operations. The product uses:

```text
ParentRouterApp + Router + PersistentChildren
```

The app restores the child table only after network resume, clears stale
foreign/corrupt state, persists changes when dirty, and clears it before
factory-reset recommissioning.

### Distributed router

A product that owns a distributed-security PAN uses:

```text
DistributedRouterApp + Router + PersistentChildren
```

It must call `set_distributed_security_link_key()` with its certified
product key before `initialize()`. Formation without a provisioned key is
rejected. The public BDB `D0..DF` key is for test/certification images only.

## Coordinator startup

`CoordinatorApp::initialize()` selects one of two typed paths:

- no persisted commissioned state: form a network and persist PAN/key state;
- valid persisted coordinator state: restart the same PAN without re-forming
  or associating.

`examples/mock-coordinator` host-tests both paths with finite initialization
and steps.

Coordinator formation still requires a real entropy source and a
`ParentMacDriver`. A platform must not return fake entropy or silently accept
unsupported parent operations.

The current coordinator and Trust-Center server validation is mock/host-only.

`TrustCenterCoordinatorApp` adds the authoritative durable Trust Center. It
restores all individual link keys and counter reservations, re-applies the
security journal's APS replay floors against those keys, and only then resumes
pending security transactions and receive. A failed restore closes the barrier;
the next `step()` retries it before RX or ticking. `TrustCenterRuntime::poll()`
and security indications never implicitly initialize a missing key table.
The public `initialize_with_security_store(device, replay_store)` API owns
this entire barrier; the separate restore/resume phases are private.
The app also exposes
`request_network_key_rotation().await`. The BDB
`trust_center_network_key_update_period` can trigger the same broadcast or
router-only unicast rotation automatically. The unicast router list is learned
from authenticated `Device_annce`; an incomplete list is reported explicitly.
Broadcast rotation also journals and restores forwarding to the coordinator's
own sleepy children. Products configure the profile maximum polling interval
before initialization; the TC propagation grace is at least 30 seconds and
never shorter than that parent's configured forwarding window.

## Persistence

Parent products use separate stores:

- `SecurityStateJournal` for network keys, outgoing counter reservations, and
  incoming NWK/APS replay floors;
- `ChildTableJournal` for child identity/configuration snapshots.
- an `ApsTableStore` for network-bound bindings, groups, and application link
  keys when the product selects `PersistentApsTables<A>`.

`application-link-key-installation` is an optional product capability.
`NoApsTables` keeps it disabled. With the feature and persistent APS tables,
receipt commits the APS snapshot before the APS replay floor and releases any
compatibility acknowledgement for a legacy incoming AR=1 command only after
both commits. Normal security commands use AR=0, with no APS retransmission.
This ordering makes repeated delivery safe on either side of a reboot without
resetting an already stored key's counters.

An authoritative Coordinator adds `TrustCenterDeviceJournal` for its device
and link-key table, APS counter reservations, Remove-Device/application-key
transactions, and Network-Key rotation progress. Rotation key material remains
in `SecurityStateJournal`; the two journals use an explicit checkpoint order
so no reboot can advertise a key that was never durably staged or clear a
transaction before local activation is durable.

Repeated `Update-Device(DeviceLeft)` for an unknown or already-revoked peer
is idempotent. A denied secured rejoin uses a separate durable removal intent,
not an admitted device or fabricated link-key entry. The intent survives a
restart until local parent-command submission/removal and replay retirement
finish. This send bookkeeping is not evidence of remote removal or APS ACK.
Protocol rejection is not a fatal error; storage and capacity errors still
propagate instead of acknowledging an uncommitted transaction.

The child record is bound to the extended PAN ID. It never stores security
counters. Factory reset clears both stores in the correct order before
recommissioning.

Child-journal version 5 records the complete parent-side NWK lifecycle:
pending Trust Center Remove-Device, selected conflict-reassignment address,
pending DeviceLeft notification, and a remove-children Leave cascade. A
sleepy child's Leave remains indirect until its MAC transmission succeeds; a
remote `Update-Device(DeviceLeft)` remains durable until successful lower-layer
submission and its journal completion, without waiting for APS ACK. Neither
enqueue nor submission proves TC processing. Failed completion writes retry
without another submission within that boot; reboot may repeat the durable
intent. The parent does not begin its own Leave/Rejoin until every child
removal and local-send completion is checkpointed.

TLSR8258 TB-04 partitions:

```text
0x70000..0x72000  APS binding/group/application-key table
0x72000..0x74000  child table
0x74000..0x76000  security state
0x76000..0x78000  factory EUI/config
```

The nRF always-on End Device has only its security journal; it has no router
or child storage.

The legacy 4 KiB-per-sector journal geometry is not a production Coordinator
default. With the full router replay table it leaves only 12 append commits
after a rollover snapshot. A Coordinator or busy parent product must provision
larger logical sectors (for example 16 KiB each), assert the resulting
`FULL_TABLE_APPEND_CAPACITY`, and qualify erase endurance under expected
traffic.

## Shared protocol behavior

The typed application wraps the same runtime/NWK implementation for:

- routing and route discovery;
- Link Status and neighbor aging;
- R22 many-to-one/source routing;
- address/PAN conflict handling;
- secured APS/NWK forwarding;
- parent command servicing and child timeout only for the parent role.

Host/runtime tests cover role splitting and protocol vectors. Timing-sensitive
forwarding, child admission, indirect delivery, and multi-router behavior
still require independent packet captures and HIL.

## Validation status

| product | validation |
|---|---|
| nRF52840 always-on End Device | builds, layout/role-symbol gates pass; complete HIL acceptance open |
| TLSR8258 parent router | current 433,756 B image exceeds the unchanged 430,080 B regression gate by 3,676 B; prior join, silent restart, Link Status, and NWK relay evidence exists; corrected-image first-attempt child join/interview open |
| coordinator | finite formation/restart host-tested; no production hardware coordinator path claimed |

Do not call a non-parent backend a router, and do not call a host-tested
coordinator hardware-supported.
