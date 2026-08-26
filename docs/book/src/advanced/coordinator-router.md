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

`router-app` has an empty default feature set. A relay, parent-router, or
coordinator product must opt in explicitly:

```toml
router-app = { path = "../../apps/router", features = ["router"] }
```

An always-on End Device leaves that feature disabled, so the dependency graph
does not carry route, parent, or child-table capacities.

Parent-only APIs are not exposed to an end device or relay. A backend without
association responses, pending transactions, and indirect delivery cannot
construct or advertise any Router descriptor.

## Application frontends

`apps/router` adds role-specific startup, persistence, and finite lifecycle:

| frontend | runtime role | children | startup |
|---|---|---|---|
| `RelayRouterApp` | `RelayRouter` | `NoChildren` | steering or persisted resume |
| `ParentRouterApp` | `Router` | `PersistentChildren<C>` | steering or persisted resume |
| `CoordinatorApp` | `Router` | `PersistentChildren<C>` | formation or persisted-PAN restart |

Relay and parent frontends cannot construct a coordinator formation future.
`CoordinatorApp` is the only public frontend that selects formation and
coordinator restart.

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
RelayRouterApp + RelayRouter + NoChildren
```

It keeps routing, route maintenance, and Link Status. It does not advertise or
implement child admission, pending transactions, or indirect delivery.

### TLSR8258

The Telink MAC supplies parent operations. The product uses:

```text
ParentRouterApp + Router + PersistentChildren
```

The app restores the child table only after network resume, clears stale
foreign/corrupt state, persists changes when dirty, and clears it before
factory-reset recommissioning.

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

## Persistence

Parent products use two separate stores:

- `SecurityStateJournal` for network keys and outgoing counter reservations;
- `ChildTableJournal` for child identity/configuration snapshots.

The child record is bound to the extended PAN ID. It never stores security
counters. Factory reset clears both stores in the correct order before
recommissioning.

TLSR8258 TB-04 partitions:

```text
0x72000..0x74000  child table
0x74000..0x76000  security state
0x76000..0x78000  factory EUI/config
```

The nRF relay has only its security journal because `NoChildren` removes child
storage.

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
| TLSR8258 parent router | join, silent restart, Link Status, and NWK relay hardware-proven; corrected-image first-attempt child join/interview open |
| coordinator | finite formation/restart host-tested; no production hardware coordinator path claimed |

Do not call a non-parent backend a router, and do not call a host-tested
coordinator hardware-supported.
