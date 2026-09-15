# Finite mock coordinator

Host demonstration of the typed `router_app::TrustCenterCoordinatorApp`.

The first app instance forms a PAN and persists its coordinator state. The
second instance uses the same security store and proves that
`initialize()` restarts the same PAN without re-forming or associating.
Both execute finite `step()` calls.

```rust,ignore
let mut app = TrustCenterCoordinatorApp::new(
    node,
    PersistentChildren::new(RamChildTableStore::new()),
    RamTrustCenterDeviceStore::new(),
    &POLICY,
    RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
)?;
```

Only the coordinator frontends can select formation/persisted-PAN restart.
`TrustCenterCoordinatorApp` additionally owns the durable per-device key
database and executes authenticated Trust Center commands internally.
`RelayRouterApp` and `ParentRouterApp` select steering instead.

## Run

```bash
cd examples/mock-coordinator
cargo +nightly-2026-03-23 run --locked
```

This proves host lifecycle behavior, not a production hardware coordinator.
