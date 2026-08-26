# Finite mock coordinator

Host demonstration of the typed `router_app::CoordinatorApp`.

The first app instance forms a PAN and persists its coordinator state. The
second instance uses the same security store and proves that
`initialize()` restarts the same PAN without re-forming or associating.
Both execute finite `step()` calls.

```rust,ignore
let mut app = CoordinatorApp::new(
    node,
    PersistentChildren::new(RamChildTableStore::new()),
    &POLICY,
    RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
)?;
```

Only `CoordinatorApp` can select formation/persisted-PAN restart.
`RelayRouterApp` and `ParentRouterApp` select steering instead.

## Run

```bash
cd examples/mock-coordinator
cargo +nightly-2026-03-23 run --locked
```

This proves host lifecycle behavior, not a production hardware coordinator.
