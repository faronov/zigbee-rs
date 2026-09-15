# Finite mock dimmable-light relay

Host demonstration of a forwarding-only
`router_app::RelayRouterApp + NoChildren`.

The example seeds persisted router security state, initializes the shared
frontend, applies On/Off and Level Control behavior through the profile, and
runs two finite `step()` calls.

```rust,ignore
let mut app = RelayRouterApp::new(
    node,
    NoChildren,
    &POLICY,
    RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
)?;

app.initialize().await?;
let events = app.step().await?;
```

It does not model a child-admitting parent. Use it to understand how a plug or
light composition synchronizes fitted hardware after `StepEvents`.

## Run

```bash
cd examples/mock-light
cargo +nightly-2026-03-23 run --locked
```
