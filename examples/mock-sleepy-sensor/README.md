# Finite mock sleepy sensor

Host demonstration of the same `sensor_sed_app::SensorApp` used by embedded
products.

It constructs:

- `MockMac`;
- a persisted commissioned `RamSecurityStateStore`;
- `DeviceProfile<TemperatureHumidityBattery>`;
- explicit `SensorSedParts`;
- a finite `initialize()` plus four `step()` calls.

```rust,ignore
let mut app = SensorApp::new(
    node,
    &POLICY,
    SensorSedParts {
        wake: HostWake::default(),
        status: NoStatus,
        environment: DemoEnvironment::default(),
        battery: FixedBattery::new(...),
        ota: NoOta,
        actions: NoUserAction,
        supervisor: HostSupervisor,
        diagnostics: HostDiagnostics,
    },
)?;

app.initialize().await?;
for _ in 0..4 {
    app.step().await?;
}
```

The example proves that the shared lifecycle can run under an outer scheduler
without calling the infinite `run()` wrapper. It also demonstrates that
`NoStatus` accepts zero blink periods and that `NoOta` is paired with an
explicit non-OTA profile.

## Run

```bash
cd examples/mock-sleepy-sensor
cargo +nightly-2026-03-23 run --locked
```

This is host-tested behavior, not radio or low-power hardware proof.
