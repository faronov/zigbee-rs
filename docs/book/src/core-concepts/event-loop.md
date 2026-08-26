# The Application Lifecycle

Platform examples should use the shared finite application frontends rather
than reproduce receive/tick/commissioning loops.

## Sleepy sensor lifecycle

`sensor_sed_app::SensorApp` owns:

- cold commissioning and persisted resume/rejoin;
- one manual parent-poll owner;
- fast and slow poll windows;
- incoming receive and runtime tick processing;
- reporting and measurement cadence;
- status, Identify, user actions, watchdog service, and diagnostics;
- security checkpoints and factory reset;
- OTA-first event routing and activation checkpointing.

```rust,ignore
let mut app = SensorApp::new(node, policy, parts)?;

app.initialize().await?;
loop {
    app.step().await?;
}
```

- `initialize()` performs one finite boot/resume lifecycle and rejects a
  second call.
- `step()` performs one finite wait/service iteration and rejects use before
  initialization.
- `run()` is convenience sugar for initialize followed by infinite steps.

Most firmware roots call `run()`. An outer scheduler, host test, or
reset-on-wake platform calls the finite methods directly.

### Wait arbitration

Each step computes the earliest deadline among:

- fast or slow parent poll;
- runtime `RunAgain`;
- sample/report cadence;
- reporting/interview grace;
- rejoin and Device Announce retry;
- status/Identify phase when status hardware exists;
- OTA service and keep-awake window;
- watchdog maximum wait.

It selects the product's `fast_sleep_depth` or `slow_sleep_depth` only while
joined and OTA-idle. Otherwise it requests `Active`.

The `WakeController` performs the atomic MAC prepare/wait/restore operation and
returns `Button` or `Timer`.

### OTA before generic events

Every `StackEvent` is first passed to `OtaLifecycle::handle_event`. Only
`NotHandled` falls through to generic sensor matching. An OTA implementation
reports pending activation; `SensorApp` checkpoints security state before
calling the reset-causing activation method.

`NoOta` is valid only with `NonOtaProfile`.

## Router/coordinator lifecycle

`router_app` exposes:

- `RelayRouterApp` — forwarding only, `NoChildren`;
- `ParentRouterApp` — child-capable, `PersistentChildren`;
- `CoordinatorApp` — formation/persisted-PAN restart,
  `PersistentChildren`.

Each has the same finite shape:

```rust,ignore
app.initialize().await?;
let StepEvents { incoming, tick } = app.step().await?;
```

`StepEvents` contains at most one event from the bounded receive path and one
from the subsequent runtime tick. A relay plug/light can apply the profile's
new state to fitted hardware after each step.

`run()` loops internally. A product that must service a reset button or other
outer-loop concern can call `step()` itself.

Startup is statically selected:

- relay and parent: steering or persisted resume;
- coordinator: formation or persisted-PAN restart.

## Stack events

The underlying `zigbee_runtime::event_loop::StackEvent` still describes:

- joined/left/commissioning state;
- reporting configuration and sent reports;
- received ZCL commands/default responses/attribute reports;
- Identify and OTA progress;
- permit-join and router/parent activity.

Shared applications match events explicitly. New event variants therefore
produce compile errors in those handlers instead of silently falling through a
wildcard.

## Low-level `ZigbeeNode`

`ZigbeeNode` retains finite receive/tick/start APIs for new reusable
application frontends and diagnostics. It integrates the device, profile, and
security store. Platform products should not call low-level methods merely to
copy the existing sensor/router lifecycle.

If a genuinely different application archetype is needed:

1. implement it once in a platform-independent app crate;
2. use finite `initialize`/`step` operations;
3. accept narrow explicit capabilities;
4. preserve security checkpoints and runtime deadlines;
5. keep the platform `main.rs` as the composition root.

## Testing

The finite API makes host tests deterministic:

- `mock-sleepy-sensor` initializes and executes four steps;
- `mock-light` executes finite relay steps;
- `mock-coordinator` forms, persists, restarts, and steps a coordinator.

These are host lifecycle proofs, not radio timing or hardware power proofs.
