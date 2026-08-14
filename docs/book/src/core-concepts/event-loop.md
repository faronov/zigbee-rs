# The Event Loop

The event loop is the heartbeat of every zigbee-rs device. It drives all stack
processing — scanning, joining, routing, ZCL command handling, and attribute
reporting — by calling two functions in a loop:

- **`device.tick(elapsed_secs, clusters)`** — periodic processing
- **`device.process_incoming(frame, clusters)`** — handle received frames

zigbee-rs uses cooperative async scheduling: you own the loop, the stack never
blocks indefinitely, and you decide when to sleep or read sensors.

## The Basic Pattern

```rust,no_run,ignore
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};

loop {
    match select(
        device.receive(),
        Timer::after(Duration::from_secs(10)),
    ).await {
        // Incoming MAC frame — process through the stack
        Either::First(Ok(frame)) => {
            if let Some(event) = device.process_incoming(&frame, &mut clusters).await {
                handle_event(event);
            }
        }
        Either::First(Err(_)) => {}  // MAC receive error, retry

        // Timer fired — tick reporting engine and read sensors
        Either::Second(_) => {
            let result = device.tick(10, &mut clusters).await;
            match result {
                TickResult::Event(evt) => handle_event(evt),
                TickResult::RunAgain(ms) => { /* schedule next tick sooner */ }
                TickResult::Idle => {}
            }
        }
    }
}
```

This `select`-based pattern is the recommended way to run zigbee-rs on any async
executor (Embassy, `async-std`, Tokio, etc.).

## `tick()` — The Processing Pipeline

```rust,ignore
pub async fn tick(
    &mut self,
    elapsed_secs: u16,
    clusters: &mut [ClusterRef<'_>],
) -> TickResult
```

Every call to `tick()` runs through these phases in order:

| Phase | What It Does |
|-------|-------------|
| **1. User actions** | Drains the `pending_action` queue — processes `Join`, `Leave`, `Toggle`, `PermitJoin`, `FactoryReset` |
| **2. ZCL responses** | Sends any queued ZCL response frames (from sync `process_incoming()` handling) |
| **3. Join check** | If not joined to a network, returns `Idle` early |
| **4. APS maintenance** | Ages the APS ACK table, retransmits unacknowledged frames, ages duplicate-detection and fragment tables |
| **4b. NWK maintenance** | Runs role-independent NWK maintenance (neighbour-cache aging). On `router`-feature builds it additionally runs parent/router maintenance — permit-join expiry, link-status / route-table / concentrator upkeep, pending routing transmissions, End Device Timeout child aging and eviction, MAC parent-command servicing, and Parent Announce. A non-routing sensor build compiles the whole parent/router maintenance subgraph out of the image. |
| **5. Reporting timers** | Ticks the ZCL reporting engine by `elapsed_secs` seconds |
| **5b. Identify / Find & Bind** | Ticks the runtime-owned Identify cluster and handles Finding & Binding target requests |
| **5c. F&B initiator** | Ticks the F&B initiator response window |
| **6. Attribute reports** | For each registered cluster, checks if any attribute reports are due and sends them |
| **7. Poll** | Performs a forced End Device Timeout poll, or an automatic sleepy poll when due, and routes any returned frame through `process_incoming()` |
| **8. End Device Timeout** | Services the R22 keepalive: response timeout, bounded retransmissions and the recurring keepalive |

The `elapsed_secs` parameter tells the reporting engine how much wall-clock time
has passed since the last tick.  Use the actual interval of your timer.

### ClusterRef — Connecting Clusters to the Runtime

The runtime needs access to application-owned cluster instances to read
attribute values for reports and handle incoming commands. Basic and Identify
are owned by `ZigbeeDevice`; pass only sensor and actuator clusters as a
`&mut [ClusterRef]`:

```rust,ignore
use zigbee_runtime::ClusterRef;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut temp_cluster }];

let result = device.tick(10, &mut clusters).await;
```

## `TickResult` — What Tick Returns

```rust,ignore
pub enum TickResult {
    /// Nothing happened — safe to sleep.
    Idle,
    /// A stack event occurred — handle it.
    Event(StackEvent),
    /// Stack needs to run again within this many milliseconds.
    RunAgain(u32),
}
```

- **`Idle`** — No pending work.  Your loop can safely wait for the next frame
  or timer.
- **`Event(evt)`** — Something happened that your application should know
  about.  See [StackEvent](#stackevent--what-the-stack-tells-you) below.
- **`RunAgain(ms)`** — The stack has pending work and needs `tick()` called
  again within `ms` milliseconds.  Schedule accordingly.

While the post-network unique Trust Center link-key handshake is running, every
tick advances it by exactly one bounded step (even when the same tick also
returns an application event, which is never dropped), and any non-event result
is shortened to `RunAgain(50)` so retransmissions and the handshake's 15 s
deadline are honoured. Send failures and rejected Confirm-Key responses remain
in a dedicated 250 ms backoff state during those 50 ms service ticks; they do
not consume another per-message attempt on every tick. A terminal transition is
reported immediately as `CommissioningComplete { success }`. See
[BDB — unique Trust Center link-key exchange](bdb.md#network-steering).

## `StackEvent` — What the Stack Tells You

`StackEvent` is the primary way the stack communicates with your application.
Events are returned from both `tick()` and `process_incoming()`.

### Network Lifecycle Events

```rust,ignore
/// Device successfully joined a network.
StackEvent::Joined {
    short_address: u16,   // Our assigned NWK address
    channel: u8,          // Operating channel (11-26)
    pan_id: u16,          // PAN identifier
}

/// Device left the network.
StackEvent::Left

/// BDB commissioning completed.
StackEvent::CommissioningComplete {
    success: bool,        // true = joined, false = failed
}

/// Permit joining status changed (coordinator/router).
StackEvent::PermitJoinChanged {
    open: bool,           // true = accepting joins
}
```

### ZCL Data Events

```rust,ignore
/// Attribute report received from another device.
StackEvent::AttributeReport {
    src_addr: u16,        // Source NWK address
    endpoint: u8,         // Source endpoint
    cluster_id: u16,      // Cluster the report belongs to
    attr_id: u16,         // Attribute that was reported
}

/// Cluster-specific command received.
StackEvent::CommandReceived {
    src_addr: u16,
    endpoint: u8,
    cluster_id: u16,
    command_id: u8,
    seq_number: u8,       // ZCL sequence (for responses)
    payload: heapless::Vec<u8, 64>,
}

/// Default Response from a remote device.
StackEvent::DefaultResponse {
    src_addr: u16,
    endpoint: u8,
    cluster_id: u16,
    command_id: u8,       // Command ID this responds to
    status: u8,           // ZCL status code
}

/// An attribute report was sent successfully.
StackEvent::ReportSent

/// A remote ZCL client (typically the coordinator, during its interview)
/// fully configured attribute reporting for one of our clusters.
StackEvent::ReportingConfigured {
    src_addr: u16,
    source_endpoint: u8,      // Remote endpoint that sent the command
    endpoint: u8,             // Local endpoint whose cluster was configured
    cluster_id: u16,
    configured_clusters: usize, // Distinct clusters remotely configured on `endpoint`
}
```

### Interview Completion — Remote vs. Local Reporting

`ReportingConfigured` is emitted **only** when a well-formed global Configure
Reporting (0x06, client→server) command was fully processed, was non-empty,
contained only Send-direction records, and *every one of its status records
returned `Success`*. A malformed or empty command, a receive-only or mixed
Send/Receive command, an unsupported or unreportable attribute, an
invalid/disabled data type, or a reporting-table capacity failure still
produces the Configure Reporting Response (0x07) required by the
specification, but yields the generic `CommandReceived` event and does **not**
count as outbound interview progress.

The runtime keeps that record in
[`zigbee_runtime::remote_reporting`](https://docs.rs/zigbee-runtime), separate
from the `ReportingEngine`:

| Query | Counts | Use for |
| --- | --- | --- |
| `device.configured_cluster_count(ep)` | every reporting configuration, including the product's own defaults | "will this device send reports?" |
| `device.remote_reporting_cluster_count(ep)` | every distinct server cluster a remote client configured in full, including unrelated clusters | diagnostics only |
| `device.remote_reporting_coverage(ep, expected)` / `remote_reporting_covers(ep, expected)` | exact expected cluster IDs present in the remote record | direct-composition interview progress/completion |
| `device.is_cluster_remotely_configured(ep, cluster)` | one cluster, remote only | per-cluster interview state |
| `device.reset_remote_reporting()` | — | start of a new commissioning/rejoin lifecycle |

`ZigbeeNode` exposes the same state against its profile's endpoint:
`remote_reporting_cluster_count()`, `remote_reporting_is_complete()`,
`is_cluster_remotely_configured(cluster)`, `expected_report_clusters()` and
`reset_remote_reporting()`. Its progress count and completion check are
filtered through the active profile's exact expected reportable cluster IDs,
so an unrelated Basic cluster cannot replace a missing sensor cluster.

Interview detection must use the profile-aware exact-membership check.
Counting either the reporting engine or arbitrary entries in the generic
remote record is insufficient: the engine includes local defaults, while the
generic record can include unrelated server clusters. An interview-timeout
fallback installs defaults precisely *because* no coordinator arrived:

```rust,ignore
// Fallback: keep reporting even though nobody interviewed us.
if awaiting_initial_configuration && now >= interview_deadline {
    node.configure_default_reporting()?;      // local defaults only
    log::info!(
        "interview timeout — using default reporting (remote configured {}/{})",
        node.remote_reporting_cluster_count(), // still 0 — no interview happened
        node.expected_report_clusters(),
    );
}

// Real interview completion.
if node.remote_reporting_is_complete() {
    // leave the fast-poll window, park the commissioning LED
}
```

The record is live network state, never persisted: it is cleared when the
device leaves the network or is factory reset, and should be reset explicitly
on a fresh join/rejoin, because the new coordinator re-runs the interview.

### OTA Events

```rust,ignore
/// OTA server has a new firmware image available.
StackEvent::OtaImageAvailable {
    version: u32,
    size: u32,
}

/// OTA download progress.
StackEvent::OtaProgress { percent: u8 }

/// OTA upgrade completed — reboot to apply.
StackEvent::OtaComplete

/// OTA upgrade failed.
StackEvent::OtaFailed

/// OTA server requested delayed activation.
StackEvent::OtaDelayedActivation { delay_secs: u32 }
```

### Maintenance Events

```rust,ignore
/// Basic cluster Reset to Factory Defaults completed for application state.
StackEvent::BasicResetToFactoryDefaults
```

## `UserAction` — What Your App Can Do

Queue actions from button presses, sensors, or application logic.  The action is
consumed on the next `tick()`:

```rust,ignore
pub enum UserAction {
    /// Join a network via BDB commissioning.
    Join,
    /// Leave the current network.
    Leave,
    /// Toggle: leave if joined, join if not.
    Toggle,
    /// Open permit joining for N seconds (coordinator/router only).
    PermitJoin(u8),
    /// Factory reset — leave network and clear all state.
    FactoryReset,
}
```

Use `device.user_action(action)` to queue:

```rust,ignore
// Button press handler
if button_pressed {
    device.user_action(UserAction::Toggle);
}
```

Actions are processed at the start of the next `tick()` call.  Only one action
can be pending at a time — if you queue a second action before tick runs, it
replaces the first.

## Handling Incoming Frames

```rust,ignore
pub async fn process_incoming(
    &mut self,
    indication: &McpsDataIndication,
    clusters: &mut [ClusterRef<'_>],
) -> Option<StackEvent>
```

When a MAC frame arrives (from `device.receive()` or `device.poll()`), pass it
to `process_incoming()`.  The stack processes the frame through the full
pipeline:

1. **NWK layer** — parses the NWK header, checks addressing, decrypts if
   NWK-secured
2. **APS layer** — handles APS framing, duplicate detection, fragmentation
   reassembly
3. **ZDO (endpoint 0)** — handles device interview commands
   (`Node_Desc_req`, `Active_EP_req`, `Simple_Desc_req`, etc.) and sends
   responses automatically
4. **ZCL (app endpoints)** — dispatches to your registered clusters for
   attribute read/write/report and command handling

Returns `Some(StackEvent)` if the frame produced an event your application
should handle, or `None` if the stack handled it internally.

## Sending Attribute Reports

The reporting engine automatically sends reports when they're due, but you can
also send reports explicitly:

```rust,ignore
use zigbee_zcl::foundation::reporting::ReportAttributes;

let report = ReportAttributes::new()
    .add(0x0000, ZclValue::I16(2350));  // MeasuredValue = 23.50°C

device.send_report(1, 0x0402, &report).await?;
```

Reports are sent to the coordinator (`0x0000`) using the APS data service with
NWK encryption enabled.

## Receiving Frames: `receive()` and `poll()`

Two methods for getting incoming frames:

```rust,ignore
// For always-on devices (routers, mains-powered end devices):
// Blocks until a frame arrives from the radio.
let frame = device.receive().await?;

// For sleepy end devices:
// Sends a MAC Data Request to the parent and returns any queued frame.
if let Some(frame) = device.poll().await? {
    device.process_incoming(&frame, &mut clusters).await;
}
```

Sleepy End Devices should call `poll()` periodically based on their poll
interval.  The power manager can tell you when it's time:

```rust,ignore
if device.power().should_poll(now_ms) {
    if let Some(frame) = device.poll().await? {
        device.process_incoming(&frame, &mut clusters).await;
    }
    device.power_mut().record_poll(now_ms);
}
```

## End Device Timeout keepalive

The runtime owns the client half of the R22 End Device Timeout lifecycle (the
NWK 0x0B/0x0C wire format and NIB state live in
[`zigbee-nwk`](./nwk.md#end-device-timeout-nwk-0x0b--0x0c)). Nothing has to be
called from the application: `tick()` drives it.

**Owned by the end-device role only.** The client lifecycle state (the keepalive
countdown, response wait, retransmission budget, failure counter and forced-poll
flag) lives in the `EndDevice` role's inline `EndDeviceState`, and the client
helpers are bounded on the `EndDeviceRole` trait. The join/rejoin/resume/tick and
receive paths reach them through static role hooks, so a `RelayRouter` or
`Router` monomorphization carries **neither the client state nor the client
code** — a parent instead runs the separate *server-side* End Device Timeout
path (child aging and the 0x0C response). The public
`send_ed_timeout_request()` API is likewise present only on an end-device-typed
device rather than as a no-op on a router.

**One request per real join.** `finish_join()` is the single choke point for
`start()`, `start_with_security_store()`, the steering branch of
`start_or_resume_with_security_store()`, `secure_rejoin()` and the
`UserAction::Join`/`Toggle` handlers, so exactly one initial request with
enumeration 14 is sent per join. A transmission failure never fails the join —
the recurring keepalive owns recovery.

**Silent persisted resume** does not renegotiate. It re-uses the stored parent
relationship and picks the cheapest keepalive:

| Stored `nwkParentInformation` | Resume / keepalive action |
|-------------------------------|---------------------------|
| valid, bit0 set (or both bits) | forced MAC poll |
| valid, `0` (pre-R22 parent) | forced MAC poll |
| valid, bit1 only | End Device Timeout Request |
| invalid (never negotiated) | End Device Timeout Request |

A **forced poll** bypasses the `automatic_polling` and sleepy gates and routes
any returned frame through `process_incoming()`, so choosing the cheap
keepalive never drops a pending indirect frame. One is scheduled after every
successful request so a sleepy device can fetch the indirect response inside
the parent's transaction persistence.

**Bounded recovery.** An unanswered request is retransmitted twice within a
5-second response window; if that budget is exhausted the parent information
stays invalid and the recurring keepalive falls back to enumeration 8. The
recurring keepalive runs at `timeout / 3` seconds, tracked in whole seconds with
saturating subtraction because a negotiated timeout can be days long. A MAC poll
refreshes that deadline — **except** for a parent that advertised bit1 only,
which does not age its children on polls. Consecutive forced keepalive TX/poll
failures schedule the existing secured-rejoin retry after a small bounded
threshold.

**Persistence.** `parent_information`, `parent_information_valid` and
`end_device_timeout` are stored in the durable security state (record version
3), so a reboot resumes with the negotiated keepalive method instead of
renegotiating. Note that **downgrading to firmware older than the current
record version (4) is unsupported** once such a record has been written.

## Complete Event Loop Example

Here's a complete event loop for a temperature sensor that reads every 60
seconds and reports automatically:

```rust,no_run,ignore
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};
use zigbee_runtime::{ClusterRef, UserAction};
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_zcl::clusters::temperature::TemperatureCluster;

// After device.build()...
let mut temp = TemperatureCluster::new(-4000, 12500);

// Start by requesting join
device.user_action(UserAction::Join);

loop {
    match select(
        device.receive(),
        Timer::after(Duration::from_secs(60)),
    ).await {
        Either::First(Ok(frame)) => {
            let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut temp }];
            if let Some(event) = device.process_incoming(&frame, &mut clusters).await {
                match event {
                    StackEvent::Joined { short_address, channel, pan_id } => {
                        log::info!(
                            "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                            short_address, channel, pan_id,
                        );
                        // Save state for fast rejoin after reboot
                        device.save_state(&mut nv_storage);
                    }
                    StackEvent::Left => {
                        log::info!("Left network — will retry...");
                        device.user_action(UserAction::Join);
                    }
                    StackEvent::CommandReceived { cluster_id, command_id, .. } => {
                        log::info!("Command 0x{:02X} on cluster 0x{:04X}", command_id, cluster_id);
                    }
                    StackEvent::BasicResetToFactoryDefaults => {
                        // Application attributes are reset by the runtime.
                        // Network and security state remain intact.
                    }
                    _ => {}
                }
            }
        }
        Either::First(Err(e)) => {
            log::warn!("MAC error: {:?}", e);
        }
        Either::Second(_) => {
            // Timer fired — read sensor and tick the stack
            let temperature = read_temperature_sensor();
            temp.set_temperature(temperature);

            let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut temp }];
            let result = device.tick(60, &mut clusters).await;
            if let TickResult::Event(event) = result {
                // Handle events from tick (reports sent, etc.)
                match event {
                    StackEvent::ReportSent => log::debug!("Report sent"),
                    _ => {}
                }
            }
        }
    }
}
```

## Key Points

- **`tick()` is cheap** — call it often.  It returns quickly when there's
  nothing to do.
- **One user action at a time** — actions are queued, not stacked.
- **`process_incoming()` is async** — it may send ZDO responses back through
  the MAC.
- **Pass references to the same cluster instances** to both `tick()` and
  `process_incoming()`. Create short-lived `ClusterRef` arrays when the
  application also needs to update a cluster directly.
- **Save state after joining** — call `device.save_state(&mut nv)` so the
  device can rejoin quickly after power loss.
