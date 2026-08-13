# ZCL Foundation Commands

Foundation commands are the **global command set shared by every ZCL cluster**. They handle attribute reading, writing, reporting, discovery, and default responses. In zigbee-rs, the runtime processes these automatically — your cluster code rarely needs to touch them directly.

All foundation types live in `zigbee_zcl::foundation`.

## Command Overview

| ID | Command | Direction | Response ID |
|----|---------|-----------|-------------|
| `0x00` | Read Attributes | Client → Server | `0x01` |
| `0x01` | Read Attributes Response | Server → Client | — |
| `0x02` | Write Attributes | Client → Server | `0x04` |
| `0x03` | Write Attributes Undivided | Client → Server | `0x04` |
| `0x04` | Write Attributes Response | Server → Client | — |
| `0x05` | Write Attributes No Response | Client → Server | — |
| `0x06` | Configure Reporting | Client → Server | `0x07` |
| `0x07` | Configure Reporting Response | Server → Client | — |
| `0x08` | Read Reporting Configuration | Client → Server | `0x09` |
| `0x09` | Read Reporting Configuration Response | Server → Client | — |
| `0x0A` | Report Attributes | Server → Client | — |
| `0x0B` | Default Response | Either | — |
| `0x0C` | Discover Attributes | Client → Server | `0x0D` |
| `0x0D` | Discover Attributes Response | Server → Client | — |
| `0x11` | Discover Commands Received | Client → Server | `0x12` |
| `0x13` | Discover Commands Generated | Client → Server | `0x14` |
| `0x15` | Discover Attributes Extended | Client → Server | `0x16` |

These are defined as a Rust enum:

```rust
pub enum FoundationCommandId {
    ReadAttributes           = 0x00,
    ReadAttributesResponse   = 0x01,
    WriteAttributes          = 0x02,
    WriteAttributesUndivided = 0x03,
    WriteAttributesResponse  = 0x04,
    WriteAttributesNoResponse = 0x05,
    ConfigureReporting       = 0x06,
    ConfigureReportingResponse = 0x07,
    ReadReportingConfig      = 0x08,
    ReadReportingConfigResponse = 0x09,
    ReportAttributes         = 0x0A,
    DefaultResponse          = 0x0B,
    DiscoverAttributes       = 0x0C,
    DiscoverAttributesResponse = 0x0D,
    DiscoverCommandsReceived = 0x11,
    DiscoverCommandsReceivedResponse = 0x12,
    DiscoverCommandsGenerated = 0x13,
    DiscoverCommandsGeneratedResponse = 0x14,
    DiscoverAttributesExtended = 0x15,
    DiscoverAttributesExtendedResponse = 0x16,
}
```

---

## Read Attributes (0x00 / 0x01)

The most common foundation command. A coordinator or binding partner reads attribute values from a cluster.

**Request** — a list of `AttributeId`s:

```rust
use zigbee_zcl::foundation::read_attributes::*;

let req = ReadAttributesRequest::parse(&payload)?;
// req.attributes: Vec<AttributeId, 16>
```

**Processing** — the runtime calls `process_read_dyn()` automatically:

```rust
use zigbee_zcl::foundation::read_attributes::process_read_dyn;

let response = process_read_dyn(cluster.attributes(), &request);
// Each record: { id, status, data_type, value }
```

Each `ReadAttributeRecord` in the response contains:
- `id` — the attribute ID that was requested
- `status` — `Success`, `UnsupportedAttribute`, or `WriteOnly`
- `data_type` / `value` — present only when `status == Success`

---

## Write Attributes (0x02 / 0x04)

Writes one or more attributes. The runtime enforces **access control** (read-only attributes are rejected) and **type checking** (mismatched data types are rejected).

```rust
use zigbee_zcl::foundation::write_attributes::*;

let req = WriteAttributesRequest::parse(&payload)?;
let resp = process_write_dyn(cluster.attributes_mut(), &req);
// resp.records: Vec<WriteAttributeStatusRecord, 16>
```

**Write Attributes Undivided** (0x03) provides all-or-nothing semantics — if any single attribute write would fail, none are applied:

```rust
let resp = process_write_undivided_dyn(cluster.attributes_mut(), &req);
```

**Write Attributes No Response** (0x05) silently writes without sending a response frame:

```rust
process_write_no_response_dyn(cluster.attributes_mut(), &req);
```

Per the ZCL spec, if all writes succeed the response is a single byte `0x00` (Success). Only failed attributes appear individually in the response.

---

## Configure Reporting (0x06 / 0x07)

Configures periodic and change-triggered attribute reports. The `ReportingEngine` stores these configurations and decides when to generate `ReportAttributes` (0x0A) frames.

```rust
use zigbee_zcl::foundation::reporting::*;

let req = ConfigureReportingRequest::parse(&payload)?;
for config in &req.configs {
    engine.configure_for_cluster(endpoint, cluster_id, config.clone())?;
}
```

Each `ReportingConfig` contains:

| Field | Description |
|-------|-------------|
| `direction` | `Send` (0x00) or `Receive` (0x01) |
| `attribute_id` | Which attribute to report |
| `data_type` | ZCL data type of the attribute |
| `min_interval` | Minimum seconds between reports |
| `max_interval` | Maximum seconds between reports (0xFFFF = disable periodic) |
| `reportable_change` | Minimum value change to trigger report (analog types only) |

Each record gets its own status in the 0x07 response — `UNSUPPORTED_ATTRIBUTE`,
`UNREPORTABLE_ATTRIBUTE`, `INVALID_DATA_TYPE` or `INSUFFICIENT_SPACE` for a
record the device cannot honour, and `SUCCESS` for the rest. Records that
succeeded stay configured even when a sibling record in the same command
failed.

### Interview state (runtime)

`zigbee-runtime` additionally records which `(endpoint, cluster)` pairs a
**remote** client configured, in
[`remote_reporting`](https://docs.rs/zigbee-runtime) — separate from the
`ReportingEngine`, which also holds the product's own default configurations.
A cluster is recorded only when the whole command was well formed *and every
record returned `SUCCESS`*, the command was non-empty, and every record used
the Send direction. Receive-only and mixed-direction commands do not advance
outbound reporting progress. A qualifying command is surfaced as
`StackEvent::ReportingConfigured` instead of the generic `CommandReceived`.
Profile-backed applications determine completion by checking the active
profile's exact expected cluster IDs; an unrelated configured server cluster
cannot replace a missing reportable cluster. See
[the event loop chapter](../core-concepts/event-loop.md).

---

## Report Attributes (0x0A)

Sent by the server when a configured report triggers. The `ReportingEngine` handles this:

```rust
// In the main loop, advance timers:
engine.tick(elapsed_seconds);

// Then check each cluster for due reports:
let mut reports = heapless::Vec::new();
engine.check_and_collect_dyn(
    endpoint, cluster_id, cluster.attributes(), &mut reports,
);
if !reports.is_empty() {
    let payload = ReportAttributes { reports };
    // Send payload as ZCL frame with command ID 0x0A
}
```

The engine tracks per-attribute state:
- **Elapsed time** since last report
- **Last reported value** for change detection
- For **analog types**: checks if change exceeds the configured threshold
- For **discrete types**: any value change triggers a report

---

## Default Response (0x0B)

Sent in reply to any command that lacks a cluster-specific response, unless the sender set the "disable default response" flag.

```rust
use zigbee_zcl::foundation::default_response::DefaultResponse;

let dr = DefaultResponse {
    command_id: 0x00,           // The command this responds to
    status: ZclStatus::Success, // Result
};
let mut buf = [0u8; 2];
dr.serialize(&mut buf);
```

The runtime generates Default Responses automatically when `handle_command()` returns `Ok(empty_vec)` or `Err(status)`.

---

## Discover Attributes (0x0C / 0x0D)

Lets a client enumerate which attributes a cluster supports, starting from a given attribute ID.

```rust
use zigbee_zcl::foundation::discover::*;

let req = DiscoverAttributesRequest::parse(&payload)?;
let resp = process_discover_dyn(cluster.attributes(), &req);
// resp.complete: bool (true = all attributes returned)
// resp.attributes: Vec<DiscoverAttributeInfo, 16>
//   each: { id: AttributeId, data_type: ZclDataType }
```

---

## Discover Attributes Extended (0x15 / 0x16)

Like Discover Attributes, but also returns access control flags per attribute:

```rust
let resp = process_discover_extended_dyn(cluster.attributes(), &req);
// resp.attributes: Vec<DiscoverAttributeExtendedInfo, 16>
//   each: { id, data_type, access_control }
//   access_control bits: 0x01=readable, 0x02=writable, 0x04=reportable
```

---

## Discover Commands (0x11–0x14)

Enumerates which cluster-specific commands a cluster supports. Each cluster implements `received_commands()` and `generated_commands()`:

```rust
// The Cluster trait provides:
fn received_commands(&self) -> heapless::Vec<u8, 32>;
fn generated_commands(&self) -> heapless::Vec<u8, 32>;

// Processing:
let resp = process_discover_commands(
    &cluster.received_commands(), req.start_command_id, req.max_results,
);
```

---

## How the Runtime Handles Foundation Commands

You almost never handle foundation commands yourself. The zigbee-rs runtime:

1. **Parses** the incoming ZCL frame and checks `frame_control.frame_type`
2. **Foundation frames** (frame_type = 0b00) are dispatched to the appropriate handler:
   - Read Attributes → `process_read_dyn()`
   - Write Attributes → `process_write_dyn()` or `process_write_undivided_dyn()`
   - Configure Reporting → `ReportingEngine::configure_for_cluster()`
   - Discover → `process_discover_dyn()` / `process_discover_extended_dyn()`
3. **Cluster-specific frames** (frame_type = 0b01) are dispatched to your cluster's `handle_command()`
4. **Reporting** is driven by the event loop calling `engine.tick()` + `check_and_collect_dyn()` periodically

This means your `Cluster` implementation only needs to:
- Register attributes with correct `AttributeAccess` modes
- Implement `handle_command()` for cluster-specific commands
- Implement `received_commands()` / `generated_commands()` for discovery
