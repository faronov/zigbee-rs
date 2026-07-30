# Quick Start

This chapter gets you from zero to a running Zigbee sensor simulation in under
five minutes. No hardware required — the mock MAC backend runs the full
protocol stack on your laptop.

## Prerequisites

You need a working Rust toolchain. zigbee-rs uses the **2024 edition**, so a
recent nightly or stable toolchain is required:

```bash
# Install Rust (if you haven't already)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Verify your installation
rustc --version
cargo --version
```

That's it for the mock examples. No cross-compilation target, no probe, no
SDK — just `rustc` and `cargo`.

> **Hardware targets** (ESP32, nRF52, etc.) need additional setup covered in
> the [Platform Guides](../platform-guides/esp32.md). The mock examples are
> the fastest way to explore the stack.

## Clone the Repository

```bash
git clone https://github.com/faronov/zigbee-rs.git
cd zigbee-rs
```

Verify the workspace builds:

```bash
cargo build
```

This compiles all 9 library crates and the 4 mock examples. The first build
downloads dependencies and takes about a minute; subsequent builds are
incremental.

## Run the Mock Sensor

The `mock-sensor` example simulates a Zigbee temperature and humidity sensor.
It exercises the full stack: MAC scanning, network association, ZCL cluster
creation, and attribute reporting.

```bash
cargo run -p mock-sensor
```

### What You'll See

The output walks through seven steps of the device lifecycle:

```text
╔══════════════════════════════════════════════════════╗
║  zigbee-rs Mock Temperature + Humidity Sensor       ║
╚══════════════════════════════════════════════════════╝

── Step 1: Configure Mock MAC Layer ──
  Created MockMac with IEEE address: AA:BB:CC:DD:11:22:33:44
  Added coordinator beacon: PAN 0x1A62, channel 15, LQI 220
  Set association response: short addr 0x796F (Success)

── Step 2: Build Sensor Device ──
  Built temperature + humidity sensor device
  Device type: EndDevice
  Profile: Home Automation (0x0104)
  Endpoint 1 server clusters:
    - Basic (0x0000)
    - Power Configuration (0x0001)
    - Identify (0x0003)
    - Temperature Measurement (0x0402)
    - Relative Humidity (0x0405)
```

Let's break down what's happening:

**Step 1 — MockMac setup.** A simulated MAC layer is created with a
pre-configured coordinator beacon and association response. In real firmware
these come over the air; here we inject them so the stack has something to
find.

**Step 2 — DeviceBuilder.** The `templates::temperature_humidity_sensor()`
helper creates a fully configured end device with the right HA profile and
cluster set. Here's the core of it:

```rust
use zigbee_runtime::templates;
use zigbee_types::*;

let device = templates::temperature_humidity_sensor(mac)
    .manufacturer("Zigbee-RS")
    .model("MockTempHumid-01")
    .sw_build("0.1.0")
    .channels(ChannelMask::PREFERRED)
    .build();
```

**Step 3 — Network join.** The stack performs the standard Zigbee join
sequence using raw MAC primitives:

1. `MLME-RESET` — initialize the radio
2. `MLME-SCAN(Active)` — discover nearby PANs
3. `MLME-ASSOCIATE` — request a short address from the coordinator
4. `MLME-START` — begin operating on the assigned PAN

```text
── Step 3: Network Join Sequence ──
  [3a] MLME-RESET.request(setDefaultPIB=true) → OK
  [3b] MLME-SCAN.request(Active, preferred channels) → found 1 PAN(s)
       PAN[0]: channel 15, LQI 220, association_permit=true
  [3c] MLME-ASSOCIATE.request → status=Success, short_addr=0x796F
  [3d] MLME-START.request → joined PAN 0x1A62 on channel 15
```

**Steps 4–6 — ZCL clusters.** Temperature and humidity clusters are created,
sensor readings are simulated, and attributes are read back through the
typed cluster API:

```rust
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;

// Values are in hundredths: 2350 = 23.50°C, 6500 = 65.00%
let mut temp = TemperatureCluster::new(-4000, 12500);
let mut humid = HumidityCluster::new(0, 10000);

temp.set_temperature(2350);
humid.set_humidity(6500);
```

```text
── Step 5: Simulate Sensor Readings ──
  Reading #1: temperature=23.50°C, humidity=65.00%
  Reading #2: temperature=24.10°C, humidity=63.80%
  Reading #3: temperature=22.75°C, humidity=71.00%
  Reading #4: temperature=18.90°C, humidity=82.50%
```

The example finishes with a summary confirming that MockMac configuration,
MAC-level join, and ZCL attribute read/write all work correctly.

## Run the Mock Coordinator

The `mock-coordinator` example shows the other side of the network — forming
a PAN and accepting joining devices:

```bash
cargo run -p mock-coordinator
```

This example:

1. Performs an **energy detection scan** to pick the quietest channel
2. Forms the network with `MLME-START` as PAN coordinator
3. Sets up a **Trust Center** with the default link key
4. Simulates three devices joining the network
5. Builds a coordinator `ZigbeeDevice` with Basic and Identify clusters

```text
── Step 2: Energy Detection Scan ──
  MLME-SCAN.request(ED) → 4 measurements
    Channel 11: energy level 180
    Channel 15: energy level 45 ← best
    Channel 20: energy level 90
    Channel 25: energy level 60
  Selected channel 15 (energy=45)

── Step 3: NLME-NETWORK-FORMATION ──
  MLME-START.request → Network formed!
    PAN ID:          0x1A62
    Channel:         15
    Short address:   0x0000 (coordinator)
    Beacon order:    15 (non-beacon)
    Association:     PERMITTED
```

The coordinator allocates short addresses to joining devices and distributes
the network key through the Trust Center — the same flow that happens on
production Zigbee coordinators.

## Other Mock Examples

Two more mock examples are available:

```bash
# Dimmable light (On/Off + Level Control clusters)
cargo run -p mock-light

# Sleepy end device (full SED lifecycle with polling)
cargo run -p mock-sleepy-sensor
```

## Running Tests

The workspace includes integration tests that exercise protocol encoding,
cluster behavior, and MAC primitives:

```bash
cargo test
```

You can also run the linter and formatter to match the project's CI checks:

```bash
cargo clippy --workspace
cargo fmt --check
```

## Next Steps

You've seen the stack in action without any hardware. From here:

- **[Your First Device](./first-device.md)** — Build a custom sensor from
  scratch using the `DeviceBuilder` API, picking your own clusters and
  endpoint configuration.
- **[Architecture Overview](./architecture.md)** — Understand how the 9
  crates fit together and how data flows from radio to application.
- **[ESP32-C6 / ESP32-H2](../platform-guides/esp32.md)** — Flash real
  firmware to hardware and join a live Zigbee network.
