# Your First Device

In this tutorial you will build a **Zigbee temperature sensor from scratch** —
without using the built-in templates — so you understand every piece of the API.
At the end you will have a working device that:

- Joins a Zigbee network
- Reports temperature readings
- Runs on your desktop using the mock MAC backend (no hardware needed)

> **Prerequisites:** Rust 2024 edition toolchain (`rustup default nightly`).
> The workspace already compiles with `cargo build`.

---

## Step 1 — Create a New Cargo Project

From the repository root:

```bash
cargo init examples/my-temp-sensor
```

Edit `examples/my-temp-sensor/Cargo.toml`:

```toml
[package]
name = "my-temp-sensor"
version = "0.1.0"
edition = "2024"

[[bin]]
name = "my-temp-sensor"
path = "src/main.rs"

[dependencies]
zigbee-types   = { path = "../../zigbee-types" }
zigbee-mac     = { path = "../../zigbee-mac", features = ["mock"] }
zigbee-nwk     = { path = "../../zigbee-nwk" }
zigbee-zcl     = { path = "../../zigbee-zcl" }
zigbee-runtime = { path = "../../zigbee-runtime" }
pollster       = "0.4"
```

The `mock` feature on `zigbee-mac` enables `MockMac` — a simulated 802.15.4
radio that lets you test the full stack on your host machine.

## Step 2 — Set Up the Mock MAC

Every Zigbee device needs a MAC layer to talk to the radio. `MockMac` simulates
one by letting you inject beacons and association responses:

```rust,ignore
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_types::*;

// Each device needs a unique IEEE address (8 bytes)
let ieee_addr: IeeeAddress = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
let mut mac = MockMac::new(ieee_addr);
```

Next, simulate a coordinator that the sensor will join:

```rust,ignore
// The coordinator's PAN and address
let coordinator_pan = PanId(0x1A62);
let coordinator_addr = ShortAddress(0x0000);
let extended_pan_id: IeeeAddress = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

// Inject a beacon so the sensor "sees" the coordinator during scan
mac.add_beacon(PanDescriptor {
    channel: 15,
    coord_address: MacAddress::Short(coordinator_pan, coordinator_addr),
    superframe_spec: SuperframeSpec {
        beacon_order: 15,
        superframe_order: 15,
        final_cap_slot: 15,
        battery_life_ext: false,
        pan_coordinator: true,
        association_permit: true,
    },
    lqi: 220,
    security_use: false,
    zigbee_beacon: ZigbeeBeaconPayload {
        protocol_id: 0x00,
        stack_profile: 2,        // ZigBee PRO
        protocol_version: 2,
        router_capacity: true,
        device_depth: 0,
        end_device_capacity: true,
        extended_pan_id,
        tx_offset: [0xFF, 0xFF, 0xFF],
        update_id: 0,
    },
});

// Pre-configure the association response the sensor will receive
let assigned_address = ShortAddress(0x1234);
mac.set_associate_response(MlmeAssociateConfirm {
    short_address: assigned_address,
    status: AssociationStatus::Success,
});
```

## Step 3 — Define the Endpoint

A Zigbee endpoint groups related clusters under a **profile** and **device ID**.
For a Home Automation temperature sensor:

| Field | Value | Meaning |
|-------|-------|---------|
| Endpoint | `1` | Application endpoints are 1–240 |
| Profile ID | `0x0104` | Home Automation |
| Device ID | `0x0302` | Temperature Sensor |

We use `DeviceBuilder` to define this. The builder uses a fluent API where you
chain `.endpoint()` calls with a closure that configures clusters:

```rust,ignore
use zigbee_runtime::ZigbeeDevice;
use zigbee_nwk::DeviceType;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::{ClusterId, DeviceId};

let device = ZigbeeDevice::builder(mac)
    .device_type(DeviceType::EndDevice)
    .manufacturer("zigbee-rs-tutorial")
    .model("MyTempSensor-01")
    .date_code("20250101")
    .sw_build("0.1.0")
    .power_source(PowerSource::Battery)
    .channels(ChannelMask::PREFERRED)
    .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
        ep.cluster_server(ClusterId::BASIC)
          .cluster_server(ClusterId::TEMPERATURE)
    })
    .build();
```

- **`cluster_server(ClusterId::BASIC)`** — the Basic cluster is mandatory on every
  endpoint. It holds the manufacturer name, model, and software version.
- **`cluster_server(ClusterId::TEMPERATURE)`** — the Temperature Measurement cluster. It
  exposes `MeasuredValue`, `MinMeasuredValue`, and `MaxMeasuredValue` attributes.

The builder registers these with the ZDO layer so that discovery requests
(`Active_EP_req`, `Simple_Desc_req`) return the correct data to coordinators
and gateways.

## Step 4 — Create Application Cluster Instances

`ZigbeeDevice` owns the standard Basic and Identify cluster instances. The
builder metadata above directly populates Basic attributes, so identity is
defined only once. You create only mutable application clusters separately:

```rust,ignore
use zigbee_zcl::clusters::temperature::TemperatureCluster;

// Temperature cluster — range -40.00°C to +125.00°C
// Values are in hundredths of a degree: -4000 = -40.00°C
let mut temp = TemperatureCluster::new(-4000, 12500);
```

## Step 5 — Join the Network

Call `device.start()` to run BDB commissioning. This performs:

1. **MAC reset** — initialize the radio
2. **Active scan** — find nearby coordinators via beacons
3. **Association** — join the best network and receive a short address

Since `start()` is `async`, we use `pollster::block_on` on the host:

```rust,ignore
pollster::block_on(async {
    match device.start().await {
        Ok(addr) => println!("Joined! Short address: 0x{:04X}", addr),
        Err(e) => println!("Join failed: {:?}", e),
    }
});
```

On real hardware with Embassy you would just `.await` directly inside an
`#[embassy_executor::main]` task.

## Step 6 — Update Temperature and Tick

In a real sensor you would read the ADC or I²C sensor periodically. Here we
simulate it:

```rust,ignore
use zigbee_runtime::ClusterRef;

// Update the temperature: 2350 = 23.50°C
temp.set_temperature(2350);
```

To drive the stack — send queued reports, handle incoming frames, manage power —
call `device.tick()`:

```rust,ignore
pollster::block_on(async {
    let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut temp }];
    let result = device.tick(10, &mut clusters).await;
    println!("Tick result: {:?}", result);
});
```

`tick(elapsed_secs, clusters)` takes:
- **`elapsed_secs`** — seconds since the last tick (drives the reporting timer)
- **`clusters`** — mutable references to your cluster instances so the runtime
  can read attributes for reports and dispatch incoming commands

## Step 7 — Verify Attribute Values

You can read back attributes at any time through the `Cluster` trait:

```rust,ignore
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE;
use zigbee_zcl::data_types::ZclValue;

let attrs = temp.attributes();
if let Some(ZclValue::I16(val)) = attrs.get(ATTR_MEASURED_VALUE) {
    println!("Temperature: {:.2}°C", *val as f64 / 100.0);
}
```

## Full Working Example

Here is the complete `src/main.rs` — paste this into
`examples/my-temp-sensor/src/main.rs` and run with `cargo run -p my-temp-sensor`:

```rust
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_types::*;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::temperature::{TemperatureCluster, ATTR_MEASURED_VALUE};
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::data_types::ZclValue;
use zigbee_zcl::{ClusterId, DeviceId};

fn main() {
    // ── 1. Set up MockMac ──────────────────────────────────────────
    let ieee_addr: IeeeAddress = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
    let mut mac = MockMac::new(ieee_addr);

    let coordinator_pan = PanId(0x1A62);
    let coordinator_addr = ShortAddress(0x0000);
    let extended_pan_id: IeeeAddress =
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    mac.add_beacon(PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(coordinator_pan, coordinator_addr),
        superframe_spec: SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 15,
            battery_life_ext: false,
            pan_coordinator: true,
            association_permit: true,
        },
        lqi: 220,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0x00,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id,
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        },
    });

    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: ShortAddress(0x1234),
        status: AssociationStatus::Success,
    });

    // ── 2. Build the device (no template) ──────────────────────────
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("zigbee-rs-tutorial")
        .model("MyTempSensor-01")
        .date_code("20250101")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::PREFERRED)
        .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
            ep.cluster_server(ClusterId::BASIC)
              .cluster_server(ClusterId::TEMPERATURE)
        })
        .build();

    // ── 3. Create cluster instances ────────────────────────────────
    let mut temp = TemperatureCluster::new(-4000, 12500);

    // ── 4. Join the network ────────────────────────────────────────
    pollster::block_on(async {
        match device.start().await {
            Ok(addr) => {
                println!("Joined network as 0x{:04X}", addr);
                println!("  Channel: {}", device.channel());
                println!("  PAN ID:  0x{:04X}", device.pan_id());
            }
            Err(e) => {
                println!("Join failed: {:?}", e);
                return;
            }
        }

        // ── 5. Simulate sensor readings ────────────────────────────
        let readings: &[i16] = &[2350, 2410, 2275, 1890];

        for (i, &value) in readings.iter().enumerate() {
            temp.set_temperature(value);

            // Read back via the Cluster trait
            if let Some(ZclValue::I16(v)) = temp.attributes().get(ATTR_MEASURED_VALUE) {
                println!(
                    "  Reading #{}: {:.2}°C",
                    i + 1,
                    *v as f64 / 100.0
                );
            }

            // ── 6. Tick the stack ──────────────────────────────────
            let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut temp }];
            let _result = device.tick(10, &mut clusters).await;
        }
    });

    println!("Done!");
}
```

### What Each Part Does

| Section | Purpose |
|---------|---------|
| **MockMac setup** | Creates a simulated radio with a fake coordinator beacon so the device can scan and associate without real hardware. |
| **`ZigbeeDevice::builder(mac)`** | Constructs the full BDB→ZDO→APS→NWK→MAC layer stack. The `.endpoint()` call registers clusters with the ZDO so discovery works. |
| **Builder identity methods** | Configure the runtime-owned Basic cluster once through `.manufacturer()`, `.model()`, `.date_code()`, `.sw_build()`, and `.power_source()`. |
| **`TemperatureCluster::new(-4000, 12500)`** | Creates the Temperature Measurement attribute store with a valid range of −40.00 °C to +125.00 °C. Values are in hundredths of a degree. |
| **`device.start().await`** | Runs BDB commissioning: MAC reset → active scan → association → NWK join. Returns the assigned short address. |
| **`temp.set_temperature(2350)`** | Updates the `MeasuredValue` attribute to 23.50 °C. On the next reporting interval, the runtime will send this to the coordinator. |
| **`device.tick(10, &mut clusters)`** | Drives one iteration of the event loop: sends queued reports, processes pending user actions, and manages APS retransmissions. The `10` means "10 seconds since last tick". |

## Running the Example

```bash
cargo run -p my-temp-sensor
```

Expected output:

```text
Joined network as 0x1234
  Channel: 15
  PAN ID:  0x1A62
  Reading #1: 23.50°C
  Reading #2: 24.10°C
  Reading #3: 22.75°C
  Reading #4: 18.90°C
Done!
```

## Next Steps

- **Add humidity** — add `cluster_server(ClusterId::HUMIDITY)` to the endpoint and create a
  `HumidityCluster`. See the `mock-sensor` example for a complete temp+humidity
  device.
- **Use a template** — `zigbee_runtime::templates::temperature_sensor(mac)`
  gives you a pre-configured `DeviceBuilder` with Basic, Power Config, Identify,
  and Temperature clusters already set up.
- **Run on real hardware** — swap `MockMac` for a platform-specific MAC backend
  (e.g., `Esp32Mac`, `NrfMac`) and use Embassy as the async executor. See the
  [Platform Guides](../platform-guides/esp32.md).
- **Configure reporting** — set up periodic attribute reports so Home Assistant
  / Zigbee2MQTT receives temperature updates automatically.
