# The Device Builder

Every zigbee-rs application starts the same way: you describe *what* your device
is, and the builder assembles the full Zigbee stack for you.  The
`DeviceBuilder` pattern lets you configure addresses, channels, endpoints,
clusters, power mode, and device metadata in a single fluent chain — then call
`.build()` to get a ready-to-run `ZigbeeDevice`.

## Creating a Builder

The entry point is always `ZigbeeDevice::builder(mac)`, where `mac` is your
platform's `MacDriver` implementation:

```rust,no_run,ignore
use zigbee_runtime::ZigbeeDevice;
use zigbee_mac::esp::EspMac;           // or nrf::NrfMac, mock::MockMac, …

let mac = EspMac::new();
let device = ZigbeeDevice::builder(mac)
    // ... configuration ...
    .build();
```

Under the hood this calls `DeviceBuilder::new(mac)`, which sets sensible
defaults:

| Field              | Default                       |
|--------------------|-------------------------------|
| `device_type`      | `DeviceType::EndDevice`       |
| `channel_mask`     | `ChannelMask::ALL_2_4GHZ`     |
| `power_mode`       | `PowerMode::AlwaysOn`         |
| `manufacturer`     | `"zigbee-rs"`                 |
| `model`            | `"Generic"`                   |
| `sw_build`         | `"0.1.0"`                     |
| `date_code`        | `""`                          |
| `power_source`     | `PowerSource::Unknown`        |

You only override what you need.

## Configuration Methods

### Device Type

Set the Zigbee role — this affects how the stack joins and routes:

```rust,ignore
use zigbee_nwk::DeviceType;

// End Device — joins a network, does not route (default)
builder.device_type(DeviceType::EndDevice)

// Router — joins a network and relays frames for others
builder.device_type(DeviceType::Router)

// Coordinator — forms a new network (PAN coordinator)
builder.device_type(DeviceType::Coordinator)
```

### Channel Mask

Control which 2.4 GHz channels (11–26) the device scans when joining:

```rust,ignore
use zigbee_types::ChannelMask;

// Scan all channels (default)
builder.channels(ChannelMask::ALL_2_4GHZ)

// Scan only channels 15, 20, and 25
builder.channels(ChannelMask::from_channels(&[15, 20, 25]))

// Single channel — useful for testing
builder.channels(ChannelMask::single(15))
```

### Power Mode

Determines sleep behavior.  This also sets `rx_on_when_idle` in the MAC
capability info sent during association:

```rust,ignore
use zigbee_runtime::power::PowerMode;

// Always on — router or mains-powered end device (default)
builder.power_mode(PowerMode::AlwaysOn)

// Sleepy End Device — wakes to poll periodically
builder.power_mode(PowerMode::Sleepy {
    poll_interval_ms: 5_000,     // poll parent every 5 s
    wake_duration_ms: 500,       // stay awake 500 ms after activity
})

// Deep sleep — wake only on timer (extreme battery savings)
builder.power_mode(PowerMode::DeepSleep {
    wake_interval_s: 3600,       // wake once per hour
})
```

When `PowerMode::Sleepy` or `PowerMode::DeepSleep` is set, the builder
automatically calls `nwk.set_rx_on_when_idle(false)` so the coordinator knows
this is a Sleepy End Device and will buffer frames for it.

### Device Metadata

These values populate the **Basic cluster** (0x0000) attributes that Zigbee
coordinators and tools like Zigbee2MQTT read during device interview:

```rust,ignore
builder
    .manufacturer("Acme Corp")       // ManufacturerName (attr 0x0004)
    .model("TempSensor-v2")          // ModelIdentifier  (attr 0x0005)
    .sw_build("1.3.0")               // SWBuildID        (attr 0x4000)
    .date_code("20260101")           // DateCode         (attr 0x0006)
    .power_source(PowerSource::Battery)
```

## Adding Endpoints

Zigbee devices expose functionality through **endpoints** (1–240).  Each
endpoint has a profile ID, a device ID, and a set of server/client clusters.

Use the `.endpoint()` method with a closure that configures the endpoint's
clusters:

```rust,ignore
builder.endpoint(
    1,        // endpoint number (1-240)
    0x0104,   // profile ID: Home Automation
    DeviceId::TEMPERATURE_SENSOR,
    |ep| {
        ep.cluster_server(ClusterId::BASIC)
          .cluster_server(ClusterId::POWER_CONFIG)
          .cluster_server(ClusterId::IDENTIFY)
          .cluster_server(ClusterId::TEMPERATURE)
    },
)
```

### EndpointBuilder Methods

The closure receives an `EndpointBuilder` with these methods:

| Method              | Description                                      |
|---------------------|--------------------------------------------------|
| `cluster_server(id)` | Add a server-side cluster to the endpoint       |
| `cluster_client(id)` | Add a client-side cluster (you send commands)   |
| `device_version(v)`  | Set the device version (default: 1)             |

**Server clusters** are clusters your device *implements* — other devices can
read attributes and send commands to them.  **Client clusters** are clusters
your device *sends commands to* — for example, a light switch has On/Off as a
client cluster. `ZigbeeDevice` owns Basic and Identify; mutable sensor and
actuator clusters are supplied by the application as `ClusterRef`s.

You can add up to **16 clusters per endpoint** and **8 endpoints per device**.

### Multiple Endpoints

Some devices expose multiple functions.  For example, a multi-sensor:

```rust,ignore
builder
    .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
        ep.cluster_server(ClusterId::BASIC)
          .cluster_server(ClusterId::TEMPERATURE)
    })
    .endpoint(2, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
        ep.cluster_server(ClusterId::HUMIDITY)
    })
    .endpoint(3, 0x0104, DeviceId::IAS_ZONE, |ep| {
        ep.cluster_server(ClusterId::IAS_ZONE)
    })
```

## Using Templates

For common device types, zigbee-rs provides **pre-built templates** in
`zigbee_runtime::templates` that set the correct device type, endpoint,
profile, device ID, and clusters for you:

```rust,ignore
use zigbee_runtime::templates;

// Temperature sensor (endpoint 1, device ID 0x0302)
// Clusters: Basic, Power Config, Identify, Temperature Measurement
let device = templates::temperature_sensor(mac)
    .manufacturer("My Company")
    .model("TH-Sensor-01")
    .build();
```

Templates return a `DeviceBuilder`, so you can chain additional configuration
after them.

### Available Templates

| Template                       | Device ID | Type       | Key Clusters                      |
|-------------------------------|-----------|------------|-----------------------------------|
| `temperature_sensor`           | 0x0302    | EndDevice  | Basic, PowerCfg, Identify, Temp   |
| `temperature_humidity_sensor`  | 0x0302    | EndDevice  | + Relative Humidity               |
| `on_off_light`                 | 0x0100    | Router     | Basic, Identify, Groups, Scenes, On/Off |
| `dimmable_light`               | 0x0101    | Router     | + Level Control                   |
| `color_temperature_light`      | 0x010C    | Router     | + Color Control                   |
| `contact_sensor`               | 0x0402    | EndDevice  | Basic, PowerCfg, Identify, IAS Zone |
| `occupancy_sensor`             | 0x0107    | EndDevice  | Basic, PowerCfg, Identify, Occupancy |
| `smart_plug`                   | 0x0009    | Router     | Basic, Identify, Groups, Scenes, On/Off, Electrical Meas |
| `thermostat`                   | 0x0301    | Router     | Basic, Identify, Groups, Thermostat, Temp |

> **Note:** Templates set the device type for you.  Lights and plugs default to
> `Router` (they're mains-powered and relay traffic).  Sensors default to
> `EndDevice`.

## Building the Device

Once configuration is complete, call `.build()` to construct the full stack:

```rust,ignore
let mut device = ZigbeeDevice::builder(mac)
    .device_type(DeviceType::EndDevice)
    .manufacturer("Acme Corp")
    .model("TempSensor-v2")
    .date_code("20260325")
    .sw_build("1.3.0")
    .power_source(PowerSource::Battery)
    .channels(ChannelMask::from_channels(&[15, 20, 25]))
    .power_mode(PowerMode::Sleepy {
        poll_interval_ms: 5_000,
        wake_duration_ms: 500,
    })
    .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
        ep.cluster_server(ClusterId::BASIC)
          .cluster_server(ClusterId::POWER_CONFIG)
          .cluster_server(ClusterId::IDENTIFY)
          .cluster_server(ClusterId::TEMPERATURE)
          .cluster_server(ClusterId::HUMIDITY)
    })
    .build();
```

### What `.build()` Does

The builder constructs the entire layer stack:

1. Creates the **NWK layer** with the MAC driver and device type
2. Sets `rx_on_when_idle` based on power mode
3. Wraps NWK in the **APS layer**
4. Wraps APS in the **ZDO layer** and registers all endpoint descriptors
5. Sets the node descriptor (logical type, power descriptor)
6. Wraps ZDO in the **BDB layer** for commissioning
7. Creates the **ReportingEngine** for automatic attribute reporting
8. Creates the **PowerManager** with the configured power mode

The result is a `ZigbeeDevice<M>` ready for `start()` and the event loop.

## Complete Example

Here's a full example of a battery-powered temperature + humidity sensor:

```rust,no_run,ignore
use zigbee_runtime::{ZigbeeDevice, ClusterRef, UserAction};
use zigbee_runtime::power::PowerMode;
use zigbee_mac::nrf::NrfMac;
use zigbee_nwk::DeviceType;
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::{ClusterId, DeviceId};

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mac = NrfMac::new(/* peripherals */);

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Acme Corp")
        .model("TH-Sensor-01")
        .sw_build("1.3.0")
        .date_code("20260325")
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 7_500,
            wake_duration_ms: 500,
        })
        .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
            ep.cluster_server(ClusterId::BASIC)
              .cluster_server(ClusterId::POWER_CONFIG)
              .cluster_server(ClusterId::IDENTIFY)
              .cluster_server(ClusterId::TEMPERATURE)
              .cluster_server(ClusterId::HUMIDITY)
        })
        .build();

    // Join the network
    device.user_action(UserAction::Join);

    // ... enter event loop (see Event Loop chapter)
}
```

## What's Next

After building, you need to:

1. **[Start the event loop](event-loop.md)** — call `tick()` and
   `process_incoming()` in a loop to drive the stack
2. **Register cluster instances** — pass `ClusterRef` slices to `tick()` so the
   runtime can handle attribute reads/writes and send reports
3. **Persist state** — call `save_state(nv)` after joining so the device can
   rejoin quickly after reboot
