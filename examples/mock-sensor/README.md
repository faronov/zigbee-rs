# Mock Temperature + Humidity Sensor

Simulates a battery-powered Zigbee temperature and humidity sensor on the host
machine using `MockMac`. No hardware required.

## What It Demonstrates

- **MockMac** — pre-configured beacons and association responses for offline testing
- **DeviceBuilder** — `templates::temperature_humidity_sensor()` for a HA-profile end device
- **MAC primitives** — MLME-RESET, MLME-SCAN (active), MLME-ASSOCIATE, MLME-START
- **ZCL clusters** — runtime-owned Basic identity plus application-owned `TemperatureCluster` and `HumidityCluster`
- **Attribute read/write** — simulated sensor readings with `set_temperature()` / `set_humidity()`, read-back via `Cluster::attributes()`

## Build & Run

```sh
cargo run -p mock-sensor
```

## Expected Output

1. Configures a MockMac with a coordinator beacon on channel 15 (PAN `0x1A62`)
2. Builds a temperature + humidity sensor (endpoint 1: Basic, Power Config, Identify, Temperature, Humidity)
3. Performs scan → associate → start join sequence, gets short address `0x796F`
4. Simulates four sensor readings (e.g. 23.50 °C / 65.00 %, 24.10 °C / 63.80 %, …)
5. Reads back all cluster attributes and prints the TX history

## Project Structure

```
mock-sensor/
├── Cargo.toml      # Dependencies: zigbee-*, pollster
└── src/
    └── main.rs     # Sensor simulation (~330 lines)
```

## Dependencies

| Crate | Purpose |
|---|---|
| `zigbee-mac` (mock) | MockMac, MAC primitives |
| `zigbee-zcl` | Temperature, Humidity, Basic clusters |
| `zigbee-runtime` | DeviceBuilder, templates |
| `zigbee-types` | IeeeAddress, PanId, ChannelMask |
| `pollster` | Block on async MAC calls |
