# Measurement & Sensing Clusters

Measurement clusters are **server-side, read-only** clusters used by sensors. They share a common pattern: a `MeasuredValue` attribute (reportable) plus `MinMeasuredValue` and `MaxMeasuredValue` bounds. The application updates the measured value via a setter method; the runtime handles attribute reads and reporting.

None of these clusters define cluster-specific commands — `handle_command()` always returns `UnsupClusterCommand`.

---

## Temperature Measurement (0x0402)

Values in **0.01°C** units (e.g. 2250 = 22.50°C).

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| MeasuredValue | `0x0000` | I16 | Report | Current temperature × 100 |
| MinMeasuredValue | `0x0001` | I16 | Read | Minimum measurable |
| MaxMeasuredValue | `0x0002` | I16 | Read | Maximum measurable |
| Tolerance | `0x0003` | U16 | Read | Measurement tolerance |

```rust
use zigbee_zcl::clusters::temperature::TemperatureCluster;

let mut temp = TemperatureCluster::new(-4000, 8500); // -40°C to 85°C
temp.set_temperature(2250); // 22.50°C
```

---

## Relative Humidity (0x0405)

Values in **0.01% RH** units (e.g. 5000 = 50.00% RH).

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| MeasuredValue | `0x0000` | U16 | Report | Current humidity × 100 |
| MinMeasuredValue | `0x0001` | U16 | Read | Minimum measurable |
| MaxMeasuredValue | `0x0002` | U16 | Read | Maximum measurable |
| Tolerance | `0x0003` | U16 | Read | Measurement tolerance |

```rust
use zigbee_zcl::clusters::humidity::HumidityCluster;

let mut hum = HumidityCluster::new(0, 10000); // 0–100%
hum.set_humidity(5000); // 50.00% RH
```

---

## Pressure Measurement (0x0403)

Values in **0.1 kPa** (= 1 hPa) units (e.g. 1013 = 1013 hPa). The scaled attributes exist for extended precision but `set_pressure()` only updates `MeasuredValue`; `ScaledValue` (default 0, `Scale` = 0) must be written by the application through `attributes_mut().set_raw()` if used.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| MeasuredValue | `0x0000` | I16 | Report | Pressure in 0.1 kPa |
| MinMeasuredValue | `0x0001` | I16 | Read | Minimum measurable |
| MaxMeasuredValue | `0x0002` | I16 | Read | Maximum measurable |
| Tolerance | `0x0003` | U16 | Read | Measurement tolerance |
| ScaledValue | `0x0010` | I16 | Report | High-precision pressure |
| MinScaledValue | `0x0011` | I16 | Read | Minimum scaled |
| MaxScaledValue | `0x0012` | I16 | Read | Maximum scaled |
| ScaledTolerance | `0x0013` | U16 | Read | Scaled tolerance |
| Scale | `0x0014` | I8 | Read | 10^Scale multiplier |

```rust
use zigbee_zcl::clusters::pressure::PressureCluster;

let mut press = PressureCluster::new(300, 1100); // 300–1100 hPa
press.set_pressure(1013); // 1013 hPa
```

---

## Illuminance Measurement (0x0400)

Measures ambient light level.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasuredValue | `0x0000` | U16 | Report |
| MinMeasuredValue | `0x0001` | U16 | Read |
| MaxMeasuredValue | `0x0002` | U16 | Read |
| Tolerance | `0x0003` | U16 | Read |
| LightSensorType | `0x0004` | Enum8 | Read |

Values use a logarithmic formula: `MeasuredValue = 10,000 × log10(lux) + 1`.
`measured_value_from_centi_lux()` performs this conversion in integer math
(0 lx → `MEASURED_VALUE_TOO_LOW`, clamped to `MEASURED_VALUE_MAX` = 0xFFFE).

```rust
use zigbee_zcl::clusters::illuminance::{IlluminanceCluster, measured_value_from_centi_lux};

let mut lux = IlluminanceCluster::new(1, 0xFFFE);
lux.set_illuminance(measured_value_from_centi_lux(12_345)); // 123.45 lx
```

---

## Flow Measurement (0x0404)

Measures flow rate in **0.1 m³/h** units.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasuredValue | `0x0000` | U16 | Report |
| MinMeasuredValue | `0x0001` | U16 | Read |
| MaxMeasuredValue | `0x0002` | U16 | Read |
| Tolerance | `0x0003` | U16 | Read |

```rust
use zigbee_zcl::clusters::flow_measurement::FlowMeasurementCluster;

let mut flow = FlowMeasurementCluster::new(0, 1000); // min, max
flow.set_flow(125); // 12.5 m³/h
```

---

## Occupancy Sensing (0x0406)

Binary occupancy detection with configurable sensor type.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| Occupancy | `0x0000` | Bitmap8 | Report | Bit 0 = occupied |
| OccupancySensorType | `0x0001` | Enum8 | Read | 0=PIR, 1=Ultrasonic, 2=PIR+US, 3=Physical contact |
| OccupancySensorTypeBitmap | `0x0002` | Bitmap8 | Read | Sensor types present |
| PIROccupiedToUnoccupiedDelay | `0x0010` | U16 | R/W | Seconds |
| PIRUnoccupiedToOccupiedDelay | `0x0011` | U16 | R/W | Seconds |
| PIRUnoccupiedToOccupiedThreshold | `0x0012` | U8 | R/W | Movement events |
| UltrasonicOccupiedToUnoccupiedDelay | `0x0020` | U16 | R/W | Seconds |
| UltrasonicUnoccupiedToOccupiedDelay | `0x0021` | U16 | R/W | Seconds |

```rust
use zigbee_zcl::clusters::occupancy::{OccupancyCluster, SENSOR_TYPE_PIR};

let mut occ = OccupancyCluster::new(SENSOR_TYPE_PIR);
occ.set_occupied(true);
```

---

## Electrical Measurement (0x0B04)

Real-time electrical measurements (voltage, current, power, power factor).

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasurementType | `0x0000` | U32 | Read |
| DcVoltage / DcCurrent / DcPower | `0x0100` / `0x0103` / `0x0106` | I16 | Read |
| AcFrequency | `0x0300` | U16 | Read |
| RmsVoltage | `0x0505` | U16 | Report |
| RmsCurrent | `0x0508` | U16 | Report |
| ActivePower | `0x050B` | I16 | Report |
| ReactivePower | `0x050E` | I16 | Read |
| ApparentPower | `0x050F` | U16 | Read |
| PowerFactor | `0x0510` | I8 | Read |
| AcVoltage/Current/Power Multiplier & Divisor | `0x0600`–`0x0605` | U16 | Read |

```rust
use zigbee_zcl::clusters::electrical::{AcScaling, ElectricalMeasurementCluster};

let mut em = ElectricalMeasurementCluster::new();
em.set_ac_scaling(AcScaling::default())?; // divisors must be non-zero
em.set_measurements(2301, 450, 100); // RMS voltage, RMS current, active power
```

---

## PM2.5 Measurement (0x042A)

Particulate matter (PM2.5) concentration in µg/m³. Requires the `float32`
feature (default).

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasuredValue | `0x0000` | Float32 | Report |
| MinMeasuredValue | `0x0001` | Float32 | Read |
| MaxMeasuredValue | `0x0002` | Float32 | Read |
| Tolerance | `0x0003` | Float32 | Read |

```rust
use zigbee_zcl::clusters::pm25::Pm25Cluster;

let mut pm = Pm25Cluster::new();
pm.set_pm25(12.5); // µg/m³
```

---

## Carbon Dioxide (0x040D)

CO₂ concentration. Per the ZCL concentration-measurement clusters the
attribute holds a fraction (ppm × 10⁻⁶); `set_co2_ppm()` does the conversion.
Requires the `float32` feature (default).

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasuredValue | `0x0000` | Float32 | Report |
| MinMeasuredValue | `0x0001` | Float32 | Read |
| MaxMeasuredValue | `0x0002` | Float32 | Read |
| Tolerance | `0x0003` | Float32 | Read |

```rust
use zigbee_zcl::clusters::carbon_dioxide::CarbonDioxideCluster;

let mut co2 = CarbonDioxideCluster::new();
co2.set_co2_ppm(415.0);
```

---

## Soil Moisture (0x0408)

Soil moisture level in **0.01%** units.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| MeasuredValue | `0x0000` | U16 | Report |
| MinMeasuredValue | `0x0001` | U16 | Read |
| MaxMeasuredValue | `0x0002` | U16 | Read |
| Tolerance | `0x0003` | U16 | Read |

```rust
use zigbee_zcl::clusters::soil_moisture::SoilMoistureCluster;

let mut soil = SoilMoistureCluster::new();
soil.set_moisture(4250); // 42.50 %
```

---

## Common Sensor Pattern

All measurement clusters follow the same usage pattern:

```rust
// 1. Declare the server cluster on an endpoint via the builder
let mut device = ZigbeeDevice::builder(mac)
    .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
        ep.cluster_server(ClusterId::TEMPERATURE)
    })
    .build();

// 2. Create the cluster instance with min/max bounds
let mut sensor = TemperatureCluster::new(-4000, 8500);

// 3. In your sensor read callback, update the value:
sensor.set_temperature(read_adc_temperature());

// 4. Pass the instance to the runtime on every process/tick call
let mut clusters = [ClusterRef { endpoint: 1, cluster: &mut sensor }];
device.tick(elapsed_secs, &mut clusters).await;

// 5. The runtime handles:
//    - Read Attributes responses
//    - Attribute reporting (when configured)
//    - Discover Attributes responses
```

### Reporting Configuration Example

A coordinator typically configures measurement clusters to report on change:

```
Configure Reporting for TemperatureMeasurement (0x0402):
  Attribute: MeasuredValue (0x0000)
  Type: I16
  Min Interval: 30 seconds
  Max Interval: 300 seconds
  Reportable Change: 50 (= 0.50°C)
```

The `ReportingEngine` tracks the last reported value and sends a new report when:
- The value changes by more than 0.50°C **and** at least 30 seconds have passed, **or**
- 300 seconds have passed regardless of change
