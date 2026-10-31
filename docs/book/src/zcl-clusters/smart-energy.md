# Smart Energy Clusters

Smart Energy clusters provide utility metering for electricity, gas, and water consumption.

---

## Simple Metering (0x0702)

Tracks cumulative energy consumption and instantaneous demand. This is a **read/report-only** cluster with no cluster-specific commands — all data is published via attribute reads and reporting.

### Attributes

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| CurrentSummationDelivered | `0x0000` | U48 | Report | Total energy delivered to premises |
| CurrentSummationReceived | `0x0001` | U48 | Report | Total energy exported (solar, etc.) |
| UnitOfMeasure | `0x0300` | Enum8 | Read | Measurement unit |
| Multiplier | `0x0301` | U24 | Read | Value multiplier |
| Divisor | `0x0302` | U24 | Read | Value divisor |
| SummationFormatting | `0x0303` | Bitmap8 | Read | Display format |
| DemandFormatting | `0x0304` | Bitmap8 | Read | Demand display format |
| MeteringDeviceType | `0x0308` | Bitmap8 | Read | Device type |
| InstantaneousDemand | `0x0400` | I32 | Report | Current power draw (signed) |
| PowerFactor | `0x0510` | I8 | Read | Power factor (-100 to +100) |

### Unit of Measure Values

| Value | Unit | Description |
|-------|------|-------------|
| `0x00` | kWh | Kilowatt hours |
| `0x01` | m³ | Cubic meters |
| `0x02` | ft³ | Cubic feet |
| `0x03` | CCF | Centum cubic feet |
| `0x04` | US gal | US gallons |
| `0x05` | IMP gal | Imperial gallons |
| `0x06` | BTU | British thermal units |
| `0x07` | L | Liters |
| `0x08` | kPa | Kilopascals (gas pressure) |

### Metering Device Types

| Value | Type |
|-------|------|
| `0x00` | Electric metering |
| `0x01` | Gas metering |
| `0x02` | Water metering |

### Value Conversion

To convert raw attribute values to engineering units:

```
Actual Value = RawValue × Multiplier ÷ Divisor
```

For example, with `Multiplier=1` and `Divisor=1000`:
- `CurrentSummationDelivered = 123456` → 123.456 kWh
- `InstantaneousDemand = 1500` → 1.500 kW

### Usage

```rust
use zigbee_zcl::clusters::metering::*;

// Electric meter: kWh, multiplier=1, divisor=1000
let mut meter = MeteringCluster::new(UNIT_KWH, 1, 1000);

// In your metering ISR / periodic callback:
meter.add_energy_delivered(100);  // Add 100 Wh
meter.set_instantaneous_demand(1500); // 1.5 kW draw

// Read cumulative total:
let total_wh = meter.get_total_delivered(); // returns u64

// After reboot, restore the counter persisted by the application:
meter.set_total_delivered(saved_wh);
```

The summation counter saturates at the 48-bit maximum; `handle_command()`
returns `UnsupClusterCommand`. `CurrentSummationReceived` and `PowerFactor`
have no setters and must be written through `attributes_mut().set_raw()`.

### Reporting Example

A typical energy monitor reports:
- **InstantaneousDemand**: every 10 seconds, or on 100W change
- **CurrentSummationDelivered**: every 5 minutes, or on 100 Wh change

```
Configure Reporting for Metering (0x0702):
  Attribute: InstantaneousDemand (0x0400), Type: I32
    Min: 10s, Max: 60s, Change: 100
  Attribute: CurrentSummationDelivered (0x0000), Type: U48
    Min: 60s, Max: 300s, Change: 100
```

---

## Electrical Measurement (0x0B04)

While technically a "Measurement & Sensing" cluster (`zigbee_zcl::clusters::electrical::ElectricalMeasurementCluster`), Electrical Measurement is closely related to Smart Energy metering. It provides real-time electrical parameters, including:

- **RMS Voltage** (0x0505)
- **RMS Current** (0x0508)
- **Active Power** (0x050B)
- **Power Factor** (0x0510)

See the [Measurement & Sensing](./measurement.md) chapter for details.
