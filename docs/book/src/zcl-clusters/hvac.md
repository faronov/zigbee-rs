# HVAC Clusters

Heating, Ventilation, and Air Conditioning clusters for climate control devices.

---

## Thermostat (0x0201)

A full-featured thermostat with heating/cooling setpoints, weekly scheduling, and automatic mode switching. Temperature values are in **0.01°C** units throughout.

### Attributes

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| LocalTemperature | `0x0000` | I16 | Report | Current sensor reading |
| OutdoorTemperature | `0x0001` | I16 | Read | Outdoor temp (optional) |
| Occupancy | `0x0002` | U8 | Read | Occupancy bitmap |
| AbsMinHeatSetpointLimit | `0x0003` | I16 | Read | Absolute minimum heat SP (700 = 7°C) |
| AbsMaxHeatSetpointLimit | `0x0004` | I16 | Read | Absolute maximum heat SP (3000 = 30°C) |
| AbsMinCoolSetpointLimit | `0x0005` | I16 | Read | Absolute minimum cool SP (1600 = 16°C) |
| AbsMaxCoolSetpointLimit | `0x0006` | I16 | Read | Absolute maximum cool SP (3200 = 32°C) |
| OccupiedCoolingSetpoint | `0x0011` | I16 | R/W | Active cooling setpoint (2600 = 26°C) |
| OccupiedHeatingSetpoint | `0x0012` | I16 | R/W | Active heating setpoint (2000 = 20°C) |
| MinHeatSetpointLimit | `0x0015` | I16 | R/W | Configurable heat SP minimum |
| MaxHeatSetpointLimit | `0x0016` | I16 | R/W | Configurable heat SP maximum |
| MinCoolSetpointLimit | `0x0017` | I16 | R/W | Configurable cool SP minimum |
| MaxCoolSetpointLimit | `0x0018` | I16 | R/W | Configurable cool SP maximum |
| ControlSequenceOfOperation | `0x001B` | Enum8 | R/W | 0x04 = Cooling and Heating |
| SystemMode | `0x001C` | Enum8 | R/W | Current operating mode |
| ThermostatRunningMode | `0x001E` | Enum8 | Read | Computed running mode |

### System Modes

| Value | Mode | Description |
|-------|------|-------------|
| `0x00` | Off | System disabled |
| `0x01` | Auto | Automatic heat/cool switching |
| `0x03` | Cool | Cooling only |
| `0x04` | Heat | Heating only |
| `0x05` | Emergency Heat | Emergency/auxiliary heating |
| `0x07` | Fan Only | Fan without heating/cooling |

### Commands

| ID | Direction | Command |
|----|-----------|---------|
| `0x00` | Client→Server | SetpointRaiseLower |
| `0x01` | Client→Server | SetWeeklySchedule |
| `0x02` | Client→Server | GetWeeklySchedule |
| `0x03` | Client→Server | ClearWeeklySchedule |
| `0x00` | Server→Client | GetWeeklyScheduleResponse |

### Usage

```rust
use zigbee_zcl::clusters::thermostat::ThermostatCluster;

let mut therm = ThermostatCluster::new();

// Update temperature from sensor (22.50°C):
therm.set_local_temperature(2250);

// In the periodic callback — advance schedule and compute running mode:
// day_of_week: bitmask (bit 0 = Sunday .. bit 6 = Saturday)
// minutes_since_midnight: current time of day
therm.tick(0b0000010, 480); // Monday, 08:00

// SystemMode and setpoints can be written remotely via Write Attributes
// RunningMode is computed automatically by tick():
//   - Auto mode: heats if temp < heat SP, cools if temp > cool SP
//   - Heat / Emergency Heat mode: heats if temp < heat SP
//   - Cool mode: cools if temp > cool SP
//   - Off / Fan Only: running mode Off
```

### Weekly Schedule

The thermostat stores up to 16 `WeeklyScheduleEntry` items (one per days-of-week + mode pair; a new SetWeeklySchedule for the same pair replaces it), each holding up to 10 `ScheduleTransition`s:

```rust
// A coordinator sends SetWeeklySchedule (0x01):
//   num_transitions: 3
//   days_of_week: 0b0111110 (Monday–Friday)
//   mode: 0x01 (heat only)
//   transitions:
//     06:00 → heat SP = 21.00°C
//     09:00 → heat SP = 18.00°C  (away)
//     17:00 → heat SP = 21.00°C  (home)

// tick() finds the latest passed transition and applies its setpoints
```

---

## Fan Control (0x0202)

Simple fan speed control with mode enumeration.

### Attributes

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| FanMode | `0x0000` | Enum8 | R/W | Current fan mode |
| FanModeSequence | `0x0001` | Enum8 | R/W | Available mode sequence |

### Fan Modes

| Value | Mode |
|-------|------|
| `0x00` | Off |
| `0x01` | Low |
| `0x02` | Medium |
| `0x03` | High |
| `0x04` | On |
| `0x05` | Auto |
| `0x06` | Smart |

### Fan Mode Sequences

| Value | Sequence |
|-------|----------|
| `0x00` | Low/Med/High |
| `0x01` | Low/High |
| `0x02` | Low/Med/High/Auto |
| `0x03` | Low/High/Auto |
| `0x04` | On/Auto |

No cluster-specific commands — fan mode is set via Write Attributes.

```rust
use zigbee_zcl::clusters::fan_control::FanControlCluster;

let mut fan = FanControlCluster::new();
assert_eq!(fan.fan_mode(), 0x05); // Auto by default
fan.set_fan_mode(0x03); // High
```

---

## Thermostat User Interface Configuration (0x0204)

Controls how the thermostat's local UI behaves.

### Attributes

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| TemperatureDisplayMode | `0x0000` | Enum8 | R/W | 0=Celsius, 1=Fahrenheit |
| KeypadLockout | `0x0001` | Enum8 | R/W | 0=No lockout, 1–5=lockout levels |
| ScheduleProgrammingVisibility | `0x0002` | Enum8 | R/W | 0=enabled, 1=disabled |

```rust
use zigbee_zcl::clusters::thermostat_ui::ThermostatUiCluster;

let mut ui = ThermostatUiCluster::new();
ui.set_display_mode(0x01); // Fahrenheit
ui.set_keypad_lockout(0x01); // Level 1 lockout
```

---

## Putting It Together: Smart Thermostat

```rust
use zigbee_zcl::clusters::thermostat::ThermostatCluster;
use zigbee_zcl::clusters::fan_control::FanControlCluster;
use zigbee_zcl::clusters::thermostat_ui::ThermostatUiCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

let mut therm = ThermostatCluster::new();
let mut fan = FanControlCluster::new();
let mut ui = ThermostatUiCluster::new();
let mut temp_sensor = TemperatureCluster::new(-1000, 5000);

// Periodic callback (every minute):
let reading = read_temperature_sensor(); // 0.01°C units
temp_sensor.set_temperature(reading);
therm.set_local_temperature(reading);
therm.tick(get_day_of_week(), get_minutes_since_midnight());

// The thermostat running mode drives the HVAC relays
```
