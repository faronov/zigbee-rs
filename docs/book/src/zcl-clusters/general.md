# General Clusters

General clusters provide core functionality required by most Zigbee devices. This chapter covers every general cluster implemented in zigbee-rs.

---

## Basic (0x0000)

**Mandatory for all Zigbee devices.** Exposes identity and version information.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| ZCLVersion | `0x0000` | U8 | Read | ZCL revision (8) |
| ApplicationVersion | `0x0001` | U8 | Read | Application version |
| StackVersion | `0x0002` | U8 | Read | Stack version |
| HWVersion | `0x0003` | U8 | Read | Hardware version |
| ManufacturerName | `0x0004` | CharString | Read | Manufacturer name |
| ModelIdentifier | `0x0005` | CharString | Read | Model identifier |
| DateCode | `0x0006` | CharString | Read | Date code |
| PowerSource | `0x0007` | Enum8 | Read | Power source (0x01=mains, 0x03=battery) |
| LocationDescription | `0x0010` | CharString | R/W | User-settable location |
| SWBuildID | `0x4000` | CharString | Read | Software build identifier |

**Commands:** `ResetToFactoryDefaults` (0x00)

```rust
use zigbee_zcl::clusters::basic::{BasicCluster, PowerSource};

let basic = BasicCluster::new(
    "zigbee-rs",     // manufacturer
    "MyDevice",      // model
    "20250101",      // date code
    "1.0.0",         // sw build
    PowerSource::Battery,
);
```

`BasicCluster::new_with_application_version()` additionally sets
ApplicationVersion (default 1). Strings longer than `MAX_STRING_LEN` are
truncated at a UTF-8 boundary.

Device applications normally configure this runtime-owned cluster through
`DeviceBuilder` rather than constructing a second instance.

---

## Power Configuration (0x0001)

Battery monitoring and alarm thresholds.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| BatteryVoltage | `0x0020` | U8 | Report | Voltage in 100 mV units |
| BatteryPercentageRemaining | `0x0021` | U8 | Report | Percentage in 0.5% units |
| BatterySize | `0x0031` | Enum8 | R/W | Battery size (3=AA, 4=AAA, …) |
| BatteryQuantity | `0x0033` | U8 | R/W | Number of battery cells |
| BatteryRatedVoltage | `0x0034` | U8 | R/W | Rated voltage (100 mV units) |
| BatteryAlarmMask | `0x0035` | Bitmap8 | R/W | Alarm enable bits |
| BatteryVoltageMinThreshold | `0x0036` | U8 | R/W | Low-voltage threshold |
| BatteryAlarmState | `0x003E` | Bitmap32 | Read | Active alarm bits |

```rust
use zigbee_zcl::clusters::power_config::PowerConfigCluster;

let mut pwr = PowerConfigCluster::new();
pwr.set_battery_voltage(33);       // 3.3V
pwr.set_battery_percentage(200);   // 100%
pwr.set_battery_size(3);           // AA
pwr.set_battery_quantity(2);
pwr.set_battery_voltage_min_threshold(24); // 2.4V alarm threshold
pwr.update_alarm_state();           // Recalculate alarms
```

---

## Device Temperature Configuration (0x0002)

Internal device temperature with min/max tracking and alarm thresholds.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| CurrentTemperature | `0x0000` | I16 | Read |
| MinTempExperienced | `0x0001` | I16 | Read |
| MaxTempExperienced | `0x0002` | I16 | Read |
| OverTempTotalDwell | `0x0003` | U16 | Read |
| DeviceTempAlarmMask | `0x0010` | U8 | R/W |
| LowTempThreshold / HighTempThreshold | `0x0011` / `0x0012` | I16 | R/W |
| LowTempDwellTripPoint / HighTempDwellTripPoint | `0x0013` / `0x0014` | U24 | R/W |

Values are stored in **centi-degrees** (2500 = 25.00 °C) rather than the
spec's whole degrees, for ZHA interoperability.

```rust
use zigbee_zcl::clusters::device_temp_config::DeviceTempConfigCluster;

let mut dev_temp = DeviceTempConfigCluster::new();
dev_temp.set_temperature(2500); // also updates min/max experienced
```

---

## Identify (0x0003)

Allows a coordinator to make a device identify itself (e.g. blink an LED).

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| IdentifyTime | `0x0000` | U16 | R/W |

**Commands:** `Identify` (0x00), `IdentifyQuery` (0x01), `TriggerEffect` (0x40)

```rust
let endpoint = 1;
if device.is_identifying(endpoint) {
    toggle_led();
}
// Check for trigger effects:
if let Some((effect_id, variant)) = device.take_identify_effect(endpoint) {
    play_effect(effect_id, variant);
}
```

`ZigbeeDevice` owns and ticks the standard Identify cluster. Applications can
query it with `device.is_identifying(endpoint)` and consume trigger effects
with `device.take_identify_effect(endpoint)`.

---

## Groups (0x0004)

Manages group membership for multicast addressing.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| NameSupport | `0x0000` | Bitmap8 | Read |

**Commands:** `AddGroup` (0x00), `ViewGroup` (0x01), `GetGroupMembership` (0x02), `RemoveGroup` (0x03), `RemoveAllGroups` (0x04), `AddGroupIfIdentifying` (0x05)

```rust
use zigbee_zcl::clusters::groups::{GroupsCluster, GroupAction};

let mut groups = GroupsCluster::new();
// After handling a command, check for APS table actions:
match groups.take_action() {
    GroupAction::Added(id) => aps_add_group(endpoint, id),
    GroupAction::Removed(id) => aps_remove_group(endpoint, id),
    GroupAction::RemovedAll => aps_remove_all_groups(endpoint),
    GroupAction::None => {},
}
```

---

## Scenes (0x0005)

Stores and recalls attribute snapshots (scenes) associated with groups.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| SceneCount | `0x0000` | U8 | Read |
| CurrentScene | `0x0001` | U8 | Read |
| CurrentGroup | `0x0002` | U16 | Read |
| SceneValid | `0x0003` | Bool | Read |
| NameSupport | `0x0004` | Bitmap8 | Read |
| LastConfiguredBy | `0x0005` | IEEE | Read |

**Commands:** `AddScene` (0x00), `ViewScene` (0x01), `RemoveScene` (0x02), `RemoveAllScenes` (0x03), `StoreScene` (0x04), `RecallScene` (0x05), `GetSceneMembership` (0x06)

The cluster maintains a fixed-capacity scene table (16 entries) with extension data for attribute snapshots.

```rust
use zigbee_zcl::clusters::scenes::ScenesCluster;

let scenes = ScenesCluster::new();
// scene_count() returns number of active scenes
```

---

## On/Off (0x0006)

The most common actuator cluster — controls a boolean on/off state.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| OnOff | `0x0000` | Bool | Report | Current state |
| GlobalSceneControl | `0x4000` | Bool | Read | Global scene recall flag |
| OnTime | `0x4001` | U16 | R/W | Timed-on remaining (1/10s) |
| OffWaitTime | `0x4002` | U16 | R/W | Off-wait remaining (1/10s) |
| StartUpOnOff | `0x4003` | Enum8 | R/W | Startup behavior |

**Commands:** `Off` (0x00), `On` (0x01), `Toggle` (0x02), `OffWithEffect` (0x40), `OnWithRecallGlobalScene` (0x41), `OnWithTimedOff` (0x42)

```rust
use zigbee_zcl::clusters::on_off::OnOffCluster;

let mut on_off = OnOffCluster::new();
assert!(!on_off.is_on());

// In the 100ms timer callback:
on_off.tick(); // manages OnTime/OffWaitTime timers

// On startup:
on_off.apply_startup(previous_state);
```

---

## On/Off Switch Configuration (0x0007)

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| SwitchType | `0x0000` | Enum8 | Read |
| SwitchActions | `0x0010` | Enum8 | R/W |

```rust
use zigbee_zcl::clusters::on_off_switch_config::*;

let cfg = OnOffSwitchConfigCluster::new(SWITCH_TYPE_TOGGLE);
```

---

## Level Control (0x0008)

Smooth dimming transitions with `TransitionManager` (`zigbee_zcl::transition`).

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| CurrentLevel | `0x0000` | U8 | Report | Current brightness (0–254) |
| RemainingTime | `0x0001` | U16 | Read | Transition time left (1/10s) |
| MinLevel | `0x0002` | U8 | Read | Minimum level |
| MaxLevel | `0x0003` | U8 | Read | Maximum level |
| OnOffTransitionTime | `0x0010` | U16 | R/W | Default transition time |
| OnLevel | `0x0011` | U8 | R/W | Level when turned on |
| Options | `0x000F` | Bitmap8 | R/W | Options bitmap (stored only) |
| StartUpCurrentLevel | `0x4000` | U8 | R/W | Level on power-up |

**Commands:** `MoveToLevel` (0x00), `Move` (0x01), `Step` (0x02), `Stop` (0x03), `MoveToLevelWithOnOff` (0x04), `MoveWithOnOff` (0x05), `StepWithOnOff` (0x06), `StopWithOnOff` (0x07)

```rust
use zigbee_zcl::clusters::level_control::LevelControlCluster;

let mut level = LevelControlCluster::new();
// In the 100ms timer callback:
level.tick(1); // 1 decisecond
let brightness = level.current_level(); // 0–254
set_pwm(brightness);
```

---

## Time (0x000A)

Provides UTC time and timezone information. `Time`, `TimeStatus`, `TimeZone`,
`DstStart`/`DstEnd`/`DstShift` and `ValidUntilTime` are writable so a
coordinator can set the time; `StandardTime`, `LocalTime` and `LastSetTime` are
read-only.

```rust
use zigbee_zcl::clusters::time::TimeCluster;

let mut time = TimeCluster::new();
time.set_time(zcl_utc_seconds); // seconds since 2000-01-01 (ZCL epoch)
let now = time.get_time();
```

---

## Alarms (0x0009)

Exposes `AlarmCount` and accepts `ResetAlarm` (0x00), `ResetAllAlarms` (0x01),
`GetAlarm` (0x02) and `ResetAlarmLog` (0x03). No alarm log is stored: Get Alarm
always answers "not found", and the reset commands clear `AlarmCount`.
Applications build outgoing Alarm notifications with
`AlarmsCluster::build_alarm_payload()`.

```rust
use zigbee_zcl::clusters::alarms::AlarmsCluster;

let payload = AlarmsCluster::build_alarm_payload(alarm_code, cluster_id);
```

---

## OTA Upgrade (0x0019)

Client cluster for over-the-air firmware updates. Tracks image version, file offset, and download status.

```rust
use zigbee_zcl::clusters::ota::OtaCluster;

let mut ota = OtaCluster::new(manufacturer_code, image_type, current_version);
// See also: zigbee_zcl::clusters::ota_image for image header parsing
```

---

## Poll Control (0x0020)

Manages polling intervals for sleepy end devices.

| Attribute | ID | Type | Access |
|-----------|----|------|--------|
| CheckInInterval | `0x0000` | U32 | R/W |
| LongPollInterval | `0x0001` | U32 | Read |
| ShortPollInterval | `0x0002` | U16 | Read |
| FastPollTimeout | `0x0003` | U16 | R/W |
| CheckInIntervalMin / LongPollIntervalMin | `0x0004` / `0x0005` | U32 | Read |
| FastPollTimeoutMax | `0x0006` | U16 | Read |

**Commands:** C→S `CheckInResponse` (0x00), `FastPollStop` (0x01),
`SetLongPollInterval` (0x02), `SetShortPollInterval` (0x03); S→C `CheckIn` (0x00).

```rust
use zigbee_zcl::clusters::poll_control::PollControlCluster;

let mut poll = PollControlCluster::new();
if poll.tick() { // call every 250 ms (quarter-second)
    let payload = poll.trigger_checkin(); // send as CheckIn (0x00)
}
```

---

## Green Power (0x0021)

Proxy/sink for Green Power devices (battery-less sensors).

```rust
use zigbee_zcl::clusters::green_power::{GreenPowerCluster, GpRole};

let gp = GreenPowerCluster::new(GpRole::Proxy); // or Sink / Combined
```

---

## Diagnostics (0x0B05)

Network and hardware diagnostic counters (MAC/APS Rx/Tx counters,
`NumberOfResets`, `LastMessageLQI`/`LastMessageRSSI`). All attributes are
read-only.

```rust
use zigbee_zcl::clusters::diagnostics::{DiagnosticsCluster, ATTR_MAC_TX_UCAST};

let mut diag = DiagnosticsCluster::new();
diag.increment_counter(ATTR_MAC_TX_UCAST);
```

---

## Analog, Binary and Multistate (Basic) (0x000C–0x0012)

BACnet-style value clusters. All expose `PresentValue` (0x0055),
`StatusFlags` (0x006F), `OutOfService` (0x0051), `Reliability` (0x0067) and
`ApplicationType` (0x0100).

| ID | Cluster | Struct | PresentValue type | Setter |
|----|---------|--------|-------------------|--------|
| `0x000C` | Analog Input | `AnalogInputCluster` | Float32 (reportable) | `set_present_value(f32)` |
| `0x000D` | Analog Output | `AnalogOutputCluster` | Float32 (R/W) | `set_present_value(f32)` |
| `0x000E` | Analog Value | `AnalogValueCluster` | Float32 (R/W) | `set_present_value(f32)` |
| `0x000F` | Binary Input | `BinaryInputCluster` | Bool (reportable) | `set_present_value(bool)` |
| `0x0010` | Binary Output | `BinaryOutputCluster` | Bool (R/W) | `set_present_value(bool)` |
| `0x0011` | Binary Value | `BinaryValueCluster` | Bool (R/W) | `set_present_value(bool)` |
| `0x0012` | Multistate Input | `MultistateInputCluster` | U16 (reportable) | `set_value(u16)` |

The Analog clusters are only compiled with the `float32` feature (enabled by
default). `MultistateInputCluster::new(number_of_states)` takes the
`NumberOfStates` value.

---

## Common Pattern

All general clusters follow the same pattern:

```rust
use zigbee_zcl::clusters::Cluster;

// 1. Create the cluster
let mut cluster = OnOffCluster::new();

// 2. The runtime dispatches foundation commands automatically
//    (read attributes, write attributes, reporting, discovery)

// 3. Cluster-specific commands go through handle_command():
let result = cluster.handle_command(CommandId(0x02), &[]); // Toggle

// 4. Read attributes from application code:
let val = cluster.attributes().get(AttributeId(0x0000));

// 5. If the cluster has a tick(), call it from your timer:
cluster.tick();
```
