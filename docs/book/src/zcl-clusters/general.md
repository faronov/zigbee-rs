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
| ManufacturerName | `0x0004` | String | Read | Manufacturer name |
| ModelIdentifier | `0x0005` | String | Read | Model identifier |
| DateCode | `0x0006` | String | Read | Date code |
| PowerSource | `0x0007` | Enum8 | Read | Power source (0x01=mains, 0x03=battery) |
| LocationDescription | `0x0010` | String | R/W | User-settable location |
| SWBuildID | `0x4000` | String | Read | Software build identifier |

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
| NameSupport | `0x0000` | U8 | Read |

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
| NameSupport | `0x0004` | U8 | Read |
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

## Level Control (0x0008)

Smooth dimming transitions with `TransitionManager`.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| CurrentLevel | `0x0000` | U8 | Report | Current brightness (0–254) |
| RemainingTime | `0x0001` | U16 | Read | Transition time left (1/10s) |
| MinLevel | `0x0002` | U8 | Read | Minimum level |
| MaxLevel | `0x0003` | U8 | Read | Maximum level |
| OnOffTransitionTime | `0x0010` | U16 | R/W | Default transition time |
| OnLevel | `0x0011` | U8 | R/W | Level when turned on |
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

Provides UTC time and timezone information. Attributes are writable so a coordinator can set the time.

```rust
use zigbee_zcl::clusters::time; // TimeCluster
```

---

## Alarms (0x0009)

Maintains an alarm table and handles alarm acknowledgement.

```rust
use zigbee_zcl::clusters::alarms; // AlarmsCluster
```

---

## OTA Upgrade (0x0019)

Client cluster for over-the-air firmware updates. Tracks image version, file offset, and download status.

```rust
use zigbee_zcl::clusters::ota; // OtaCluster
// See also: zigbee_zcl::clusters::ota_image for image header parsing
```

---

## Poll Control (0x0020)

Manages polling intervals for sleepy end devices.

```rust
use zigbee_zcl::clusters::poll_control; // PollControlCluster
```

---

## Green Power (0x0021)

Proxy/sink for Green Power devices (battery-less sensors).

```rust
use zigbee_zcl::clusters::green_power; // GreenPowerCluster
```

---

## Diagnostics (0x0B05)

Network and hardware diagnostic counters.

```rust
use zigbee_zcl::clusters::diagnostics; // DiagnosticsCluster
```

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
