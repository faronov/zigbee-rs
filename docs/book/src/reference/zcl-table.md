# ZCL Cluster Table

Complete reference of all ZCL clusters implemented in `zigbee-zcl`. Sorted by cluster ID and grouped by category.

---

## General Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0000` | **Basic** | `ZclVersion` (0x0000), `ManufacturerName` (0x0004), `ModelIdentifier` (0x0005), `DateCode` (0x0006), `PowerSource` (0x0007), `SwBuildId` (0x4000) | `ResetToFactoryDefaults` (0x00) | `basic` |
| `0x0001` | **Power Configuration** | `BatteryVoltage` (0x0020), `BatteryPercentageRemaining` (0x0021), `BatteryAlarmMask` (0x0035), `BatterySize` (0x0031), `BatteryAlarmState` (0x003E) | — | `power_config` |
| `0x0002` | **Device Temperature Configuration** | `CurrentTemperature` (0x0000), `MinTempExperienced` (0x0001), `MaxTempExperienced` (0x0002), `DeviceTempAlarmMask` (0x0010) | — | `device_temp_config` |
| `0x0003` | **Identify** | `IdentifyTime` (0x0000) | `Identify` (0x00), `IdentifyQuery` (0x01), `TriggerEffect` (0x40) | `identify` |
| `0x0004` | **Groups** | `NameSupport` (0x0000) | `AddGroup` (0x00), `ViewGroup` (0x01), `GetGroupMembership` (0x02), `RemoveGroup` (0x03), `RemoveAllGroups` (0x04), `AddGroupIfIdentifying` (0x05) | `groups` |
| `0x0005` | **Scenes** | `SceneCount` (0x0000), `CurrentScene` (0x0001), `CurrentGroup` (0x0002), `SceneValid` (0x0003) | `AddScene` (0x00), `ViewScene` (0x01), `RemoveScene` (0x02), `RemoveAllScenes` (0x03), `StoreScene` (0x04), `RecallScene` (0x05), `GetSceneMembership` (0x06) | `scenes` |
| `0x0006` | **On/Off** | `OnOff` (0x0000), `GlobalSceneControl` (0x4000), `OnTime` (0x4001), `OffWaitTime` (0x4002), `StartUpOnOff` (0x4003) | `Off` (0x00), `On` (0x01), `Toggle` (0x02), `OffWithEffect` (0x40), `OnWithRecallGlobalScene` (0x41), `OnWithTimedOff` (0x42) | `on_off` |
| `0x0007` | **On/Off Switch Configuration** | `SwitchType` (0x0000), `SwitchActions` (0x0010) | — | `on_off_switch_config` |
| `0x0008` | **Level Control** | `CurrentLevel` (0x0000), `RemainingTime` (0x0001), `MinLevel` (0x0002), `MaxLevel` (0x0003), `OnOffTransitionTime` (0x0010), `OnLevel` (0x0011), `StartupCurrentLevel` (0x4000) | `MoveToLevel` (0x00), `Move` (0x01), `Step` (0x02), `Stop` (0x03), `MoveToLevelWithOnOff` (0x04), `MoveWithOnOff` (0x05), `StepWithOnOff` (0x06), `StopWithOnOff` (0x07) | `level_control` |
| `0x0009` | **Alarms** | `AlarmCount` (0x0000) | `ResetAlarm` (0x00), `ResetAllAlarms` (0x01), `GetAlarm` (0x02), `ResetAlarmLog` (0x03) | `alarms` |
| `0x000A` | **Time** | `Time` (0x0000), `TimeStatus` (0x0001), `TimeZone` (0x0002), `DstStart` (0x0003), `DstEnd` (0x0004), `LocalTime` (0x0007) | — | `time` |
| `0x000C` | **Analog Input (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `MinPresentValue` (0x0045), `MaxPresentValue` (0x0041), `EngineeringUnits` (0x0075) | — | `analog_input` |
| `0x000D` | **Analog Output (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `RelinquishDefault` (0x0068), `EngineeringUnits` (0x0075) | — | `analog_output` |
| `0x000E` | **Analog Value (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `RelinquishDefault` (0x0068), `EngineeringUnits` (0x0075) | — | `analog_value` |
| `0x000F` | **Binary Input (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `Polarity` (0x0054), `OutOfService` (0x0051), `Reliability` (0x0067) | — | `binary_input` |
| `0x0010` | **Binary Output (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `Polarity` (0x0054), `RelinquishDefault` (0x0068) | — | `binary_output` |
| `0x0011` | **Binary Value (Basic)** | `PresentValue` (0x0055), `StatusFlags` (0x006F), `RelinquishDefault` (0x0068) | — | `binary_value` |
| `0x0012` | **Multistate Input (Basic)** | `PresentValue` (0x0055), `NumberOfStates` (0x004A), `StatusFlags` (0x006F) | — | `multistate_input` |
| `0x0019` | **OTA Upgrade** | `UpgradeServerId` (0x0000), `FileOffset` (0x0001), `CurrentFileVersion` (0x0002), `ImageUpgradeStatus` (0x0006), `ManufacturerId` (0x0007), `ImageTypeId` (0x0008) | `ImageNotify` (0x00), `QueryNextImageReq` (0x01), `QueryNextImageRsp` (0x02), `ImageBlockReq` (0x03), `ImageBlockRsp` (0x05), `UpgradeEndReq` (0x06), `UpgradeEndRsp` (0x07) | `ota` |
| `0x0020` | **Poll Control** | `CheckInInterval` (0x0000), `LongPollInterval` (0x0001), `ShortPollInterval` (0x0002), `FastPollTimeout` (0x0003) | `CheckIn` (0x00), `CheckInResponse` (0x00), `FastPollStop` (0x01), `SetLongPollInterval` (0x02), `SetShortPollInterval` (0x03) | `poll_control` |
| `0x0021` | **Green Power** | `GppMaxProxyTableEntries` (0x0010), `ProxyTable` (0x0011), `GpsFunctionality` (0x0005), `GpsSinkTable` (0x0000), `GpsSecurityLevel` (0x0004) | **C→S:** `GpNotification` (0x00), `GpPairingSearch` (0x01), `GpCommissioningNotification` (0x04), `GpPairingConfiguration` (0x09) — **S→C:** `GpPairing` (0x01), `GpProxyCommissioningMode` (0x02), `GpResponse` (0x06) | `green_power` |

---

## Closures Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0101` | **Door Lock** | `LockState` (0x0000), `LockType` (0x0001), `ActuatorEnabled` (0x0002), `DoorState` (0x0003), `OperatingMode` (0x0025), `Language` (0x0021) | `LockDoor` (0x00), `UnlockDoor` (0x01), `Toggle` (0x02), `UnlockWithTimeout` (0x03), `SetPinCode` (0x05), `GetPinCode` (0x06), `ClearPinCode` (0x07) | `door_lock` |
| `0x0102` | **Window Covering** | `WindowCoveringType` (0x0000), `CurrentPositionLiftPercentage` (0x0008), `CurrentPositionTiltPercentage` (0x0009), `ConfigStatus` (0x0007), `Mode` (0x0017) | `UpOpen` (0x00), `DownClose` (0x01), `Stop` (0x02), `GoToLiftValue` (0x04), `GoToLiftPercentage` (0x05), `GoToTiltValue` (0x07), `GoToTiltPercentage` (0x08) | `window_covering` |

---

## HVAC Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0201` | **Thermostat** | `LocalTemperature` (0x0000), `OccupiedCoolingSetpoint` (0x0011), `OccupiedHeatingSetpoint` (0x0012), `SystemMode` (0x001C), `ControlSequenceOfOperation` (0x001B), `ThermostatRunningMode` (0x001E) | `SetpointRaiseLower` (0x00), `SetWeeklySchedule` (0x01), `GetWeeklySchedule` (0x02), `ClearWeeklySchedule` (0x03) | `thermostat` |
| `0x0202` | **Fan Control** | `FanMode` (0x0000), `FanModeSequence` (0x0001) | — | `fan_control` |
| `0x0204` | **Thermostat User Interface** | `TemperatureDisplayMode` (0x0000), `KeypadLockout` (0x0001), `ScheduleProgrammingVisibility` (0x0002) | — | `thermostat_ui` |

---

## Lighting Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0300` | **Color Control** | `CurrentHue` (0x0000), `CurrentSaturation` (0x0001), `CurrentX` (0x0003), `CurrentY` (0x0004), `ColorTemperatureMireds` (0x0007), `ColorMode` (0x0008), `EnhancedCurrentHue` (0x4000), `ColorCapabilities` (0x400A) | `MoveToHue` (0x00), `MoveToSaturation` (0x03), `MoveToHueAndSaturation` (0x06), `MoveToColor` (0x07), `MoveToColorTemperature` (0x0A), `EnhancedMoveToHue` (0x40), `ColorLoopSet` (0x44), `StopMoveStep` (0x47) | `color_control` |
| `0x0301` | **Ballast Configuration** | `PhysicalMinLevel` (0x0000), `PhysicalMaxLevel` (0x0001), `BallastStatus` (0x0002), `MinLevel` (0x0010), `MaxLevel` (0x0011), `LampQuantity` (0x0020) | — | `ballast_config` |

---

## Measurement & Sensing Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0400` | **Illuminance Measurement** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003), `LightSensorType` (0x0004) | — | `illuminance` |
| `0x0401` | **Illuminance Level Sensing** | `LevelStatus` (0x0000), `LightSensorType` (0x0001), `IlluminanceTargetLevel` (0x0010) | — | `illuminance_level` |
| `0x0402` | **Temperature Measurement** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `temperature` |
| `0x0403` | **Pressure Measurement** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003), `ScaledValue` (0x0010), `Scale` (0x0014) | — | `pressure` |
| `0x0404` | **Flow Measurement** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `flow_measurement` |
| `0x0405` | **Relative Humidity** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `humidity` |
| `0x0406` | **Occupancy Sensing** | `Occupancy` (0x0000), `OccupancySensorType` (0x0001), `OccupancySensorTypeBitmap` (0x0002), `PirOToUDelay` (0x0010), `PirUToODelay` (0x0011) | — | `occupancy` |
| `0x0408` | **Soil Moisture** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `soil_moisture` |
| `0x040D` | **Carbon Dioxide (CO₂)** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `carbon_dioxide` |
| `0x042A` | **PM2.5 Measurement** | `MeasuredValue` (0x0000), `MinMeasuredValue` (0x0001), `MaxMeasuredValue` (0x0002), `Tolerance` (0x0003) | — | `pm25` |

---

## Security (IAS) Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0500` | **IAS Zone** | `ZoneState` (0x0000), `ZoneType` (0x0001), `ZoneStatus` (0x0002), `IasCieAddress` (0x0010), `ZoneId` (0x0011), `CurrentZoneSensitivityLevel` (0x0013) | **C→S:** `ZoneEnrollResponse` (0x00), `InitiateNormalOpMode` (0x01), `InitiateTestMode` (0x02) — **S→C:** `ZoneStatusChangeNotification` (0x00), `ZoneEnrollRequest` (0x01) | `ias_zone` |
| `0x0501` | **IAS ACE** | `PanelStatus` (0xFF00) | **C→S:** `Arm` (0x00), `Bypass` (0x01), `Emergency` (0x02), `Fire` (0x03), `Panic` (0x04), `GetZoneIdMap` (0x05), `GetPanelStatus` (0x07) — **S→C:** `ArmResponse` (0x00), `PanelStatusChanged` (0x04) | `ias_ace` |
| `0x0502` | **IAS WD** | `MaxDuration` (0x0000) | `StartWarning` (0x00), `Squawk` (0x01) | `ias_wd` |

---

## Smart Energy Clusters

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x0702` | **Metering** | `CurrentSummationDelivered` (0x0000), `CurrentSummationReceived` (0x0001), `UnitOfMeasure` (0x0300), `Multiplier` (0x0301), `Divisor` (0x0302), `InstantaneousDemand` (0x0400), `MeteringDeviceType` (0x0308) | — | `metering` |
| `0x0B04` | **Electrical Measurement** | `MeasurementType` (0x0000), `RmsVoltage` (0x0505), `RmsCurrent` (0x0508), `ActivePower` (0x050B), `ReactivePower` (0x050E), `ApparentPower` (0x050F), `PowerFactor` (0x0510), `AcVoltageMultiplier` (0x0600), `AcVoltageDivisor` (0x0601) | — | `electrical` |
| `0x0B05` | **Diagnostics** | `NumberOfResets` (0x0000), `MacRxBcast` (0x0100), `MacTxBcast` (0x0101), `MacRxUcast` (0x0102), `MacTxUcast` (0x0103), `MacTxUcastFail` (0x0105), `LastMessageLqi` (0x011C), `LastMessageRssi` (0x011D) | — | `diagnostics` |

---

## Touchlink

| ID | Name | Key Attributes | Key Commands | Module |
|----|------|----------------|--------------|--------|
| `0x1000` | **Touchlink Commissioning** | `TouchlinkState` (0xFF00) | **C→S:** `ScanRequest` (0x00), `IdentifyRequest` (0x06), `ResetToFactoryNewRequest` (0x07), `NetworkStartRequest` (0x10), `NetworkJoinRouterRequest` (0x12), `NetworkJoinEndDeviceRequest` (0x14) — **S→C:** `ScanResponse` (0x01), `NetworkStartResponse` (0x11) | `touchlink` |

---

## Summary by Category

| Category | Count | Cluster ID Range |
|----------|-------|------------------|
| General | 21 | `0x0000` – `0x0021` |
| Closures | 2 | `0x0101` – `0x0102` |
| HVAC | 3 | `0x0201` – `0x0204` |
| Lighting | 2 | `0x0300` – `0x0301` |
| Measurement & Sensing | 10 | `0x0400` – `0x042A` |
| Security (IAS) | 3 | `0x0500` – `0x0502` |
| Smart Energy | 3 | `0x0702` – `0x0B05` |
| Touchlink | 1 | `0x1000` |
| **Total** | **45** | |

> **Note:** The `ota_image` module is a helper for OTA image parsing and is not a separate cluster.
>
> The `analog_input`, `analog_output`, `analog_value`, `carbon_dioxide` and `pm25` modules use Float32 attributes and are only compiled with the `float32` feature of `zigbee-zcl` (enabled by default).

---

## IAS Zone Types

Common `ZoneType` values for the IAS Zone cluster (0x0500):

| Value | Name | Typical Device |
|-------|------|----------------|
| `0x0000` | Standard CIE | CIE device |
| `0x000D` | Motion Sensor | PIR sensor |
| `0x0015` | Contact Switch | Door/window sensor |
| `0x0028` | Fire Sensor | Smoke detector |
| `0x002A` | Water Sensor | Leak detector |
| `0x002B` | CO Sensor | Carbon monoxide detector |
| `0x002D` | Personal Emergency | Panic button |
| `0x010F` | Remote Control | Keyfob |
| `0x0115` | Key Fob | Key fob |
| `0x021D` | Keypad | Security keypad |
| `0x0225` | Standard Warning | Siren/strobe |

---

## Thermostat System Modes

| Value | Mode |
|-------|------|
| `0x00` | Off |
| `0x01` | Auto |
| `0x03` | Cool |
| `0x04` | Heat |
| `0x05` | Emergency Heat |
| `0x07` | Fan Only |

---

## Metering Unit Types

| Value | Unit |
|-------|------|
| `0x00` | kWh (electric) |
| `0x01` | m³ (gas) |
| `0x02` | ft³ |
| `0x03` | CCF |
| `0x04` | US Gallons |
| `0x05` | Imperial Gallons |
| `0x06` | BTU |
| `0x07` | Liters |
| `0x08` | kPa (gauge) |

---

## Color Control Modes

| Value | Mode | Description |
|-------|------|-------------|
| `0x00` | Hue/Saturation | HSV color space |
| `0x01` | XY | CIE 1931 color space |
| `0x02` | Color Temperature | Mireds (reciprocal megakelvins) |
