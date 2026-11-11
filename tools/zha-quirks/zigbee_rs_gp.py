"""ZHA Green Power quirks for the zigbee-rs Green Power Devices.

- Sensor (examples/efr32mg22-gpd, bin efr32mg22-gpd-sensor): DeviceID 0x33
  (IndoorEnvironmentSensor), manufacturer 0x6D6B, model 0xF300. Sends
  security level 3 Multi-Cluster Reporting (0xA2) frames with temperature
  (0x0402/0x0000, int16, 0.01 degC), humidity (0x0405/0x0000, uint16, 0.01 %)
  and optionally battery (0x0001/0x0021, uint8, 0.5 %).
- Switch (bin efr32mg22-gpd-switch): DeviceID 0x02 (OnOffSwitch),
  manufacturer 0x6D6B, model 0xF400. Sends Off (0x20), On (0x21) and
  Toggle (0x22).

On the BRD4183A dev kit the temperature is the EFR32 die sensor and the
humidity is SYNTHETIC (no humidity sensor fitted).

Copy to <HA config>/custom_zha_quirks/ (requires ZHA with Green Power
support, Home Assistant 2026.11 or later).
"""

from __future__ import annotations

import logging
from typing import Any

from zha.application.platforms.sensor import BaseSensor
from zha.application.platforms.sensor.const import SensorDeviceClass, SensorStateClass
from zha.units import PERCENTAGE, UnitOfTemperature
from zigpy.device import GreenPowerCommandReceived
from zigpy.zgp.types import DeviceID, GPDCommandID

from zhaquirks.builder import GreenPowerQuirkBuilder

try:  # Event entities need a ZHA with the event platform.
    from zha.application.platforms.event import BaseEvent
    from zha.application.platforms.event.const import EventDeviceClass
except ImportError:  # pragma: no cover - older ZHA: sensor quirk only
    BaseEvent = None

_LOGGER = logging.getLogger(__name__)

ZIGBEE_RS_MANUFACTURER_ID = 0x6D6B
ZIGBEE_RS_GP_SENSOR_MODEL_ID = 0xF300
ZIGBEE_RS_GP_SWITCH_MODEL_ID = 0xF400

# GPD command ID -> event type of the switch.
SWITCH_COMMANDS: dict[int, str] = {0x20: "off", 0x21: "on", 0x22: "toggle"}


class ZigbeeRsGreenPowerSensor(BaseSensor):
    """Sensor updated from one ZCL attribute of the GPD attribute reports."""

    _cluster_id: int
    _attribute_id: int
    _invalid: int
    _divisor: float
    _attr_state_class = SensorStateClass.MEASUREMENT

    def __init__(self, device: Any, **kwargs: Any) -> None:
        """Initialize the sensor without a value."""
        self._value: float | None = None
        super().__init__(device, **kwargs)

    @property
    def native_value(self) -> float | None:
        """Return the last reported value."""
        return self._value

    def on_add(self) -> None:
        """Subscribe to the commands sent by the GPD."""
        super().on_add()
        self._on_remove_callbacks.append(
            self.device.device.on_event(
                GreenPowerCommandReceived.event_type, self._handle_gp_command
            )
        )

    def _handle_gp_command(self, event: GreenPowerCommandReceived) -> None:
        if event.command is None:
            return
        if event.command_id == GPDCommandID.MultiClusterReporting:
            records = [(r.cluster_id, r.attribute) for r in event.command.reports]
        elif event.command_id == GPDCommandID.AttributeReporting:
            records = [(event.command.cluster_id, a) for a in event.command.attributes]
        else:
            return
        for cluster_id, attribute in records:
            if cluster_id == self._cluster_id and attribute.attrid == self._attribute_id:
                raw = attribute.value.value
                self._value = None if raw == self._invalid else raw / self._divisor
                self.maybe_emit_state_changed_event()


class ZigbeeRsTemperature(ZigbeeRsGreenPowerSensor):
    """Temperature (0x0402 MeasuredValue, 0.01 degC)."""

    _cluster_id, _attribute_id, _invalid, _divisor = 0x0402, 0x0000, -0x8000, 100
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_suggested_display_precision = 1


class ZigbeeRsHumidity(ZigbeeRsGreenPowerSensor):
    """Relative humidity (0x0405 MeasuredValue, 0.01 %)."""

    _cluster_id, _attribute_id, _invalid, _divisor = 0x0405, 0x0000, 0xFFFF, 100
    _attr_device_class = SensorDeviceClass.HUMIDITY
    _attr_native_unit_of_measurement = PERCENTAGE
    _attr_suggested_display_precision = 0


class ZigbeeRsBattery(ZigbeeRsGreenPowerSensor):
    """Battery remaining (0x0001 BatteryPercentageRemaining, 0.5 %)."""

    _cluster_id, _attribute_id, _invalid, _divisor = 0x0001, 0x0021, 0xFF, 2
    _attr_device_class = SensorDeviceClass.BATTERY
    _attr_native_unit_of_measurement = PERCENTAGE
    _attr_suggested_display_precision = 0


(
    GreenPowerQuirkBuilder()
    .applies_to(
        device_id=DeviceID.IndoorEnvironmentSensor,
        manufacturer_id=ZIGBEE_RS_MANUFACTURER_ID,
        model_id=ZIGBEE_RS_GP_SENSOR_MODEL_ID,
    )
    .adds_entity(
        ZigbeeRsTemperature, unique_id_suffix="temperature", fallback_name="Temperature"
    )
    .adds_entity(ZigbeeRsHumidity, unique_id_suffix="humidity", fallback_name="Humidity")
    .adds_entity(ZigbeeRsBattery, unique_id_suffix="battery", fallback_name="Battery")
    .friendly_name(manufacturer="zigbee-rs", model="GP sensor")
    .add_to_registry()
)

if BaseEvent is None:
    _LOGGER.warning("ZHA has no event platform: zigbee-rs GP switch quirk disabled")
else:

    class ZigbeeRsSwitchEvent(BaseEvent):
        """Event entity firing on/off/toggle for the switch commands."""

        _attr_device_class = EventDeviceClass.BUTTON
        _attr_event_types = list(SWITCH_COMMANDS.values())

        def on_add(self) -> None:
            """Subscribe to the commands sent by the switch."""
            super().on_add()
            self._on_remove_callbacks.append(
                self.device.device.on_event(
                    GreenPowerCommandReceived.event_type, self._handle_command
                )
            )

        def _handle_command(self, event: GreenPowerCommandReceived) -> None:
            event_type = SWITCH_COMMANDS.get(int(event.command_id))
            if event_type is not None:
                self._trigger_event(event_type)

    (
        GreenPowerQuirkBuilder()
        .applies_to(
            device_id=DeviceID.OnOffSwitch,
            manufacturer_id=ZIGBEE_RS_MANUFACTURER_ID,
            model_id=ZIGBEE_RS_GP_SWITCH_MODEL_ID,
        )
        .adds_entity(ZigbeeRsSwitchEvent, unique_id_suffix="button", fallback_name="Button")
        .friendly_name(manufacturer="zigbee-rs", model="GP switch")
        .add_to_registry()
    )
