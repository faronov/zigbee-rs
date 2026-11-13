"""ZHA quirk (QuirkBuilder v2) for the Zigbee-RS STM32WB5MM-DK router.

Device: manufacturer "Zigbee-RS", model "STM32WB5MM-DK-Router", mains-powered
Zigbee Router (manufacturer code 0).

ZHA only creates entities for the Analog Input cluster (0x000C) for a few
vendors, so endpoints 2..4 show nothing without this quirk. The quirk adds one
sensor entity per endpoint on AnalogInput.present_value (0x0055, single float)
and renames the IMU occupancy:

    EP1  Basic / Identify / Temperature Measurement / Occupancy Sensing
         -> ZHA's default temperature entity is kept.
         -> Occupancy (0x0000) is IMU *motion* (sensor type 3, physical
            contact: the board was moved), not presence. ZHA's unnamed
            default occupancy entity is replaced by a "Motion" binary sensor
            (device class motion: Detected / Clear).
         -> PIROccupiedToUnoccupiedDelay (0x0010) is the motion hold time,
            writable and kept across resets by the firmware (default 60 s):
            ZHA's "Occupied to unoccupied delay" entity is kept.
         -> The firmware has no unoccupied -> occupied delay (0x0011 reads
            UNSUPPORTED_ATTRIBUTE), so ZHA's "Unoccupied to occupied delay"
            entity is not created.
    EP2  AnalogInput  "Distance"     VL53L0X time-of-flight distance, mm,
                                     0..2000 (EngineeringUnits 30).
    EP3  AnalogInput  "Inclination"  tilt from vertical derived from the IMU
                                     gravity vector, degrees, 0..180 (units 90).
    EP4  AnalogInput  "Sound level"  PDM microphone level, dB, 0..130
                                     (units 199).

Notes:
  * EP2: when no target is in range the device reports present_value = 2000
    together with Reliability (0x0067) = 2 (OVER_RANGE), i.e. "no target"
    (2000 mm is not a real measurement). Reliability = 1 (NO_SENSOR) means the
    VL53L0X is missing. Reliability is not mapped to an entity here.
  * EP4: the sound level is an UNCALIBRATED, unweighted estimate based on the
    microphone's nominal sensitivity, not a calibrated SPL (dBA) reading.
  * AnalogInput Description (0x001C) is not implemented by the firmware, so
    entity names come from translation_key / fallback_name only.
  * zha-quirks 2.4.x drops suggested_display_precision, so HA would show the
    raw single float (19.0200004577637); attribute_converter rounds instead.
  * ZHA would make the temperature entity "primary" and name it after the
    device; the quirk marks it non-primary so it stays "Temperature".
"""

try:
    # Canonical location in newer ZHA (quirks v2 API moved to zha-quirks).
    from zhaquirks.builder import (
        DEGREE,
        BinarySensorDeviceClass,
        EntityType,
        QuirkBuilder,
        ReportingConfig,
        SensorDeviceClass,
        SensorStateClass,
        UnitOfLength,
        UnitOfSoundPressure,
    )
except ImportError:
    # Older zigpy where the quirks v2 API lives in zigpy itself.
    from zigpy.quirks.v2 import EntityType, QuirkBuilder, ReportingConfig
    from zigpy.quirks.v2.homeassistant.binary_sensor import BinarySensorDeviceClass
    from zigpy.quirks.v2.homeassistant import (
        DEGREE,
        UnitOfLength,
        UnitOfSoundPressure,
    )
    from zigpy.quirks.v2.homeassistant.sensor import (
        SensorDeviceClass,
        SensorStateClass,
    )

from zigpy.zcl.clusters.general import AnalogInput
from zigpy.zcl.clusters.measurement import OccupancySensing, TemperatureMeasurement

(
    QuirkBuilder("Zigbee-RS", "STM32WB5MM-DK-Router")
    # ZHA names its weight-picked "primary" entity after the device. With the
    # default occupancy entity gone, temperature would win; this multi-sensor
    # board has no single main feature, so keep every entity named.
    .change_entity_metadata(
        endpoint_id=1,
        cluster_id=TemperatureMeasurement.cluster_id,
        new_primary=False,
    )
    # EP1: ZHA's default occupancy entity has no name (HA shows the device
    # name); "Motion" below replaces it.
    .prevent_default_entity_creation(
        endpoint_id=1,
        cluster_id=OccupancySensing.cluster_id,
        function=lambda entity: type(entity).__name__ == "Occupancy",
    )
    # EP1: 0x0011 is not implemented by the firmware.
    .prevent_default_entity_creation(
        endpoint_id=1,
        cluster_id=OccupancySensing.cluster_id,
        unique_id_suffix="pir_u_to_o_delay",
    )
    # EP1: IMU motion, held for PIROccupiedToUnoccupiedDelay seconds.
    .binary_sensor(
        OccupancySensing.AttributeDefs.occupancy.name,
        OccupancySensing.cluster_id,
        endpoint_id=1,
        entity_type=EntityType.STANDARD,
        device_class=BinarySensorDeviceClass.MOTION,
        reporting_config=ReportingConfig(
            min_interval=0, max_interval=900, reportable_change=1
        ),
        unique_id_suffix="motion",
        translation_key="motion",
        fallback_name="Motion",
    )
    # EP2: VL53L0X ToF distance (mm). 2000 + Reliability OVER_RANGE = no target.
    .sensor(
        AnalogInput.AttributeDefs.present_value.name,
        AnalogInput.cluster_id,
        endpoint_id=2,
        device_class=SensorDeviceClass.DISTANCE,
        state_class=SensorStateClass.MEASUREMENT,
        unit=UnitOfLength.MILLIMETERS,
        suggested_display_precision=0,
        attribute_converter=round,
        unique_id_suffix="distance",
        translation_key="distance",
        fallback_name="Distance",
    )
    # EP3: inclination from vertical (degrees), IMU gravity vector.
    .sensor(
        AnalogInput.AttributeDefs.present_value.name,
        AnalogInput.cluster_id,
        endpoint_id=3,
        state_class=SensorStateClass.MEASUREMENT,
        unit=DEGREE,
        suggested_display_precision=1,
        attribute_converter=lambda value: round(value, 1),
        unique_id_suffix="inclination",
        translation_key="inclination",
        fallback_name="Inclination",
    )
    # EP4: sound level (dB). UNCALIBRATED, unweighted estimate from a PDM
    # microphone using its nominal sensitivity -- treat as relative only.
    .sensor(
        AnalogInput.AttributeDefs.present_value.name,
        AnalogInput.cluster_id,
        endpoint_id=4,
        device_class=SensorDeviceClass.SOUND_PRESSURE,
        state_class=SensorStateClass.MEASUREMENT,
        unit=UnitOfSoundPressure.DECIBEL,
        suggested_display_precision=0,
        attribute_converter=round,
        unique_id_suffix="sound_level",
        translation_key="sound_level",
        fallback_name="Sound level",
    )
    .add_to_registry()
)
