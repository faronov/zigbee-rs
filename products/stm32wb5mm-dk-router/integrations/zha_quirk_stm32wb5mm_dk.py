"""ZHA quirk (QuirkBuilder v2) for the Zigbee-RS STM32WB5MM-DK router.

Device: manufacturer "Zigbee-RS", model "STM32WB5MM-DK-Router", mains-powered
Zigbee Router (manufacturer code 0).

ZHA only creates entities for the Analog Input cluster (0x000C) for a few
vendors, so endpoints 2..4 show nothing without this quirk. The quirk adds one
sensor entity per endpoint on AnalogInput.present_value (0x0055, single float):

    EP1  Basic / Identify / Temperature Measurement / Occupancy Sensing
         -> left untouched; ZHA's default temperature and occupancy entities
            are kept.
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
"""

try:
    # Canonical location in newer ZHA (quirks v2 API moved to zha-quirks).
    from zhaquirks.builder import (
        DEGREE,
        QuirkBuilder,
        SensorDeviceClass,
        SensorStateClass,
        UnitOfLength,
        UnitOfSoundPressure,
    )
except ImportError:
    # Older zigpy where the quirks v2 API lives in zigpy itself.
    from zigpy.quirks.v2 import QuirkBuilder
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

(
    QuirkBuilder("Zigbee-RS", "STM32WB5MM-DK-Router")
    # EP2: VL53L0X ToF distance (mm). 2000 + Reliability OVER_RANGE = no target.
    .sensor(
        AnalogInput.AttributeDefs.present_value.name,
        AnalogInput.cluster_id,
        endpoint_id=2,
        device_class=SensorDeviceClass.DISTANCE,
        state_class=SensorStateClass.MEASUREMENT,
        unit=UnitOfLength.MILLIMETERS,
        suggested_display_precision=0,
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
        unique_id_suffix="sound_level",
        translation_key="sound_level",
        fallback_name="Sound level",
    )
    .add_to_registry()
)
