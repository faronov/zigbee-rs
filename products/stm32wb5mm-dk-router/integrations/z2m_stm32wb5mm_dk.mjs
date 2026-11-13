// Zigbee2MQTT external converter for the zigbee-rs STM32WB5MM-DK router.
//
// Install: copy this file into `<z2m data dir>/external_converters/` and
// restart Zigbee2MQTT (re-run "Reconfigure" if the device was already paired).
//
// Device (mains-powered Zigbee Router, manufacturer "Zigbee-RS", code 0):
//   EP1  0x0104 / 0x0008 Range Extender
//        Basic, Identify,
//        msTemperatureMeasurement (centi-degC, -40..125 C),
//        msOccupancySensing - occupancy is derived from IMU *motion*; it is
//        NOT a PIR sensor. The hold after the last motion is pirOToUDelay
//        (0x0010, writable, kept across resets, default 60 s), published as
//        `occupancy_timeout`. pirUToODelay/threshold are not implemented.
//   EP2  genAnalogInput - VL53L0X ToF distance, presentValue in mm, 0..2000.
//        NOTE: 2000 mm means "no target in range": the device then reports
//        presentValue = 2000 together with reliability (0x0067) = 2
//        (OVER_RANGE). reliability = 1 (NO_SENSOR) means the ToF sensor is
//        missing/failed.
//   EP3  genAnalogInput - inclination from vertical in degrees, 0..180.
//   EP4  genAnalogInput - sound level in dB, 0..130.
//        NOTE: the sound level is an UNCALIBRATED, UNWEIGHTED estimate
//        computed from the on-board PDM microphone. It is useful for
//        relative changes (quiet vs. loud), not as an SPL meter reading.
//
// Reporting: the firmware configures its own default reporting
// (temperature 10..300 s / 0.2 C, occupancy 0..3600 s, distance 5..300 s /
// 20 mm, inclination 5..300 s / 2 deg, sound 10..300 s / 3 dB) and, while no
// binding exists, sends reports to the coordinator. This converter therefore
// does NOT overwrite the device-side reporting (`reporting: false`); it only
// reads the initial values at configure time and allows on-demand reads
// (access STATE_GET).
// If you bind a cluster to some other device/group, the firmware stops
// reporting that cluster to the coordinator. In that case, replace `false`
// with an explicit config, e.g. {min: 5, max: 300, change: 1}. Z2M then also
// binds the cluster to the coordinator.
//
// The genBasic "description" attribute is not implemented by the firmware.

import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

// Endpoint names are used as the property suffix by the multi-endpoint
// numeric() helper, so the published keys are:
//   distance_tof, inclination_imu, sound_level_mic
// (temperature, occupancy and identify stay unsuffixed on EP1).
const EP = {main: 1, tof: 2, imu: 3, mic: 4};

export default {
    zigbeeModel: ['STM32WB5MM-DK-Router'],
    model: 'STM32WB5MM-DK-Router',
    vendor: 'Zigbee-RS',
    description: 'STM32WB5MM-DK Zigbee router with temperature, motion, ToF distance, inclination and sound level',
    extend: [
        m.deviceEndpoints({endpoints: EP}),
        m.identify(),
        m.temperature({
            reporting: false, // device self-configures 10..300 s, 0.2 C
            valueMin: -40,
            valueMax: 125,
        }),
        m.occupancy({
            reporting: false, // device self-configures 0..3600 s
            pirConfig: ['otu_delay'], // motion hold, seconds
        }),
        m.numeric({
            name: 'distance',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            reporting: false, // device self-configures 5..300 s, 20 mm
            description:
                'VL53L0X time-of-flight distance. 2000 mm means no target in range ' +
                '(device sets reliability OVER_RANGE).',
            unit: 'mm',
            valueMin: 0,
            valueMax: 2000,
            precision: 0,
            access: 'STATE_GET',
            endpointNames: ['tof'],
        }),
        m.numeric({
            name: 'inclination',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            reporting: false, // device self-configures 5..300 s, 2 deg
            description: 'Inclination of the board from vertical (IMU accelerometer)',
            unit: '°',
            valueMin: 0,
            valueMax: 180,
            precision: 1,
            access: 'STATE_GET',
            endpointNames: ['imu'],
        }),
        m.numeric({
            name: 'sound_level',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            reporting: false, // device self-configures 10..300 s, 3 dB
            description:
                'Sound level estimate from the PDM microphone (uncalibrated, unweighted; ' +
                'relative indication only)',
            unit: 'dB',
            valueMin: 0,
            valueMax: 130,
            precision: 1,
            access: 'STATE_GET',
            endpointNames: ['mic'],
        }),
    ],
};
