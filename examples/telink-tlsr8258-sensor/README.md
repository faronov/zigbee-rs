# TLSR8258 Zigbee Sensor

A pure-Rust polling Zigbee end device for the Telink TLSR8258/TB-04 board.
This directory contains one firmware role and no hardware bring-up code.

## What to read

```text
src/main.rs      reset glue and application selection
src/app.rs       Zigbee device, clusters, commissioning, polling, reporting
src/board.rs     TB-04 status LEDs
src/flash_nv.rs  crash-safe security journal adapter
memory.x         production flash/SRAM layout
```

The application exposes Basic, Power Configuration, Identify, Temperature,
and Relative Humidity clusters. Synthetic temperature and humidity values
change every 30 seconds so reporting can be tested without an external sensor.

This is a polling Zigbee end device (`rx_on_when_idle = false`), but it does
not yet enter TLSR8258 retention sleep. A separate sleepy-end-device example
will be added only after retention startup and radio resume are hardware-proven.

## Build

Install the `tc32-stage2-tc32-45` toolchain under
`.toolchains/tc32-stage2-tc32-45`, then run from the repository root:

```bash
./scripts/tlsr8258.sh build sensor
```

The flashable image is:

```text
examples/telink-tlsr8258-sensor/target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

Hardware diagnostics and the legacy manual stack live in
`tools/telink-tlsr8258-lab`, not in this example.
