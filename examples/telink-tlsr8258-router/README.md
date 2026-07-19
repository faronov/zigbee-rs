# TLSR8258 Zigbee Router

A pure-Rust always-on TLSR8258 router/relay example. It is intentionally a
separate crate from the end-device sensor, matching the Nordic example layout.

## What to read

```text
src/main.rs      reset glue and application selection
src/app.rs       router role, commissioning, receive and maintenance loop
../../boards/tlsr8258-tb04
                 TB-04 LEDs, security partition, and linker layout
```

The firmware joins as an FFD, keeps the radio in continuous receive, relays
NWK traffic, and sends normal router maintenance frames. It does not yet admit
children: association responses, beacons, permit joining, and indirect queues
are not implemented by the Telink MAC backend.

## Build

```bash
./scripts/tlsr8258.sh build router
```

The flashable image is:

```text
examples/telink-tlsr8258-router/target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```
