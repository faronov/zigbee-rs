# TLSR8258 always-on End Device

Pure-Rust, mains-powered, RX-on-when-idle TB-04 End Device using
`router_app::AlwaysOnEndDeviceApp`. It joins as a non-routing End Device and
accepts no children.

## Architecture

```text
always-on End Device product behavior (identity, policy, role tag)
        ↓
products/tlsr8258-tb04 (feature `end-device`)
        ↓
boards/tlsr8258-tb04
        ↓
tlsr8258-hal + TelinkMac (SoftMacCore<TelinkRadioPhy, EndDevice>) + tlsr8258-rt
```

This example is the composition root. `src/main.rs` holds the startup and IRQ
entry. `src/app.rs` holds the TC32 static storage, the observer and the
`TELINK_ED_METRICS` diagnostics block. Steering/resume, secured rejoin,
bounded receive/tick processing and security checkpoints come from
`router-app`; this example contains no local state machine.

The receiver stays on between operations. The HAL services RX and generates
the Imm-Ack from its IRQ path, so `irq_handler` forwards to
`tlsr8258_hal::radio::handle_irq()`, as in the router. The image links no
parent MAC, router lifecycle, sleep or LOW32K retention code.

## Identity and persistence

| item | value |
|---|---|
| model | `TLSR8258-ED` |
| IEEE | factory EUI-64, byte 0 + `0x45` (sensor `0x33`, router `0x52`) |
| profile | endpoint 1, Basic + Identify (Simple Sensor device ID) |
| security journal | `0x74000..0x76000`, role tag `END_DEVICE` (`0x5202`) |
| APS / child journals | reserved by the product layout, unused by this role |

A role switch to or from the sensor or router image keeps the frame-counter
limits, clears the commissioned network and joins again.

The TB-04 has no button. After a Leave the device stays factory-new until the
next power cycle.

## Build

```bash
./scripts/tlsr8258.sh build ed
```

Flashable image:

```text
examples/telink-tlsr8258-ed/target/tc32-unknown-none-elf/release/telink-tlsr8258-ed.bin
```

The image is built at `opt-level=s` because the TC32 backend cannot build at
`opt-level=z`. `tools/tlsr8258-firmware.sh` fails the build if the image:

- links any parent or router symbol;
- lacks the End Device Timeout client symbols;
- links a sleep or retention path;
- has anything other than exactly one `tlsr8258_rt::block_on` root;
- moves off the product-wide 20 KiB SVC (64 B guard below it) / 1 KiB IRQ
  stack layout.

## Validation

The image is build-, layout- and role-symbol-checked only. Join, interview
and resume after reset have not been run on hardware. Reading
`TELINK_ED_METRICS` over SWire halts the target, so treat each read as a
reset/resume event.
