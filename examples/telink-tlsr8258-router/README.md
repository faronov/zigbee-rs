# TLSR8258 parent router

Pure-Rust always-on TB-04 router using
`router_app::ParentRouterApp + PersistentChildren`.

## Architecture

```text
tlsr8258-hal + TelinkMac + tlsr8258-rt
        ↓
boards/tlsr8258-tb04
        ↓
products/tlsr8258-tb04
        ↓
this root
        ↓
router_app::ParentRouterApp
```

The shared app owns steering/resume, bounded receive/tick processing,
security checkpoints, child-table restore/save/clear, rejoin, and factory
reset ordering. `src/app.rs` supplies TC32 static storage, product adapters,
and retained diagnostics; it is not a duplicate router state machine.

`TelinkMac` implements `ParentMacDriver`, so this product can construct a true
parent router. This differs from the nRF52840 forwarding relay.

## Persistence

```text
0x00000..0x72000  application
0x72000..0x74000  child-table journal
0x74000..0x76000  security journal
0x76000..0x77000  factory EUI-64
0x77000..0x78000  factory config/calibration
```

The child journal is restored only after network resume and is bound to the
extended PAN ID. Factory reset clears child state and security state before
fresh commissioning.

## Build

```bash
./scripts/tlsr8258.sh build router
```

Flashable image:

```text
examples/telink-tlsr8258-router/target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```

Current size: **343,660 B**.

## Hardware evidence

Hardware-proven:

- association, Transport-Key, Device Announce, TCLK exchange, and interview;
- hardware AES and monotonic security counters;
- silent durable restart on the same short address;
- Link Status and NWK relay;
- first-association Verify-Key/Confirm-Key behavior against ZiGate.

Still open:

- corrected-image first-attempt sleepy-child admission and full interview;
- ACK turnaround and Frame Pending on child polls;
- deferred Association Response and indirect delivery;
- long-duration parent/router stability.

Retained `TELINK_JOIN_METRICS` and queue-overload counters support HIL, but
reading them over SWire halts/restarts the tested target and must be treated as
another reset/resume event.
