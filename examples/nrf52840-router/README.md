# nRF52840 always-on Zigbee End Device

This `no_std` nRF52840-DK image composes the shared
`router_app::AlwaysOnEndDeviceApp`.

The Nordic MAC backend does not implement `ParentMacDriver`, association
responses, pending transactions, or indirect delivery. The image therefore
builds `DeviceType::EndDevice` with `PowerMode::AlwaysOn`, which advertises
`macRxOnWhenIdle = true` without claiming router or child-parent behavior.

The frontend exposes finite `initialize()` and `step()` calls. This root uses
them directly so it can service the three-second reset button between bounded
always-on End Device steps.

## Product behavior

- Home Automation **Simple Sensor** profile on endpoint 1:
  Basic and Identify servers only.
- Always-on power mode and continuous bounded receive slices; no
  application-side sleep between slices.
- Crash-safe security-state journal in the protected top 8 KiB of flash
  (`0x000FE000..0x00100000`).
- Factory-programmed FICR EUI-64, with persisted identity mismatch guard.
- Nordic ECB hardware AES installed only after both startup known-answer tests.
- DK external high-frequency crystal (`ExternalXtal`) and both applicable
  DC-DC regulators enabled.
- Button 1 held for three seconds performs a journal-aware factory reset and
  only then resets the MCU. A short press has no protocol action.

## Indicators

- **LED1 / P0.13 — semantic status:** solid on when online, alternate state on
  commissioning/rejoin attempts, dark during Identify, solid on for reset or
  fault.
- **LED2 / P0.14 — RX activity:** toggles for each normal MAC data indication.
  It indicates reception, not proof that the frame was forwarded.

## Build and flash

CI and release measurements use the pinned compiler:

```bash
cd examples/nrf52840-router
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 run --release --locked
```

The linker consumes `products/nrf52840-router/link/memory.x`; the product's
Rust constants and linker `ASSERT`s independently protect the two-page
security journal.

The current raw image budget is checked by CI; rebuild after this role change
before treating any previous forwarding-relay measurement as applicable.

## Hardware validation still required

Before release, verify on an nRF52840-DK that commissioning/resume survives
power loss, LED2 follows RX activity, the three-second reset clears membership
without counter rollback, and the always-on End Device remains reachable
without advertising or admitting children.
