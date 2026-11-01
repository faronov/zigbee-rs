# nRF52840-DK always-on End Device and Router

This `no_std` nRF52840-DK crate has one bin per role. The role features are
mutually exclusive:

| bin | feature | composition |
|---|---|---|
| `nrf52840-ed` | `ed` (default) | `router_app::AlwaysOnEndDeviceApp` over the end-device `NrfSoftMac`; defmt over RTT |
| `nrf52840-router` | `router` | `router_app::ParentRouterApp` over `NrfRouterMac` = `SoftMacCore<NrfIsrRadioPhy, Parent<8, 8, 0>>`; text log on VCOM |

The End Device builds `DeviceType::EndDevice` with `PowerMode::AlwaysOn`,
which advertises `macRxOnWhenIdle = true` without claiming router or
child-parent behavior. The router sends Imm-Acks from the RADIO ISR, keeps
durable child and APS tables in the product's `ROUTER_TABLES` region
(`0x000FA000..0x000FE000`), and stores the role tag in its security journal.
See the [nRF platform guide](../../docs/book/src/platform-guides/nrf.md#node-roles).

The frontend exposes finite `initialize()` and `step()` calls. This root uses
them directly so it can service Button 1 between bounded always-on End Device
steps.

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
  only then resets the MCU.
- Button 1 short press (50 ms up to three seconds, acted on at release)
  requests commissioning. After a Leave without rejoin (coordinator
  Mgmt_Leave, NWK Leave, Trust Center Remove-Device, or a local Leave) the
  device stays factory-new with LED1 dark and does not search for a network
  until this press or a power cycle; a press also skips a running join
  backoff. While joined or while a reset is pending the press is ignored.
  Gesture classification is host-tested in
  `products/nrf52840-router/src/button.rs`.

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
cargo +nightly-2026-03-23 build --release --locked          # nrf52840-ed
cargo +nightly-2026-03-23 build --release --locked --no-default-features --features router
```

The linker consumes `products/nrf52840-router/link/memory.x`; the product's
Rust constants and linker `ASSERT`s independently protect the two-page
security journal.

Physical memory, protected-partition, stack and role-symbol checks are
mandatory in CI. Both nRF52840 images are build-, layout- and symbol-checked
only. The same router code is hardware-tested on the nRF52833 DK
(`examples/nrf52833-sensor`, bin `nrf52833-router`).

## Hardware validation still required

Before release, verify on an nRF52840-DK that commissioning/resume survives
power loss, LED2 follows RX activity, the three-second reset clears membership
without counter rollback, a short press after a coordinator Leave starts
commissioning again, and the always-on End Device remains reachable
without advertising or admitting children.
