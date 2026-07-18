# Examples by Zigbee role

Start with the host examples to understand stack behavior, then use the
platform example for the target MCU. Hardware examples are thin board/runtime
shells where possible; platform startup belongs in HAL/runtime crates rather
than in application logic.

## Router

| Example | Status |
|---|---|
| `mock-light` | Canonical host FFD/router application |
| `nrf52840-router` | Hardware router example |
| `telink-tlsr8258-sensor` with `runtime-router` | Join/relay-only prototype; no child admission |

## Always-on end device

| Example | Status |
|---|---|
| `mock-sensor` | Canonical host end-device sensor |
| `efr32mg1-sensor` | Platform example with always-on and sleepy build paths |

## Sleepy end device

| Example | Status |
|---|---|
| `mock-sleepy-sensor` | Canonical host SED lifecycle |
| `nrf52840-sensor` | Nordic platform SED |
| `nrf52833-sensor` | Nordic platform SED |
| `esp32c6-sensor`, `esp32h2-sensor` | Espressif platform SEDs |
| `cc2340-sensor` | TI platform SED |
| `phy6222-sensor` | Compile-tested PHY62x2 SED; deep sleep disabled |
| `telink-tlsr8258-sensor` with `runtime-sensor` | Hardware-proven Telink end-device runtime; production SED retention integration remains |

## Coordinator

| Example | Status |
|---|---|
| `mock-coordinator` | Host coordinator behavior |

## TLSR8258 layout

The Telink project intentionally contains three separate surfaces:

- `tlsr8258-rt`: reset vector, IRQ context, and cold-start RAM initialization;
- `telink-tlsr8258-runtime` / `telink-tlsr8258-router`: small production role
  entry points and application modules;
- `telink-tlsr8258-lab`: hardware bring-up and diagnostic firmware.

Diagnostic startup and SRAM instrumentation remain in the lab binary and are
not part of the production sensor/router applications.
