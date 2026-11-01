# Nordic nRF52840 / nRF52833

Nordic sensor products use Embassy's chip HAL, the shared software MAC
(`SoftMacCore<NrfRadioPhy>`, alias `zigbee_mac::nrf::NrfSoftMac`), board
resource crates, product-owned profiles/policy/storage, and
`sensor_sed_app::SensorApp`.

## Layering

```text
environmental profile / always-on End Device behavior
        ↓
nrf52840-sensor / nrf52833-sensor / nrf52840-router product
        ↓
nrf52840-dk / nrf52833-dk / UF2 board crate
        ↓
zigbee_mac::nrf::NrfSoftMac = SoftMacCore<NrfRadioPhy>
        ↓
embassy-nrf RADIO / RNG / ECB
```

The short example root constructs and connects these layers.

`apps/nrf-sensor` contains Nordic capability adapters such as
`NrfWakeController`, `NrfStatus`, `NrfBattery`, diagnostics, and supervisor.
It delegates lifecycle behavior to `apps/sensor-sed`; every Nordic sensor
composition root constructs that lifecycle directly.

## Sensor composition

Nordic sensor roots construct explicit parts:

```rust,ignore
SensorSedParts {
    wake: NrfWakeController::new(button),
    status: NrfStatus::new(led),
    environment,
    battery: NrfBattery::<Battery>::new(saadc),
    ota: NoOta,
    actions: nrf52840_sensor_product::policy::USER_ACTIONS,
    supervisor: NrfSupervisor,
    diagnostics: NrfDiagnostics,
}
```

Both product policies select:

```text
fast wait: Idle
slow wait: Idle
```

The wake adapter quiesces the radio and uses a retained System-ON wait. This is
not System OFF and no battery-life estimate is inferred from the build.
Profiles are explicitly non-OTA and pair with `NoOta`.

The device builder disables automatic polling so `SensorApp` is the only
parent-poll owner.

## Boards

### Development kits

| board | MCU | status | action | sensor I²C |
|---|---|---|---|---|
| PCA10056 | nRF52840 | LED1 P0.13, active low | Button 1 P0.11 | P0.26/P0.27 |
| PCA10100 | nRF52833 | LED1 P0.13, active low | Button 1 P0.11 | P0.26/P0.27 |

The default profile uses the on-chip temperature source plus synthetic
humidity. `sensor-bme280` and `sensor-sht31` select external sensor adapters
and matching product profile composition. The `zigbee-bme280`, `zigbee-sht3x`
and `zigbee-sensor` crates come from
[`zigbee-rs-drivers`](https://github.com/faronov/zigbee-rs-drivers) as git
dependencies pinned by `rev`.

### UF2 boards

`examples/nrf52840-sensor-uf2` supports, one Cargo feature each:

- `board-promicro` (default): nice!nano-compatible ProMicro;
- `board-mdk`: Makerdiary MDK USB Dongle;
- `board-nrf-dongle`: Nordic PCA10059;
- `board-nrf-dk`: Nordic PCA10056 DK.

Each feature selects a board crate and a product linker layout. Preserve the
resident bootloader/SoftDevice regions and always package ELF → Intel HEX →
UF2; do not infer an address from a flat binary.

## Persistence and identity

| product | application end | security journal |
|---|---:|---:|
| nRF52840 DK sensor | `0x000FE000` | `0x000FE000..0x00100000` |
| nRF52840 DK always-on End Device / router | `0x000FA000` | `0x000FE000..0x00100000` |
| nRF52833 DK (all roles) | `0x0007A000` | `0x0007E000..0x00080000` |

The router images keep their child-table and APS journals in a
product-owned 16 KiB `ROUTER_TABLES` region directly below the security
journal (`0x0007A000` on nRF52833, `0x000FA000` on nRF52840). All role images
of a product link the same map, so no role's code can overlap another role's
persistence; the linker asserts these boundaries.

The product constructs `SecurityStateJournal` over Embassy NVMC. FICR-derived
EUI-64 is checked before resume; a changed identity clears incompatible
persisted membership rather than using keys/counters under another address.

Nordic ECB hardware AES is token-owned. Both startup known-answer tests must
pass; production images do not silently fall back to software AES.

## Node roles

Each board has one example crate with one bin per role, following the
EFR32 Series 2 role template (see
[Soft-MAC porting](../advanced/soft-mac-porting.md#role-template-one-example-crate-per-board)).
The role features `sed`, `ed` and `router` are mutually exclusive.

| board | Sleepy End Device | always-on End Device | Router |
|---|---|---|---|
| nRF52833 DK (`examples/nrf52833-sensor`) | `nrf52833-sensor` (default) | `nrf52833-ed` | `nrf52833-router` |
| nRF52840 DK (`examples/nrf52840-router`, `examples/nrf52840-sensor`) | `nrf52840-sensor` crate | `nrf52840-ed` (default) | `nrf52840-router` |

The always-on End Device runs `AlwaysOnEndDeviceApp`; the router runs
`ParentRouterApp` over `NodeArchetype::ParentRouter`, so it routes, sends Link
Status, admits children and keeps their indirect frames.

### MAC and ACK path

The sensor and the nRF52840 `nrf52840-ed` images keep `NrfSoftMac`
(`SoftMacCore<NrfRadioPhy>`), which sends Imm-Acks from task context. That is
fine for a sleepy device, which receives almost nothing unsolicited.

The router and the nRF52833 always-on End Device enable the `zigbee-mac`
feature `nrf-isr-phy` and use `NrfIsrRadioPhy`:

```text
RADIO END ──PPI──▶ TIMER0 CAPTURE[2]          (rx_end)
END→DISABLE short ─▶ DISABLED IRQ              (classify, FP lookup)
ISR: CC[0] = rx_end + 129 µs
TIMER0 COMPARE[0] ──PPI──▶ RADIO TXEN ─READY→START short─▶ Imm-Ack on air
RADIO ADDRESS ──PPI──▶ TIMER0 CAPTURE[1]       (turnaround estimate)
```

The 129 µs delay is Nordic's `nrf_802154` budget for a 192 µs aTurnaroundTime:
40 µs TX ramp-up and 23 µs END-event latency. The ISR copies the frame into a
24-slot ring (128 B per slot) that the task drains through
`RadioPhy::receive`. A frame that arrives while the ring is full is not
acknowledged, so the sender retries it. When the ISR runs after the compare
deadline, it abandons the ACK and counts it in `acks_late`. The PHY reports
`hardware_auto_ack = true`.

Frame Pending follows the core's `PENDING = 0` model: `NrfRouterMac` is
`SoftMacCore<NrfIsrRadioPhy, Parent<8, 8, 0>>`, and the core arms a 16-entry
HAL source-match table (short and extended addresses) through
`RadioPhy::set_ack_frame_pending`; the ISR sets FP in the ACK to a Data
Request from a matching source.

`NrfRadioCounters` counts received frames, CRC errors, ring overflows, ACKs
sent, ACKs with FP, late ACKs, the turnaround estimate and TX results. The
new role bins log on the DK's VCOM UART (UARTE0, P0.06 TX, 115200 baud); the
routers print these counters every 60 s. The SED and `nrf52840-ed` images keep
defmt over RTT. The turnaround figure is an on-chip estimate from
TIMER0 captures, not an on-air measurement.

### Commissioning and recovery

Button 1 is product-mapped: a short press calls `request_commissioning()` (the
only automatic-search trigger, besides a power cycle, after a Leave without
rejoin), and a three-second hold commits the journal-aware factory reset.
After a Leave, the router waits 300 s or a Button 1 press before it searches
again.

The composition roots retry a step error such as a binding `NoAck` with a
bounded backoff (250 ms, up to 40 attempts) before treating it as fatal. A
freshly joined router may not yet have a route back to the coordinator.

### Role journals and switching roles

The security journal stores a role tag (`ROUTER = 0x5201` and
`END_DEVICE = 0x5202` on nRF52833; `ROUTER = 0x5401` on nRF52840). A role image
that finds a different tag clears the persisted membership and joins again.
The SED image predates role tags and does not check them, so leave the network
or factory-reset before you flash the SED over a router or ED.

These compositions use Embassy's pinned-nightly static task allocation. The
compiler emits storage for the actual main future; it no longer tries to
allocate that future from the insufficient default 4 KiB arena. The linker
also requires at least 16 KiB between static storage and the initial stack
pointer. This is a linked reserve, not a measured stack high-water mark.

## Build

All Nordic images use `nightly-2026-03-23`:

```bash
cd examples/nrf52840-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31

cd ../nrf52833-sensor
cargo +nightly-2026-03-23 build --release --locked          # SED
cargo +nightly-2026-03-23 build --release --locked --no-default-features --features ed
cargo +nightly-2026-03-23 build --release --locked --no-default-features --features router

cd ../nrf52840-router
cargo +nightly-2026-03-23 build --release --locked          # always-on End Device
cargo +nightly-2026-03-23 build --release --locked --no-default-features --features router

cd ../nrf52840-sensor-uf2
cargo +nightly-2026-03-23 build --release --locked   # board-promicro
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-mdk
```

Measured release images on 2026-09-06, before the move to `SoftMacCore`:

| image | bytes | former regression budget |
|---|---:|---:|
| nRF52840 default / BME280 / SHT31 | 224,472 / 231,792 / 228,216 | 225,280 / 245,760 / 241,664 |
| nRF52833 default / BME280 / SHT31 | 224,464 / 231,784 / 228,208 | 225,280 / 245,760 / 241,664 |
| nRF52840 always-on End Device | 210,072 | 253,952 |
| UF2 ProMicro / MDK / PCA10059 / DK | 222,968 / 222,848 / 224,536 / 224,552 | 237,568 each |

The `SoftMacCore` port reduced sensor flash by 352–724 B and router flash by
636 B; see the MAC unification section of the
[firmware-size chapter](../advanced/firmware-size.md). The former budgets are
historical comparisons, not enforced thresholds.
CI still measures image bytes and checks physical memory limits, product
partition symbols, hardware-AES symbols, and role-specific symbol removal.

## Validation

Hardware runs on 2026-11-01 used an nRF52833 DK and a Home Assistant ZHA
coordinator (channel 15, PAN 0xDFE9). Session IDs refer to hwlab VCOM
captures.

| image | level | evidence |
|---|---|---|
| `nrf52833-router` | lab-tested | `2026-11-01T103106Z-nrf52833-router-join`, `…103819Z-nrf52833-router-stability`, `…105237Z-nrf52833-router-v4-stability` |
| `nrf52833-ed` | lab-tested | `2026-11-01T110833Z-nrf52833-ed-join`, `…111210Z-nrf52833-ed-identify-after-reset`, `…111329Z-nrf52833-ed-leave` |
| `nrf52833-sensor` (SED) | lab-tested | ZHA side only (defmt RTT was not captured): joined as 0x9254 with MAC capabilities 0x80, interviewed, reported at 11:31:20Z, seen again on 0x9254 at 11:33:46Z after a reset |
| `nrf52840-router`, `nrf52840-ed` | build/layout-tested | no nRF52840 hardware run |

Router results:

- It joined, ZHA completed the interview, and ZHA lists it as a Router.
- It answered Mgmt_Lqi and Mgmt_Rtg with 13–14 neighbours.
- It resumed after reset on the same short address without a rejoin.
- It stayed up for 13 minutes (`v4-stability`) without a reset.
- The TIMER0 turnaround estimate was a constant 172 µs. There were 0 late
  ACKs in 10,541 received frames.
- The 24-slot ring overflowed on 7.9% of frames: none in the first two
  minutes, and 4.2% outside a permit-join burst.
- A third-party End Device associated through the router. The association
  response went out indirectly with FP set, and Update-Device reached the
  Trust Center. The child then failed to authenticate because the forwarded
  Transport-Key got a MAC NoAck. A complete child join through an nRF router
  is still open.

The always-on End Device joined through a ZHA router, and ZHA completed the
interview (`EndDevice`, MAC capabilities 0x8C). It answered Identify both
before and after a reset, resumed without a rejoin, and handled a ZHA remove
with Leave and a journal reset.

The SED joined through a ZHA router and exposed Basic, Power
Configuration, Identify, Temperature and Humidity on endpoint 1. Humidity
stays unknown on the DK because no humidity sensor is fitted. ZHA received
a periodic temperature report at its own 900 s maximum interval. After a
probe reset, ZHA saw the device again on the same short address, and the
device was not interviewed again.

The turnaround figure is an on-chip estimate. On-air ACK timing has not been
confirmed with a sniffer. Low-power current for the SED has not been
measured.
