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

### Radio lab dongle

`tools/nrf52840-radio-lab` is a sniffer and raw-frame transmitter for a
PCA10059 that still has Nordic's Open USB DFU bootloader (not a UF2
bootloader). It is a test instrument, not a product image. See
[Sniffing & Frame Injection](../advanced/radio-lab.md).

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
Request from a matching source. A Data Request whose ACK it built with
FP = 0 keeps that decision in bit 7 (reserved) of its ring slot's PHR byte
(`nrf_ack::committed_phr`), written after the ACK is armed so the check
never delays it; `receive` reports it as
`PhyRxFrame::acked_without_frame_pending` and the NWK keeps a frame queued
since for the child's next poll (host-tested and build-checked; no
hardware run yet).

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

## RF front-end modules (PA/LNA)

> **Status: hardware-unverified.** The register plan, power split and level
> correction are host-tested, and both nRF52840 role images build with a FEM
> in CI. No board with a FEM has run this code. Do not treat it as a
> supported radio configuration until the hardware gates below pass.

The `zigbee-mac` feature `nrf-fem` adds `NrfFem` and the `NrfFemPhy`
decorator. `NrfFem` drives an external front-end module with CTX (PA
enable) and CRX (LNA enable) lines, for example an AT2401C, an RFX2401C or a
SKY66112-11 in two-pin mode. An optional third line (CSD, PDN or
chip-enable) can be added. A PA-only single-pin variant is also available.
`NrfFemPhy` wraps either nRF PHY (`NrfRadioPhy` or `NrfIsrRadioPhy`). Each
PHY changes in one line: the LQI of a received frame goes through
`nrf_lqi`, which is the unchanged `lqi::nrf_from_hardware` without the
feature. The ISR PHY also gets FEM-only hooks (`arm_fem`, `disarm_fem`)
appended at the end of its file. With the feature off, every nRF image is
byte-identical to master. Edits in front of panic locations keep their line
count, so even the `file:line` constants do not move.

### Configuration

Responsibilities follow the usual layering:

| layer | owns | AT2401C example |
|---|---|---|
| board (`nrf52840-dk`, feature `fem-at2401c`) | pins, active levels, PA/LNA gain, settle times | CTX P1.04, CRX P1.02, active high, PA 22 dB, LNA 5 dB, CTX 23 µs, CRX 5 µs |
| product (`nrf52840-router-product`, feature `fem-at2401c`) | regulatory output cap, default output | `MAX_OUTPUT_DBM = 18`, `DEFAULT_OUTPUT_DBM = 18` |
| composition (`examples/nrf52840-router/src/fem.rs`) | GPIOTE/PPI channel reservation, wrapping | GPIOTE 6/7, PPI 16–19, both role bins |

The example mirrors devbis/nrf-ncp's `nrf52840dk_nrf52840.overlay`
(`skyworks,sky66112-11` with `rx-gain-db = <5>`). The values that overlay
leaves unset are the SKY66112-11 binding defaults. Build either role with
`--features fem-at2401c`:

```rust
let fem = NrfFem::two_pin(
    FemParams {
        tx_gain_db: 22,
        rx_gain_db: 5,
        pa_settle_us: 23,
        lna_settle_us: 5,
        max_output_dbm: 18,
        default_output_dbm: 18,
    },
    FemLine::new(p.P1_04, true),   // CTX, active high
    FemLine::new(p.P1_02, true),   // CRX, active high
    None,                          // optional PDN/CSD line
    p.GPIOTE_CH6,
    p.GPIOTE_CH7,
    [p.PPI_CH16.degrade(), p.PPI_CH17.degrade(), p.PPI_CH18.degrade(), p.PPI_CH19.degrade()],
)?;
// `phy`: NrfRadioPhy or NrfIsrRadioPhy with its AES engine installed.
let mac = NrfFemPhy::new(phy, fem).into_parent_mac()?; // or into_soft_mac()
```

The MAC types are `NrfFemRouterMac`, `NrfFemIsrSoftMac` and
`NrfFemSoftMac`. `NrfFemPhy::inner()` reaches the wrapped PHY, and
`counters()` is forwarded for the ISR PHY.

The GPIOTE and PPI channels are Embassy singletons, so no other driver can
claim them. Only configurable PPI channels 0–19 are accepted. The ISR PHY's
pre-programmed channels 20, 26 and 27 stay reserved, and only the FORK of
channel 20 is used. The constructor rejects:

- a shared pin or channel, or an invalid channel;
- a gain above 40 dB;
- a cap below the lowest achievable output (radio −40 dBm plus the PA gain);
- a default output outside the achievable range;
- a PA settle time above 40 µs;
- an LNA settle time above 16 µs;
- a second FEM on the same radio.

### Pin timing

The RADIO's own events switch the PA and LNA through PPI and GPIOTE task
channels. The CPU never times them, so interrupt latency does not move
them.

```text
RADIO CCAIDLE  ─PPI a─▶ CTX on  + FORK CRX off   CCA clear; the SHORT starts TXEN
TIMER0 CC[0]   ─PPI 20─▶ TXEN   + FORK CTX on    ISR PHY Imm-Ack, at the TXEN instant
RADIO PHYEND   ─PPI b─▶ CTX off                  last TX bit on air
RADIO RXREADY  ─PPI c─▶ CRX on  + FORK CTX off   receiver ramped up
RADIO DISABLED ─PPI d─▶ CRX off                  any path to DISABLED

CCA-gated data TX:
  RXEN ─▶ RXRU ─RXREADY─▶ CCA (128 µs) ─CCAIDLE─▶ TXRU (40/130 µs) ─▶ frame ─PHYEND─▶ DISABLED
  CRX  ________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______________________________________________
  CTX  __________________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________

Imm-Ack after RX (ISR PHY):
  frame ─END→DISABLE─▶ DISABLED … CC[0]: TXEN ─TXRU 40 µs─▶ ACK ─PHYEND─▶ DISABLED ─▶ RXEN
  CRX  ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________________________________________________________/‾
  CTX  ______________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_____________________
```

- **PA lead.** CTX rises when TX ramp-up starts, either at CCAIDLE or at the
  TXEN compare. That gives 40 µs before the first preamble symbol with the
  ISR PHY's fast ramp-up, and 130 µs with Embassy's default ramp-up. Both
  exceed the SKY66112-11 `ctx-settle-time-us` default of 23 µs.
- **PA off.** CTX falls at PHYEND or RXREADY. `NrfFemPhy` also drops both
  lines in software when a transmission returns an error, when its future
  is cancelled, and on sleep. CTX does not fall at DISABLED: the RX→TX
  switch of a CCA transmission passes through DISABLED after CCAIDLE, and
  the ISR PHY already handles that intermediate DISABLED event. If the ISR
  PHY recovers a stalled Imm-Ack (a hardware anomaly), CTX stays on until
  the next RXREADY or sleep.
- **LNA lead.** CRX rises at RXREADY, which is also when the receiver starts
  listening (`RXREADY→START`, `RXREADY→CCASTART`), so the lead is zero.
  During the CRX settle time (5 µs default, 16 µs accepted), a frame whose
  preamble started just before can lose up to one of its eight preamble
  symbols. With the ISR PHY, an ACK expected after a transmission arrives
  more than 100 µs after RXREADY, so it is not affected.
- **Turnarounds.** Every RX↔TX path drops the other line in hardware: RX→TX
  through the CCAIDLE fork, and TX→RX through the RXREADY fork. Both
  directions also pass through DISABLED or PHYEND. CTX and CRX are never
  driven active together.
- **Sleep.** `NrfFemPhy::sleep` (and `enter_low_power_idle` on the
  Embassy PHY) lets the wrapped PHY disable the RADIO, then drives both
  lines inactive and releases PDN. PDN is asserted again by `wake()` or by
  the next transmit or receive. The RX ramp-up of at least 40 µs that
  follows covers the PDN settle time.

### Antenna-referred values

- **TX power.** `phyTransmitPower` is the requested antenna output. The PHY
  caps it at `max_output_dbm` and programs the highest radio step that does
  not exceed `requested − tx_gain_db`, as Nordic MPSL's power split does. It
  then reports `{requested, radio, antenna, capped}` through `NrfFem::tx_power()`
  and in the log. `PhyCapabilities` advertises the antenna-referred range.
  The MAC starts at `default_output_dbm`, and MLME-RESET restores it
  (`RadioPhy::default_tx_power_dbm`, which defaults to the
  `DEFAULT_TX_POWER_DBM` constant for every other PHY).

  | requested | radio | antenna (AT2401C example) |
  |---:|---:|---:|
  | 18 (default) | −4 | 18 |
  | 20 | −4 (capped at 18) | 18 |
  | 0 | −30 | −8 |
  | −30 | −40 | −18 (lowest achievable) |

- **LQI.** The Nordic ED/LQI byte (1 dB per unit,
  `P = ED_RSSIOFFS + value`) has the LNA gain removed before the
  `×4` IEEE mapping, in both PHYs.
- **CCA.** The ISR PHY's energy-detect CCA threshold (0x14) is raised by the
  LNA gain, so the busy level at the antenna does not change. The Embassy
  PHY uses carrier-sense CCA, which has no threshold.
- **RSSI and ED.** Neither nRF PHY measures per-frame RSSI (`rx-rssi`
  reports −128) or ED scans (`Unsupported`), so today there is nothing else
  to correct. `NrfFemPhy::energy_detect` already subtracts `4 × rx_gain_db`
  from a future ED result on the IEEE scale (a saturated 255 stays 255).
  `zigbee_mac::fem::rssi_at_antenna` is the RSSI correction to use when
  per-frame RSSI is added.

### Caveats

- **Regulatory.** `max_output_dbm` is a product decision. The example's
  18 dBm conducted output assumes an antenna gain of 2 dBi or less under the
  ETSI EN 300 328 limit of 20 dBm EIRP. Re-derive it for the real antenna,
  cable loss and target market, and measure the conducted power. The PA
  gain is a datasheet figure, not a calibration.
- **Not driven.** SKY66112-11 CPS, CHL and ANT_SEL are not driven; strap
  them in hardware. nRF21540 needs SPI/GPIO gain control and PDN timing, so
  it is not supported.
- **Unchanged.** The ISR PHY's CCA TX does not use the
  `CCAIDLE→STOP` short that the product specification recommends together
  with `CCAIDLE→TXEN`. This is existing behaviour, and the FEM plan does not
  rely on it.

### Size

Measured on 2026-11-19 with release builds from the same worktree path
(`nightly-2026-03-23`). The flash figure is the `.bin` size. The RAM figure
is `.bss`, which holds the larger MAC futures and the `NrfFem` state.

| image | flash without FEM | with `fem-at2401c` | RAM (`.bss`) |
|---|---:|---:|---:|
| `nrf52840-router` (ISR PHY) | 394,108 | 399,284 (+5,176) | +188 |
| `nrf52840-ed` (Embassy PHY) | 228,440 | 233,272 (+4,832) | +348 |

Without the feature, the nine nRF images below are byte-identical to
master `370ef47` when built from the same path:

- `nrf52840-ed`, `nrf52840-router` and `router-core`;
- `nrf52840-sensor` default and BME280;
- `nrf52833-sensor` SED, ED and router;
- the ProMicro UF2 image.

Builds from two different paths, even of equal length, are not comparable:
the path-dependent crate hashes change symbol order.

### Hardware gates still open

1. Capture CTX and CRX against a RADIO event GPIO with a logic analyser for
   a CCA TX, an Imm-Ack after RX, an ACK RX after TX, a busy CCA, a
   cancelled TX and sleep. Check the ≥ 23 µs PA lead and that CTX and CRX
   are never both active.
2. Measure conducted output power and the adjacent-channel spectrum at the
   default and capped settings.
3. Measure RSSI/LQI and receive sensitivity against a reference, with and
   without the FEM.
4. Measure SED sleep current with the lines idle and PDN released.
5. Run an Imm-Ack turnaround sniffer capture and a full commissioning,
   interview and reporting cycle with the FEM fitted.

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
# AT2401C front-end add-on (hardware-unverified), either role:
cargo +nightly-2026-03-23 build --release --locked --features fem-at2401c
cargo +nightly-2026-03-23 build --release --locked --no-default-features --features router,fem-at2401c

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
