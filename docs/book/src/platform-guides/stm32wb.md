# STM32WB (STM32WB5MM-DK)

The STM32WB5MM-DK runs zigbee-rs as a **Zigbee router**. CPU1 (Cortex-M4)
runs the whole pure-Rust Zigbee stack: NWK, APS, ZDO, BDB, ZCL and the
runtime. CPU2 (Cortex-M0+) owns the radio and runs ST's IEEE 802.15.4 MAC
coprocessor firmware. No ST Zigbee stack is involved. CPU1 issues
MLME/MCPS primitives over the IPCC mailbox.

## Layering

```text
apps/router (Basic + Identify, range extender, endpoint 1)
        ↓
products/stm32wb5mm-dk-router   identity, NV journals, linker layout,
                                button/touch actions, LED palette,
                                LCD pages, sensor → ZCL mapping
        ↓
boards/stm32wb5mm-dk            pins and fitted parts: B1/B2, USART1 VCP,
                                RGB LED, OLED, TSC key, I2C3 sensors, PDM mic
        ↓
stm32wb-hal                     clocks, HSEM-guarded flash, identity,
                                IPCC MAC transport (embassy-stm32-wpan)
        ↓
zigbee-mac::stm32wb::Stm32wbMac (MLME/MCPS mapping, ParentMacDriver)
```

`examples/stm32wb5mm-dk-router` is the composition root. It holds the
platform startup, the VCP logger, the event loop and one small task per
peripheral group (`ui`, `sensing`, `sound`). Generic part drivers come
from [`zigbee-rs-drivers`](https://github.com/faronov/zigbee-rs-drivers),
pinned by revision.

## On-board peripherals

Every peripheral is a Cargo feature of the example. The default build
enables all of them; `--no-default-features` is the plain router.

| Feature | Part (UM2825) | Use |
|---|---|---|
| `rgb-led` | TLC59731 RGB LED | lifecycle colour |
| `lcd` | SSD1315 128×64 OLED, SPI1 | three text pages |
| `touch` (implies `lcd`) | TSC key (group 6, shield group 4) | page navigation |
| `sensors` | STTS22H, ISM330DHCX, VL53L0X on I2C3 | ZCL endpoints 1–3 |
| `mic` | MP23DB01HP PDM microphone, SAI1 + DMA | ZCL endpoint 4 |

The IR LED and the QSPI flash are fitted but not used.

### Inputs

| Input | Gesture | Action |
|---|---|---|
| B1 | short press | factory-new: start commissioning; joined: open this router for joining for 180 s (local NWK permit only, no Mgmt_Permit_Joining broadcast) |
| B1 | hold 3 s | journal-aware factory reset |
| B2 | short press | identify endpoint 1 for 10 s |
| B2 | hold 3 s | close this router for joining |
| touch key | tap | next LCD page |
| touch key | hold | first LCD page |

### LED colours

| State | Colour |
|---|---|
| booting | dim white |
| searching | orange blink |
| joined | green, off after 30 s |
| permit-join open | slow blue blink |
| identify | fast magenta blink |
| error | red |

### LCD pages

- **Network:** role and NWK address, PAN ID and channel, extended PAN ID,
  parent and its LQI, neighbour/child/route counts, permit-join countdown.
- **Traffic:** MAC TX ok/no-ACK/CCA/other failures, indirect frames
  delivered/handed to CPU2, RX drops, NWK frame counter, uptime.
- **Sensors:** temperature, distance, inclination and motion, sound level.

### ZCL mapping

| EP | Device type | Cluster | Source |
|---|---|---|---|
| 1 | Range Extender 0x0008 | Temperature Measurement 0x0402 | STTS22H |
| 1 | | Occupancy Sensing 0x0406 | ISM330DHCX motion, 60 s hold (not a PIR) |
| 2 | Simple Sensor 0x000C | Analog Input 0x000C, mm (BACnet units 30), 0–2000 | VL53L0X; no target = 2000 with Reliability OVER_RANGE |
| 3 | Simple Sensor 0x000C | Analog Input, degrees from vertical (units 90), 0–180 | ISM330DHCX gravity |
| 4 | Simple Sensor 0x000C | Analog Input, dB (units 199), 0–130 | MP23DB01HP; uncalibrated, unweighted, nominal −41 dBFS sensitivity |

A missing or failing sensor reports Reliability NO_SENSOR with the FAULT
status flag; temperature reports the invalid value 0x8000.

ZHA creates the temperature and occupancy entities on its own. It creates
Analog Input entities only for some manufacturers, so endpoints 2–4 need
the quirk in
`products/stm32wb5mm-dk-router/integrations/zha_quirk_stm32wb5mm_dk.py`
(through `custom_quirks_path`).
`integrations/z2m_stm32wb5mm_dk.mjs` is the matching Zigbee2MQTT external
converter. Both were checked against current zigpy/ZHA and
zigbee-herdsman-converters with simulated devices only.

## Coprocessor prerequisites

CPU2 must run the following firmware:

| Component | Version | Source |
|---|---|---|
| FUS | 1.2.0 | STM32CubeWB `Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x/stm32wb5x_FUS_fw.bin` |
| Wireless stack | 802.15.4 MAC 1.24.0 | `stm32wb5x_Mac_802_15_4_fw.bin` |

ST binaries are never committed to this repository. Install them with
HWLab's STM32WB FUS operations:

1. read the versions;
2. upgrade the FUS;
3. delete the stack;
4. install the stack;
5. start the stack.

For a 1 MiB part, use the install addresses from ST's release notes. With
only the FUS, or with a BLE or Thread stack on CPU2, MAC start-up fails
explicitly. `stm32wb_hal::mac::start` returns `WirelessStackNotRunning`
when the mailbox never becomes ready, and `MacInit` when MAC
initialisation times out.

After installation the secure-flash start is **SFSA = 0xE1**, so CPU2 owns
everything from `0x080E_1000` upwards. A different stack or FUS version
moves SFSA. The product asserts the boundary it was built for:
`layout::EXPECTED_SECURE_START`, plus the assertions in `link/memory.x`.

## Memory layout (1 MiB STM32WB55VG)

| Region | Range | Owner |
|---|---|---|
| Application | `0x0800_0000..0x080D_5000` (852 KiB) | product linker script |
| Child-table journal | `0x080D_5000..0x080D_9000` | product |
| APS-table journal | `0x080D_9000..0x080D_D000` | product |
| Security journal (frame counters) | `0x080D_D000..0x080E_1000` | product |
| CPU2 secure area | `0x080E_1000..` | FUS / MAC stack (untouchable) |
| SRAM1 (CPU1) | `0x2000_0000..0x2003_0000` | application |
| SRAM2a/2b shared | `0x2003_0000..` | IPCC mailbox (`TL_REF_TABLE` at `0x2003_0000`) |

Each journal uses two 4 KiB logical sectors. STM32WB flash programs 64-bit
double-words exactly once between erases, so the HAL's
word-per-double-word adapter (`stm32wb-hal::word_flash`) stores each
logical sector on two physical pages. The adapter preserves the journal's
crash-safe reservation semantics. Every erase and program step follows AN5289 and RM0434 §3.3.12
(`stm32wb-hal::flash`, `stm32wb-hal::hsem`):

1. take HSEM 2 for the flash controller;
2. take HSEM 7, which CPU2 holds during timing-critical radio windows,
   around the single step;
3. let embassy-stm32 wait out `PESD`, `CFGBSY` and `BSY`.

Writes at or above SFSA are refused twice: by embassy-stm32, and by the
HAL's `FlashWindow` bound.

## MAC capability findings

| Router need | ST MAC 1.24.0 | Evidence |
|---|---|---|
| Router start (`PANCoordinator = FALSE`, BO = SO = 15) | supported | on air: ZHA lists the node as Router; Link Status every ~15 s |
| Address filtering (short + extended address, PAN ID) | PIB | on air: joined as `0xC1E6` with the factory EUI-64 |
| Data, ACK, CSMA, retries | supported | on air: ACK ~2 ms after TX; 1 + 3 retries on no-ACK |
| Relay of broadcasts, route discovery (RREQ/RREP) | NWK-layer | on air |
| Beacon on Beacon Request | autonomous (CPU2); payload from `macBeaconPayload` | **not yet observed on air** |
| Association indication/response | MLME-ASSOCIATE.indication / .response | **not yet observed on air** |
| Indirect queue, Frame Pending, data poll | `TxOptions = indirect`; MLME-POLL.indication | **not yet observed on air** |
| TX power | `phyTransmitPower` (−20..+6 dBm) | programmed; level not measured |
| MAC-layer security | not used (Zigbee secures at NWK/APS) | — |
| PAN-coordinator start | not offered | — |
| CCA threshold | not exposed (only `phyCCAMode`) | — |

The module documentation of `zigbee-mac::stm32wb` lists the semantic
deviations. Three of them matter most:

- Beacons are autonomous, so permit-join changes reach the beacon payload
  within `BEACON_REFRESH_US`.
- `mcps_indirect_data` completes when the frame is handed to CPU2, not when
  it is delivered over the air.
- LQI is not reported for MAC commands.

## Coprocessor ABI: vendored embassy-stm32-wpan

`embassy-stm32-wpan` at Embassy rev `7419ec0` mis-describes two MAC
structures in its ABI. The first hardware run therefore wedged the IPCC
mailbox: the 185-byte scan confirm did not match embassy's 173-byte struct,
the parse failed, and the event was never acknowledged. This repository
vendors the crate in `vendor/embassy-stm32-wpan` and records every change in
`LOCAL-PATCHES.md`:

- `PanDescriptor` and `MacAddress` are `repr(C)` with ST's padding.
- `StartRequest` gains `coord_realign_key_id_mode`: 35 bytes, as in ST's
  header, instead of 34.
- A MAC event that fails to parse is still acknowledged, so the mailbox
  cannot wedge.

`stm32wb-hal/src/st_abi.rs` pins about 210 sizes and field offsets. The
values are generated with `arm-none-eabi-gcc` from ST's
`mac_802_15_4/core/inc` headers (stm32-mw-wpan `1a66948e`) and checked as
compile-time assertions. Any upstream change that breaks the ABI therefore
fails the build.

## Build and flash

```sh
cd examples/stm32wb5mm-dk-router
cargo build --release
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/stm32wb5mm-dk-router wb-router.bin
```

Flash through HWLab's STM32WB flash driver over the on-board ST-LINK (CN11).
The driver backs up `0x0800_0000..SFSA`, programs, and verifies by
read-back. The VCP is USART1 (PB6/PB7) at 115200 baud.

Product identity is `Zigbee-RS` / `STM32WB5MM-DK-Router`. The IEEE address
is the factory EUI-64 from the OTP/UID.

## Validation status

The stack runs on one DK, joined to a ZHA network on channel 15.
Hardware runs observed:

- active scan;
- association;
- NWK key delivery;
- TCLK exchange (Confirm-Key SUCCESS);
- router start;
- the ZHA interview, with the node reported as a mains-powered Router
  (LQI 184);
- Link Status;
- broadcast relay;
- RREP;
- reset/resume from the NV journals without a rescan.

Host tests cover:

- the MLME/MCPS mapping in `zigbee-mac` (scripted transport);
- the HAL flash adapter and identity;
- the product layout and storage.

CI builds and clippy-checks the HAL, the board, the peripheral lab, and
the example in its default, plain and single-peripheral configurations.
The CI job also checks that the image contains the router symbols
(`tools/check-role-symbols.sh parent`).

Peripheral image (all features), on the same DK:

- each part passed its own diagnostic in `tools/stm32wb5mm-dk-lab` first;
- all three I2C sensors probed;
- after a ZHA remove (Mgmt_Leave), the node cleared its credentials and
  security journal, rejoined as a new address and was interviewed with
  all four endpoints;
- ZHA shows the temperature (20.0 °C) and occupancy entities, and the
  node sends reports for 0x0402, 0x0406 and the three Analog Inputs.

Not yet confirmed by an observer: the LED colours, the LCD pages, touch
navigation, and B1/B2 on the peripheral image. A VL53L0X in-range
reading has not been confirmed either. The sound level is uncalibrated.

Open hardware gates:

- beacon response to a third-party Beacon Request;
- association of a child;
- indirect delivery to a sleepy child.

None of these has been observed on air yet.

A 30-minute run with an nRF52840 sniffer on channel 15 ended with these
counters:

- 442 MAC transmissions confirmed, 0 no-ACK, 8 channel-access failures;
- 18 neighbours;
- no IPCC fault and no reset.

The air capture shows 265 frames sent by the router.

During the run, one scan by another device drew beacons from six
routers (at about 603, 609 and 615 s). This node sent none, because it
fell in the transmit stall described below. The gate therefore stays
open; it is not a confirmed failure.

Also open:

- the cause of self-recovering transmit stalls. Two have been observed:
  about 135 s, and about 45 s in the 30-minute run. During a stall:
  - CPU2 reports `CHANNEL_ACCESS_FAILURE` for every transmission;
  - the sniffer sees nothing from the node;
  - reception continues normally;
  - an energy-detect scan taken during the stall read 14/255, so the
    channel looked quiet, which makes external interference unlikely.

  No mitigation has been added; the CSMA streak is counted in
  `Stm32wbMacStats`;
- OTA;
- runs longer than 30 minutes.
