# Silicon Labs EFR32

The repository supports these products:

- EFR32MG1P TRÅDFRI environmental sleepy end device (Series 1);
- EFR32MG22 BRD4183A development sensor (Series 2, hardware-proven);
- EFR32MG21 BRD4181A development sensor (Series 2, builds; no hardware run);
- Home Assistant Connect ZBT-1 always-on End Device (EFR32MG21, lab-tested);
- EFR32MG24 ZRHP NCP and Router for an RF-BM-MG24B1 dev board, the SONOFF
  Dongle-M, a BRD4187 radio board on a BRD4001A WSTK and the xG24 Explorer
  Kit EK2703A (Series 2). The xG24 radio at 39 MHz is lab-tested with the
  20 dBm (HP) PA on the BRD4187A and the 10 dBm (MP) PA on the EK2703A,
  where the Router joined a test network; the NCP ran on the BRD4187A
  against `hubctl`. The Dongle-M (38.4 MHz) uses the PHY of its stock
  EmberZNet image and is packaged for its stock Gecko bootloader; the NCP
  has run on one unit with a restored EmberZNet network and received at
  38.4 MHz, while its transmission to devices is not yet proven (first
  HIL, partial). The RF-BM-MG24B1 has no hardware run.

All use the shared `SensorApp`; their boards, products, clocks, storage, and
power behavior are intentionally separate. Series 2 chips share one HAL
(`efr32s2-hal`, chip chosen by the `efr32mg21` / `efr32mg22` /
`efr32mg24-pa10` / `efr32mg24-pa20` feature) and
one MAC backend (`zigbee_mac::efr32s2::Efr32s2RadioPhy` on `SoftMacCore`).

## EFR32MG1P TRÅDFRI

### Layering and fitted hardware

```text
environmental OTA profile
        ↓
products/efr32mg1-tradfri
        ↓
boards/efr32mg1-tradfri
        ↓
efr32mg1-hal + Efr32Mac
```

The typed board resources keep fitted peripherals independently testable:

| resource | wiring |
|---|---|
| direct or TIMER0 PWM LED | PA0, active high |
| user button | PB13, active low |
| SHT3x I²C | I2C0, PC10 SDA / PC11 SCL, 10 kHz |
| supply measurement | ADC0 AVDD |
| direct external flash | USART0: PD13 CLK, PD14 MISO, PD15 MOSI, PB11 CS |
| wake timer | RTCC/LFRCO |

Direct USART0 access and Gecko Bootloader storage access consume alternative
owners of the same external-flash path. They cannot coexist through the typed
board API.

The product owns identity, profile, battery chemistry, sensor mapping, policy,
linker regions, persistence, and Gecko Bootloader OTA selection.
It selects the orthogonal `compact-single-endpoint` capacity only after
profile assertions prove that two application-endpoint slots and four
reporting entries are sufficient; no Zigbee behavior is removed.

### Power and lifecycle

The product policy selects:

```text
fast wait: Active
slow wait: Retention
```

Fast commissioning/interview/OTA windows remain active. Joined steady-state
polls use RTCC/LFRCO wake and EM2 after the platform has quiesced the radio and
applied the Series-1 DCDC safety gate. PB13 remains an external wake source.

The shared app routes OTA first, checkpoints security, and only then calls the
product activation backend.

### Memory

The product linker layout preserves bootloader, application, generic NV, and
security regions. The usable SRAM region is exactly `0x7C00` bytes. It is not
the nominal rounded 32 KiB total.

Recorded measurement (2026-10-31, local build of `7d45e7e`):

| value | bytes |
|---|---:|
| raw image | 171,792 |
| former regression budget (not enforced) | 167,936 |
| `.data` | 288 |
| `.bss` | 15,056 |
| static total | 15,344 |
| available linked stack | 16,400 |

After the move to the shared `SoftMacCore` (`Efr32Mac` =
`SoftMacCore<Efr32RadioPhy, EndDevice, 1>`, build- and layout-checked only),
the raw image is 174,084 B, `.bss` is 14,736 B and the linked stack is
16,720 B (336 B above the 16 KiB gate). See
[Firmware size](../advanced/firmware-size.md#phase-b-efr32mg1-approved-flash-exception).

The former image-size budget is no longer enforced; physical Flash/RAM,
protected-partition, OTA, and stack checks remain mandatory.
The stack has only 16 B above the 16 KiB gate. The previous 162,538 B Zigbee OTA
container has not been regenerated for this image. OTA packaging assumes
the resident Gecko bootloader.

### Validation

The exact raw image above is build/layout-tested, not OTA-qualified. Earlier
TRÅDFRI images produced hardware evidence for:

- commissioning and security;
- CRYPTO hardware AES;
- ZHA interview/reporting;
- SHT3x and supply/battery measurement;
- Identify and button behavior;
- crash-safe persistence and reset/resume;
- RTCC wake and EM2.

Still open: a real Zigbee OTA download, bootloader install, reboot into the new
version, and retained commissioned state.

### Node roles

`examples/efr32mg1-sensor` follows the role template (mutually exclusive
`sed` | `ed` | `router` features, one bin per role):

| bin | build | result |
|---|---|---|
| `efr32mg1-sensor` | default (`sed`) | unchanged sleepy sensor |
| `efr32mg1-ed` | `--no-default-features --features ed --bin efr32mg1-ed` | builds: 161,340 B flash, 12,600 B `.bss`, 18,856 B linked stack |
| `efr32mg1-router` | `--no-default-features --features router --bin efr32mg1-router` | **link rejected** (`products/efr32mg1-tradfri/link/router-tables.x`) |

The router does not fit the TRÅDFRI module. Measured against an enlarged
measurement map with the `constrained-memory` ZCL/ZDO sizes and the
unmodified NWK router tables: flash 248,344 B against the 192,512 B router
window `0x4000..0x33000` (55,832 B over; the child and APS journals take
`0x33000..0x37000`), and static RAM 29,072 B (`ZigbeeDevice<_, Router>`
18,032 B, main future 4,568 B, security journal 2,464 B, app 2,388 B, radio
buffers 768 B), leaving 2,672 B of stack in the `0x7C00`-byte SRAM against the
16 KiB floor (13,712 B short). The router-only growth of `ZigbeeDevice`
(11,120 B over the End Device) is smaller than the RAM shortfall, so no table
shrink alone makes it fit. CI asserts the link failure.

Frame Pending for a parent would have to come from the FRC ISR that already
sends the Imm-Ack, consulting a short-address table armed through
`RadioPhy::set_ack_frame_pending` (core `PENDING = 0`). That table does not
exist yet, so the parent MAC fails explicitly. Role tags: `0x5201` router,
`0x5202` End Device; the original SED writes none. The End Device is build-,
layout-, clippy- and symbol-checked only.

## EFR32 Series 2 (MG21 / MG22)

### Driver provenance

No RAIL object code, library or sequencer microcode is linked or committed.
Register addresses and field layouts come from the GSDK 4.5.0 device headers
(`platform/Device/SiliconLabs/EFR32MG2{1,2}`) and the radio configurator
register maps. The PHY/analog *values* were taken from SWD register dumps of
a lab-only GSDK RAIL program (`RAIL_IEEE802154_Config2p4GHzRadio()`) on the
target die. That program and its dumps are not in the repository. The tables
were reduced to read-write registers that differ from reset.

RAIL normally runs a binary sequencer program on the radio's own core. Here
the sequencer clock is gated (`RAC.CTRL.SEQCLKDIS`) and the CPU does that
work:

- releases the radio state machine (RSM) SEQIF hand-shakes;
- writes the per-state analog values (idle/RX/TX);
- calibrates the VCO with the SYNTH LO counter, caching the result per
  channel.

The sequencer microcode is therefore not needed. The cost is CPU time during
warm-up, about 82 µs from idle to RX.

The xG21 RSM cannot have its sequencer clock gated, and it waits for the
sequencer at every state it enters. The driver therefore loads its own
64-byte vector table into sequencer RAM. Every slot is `END` except the two
warm states. Those park on a self-`JUMP` while the CPU locks the
synthesizer, and writing `END` over the jump releases them. The two
encodings were determined on the hardware; no RAIL sequencer code is used.

The xG21 tables were captured the same way on the ZBT-1 die
(EFR32MG21A020F1024IM32, rev A1). The capture program ran above the
resident Gecko Bootloader at `0x4000`.

### Audit of the former MG21 scaffold

The pre-Series-2 `efr32s2` backend had never run. It used Series 1 register
values that are wrong for Series 2:

| area | scaffold | Series 2 (GSDK headers / hardware) |
|---|---|---|
| radio block bases | FRC 0x40090000, BUFC 0x40091000, SYNTH 0x40092000, RAC 0x40093000, MODEM 0x40094000, AGC 0x40095000 | FRC 0xB8004000, AGC 0xB800C000, MODEM 0xB8014000, SYNTH 0xB8018000, RAC 0xB8020000; BUFC 0x54004000 (xG21) / 0xBA000000 (xG22) |
| radio clocks | Series 1 `CMU_HFRADIOCLKEN0` bits | xG21 radio clocks are always on; xG22 needs `CMU.CLKEN1` radio bits; `CMU_HFPERCLKEN0` does not exist |
| HFXO | none (ran on HFRCO) | HFXO0 block: CTUNE XI/XO, core bias, wait for `STATUS.RDY` / `COREBIASOPTRDY`, then select it as SYSCLK. CTUNE comes from DEVINFO (modules), else `MFG_CTUNE` (0x0FE00100), else the board value (BRD4181A 133, BRD4183A 120, xG21 default 140) |
| RAC commands/states | `TXEN=bit0`, `RXEN=bit1`, `STATE` 0..3 | `RAC.CMD` and `RXENSRCEN` (SWRXEN); the RSM has 14 states (Off, RxWarm, RxSearch, RxFrame, RxPD, Rx2Rx, RxOverflow, Rx2Tx, TxWarm, Tx, TxPD, Tx2Rx, Tx2Tx, Shutdown) |
| SYNTH | `FREQ`/`CHSP`/`CHNO` at Series 1 offsets, no calibration | Series 2 `SYNTH.FREQ` (0x34)/`IFFREQ`/`CHCTRL`/`VCOTUNING`, with VCO calibration by LO-counter binary search |
| FRC/BUFC | flat FIFO data registers | BUFC buffers with per-chip stride, and FRC dynamic-length frame descriptors; RX is read from the buffer RAM backing because the `READDATA` prefetch makes read-and-rewind unsafe |
| PA | `RAC_PACTRL` guess | Per-chip PA tables. xG22: 0 dBm PA up to 6 dBm, ramping through `MODEM.RAMPCTRL`/`RAMPLEV`. xG21: 20 dBm HP-PA curve from `RAIL_SetTxPowerDbm`, power in `RAC.PACTRL[23:16]`, ramped with `PACTRL.PAENPOWERRAMPINGCLK` held |
| RSSI | raw `AGC_RSSI` | RAIL quarter-dB conversion; xG22 offset -50 dB (checked against `RAIL_GetRssi`) |
| EUI-64 | `0x0FE081A0` (TODO) | DEVINFO `EUI64L/H` (same offset on xG21/xG22) |
| IRQ numbers | Series 1 | GSDK vector tables per chip (xG21 61 IRQs, xG22 64): RTCC 10/12, RAC_RSM 37 |
| RTCC compare flags | (none) | `IF`/`IEN` CC bits differ: xG21 CC0/CC1/CC2 = bits 2/3/4, xG22 = bits 4/6/8 (`efr32mg2{1,2}_rtcc.h`) |

Problems found later on hardware:

- **MSC.** `STATUS.TIMEOUT` is an end-of-burst flag, not an error (GSDK
  `em_msc.c`). Each programmed burst is read back instead.
- **RSM.** `RXDIS` only takes effect at the end of the frame being received,
  and `RxOverflow` is left only through `CLEARRXOVERFLOW`. `force_off`
  therefore uses FRC `RXABORT` and `CLEARRXOVERFLOW`. A SEQIF clear can race
  the flag, so the release loop runs until the state changes.
- **EM2.** RAC/FRC radio state is lost in EM2, so `Radio::resume` re-programs
  the PHY after wake. The channel, TX power and VCO cache are kept.
- **Stack.** The 32 KiB xG22 needs the `#[inline(never)] compose()` pattern
  to keep the static stack bound low.

Problems found on xG21 (ZBT-1):

- **Radio clock.** `CMU.RADIOCLKCTRL.EN` must be set before the first radio
  register access. Without it the access is a bus fault (HardFault).
- **HFXO.** Rev-A dies need GSDK `CHIP_Init` HFXO errata writes before the
  crystal starts.
- **VCO search.** The open-loop VCO binary search must run with
  `SYNTH.VCDACCTRL.VCDACEN` set, as RAIL does. Without it, codes depended on
  the previous lock and the TX→RX relock missed the 192 µs ACK. ACKs went
  from 0/20 to 19/20; the MG22 gate is unchanged at 17/20.
- **RTCC.** The HAL used the xG22 CC flag bits on both chips. On xG21 the
  Embassy alarm was never enabled, so the executor slept in `WFE` forever
  right after "Scanning". The bits are now per chip and covered by a host
  test against the GSDK headers. The fix also affects the MG21 sensor's EM2
  wake (CC1). The other MG21/MG22 header differences (CMU, HFXO, EMU, GPIO,
  MSC, SYSCFG, USART, I2C, LDMA, TIMER) were compared and are either gated
  by chip or additive xG22 fields.

### EFR32MG22 BRD4183A (hardware-proven)

| item | value |
|---|---|
| boards | BRD4183A on BRD4001A WSTK |
| MCU | EFR32MG22C224F512IM40, 512 KiB flash, 32 KiB RAM |
| HFXO | 38.4 MHz, CTUNE 120 |
| LED0 / BTN0 | PB1 active high / PB0 active low (EM2 wake) |
| VCOM | USART TX PA5, enable PA4, 115200 |
| layout | app `0x0..0x74000` (no bootloader), router child/APS tables `0x74000..0x7C000`, journal `0x7C000..0x80000` |

The WSTK Si7021 is not reachable from this QFN32 radio board (lab gate
`s2-i2c-scan`). The example reports die temperature (EMU_TEMP) and synthetic
humidity instead.

**Lab gates.** `tools/efr32s2-lab` contains `s2-blink`, `s2-i2c-scan`,
`s2-radio` and `s2-em2`. On channel 15 they showed:

- FCAL on all channels;
- energy detect and CCA;
- beacon request and data-request ACK reception;
- software-ACK turnaround from RX to TX and back of 517–522 µs, including
  352 µs of airtime;
- EM2 sleep with radio resume.

**ZHA results.** Fresh join with the full interview, temperature and
humidity reports, and resume from the security journal.

**Power (AEM).** The slow-poll average is 13.2 µA and the EM2 floor is about
9.1 µA. These figures are without DC-DC and without putting the MX25 SPI
flash into deep power-down.

**Stack.** High-water marks are 9.9 KiB for a fresh join and 11.2 KiB for
resume. The linked stack is 17.6 KiB.

### EFR32MG21 BRD4181A

| item | value |
|---|---|
| MCU | EFR32MG21A010F1024IM32 (GSDK `brd4181a.slcc`), 1 MiB flash, 64 KiB RAM |
| HFXO | 38.4 MHz, CTUNE 133 |
| LED0 / BTN0 | PB0 active high / PD2 active low (no EM2 wake on port D) |
| VCOM | USART0 TX PA5, enable PD4, 115200 |
| layout | Gecko bootloader `0x0..0x4000`, app `0x4000..0x74000`, router child/APS tables `0x74000..0x7C000`, journal `0x7C000..0x80000` (lower 512 KiB) |

`examples/efr32mg21-sensor` is the same composition as MG22: RadioPhy and
SoftMac, RTCC time driver, EM2 and VCOM. It builds and passes the layout
checks, but no BRD4181A is in the lab, so this image has not run.

The xG21 radio itself is lab-tested on the ZBT-1. Still unverified on
BRD4181A:

- the A010 part has no 20 dBm PA, so the captured HP-PA settings may not
  transmit;
- EM2 and RTCC CC1 wake have not run on xG21.

| image (`tools/mac-footprint.py`) | flash | text | bss | task pool | static stack |
|---|---:|---:|---:|---:|---:|
| former scaffold MAC | 208,956 | 177,048 | 18,888 | 11,040 | 25,524 |
| efr32s2 RadioPhy + SoftMac (xG21 tables) | 231,248 | 187,924 | 14,600 | 2,744 | 20,652 |

### Home Assistant Connect ZBT-1 (lab-tested as Router and End Device)

`boards/ha-connect-zbt1` describes the ZBT-1:

- EFR32MG21A020F1024IM32 (1 MiB flash, 96 KiB RAM, SE firmware 0.1.6)
  behind a CP2102N;
- USART0 TX PA6, RX PA5, CTS PB1, RTS PB0, 115200 with hardware flow
  control, per the Nabu Casa firmware-builder manifests;
- HFXO CTUNE from the `MFG_CTUNE` token, otherwise the board default 140.
  On the inspected unit every MFG token is erased and DEVINFO has no CTUNE.
  The stock firmware's HFXO0 `XTALCTRL`, read over SWD while it was
  running, holds CTUNE 140;
- EUI-64 from DEVINFO (the custom-EUI token is erased);
- no EFR-driven LED or button is confirmed.

The shipped Gecko Bootloader (2.4.2) occupies `0x0..0x4000` and accepts
GBL files over XMODEM on that UART. The stock flash image has the NCP at
`0x4000..0x3E9D8`, NVM3 at `0x74000..0x7E000`, and the upper 512 KiB
erased. The NCP's Application Properties use signature type NONE, so
secure boot is not enforced.

`products/ha-connect-zbt1` and `examples/ha-connect-zbt1` (bin
`ha-connect-zbt1-ed`) make it an always-on (RX-on-when-idle) End Device
with Basic and Identify on endpoint 1. The same crate also builds the
`ha-connect-zbt1-router` and `ha-connect-zbt1-sed` bins (see
[Node roles](#node-roles-router-always-on-ed-sed)).

| item | value |
|---|---|
| identity | `Zigbee-RS` / `ZBT-1-AlwaysOn-ED` |
| layout | bootloader `0x0..0x4000` (kept), app `0x4000..0xF4000`, router child/APS tables `0xF4000..0xFC000`, journal `0xFC000..0x100000` (2 × 8 KiB), RAM 96 KiB |
| boot check | DEVINFO must report 1 MiB of flash, otherwise the firmware halts |
| TX power | 10 dBm default (the ZBT-1's certified limit is not published) |
| logs | SEGGER RTT only; the UART stays free for a future bootloader trigger |
| after Leave | Network Steering re-arms after 5 min, longer than ZHA's 254 s permit window |

The stock NVM3 is overwritten. A full flash backup was taken over SWD
before every write.

Measured sizes: 230,012 B file-backed from `0x4000`, 20,748 B `.bss`, and
77,044 B linked stack headroom.

Hardware evidence (2026-10-31, ZHA channel 15, PAN `0xDFE9`):

- **Radio gates** (`tools/efr32s2-lab`, ZBT-1 variant): TX loop, ED
  monitor, and 19/20 ACKs with a 594 µs RX→TX→RX turnaround.
- **Join.** Scanning found 10 parents. The first choice (`0xD586`) did not
  deliver the Transport Key within 6 s, so the device fell back to `0x85FA`.
  It joined as `0x888A`, received the network key, answered the interview
  (node, active-endpoint and simple descriptors, Basic reads), and sent its
  unique-TCLK verify. The parent accepted the End Device Timeout.
- **Identify.** A press in HA reached the device, which stopped
  identifying after about 4 s.
- **Reset.** After an SWD reset the security journal was preserved and
  resume (`addr=0x888A`, parent `0x85FA`) took 14 ms. A second Identify
  worked.

Not tested yet: long-duration stability, OTA, and a deliberate Leave
followed by automatic recommissioning.

**Recovery.** The bootloader has no GPIO activation. Once our application
replaces the NCP, the only ways back into the bootloader are:

- SWD pads;
- `efr32s2_hal::gecko_bootloader::reboot_to_bootloader()`, which writes
  `{0x0202, 0xF00F}` at `SRAM_BASE` and then triggers `SYSRESETREQ` (GSDK
  `btl_main.c`).

The application needs a trigger for this, and that requires a UART
receiver, which the HAL does not have yet.

### Node roles (router, always-on ED, SED)

Each Series 2 example crate builds one bin per role. The features `ed`,
`sed` and `router` are mutually exclusive (see the
[role template](../advanced/soft-mac-porting.md#role-template-one-example-crate-per-board)).

| board | SED | always-on ED | Router |
|---|---|---|---|
| ZBT-1 (MG21, 96 KiB RAM) | `ha-connect-zbt1-sed` | `ha-connect-zbt1-ed` (default) | `ha-connect-zbt1-router` |
| BRD4181A (MG21, 64 KiB RAM) | `efr32mg21-sensor` (default) | `efr32mg21-ed` | `efr32mg21-router` |
| BRD4183A (MG22, 32 KiB RAM) | `efr32mg22-sensor` (default) | `efr32mg22-ed` | does not fit, link rejected |

**MAC.** The router uses `Efr32s2RouterMac = SoftMacCore<Efr32s2RadioPhy,
Parent<8, 8, 0>>` with the HAL feature `isr-ack`. The `FRC` RX-done
interrupt sends every Imm-Ack (see *Interrupt Imm-Ack* below), so the PHY
reports `hardware_auto_ack` and the core never calls `send_ack`. The Frame
Pending bit comes from an 8 short + 8 extended source-match table in the
HAL that the core arms through `RadioPhy::set_ack_frame_pending` whenever
it queues indirect data or an association response. `PENDING = 0`: the HAL
table is the only Frame Pending owner, as on TLSR8258. A Data Request
the interrupt acknowledged with FP = 0 is queued with a flag (bit 7 of its
record's length byte), which reaches the stack with that poll
(`RxFrame::acked_without_frame_pending` → `PhyRxFrame` (bit 7 of its
length byte) →
`MlmeDataRequestIndication::acked_without_frame_pending`). The child turns
its receiver off after such an ACK. When the stack serves that poll after it
has queued a frame (the poll event waited behind slower work), the frame
stays queued and armed for the next poll; the soft MAC also keeps an
Association Response armed after it. Without the flag, the frame was sent to
a child that no longer listened, and was lost. The decision travels with the
poll, so no per-child state exists to fill up: a first version recorded it
in an 8-entry per-source table, which the first eight sleepy children
polling without data occupied for good (review of #23). The core still
builds beacons, association and orphan responses and indirect data;
`PENDING = 0` compiles out only the core's software Frame Pending table. The ED and SED bins keep the
polled HAL ACK: the HAL acknowledges matching frames while `receive` polls
it, always with FP = 0. `tools/check-role-symbols.sh` checks in CI that the
ED and SED images contain neither parent code nor `radio::isr_ack`.

**Persistence.** Router images store the child table and the APS
duplicate/fragment state in their own flash partitions, which sit between
the application and the security journal (`0x74000..0x7C000` on MG2x,
`0xF4000..0xFC000` on the ZBT-1). Every role's linker script reserves these
partitions, so the security journal never moves. The role tag in the
journal makes a reflash that changes role rejoin while it keeps the frame
counters.

**Interrupt Imm-Ack (router images).** In the first router release the
core sent each ACK in software. It often came too late: the core handles one
frame at a time (software AES-CCM*, RTT logging), so the next frame aged
past the ACK window and the PHY had to drop it unacknowledged. With one
polling child the ZBT-1 dropped 123 frames and ACKed 158 in 16 minutes
(session `…202457Z-mg22-sed-via-zbt1-steady`). The `isr-ack` HAL feature
moves the ACK into the `FRC` interrupt (IRQ 34, `FRC_IEN` = RXDONE | RXOF).
For each newly completed frame in the RX buffer, the handler:

- reads the MHR (24 bytes at most) and checks the PAN ID and destination
  the way the HAL's ACK filter does;
- sends a precomputed 5-byte ACK at once if AR = 1 and the frame is for us,
  setting FP = 1 only for an unsecured MAC Data Request (command 0x04) whose
  source address is in the FP table. An ACK to a data frame carries FP = 0
  (802.15.4-2006 7.2.2.3.1; we do not use the "FP on every frame from a
  pending child" variant);
- drops ACK frames, frames for other nodes or PANs, and frames it could no
  longer ACK within 400 µs of their end (the sender retransmits, so such a
  frame must not be delivered twice);
- moves every kept frame (`[len, MPDU, raw RSSI]`) out of the 1 KiB
  hardware RX buffer into a 1 KiB software queue, and always advances the
  buffer's read offset. The hardware buffer is therefore empty after every
  interrupt. Frames without an ACK request (mostly relayed broadcast copies)
  must leave two maximum-length records (254 B) free, so a broadcast storm
  cannot starve unicasts to this node; such drops count in
  `AckStats::queue_full`. An ACK-requested frame that still finds no room is
  left unacknowledged, so the sender retries
  (`AckStats::queue_full_unacked`). A corrupt length byte clears the buffer
  (`AckStats::resyncs`).

Task-side radio operations mask the `FRC` line, so the interrupt never sees
the radio mid-operation. When the lock is released, it re-pends the
interrupt if a frame completed meanwhile, and that frame is aged from the
start of the lock. `receive` pops only from the software queue, which holds
frames the interrupt has already examined, so the ACK decision always comes
before delivery. The sequencer's
hardware auto-ACK was evaluated and rejected: it is driven by RAIL's
sequencer microcode, which this driver does not load (the sequencer stays
clock-gated).

**RREQ relay and RX overflow.** A router relays Route Requests three times,
254 ms apart, after a 4–256 ms jitter. The core does not receive while these
NWK delays run (`SoftMacCore::delay_micros`), and a storm of relayed RREQs
from neighbours overflowed the old 512-byte RX buffer (1458 overflows in
12.7 h on the software-ACK router). `transmit_cca` clears a pending overflow
before it enables RX, so CCA never sees a stale RxOverflow. On the
interrupt-ACK router a first version left foreign frames in a 2 KiB
hardware buffer until the task read past them. Channel 15 carries about
25 frames/s, of which about 2.6/s are for us, so a 0.8 s RREQ relay
(`nwk_commands.rs`) or broadcast jitter (`nlde.rs`) still overflowed it
(86 overflows in 22 min, session `…102430Z-s2ack-mg22-sed-via-zbt1-israck-v2`).
Router images now drain the radio in the HAL instead of the core. The
interrupt empties the hardware buffer on every frame and keeps only the
frames for us in the software queue, already ACKed, which the core reads
after the delay. The hardware buffer shrank from 2 KiB to 1 KiB, so the
1 KiB queue costs no extra RAM. Since then, the NWK layer schedules broadcast
and RREQ relays instead of waiting inline (see the broadcast relay schedule in
[NWK](../core-concepts/nwk.md)), so the router loop keeps reading the queue
during jitter and retry intervals. The figures above predate that change and
have not been re-measured on the EFR32 routers.

**RAM budget (Cortex-M33, default tables).**

| image | `.bss` | linked stack | notes |
|---|---:|---:|---|
| ZBT-1 router | 38,672 | 58,608 | 96 KiB part; worst-case call chain 30,944 |
| MG21 router (BRD4181A) | 31,504 | 33,520 | 64 KiB part; worst-case call chain 30,944 |
| MG22 router | about 31,500 | below the 14 KiB floor | does not fit 32 KiB |

The `isr-ack` HAL state is 1 KiB of hardware RX buffer, the 1 KiB
software RX queue and the FP table (8 short + 8 extended entries). It adds
about 1 KiB of `.bss` to the router images compared with the 512-byte RX
buffer of the leaf images.

The MG22 router's `.bss` breaks down as follows (measured at `4f842fd`,
before `isr-ack`, which adds a net 1,032 B): `ZigbeeDevice<_, Router>`
19,824 B, main future pool 4,520 B, security journal 2,464 B, application
2,392 B, radio 768 B. Including the 14 KiB stack floor, this exceeds the
32 KiB of SRAM. The product linker script `ASSERT` rejects the link, and CI
asserts that it fails. Shrinking the routing, neighbour and child tables
would make it fit, but that is a product decision; nothing shrinks them
silently.

**Hardware evidence** (2026-10-31, ZHA channel 15, PAN `0xDFE9`; hwlab
session IDs):

| board | role | result | sessions |
|---|---|---|---|
| ZBT-1 | Router | Joined as `0xBAAF` and completed the interview; ZHA shows a Router. After a topology update, ZHA lists 16 neighbours with LQI and an active route to `0x0000` via `0xD586`, which proves Mgmt_Lqi and Mgmt_Rtg. Resume from the journal after reflash. | `2026-10-31T195202Z-zbt1-router-join1`, `…195442Z-zbt1-router-topology`, `…200245Z-zbt1-router-topology2`, `…201147Z-zbt1-router-v2-overflowfix2` |
| ZBT-1 → MG22 | Router as parent | ZHA permitted joining only through the ZBT-1. The MG22 SED's scan showed only `0xBAAF` permitting, and it associated as `0xEE0F`. The ZBT-1 relayed the Transport Key indirectly to the polling child, saw the key-possession proof, and saved its child table. The interview and Configure Reporting went through `0xBAAF`. ZHA shows the MG22 as `0xEE0F` with temperature and humidity updating. The ZBT-1 reports `children 1`, `child 0xEE0F rx_on=false`. | `…202132Z-mg22-sed-via-zbt1-router`, `…202457Z-mg22-sed-via-zbt1-steady` |
| ZBT-1 | always-on ED | See the ZBT-1 section above. | earlier sessions |
| ZBT-1 | SED (`em2-debug` build) | After the reflash from the router role, the journal's role tag cleared the network state but kept the frame counters. The SED joined through `0xD586`. The Transport Key came by indirect poll, then Request-Key, Verify-Key and Confirm-Key SUCCESS. The parent accepted the ED Timeout Request (enum 14). Device_annce went out, and the fast-poll window ended after 120 s. In 30 s long polls, `power::EM2_ENTRIES`, read over SWD with hwlab `memory-read`, rose from 7 to 9 in 60 s. ZHA kept the IEEE's old Router descriptor, because zigpy doesn't re-interview a known device on rejoin. Current in EM2 was not measured: the ZBT-1 has no AEM. | `…203748Z-zbt1-sed-join` |
| ZBT-1 | Router (final image, after the SED) | The role tag cleared the SED state. The first join reused `0xBAAF`, but the Node_Desc_rsp from the TC never arrived: the previous parent `0xD586` still held `0xBAAF` as its sleepy child. The TCLK exchange failed, so the router left and rejoined 12 s later as `0x794A`, with Confirm-Key SUCCESS. ZHA shows a Router at `0x794A`. Its former child, the MG22, found a new parent by itself (`0xD066`) and kept reporting. | `…204449Z-zbt1-router-final` |
| ZBT-1 | Router (`service_overflow` out of line) | Resumed `0x794A` from the journal without rejoining. In 240 s: 15–17 neighbours, a route, 3 RX overflows cleared, 28 transmissions, 0 TX failures. ZHA still lists it as an available Router. | `…210701Z-zbt1-router-step7-inline-never` |
| MG22 | SED | See the BRD4183A section above. | earlier sessions |

**Interrupt Imm-Ack on the ZBT-1 router** (2026-11-01, same network; the
MG22 SED `0xEE0F` is the ZBT-1's child in every run; counters are at the end
of each capture):

| build | duration | ACKs (FP = 1) | late | RX overflow | queue-full (unACKed) | other | sessions |
|---|---:|---:|---:|---:|---:|---|---|
| software ACK, `Parent<8,8,8>` (`4f842fd`) | 16 min | 158 | 123 dropped as stale | 1458 in 12.7 h | — | | `2026-10-31T202457Z-mg22-sed-via-zbt1-steady`, `2026-11-01T094924Z-s2ack-pre-state` |
| `isr-ack` v2, 2 KiB hardware buffer | 22 min | 451 (35) | 1 | 86 | — | max turnaround 159 µs | `…102430Z-s2ack-mg22-sed-via-zbt1-israck-v2` |
| v3, interrupt drains into the queue | 21 min | 351 (31) | 0 | 0 | 2491 (not split) | broadcast storm at 180–420 s; tx-fail 0; resync 7; max 167 µs | `…110156Z-s2ack-mg22-sed-via-zbt1-israck-v3-queue` |
| v4, split counters | 18 min | 461 (0) | 0 | 0 | 903 (88) | the 88 at 0–415 s; one MG22 report lost to NoAck at 210 s; tx-fail 0; max 157 µs | `…113132Z-s2ack-mg22-sed-via-zbt1-israck-v4` |
| v5, 254 B reserved for ACK-requested frames | 18 min | 743 (36) | 0 | 0 | 319 (0) | resync 3; tx-fail 0; ack-fail 0; max 295 µs | `…115456Z-s2ack-mg22-sed-via-zbt1-israck-v5-reserve` |

`late` counts ACK-requested frames the interrupt found more than 400 µs
after their end. `max turnaround` is measured from interrupt entry to the
ACK's TX start; it stays below the 400 µs deadline and the sender's 864 µs
`macAckWaitDuration`. It was not cross-checked with a sniffer. In every run
the router resumed `0x794A` from the journal, restored its children and sent
Parent_annce. The MG22 reported temperature and humidity through it without
a MAC failure, except for the one v4 loss. Third-party sleepy children
joined through the ZBT-1 and completed the ZHA interview during these runs:
an nRF52833 SED (`0x9254`, v3) and a TLSR8258 SED (`0xBDF2`, v5). The FP = 1
ACKs are from their Transport Key and interview traffic. After v5, ZHA lists
the ZBT-1 Router `0x794A` and the end devices `0xEE0F`, `0x9254` and
`0xBDF2` as available.

Not run on hardware: the MG22 always-on ED, every BRD4181A image (no board
in the lab), and the MG21 router on BRD4181A.

### Hardware crypto (Series 2)

Zigbee AES (NWK/APS CCM*, AES-MMO key derivation, install codes,
Trust Center Verify-Key, Green Power GPDF security) runs only on the
chip's AES engine on xG22. No MG22 image (sensor, End Device,
loader-linked builds, both GPDs) links the RustCrypto software core; CI
fails an MG22 image that contains `aes::soft`/`SoftwareAes128` or lacks
the CRYPTOACC symbols.

| Chip | Engine | Reference |
| --- | --- | --- |
| xG22 (BRD4183A) | CRYPTOACC Silex BA411E behind the CRYPTOMASTER scatter/gather DMA: config, key and block fetch descriptors, one push descriptor | GSDK 4.5 `sx_blk_cipher_ecb`, `cryptodma_internal.c`, `ba411e_config.h`, `cryptoacc_management.c` |
| xG21 (BRD4181A, ZBT-1) | Still software AES (`zigbee-crypto`), as before. The HSE Secure Engine backend (`AES_ENCRYPT \| MODE_ECB` through the `SEMAILBOX_HOST` mailbox) is pending on branch `aes-hw-efr32mg21` until it is proven on MG21/ZBT-1 silicon. | — |

Code: `efr32s2-hal/src/crypto/{mod,cryptoacc}.rs` (mechanism, bounded
polls, host model), `zigbee-mac/src/efr32s2/aes.rs` (provider). The
composition root passes the board's `crypto` token to
`Efr32s2RadioPhy::install_aes_engine`, which runs a two-vector known-answer
test on the silicon; on failure the image logs the error and halts before
Zigbee starts (`AES_KAT_PASS` on success). Without an installed engine every
cipher request fails, so frames are dropped rather than sent or accepted
unprotected. After any engine fault the next request re-runs the KAT before
it may publish output.

Ownership and conflicts (xG22):

- The TRNG (`efr32s2_hal::trng`, BA431 block, used by the Green Power
  Device for key provisioning) and the AES core share the CRYPTOACC
  bus/AES/PK clock gates. `clock::cryptoacc_acquire/release` keep a holder
  set (TRNG, AES), so neither gates the clocks under the other. The AES
  engine opens them per request, like GSDK. The xG22 VSE mailbox is not
  used.
- The 116 B DMA area is a static owned through the singleton token, so a
  late engine write after a timeout cannot hit reused memory; key and data
  copies are wiped after every request.

Lab gate: `tools/efr32s2-lab` `s2-aes` (BRD4183A, default `wstk` feature)
runs the self test, the SP 800-38A ECB vectors and a 1,000-block re-keyed
chain checked against a host value, and reports cycles per block.

Validation level:

- **xG22 CRYPTOACC: production path proven on hardware** (BRD4183A,
  2026-11-21). The `efr32mg22-sensor` SED logged `AES_KAT_PASS CRYPTOACC`,
  joined ZHA from factory-new (Transport-Key, unique TCLK, Verify-Key ->
  Confirm-Key SUCCESS, Device_annce), finished its reporting configuration
  and reported; after a reset it resumed as the same node (`0xDFE8`) and
  kept reporting. The GPD sensor's security level 3 GPDFs were accepted by
  the ZHA sink. `s2-aes`: KAT, SP 800-38A 4/4 and the 1,000-block chain
  pass; about 900 CPU cycles (23 µs at 38.4 MHz) per block including the
  per-request clock gating. Built and gated only: the always-on ED, the
  loader-linked images and the GPD switch. EM2 current with the engine was
  not measured.
- **xG21 / ZBT-1: software AES** (unchanged). The Secure Engine backend on
  branch `aes-hw-efr32mg21` is compiled and host-tested only.

### Build

All EFR32 examples use `nightly-2026-03-23`:

```bash
cd examples/efr32mg1-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor

cd ../efr32mg22-sensor   # or efr32mg21-sensor, ha-connect-zbt1
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg22-sensor
```

The ZBT-1 binary is named `ha-connect-zbt1-ed`, which keeps it apart from the
board crate in the lockfile.

## EFR32MG24 (xG24): BRD4187A and EK2703A radio labs, RF-BM-MG24B1 dev board, SONOFF Dongle-M

**Status.** The xG24 radio runs with a 39 MHz crystal on both PA variants.
The 20 dBm (HP) PA was lab-tested on one BRD4187A (EFR32MG24B220F1536IM48)
on a BRD4001A WSTK:

- raw TX and RX over the air;
- Imm-Acks, energy detect and RSSI;
- TX and RX current equal to RAIL's;
- the Router (`--features wstk-brd4187`) joined an EmberZNet-coordinated
  test network, passed the interview and resumed from its journal after a
  reset.

The 10 dBm (MP) PA was lab-tested on one xG24 Explorer Kit EK2703A
(EFR32MG24B210F1536IM48):

- TX output equal to RAIL's from −30 to +10 dBm (sniffer);
- RX, Imm-Acks, energy detect and beacon reception;
- the Router (`--features ek2703a`) joined the same network through the
  BRD4187A router as its parent and resumed after a reset.

**MP current unmeasured:** the EK2703A has no current monitor, so the
10 dBm PA's TX current has never been compared with RAIL's.

See [Radio](#radio-captured-xg24-tables),
[Hardware validation](#hardware-validation-brd4187a) and
[10 dBm PA](#10-dbm-mp-pa-ek2703a).

Not proven on hardware:

- **RF-BM-MG24B1**: the radio stays enabled. The board uses the same radio
  configuration as the EK2703A (10 dBm PA, 39 MHz), with the tables captured
  on the B210. The board-level items are unverified: the A410 die, the RF
  path to the module's antenna, and CTUNE 92. Its first run should measure
  the CTUNE frequency offset, the output against RAILtest and the TX
  current.
- **Dongle-M (38.4 MHz, 20 dBm PA)**: the radio is enabled
  (`hfxo-38m4hz`) with the stock Dongle-M EmberZNet image's PHY, which is
  GSDK's 38.4 MHz 802.15.4 configuration byte for byte, and RAIL's
  per-channel words. **First HIL (one unit, partial)**: the NCP image,
  uploaded through the stock bootloader, ran 22 minutes at 460 800 Bd with
  the EmberZNet network restored onto it (channel 20) and decrypted the
  NWK-secured rejoins of an end device; transmission to a device, the
  output and the CTUNE offset are not yet measured, and the table still
  carries the 39 MHz B220 die's runtime trims. See
  [SONOFF Dongle-M over the stock firmware](#sonoff-dongle-m-over-the-stock-firmware).
  An xG24 build that names no crystal keeps the radio `Unsupported`.
- **NCP**: ran on the BRD4187A only, against `hubctl` over the WSTK VCOM
  at 115 200 (formation, join, ZCL, backup/restore, reset/resume; see
  [ZRHP NCP](#zrhp-ncp-on-the-brd4187a-wstk-vcom)). Not run on the
  RF-BM-MG24B1 / ESP32-S3 link, at 460 800 or 921 600, or on an MP die.
- **Power**: EM2 and the DC-DC converter are untested.

### Crates

```text
examples/efr32mg24-ncp      ZRHP NCP composition root  (product feature `ncp`)
examples/efr32mg24-router   Router composition root    (product feature `router`)
        ↓
products/efr32mg24          identity, memory map, journals, Gecko record,
                            NCP transport policy, router policy
        ↓
boards/rf-bm-mg24b1 | boards/sonoff-dongle-m | boards/efr32mg24-brd4187
  | boards/efr32mg24-ek2703a
        ↓
efr32s2-hal (`efr32mg24-pa10` / `efr32mg24-pa20`, `hfxo-39mhz` / `hfxo-38m4hz`)
        + zigbee-mac `efr32mg24` (Efr32s2RadioPhy on SoftMacCore)
tools/efr32-gbl             ELF → raw image, Intel HEX and GBL3
tools/efr32s2-lab           lab gates: `s2-mg24-hello` (RF-BM), `s2-mg24-air`
                            (BRD4187 / EK2703A radio console); xG24 captures
```

No PAC is used. The only published xG24 PAC (`efr32mg24-pac` 0.1.0, one
B220 part) has no radio blocks, and the HAL already uses MMIO addresses
checked against the GSDK headers.

### HAL: what changes on xG24

Every value below was compared against GSDK 4.5.0
(`platform/Device/SiliconLabs/EFR32MG24/Include`, A410F1536IM48 and
A420F1536IM48) with `offsetof()` dumps from the headers. The MG21/MG22 paths
are unchanged.

| block | xG21 / xG22 | xG24 |
|---|---|---|
| flash | at `0x0`, 512 KiB / 1 MiB | at `0x0800_0000`, 1536 KiB, 8 KiB pages; `flash` offsets are relative to `chip::FLASH_BASE` |
| SRAM | 64/96 KiB / 32 KiB | 256 KiB |
| HFXO | `0x4000_C000`; `CTRL.FORCEEN`/`DISONDEMAND` bits 0/1; `FSMLOCK` (+ `MANUALOVERRIDE` on xG22) | `0x4A00_4000`; bits 16/24; no `FSMLOCK`, wait for `STATUS.SYNCBUSY`; XO CTUNE = XI + 40; fixed capacitor on XO only on 20 dBm parts ([PM-5131]) |
| crystal | 38.4 MHz | named by the board: `hfxo-38m4hz` (Dongle-M, 38.4 MHz) or `hfxo-39mhz` (RF-BM-MG24B1, BRD4187A, EK2703A); neither = radio `Unsupported` |
| identity and CTUNE | DEVINFO EUI-64; `MFG_CTUNE` | `devinfo::ieee_address`: `MFG_CUSTOM_EUI_64` (`0x0FE0_0002`) unless all `FF`, else DEVINFO (EmberZNet's precedence; xG21/xG22 MACs keep DEVINFO). `devinfo::hfxo_ctune`: `MODXOCAL` when `MODULEINFO.HFXOCALVAL` is valid, then `MFG_CTUNE` (`0x0FE0_0100`), then the board default (GSDK `sl_device_init_hfxo`); MG24 images log value and source at boot |
| bus clocks | xG22 `CLKEN0/1` | `CLKEN0/1` at other bit positions (GPIO 26, HFXO0 20, USART0 9, LFRCO 22, SYSRTC0 30, MSC 16 in CLKEN1, SE mailbox 10) |
| GPIO | ports at `+0x00` | ports at `+0x30`; USART0 route at `0x720` |
| time base | RTCC | **SYSRTC0** group 0 (compare 0/1, OVF/CMP0/CMP1 at bits 0/1/2), same `rtcc` API and time driver (`SYSRTC_APP` IRQ 67) |
| UART | USART0/1 | USART0 only (+ EUSART0/1, not supported); host link `usart::host` (feature `usart0-host`): LDMA receive into an 8 KiB ring with SYNC-gated quarters (no RX interrupt), interrupt-driven 2 KiB TX ring, overrun and framing counters |
| LDMA | — | `ldma`: register map and descriptors (same on xG21/xG22/xG24 except the LDMAXBAR `REQSEL` offset); one user, the host UART receiver (channel 0, `SYNCTRIG` bits 0..=3) |
| IRQs | 61 / 64 | 76 (15 and 64 reserved); FRC 33, RAC_RSM 36, USART0_RX/TX 9/10 |
| WDOG0, I2C0 | `0x4A01_8000`, `0x4A01_0000` | `0x4B00_4000`, `0x4B00_0000` |
| TRNG | xG22 CRYPTOACC; xG21 none | Secure Engine mailbox `TRNG_GET_RANDOM` (`se`), wired to `PlatformServices::fill_random` on xG24 only |
| AES (NWK/APS CCM*, MMO) | xG22 CRYPTOACC hardware; xG21 software | software (`zigbee-crypto`), like xG21, until a Secure Engine AES backend (branch `aes-hw-efr32mg24`) is proven on hardware |
| reset cause | — | `reset::take_reset_cause` (`EMU_RSTCAUSE`, same bits on all three dies) |

The SE writes the TRNG output descriptor and buffer by DMA, possibly after
the bounded wait gave up. Both live in a `static` area that only the holder
of the mailbox lock (`se::MailboxLock`) touches; a submitted command that is
not answered in time, or rejected with `TX_STATUS.TXERROR`, poisons the
mailbox until reset, so a late completion only writes that unused area and
its response is never taken for a later command's. A failed request zeroes
the caller's buffer. Host-model tests in `efr32s2_hal::se` cover the late
completion; host builds have no SE and fail with `SeError::Unavailable`.

`fill_random` reports SE failures as specific MAC errors: no SE
(`Unavailable`) is `Unsupported`, the platform's "no entropy source";
`Timeout`, `MailboxError` and `Poisoned` are `RadioError` (a hardware fault,
as a failed TRNG on CC26x2); a non-OK SE result is `SecurityError`; a mailbox
held by another caller is `Other`. Every stack caller fails closed on any of
them (formation, PAN ID, Trust Center keys).

**Poisoned Secure Engine is fatal** (product policy, `fatal` module of
`products/efr32mg24`). A poisoned mailbox only recovers with a reset; on
xG24 the TRNG is the NCP's only entropy source, so the NCP could never form
a network or generate a key again while it kept answering the host. Both
compositions check `se::is_poisoned()` after every main-loop step. On a
fault the NCP logs it to the ZRHP LOG ring and serves the host link for
100 ms; the Router logs it on its UART and flushes it. Both then reset with
the cause recorded in the Gecko `BootloaderResetCause_t` word at
`SRAM_BASE` (reason `0x0980`, the EmberZNet `RESET_FATAL` group with a
zigbee-rs extension code; not a bootload reason, so the resident bootloader
starts the application). The next boot takes and clears the record: the
NCP reports `EVT_NCP_READY.reset_reason` 6 (fatal) and both images log the
cause. The product linker script starts RAM at `0x2000_0004` so start-up
never overwrites that word (asserted in `link/memory.x` and by
`verify-layout.py`). The Router links no SE client today, so the compiler
removes its check; it becomes live if one is added.

The other fatal resets use the same record, so the host can tell them from
its own `SYS_RESET` (`reset_reason` 2):

| cause | recorded reason | where |
|---|---|---|
| poisoned Secure Engine | `0x0980` | main loop of both images (logged first) |
| panic | `0x0981` | `#[panic_handler]` of both images |
| HardFault | `0x0982` | `HardFault` handler of both images (`verify-layout.py` rejects cortex-m-rt's spinning default) |
| unrecoverable router-app error | `0x0983` | the Router's `Supervisor::reset`, after the app's `Fatal` diagnostic |

The NCP reports each as `reset_reason` 6 and logs the cause to the ZRHP LOG
ring at boot; the Router logs it. The policy, the reason encoding (every
code decodes to itself and is outside the bootload set, const-asserted) and
the ZRHP reset-reason mapping are host-tested; no MG24 has run any of these
paths.

Not changed and not yet checked on xG24 silicon: EM2 entry (`power`), DC-DC
(left in bypass), the LFXO (the RF-BM module has one; the time base still uses
the LFRCO), I2C and IADC (not used by these products).

### Radio: captured xG24 tables

The xG24 register map (`efr32_multiphy_configurator/.../efr32xg24/revA1`)
was compared field by field with xG22 rev A2:

- FRC, AGC, MODEM, SYNTH, RAC and RFCRC sit at the xG22 addresses. Every
  offset the shared driver uses is unchanged.
- The RSM has the xG22 handshake (`RAC.CTRL.SEQCLKDIS`, `RAC.SEQIF` with
  the same state bits), so the clock-gated sequencer approach carries over.
- `MODEM.RAMPCTRL/RAMPLEV` moved to `0xD8/0xDC`, and `IRCALCOEFWR0/1` moved
  to `0x278/0x27C`.
- The analog front end is new. The RAC analog registers moved: `RFPATHEN0`
  0x160, `RX` 0x168, `TX` 0x16C, `SYEN` 0x180, `SYLOEN` 0x184,
  `AGCOVERWRITE0/1` 0x1A0/0x1A4, `PACTRL` 0x1C8. The PA comes in a 10 dBm
  (MP) and a 20 dBm (HP) variant.

The values that only silicon can give were captured on the BRD4187A.
GSDK 4.5.0 RAILtest (`config2p4GHz802154`, 39 MHz) ran on the board and was
read over SWD in idle, RX on every channel, CW and packet TX at every HP
level, and during RAIL's PA transitions. The method, raw dumps and decoded
summaries are in `tools/efr32s2-lab/captures/xg24-brd4187a`.

| value | xG24 (BRD4187A, EFR32MG24B220) |
|---|---|
| BUFC | base `0xBA00_0000`, buffer 0 at `+0x0C`, stride `0x40` (xG22 layout) |
| static PHY | 151 writes. 192 of the 208 registers in GSDK's `sl_rail_ieee802154_config_39MHz.c` equal the silicon. The other 16 are per-state, per-channel or runtime calibrations. |
| per-state analog | idle, RX and TX values of 14 RAC registers, plus `SYNTH.VCDACCTRL/VCORANGE` and `MODEM.IRCAL/PADEBUG` |
| per-channel | RAIL rescales the IF chain on every channel. At 39 MHz, `MODEM.SRCCHF.SRCRATIO2` = 0x3C29 + 0x20·(ch−11) and `DIGMIXCTRL.DIGMIXFREQ` ∝ 1/(`FREQ`+`IFFREQ`) (±1 LSB of RAIL's rounding). `SYNTH.CHCTRL` stays 0 and the channel is set through `SYNTH.FREQ`. |
| HP PA | RAIL's 29 levels from −24.9 to +20.0 dBm. `RAC.PACTRL` = slice<<12 \| power<<4 (+ bit 10 in TX). `MODEM.RAMPLEV` = the DEVINFO byte of the slice (`0x0FE0_8250..5B`, `D6..D0` on this die). |
| PA power-up | RAIL's order, sampled on the target: see the PA current fix below |
| RSSI | offset −50 dB: RAIL reports `floor(FRAMERSSI/4) − 50`, and the driver's trailer RSSI matches |
| IR calibration | DEVINFO `0x0FE0_8248/824C` → `MODEM.IRCALCOEFWR0/1` (0x80FBFFEF on this die) |
| `RAC.RFLOCK0` | `0x0006_0000` is set at boot (0 and 10 dBm PAs locked on this part); the driver never writes it |

**Crystal.** The 39 MHz tables come from the silicon (`hfxo-39mhz`). The
38.4 MHz build (`hfxo-38m4hz`, Dongle-M) differs in the 21 registers in
which GSDK's `…_38M4Hz.c` differs from the 39 MHz file. Evidence for it:

- The stock Dongle-M image (`zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`)
  contains GSDK 4.5.0's `sl_rail_ieee802154_phy_2G4Hz_38M4Hz_modemConfigBase`
  and `_modemConfig` byte for byte (238 + 66 configuration words; the
  pointers differ) and the same `phyInfo` (`src2Denominator` 60 MHz,
  `synthCache` `0x02104911`). The FEM and antenna-diversity variants
  match only partly, and no 39 MHz value occurs
  (`tools/efr32s2-lab/captures/dongle-m-stock`, reproducible with
  `efr32_radio_capture.py stock-gbl`).
- RAIL computes `SRCRATIO2` and `DIGMIXFREQ` per channel at run time; they
  are not in the image. `xg24::channel_words` reproduces the computation:
  `SRCRATIO2 = round(2^11 · (f_LO/8) / src2Denominator)` (fitted) and
  `DIGMIXFREQ = round(IFFREQ · dec0 · 2^26 / (FREQ + IFFREQ))` (GSDK radio
  configurator, `lynx`). Both reproduce all 16 channels of the 39 MHz
  silicon exactly, and GSDK's channel-11 words at both crystals. The
  earlier proportional scaling was within one LSB.

**It has not run on silicon**, and it carries the 39 MHz B220 die's
runtime trims (below). The first run on a Dongle-M must compare raw TX/RX,
ACKs, output and the CTUNE offset against the stock image, and capture
RAIL's per-state values at 38.4 MHz to confirm the shared analog states.

**Die-specific values.** RAIL computes some trims at run time:
`SYNTH.LPFCTRL2RX/TX`, `RAC.IFADCTRIM0`, `LNAMIXTRIM4`, `SYTRIM1`,
`SYNTHREGCTRL` and the `AGC.CTRL0` RSSI shift. The table carries this die's
values. Whether other dies need their own values is open.

**PA current: found and fixed on hardware.** The first PA ramp followed the
xG21 order: regulators, then stages, then slice and power enables, then the
ramp clock. The output matched RAIL, but the PA drew more than the AEM's
95.7 mA at every level, including 0 dBm, where RAIL draws 25 mA. Snapshots
of steady TX registers were identical to RAIL's.

An on-target sampler in RAILtest (`railtest-sampler/app.c`, one row per
register change, every ~11 µs) showed RAIL's order:

1. Hold the slices in reset (`PACTRL.TXPASLICERST`).
2. Select slice and power manually (`MODEM.PADEBUG` =
   `ENMANPAPOWER|ENMANPASELSLICE`).
3. Power the regulators with their bleeders (`RAC.TX` = 0xC401_3001).
4. After 11–40 µs, release the reset and set `PARAMP` and the output stages
   with `TXPAENRAMPCLK` (0xC3C1_F003).
5. After the ramp, drop the clock and the bleeders (0xC3C1_C001).

`xg24::pa_ramp_up` follows this order, with 40 µs and 16 µs steps. The
current and output now match RAIL (see the table below).

### Hardware validation (BRD4187A)

**Setup**

- BRD4187A Rev A01 on a BRD4001A, driven through hwlab: J-Link OB, VCOM,
  and AEM at 3.3 V with DC-DC bypassed.
- nRF52840 802.15.4 sniffer and injector.
- Lab binary `s2-mg24-air` (`tools/efr32s2-lab`, `--features brd4187`,
  linked at `0x0800_0000`): a VCOM console for channel, power, TX bursts,
  RX with Imm-Ack, ED and register dumps.
- Router `examples/efr32mg24-router --features wstk-brd4187`, running above
  a GSDK 4.5.0 `bootloader-uart-xmodem` built for `brd4187c` at
  `0x0800_0000`.
- Before any write, a verified backup of the original flash was taken
  (fully erased as delivered, hwlab artifact
  `jlink-flash-…-852aad42795f.bin`).

**TX current and output, RAIL vs zigbee-rs**

Channel 26, 125-byte frames back to back. Mean current over the burst;
RSSI is the sniffer's reading. The two images were measured back to back.

| | RAILtest | zigbee-rs |
|---|---|---|
| radio off | 4.1 mA | 3.6 mA |
| RX | 10.3 mA | 9.9–10.1 mA |
| TX 0 dBm | 24.8 mA, −53.0 dBm | 23.7 mA, −52.9 dBm |
| TX 10 dBm | 57.3 mA, −43.0 dBm | 56.5 mA, −42.9 dBm |
| TX 15 dBm | AEM saturated, −37.9 dBm | AEM saturated, −37.3 dBm |
| TX 20 dBm | AEM saturated, −33.0 dBm | AEM saturated, −32.9 dBm |

The two images were also measured alternately on channel 20, 4 × 500
frames: −46, −47, −46 and −45 dBm. During one ten-minute window, the
sniffer read both our lab binary and the Router 10–20 dB lower. Other
devices stayed stable and the TX current did not change. The readings
recovered by themselves. This points to the bench environment, but that is
not confirmed.

**Other gates**

| gate | result |
|---|---|
| RX | Received frames injected by the nRF. The trailer RSSI follows RAIL's convention. |
| synthesizer | Locked on all 16 channels for RX and TX. ED scan ran on every channel. |
| Imm-Ack (lab) | The nRF's ACK-wait got 11 of 11 ACKs for frames to the filter address, and none for other destinations. |
| Imm-Ack (Router, interrupt path) | During a rejoin, the coordinator sent 8 ACK-requested frames. All 8 were ACKed 174–181 µs after the frame end (sniffer). HAL: 9 ACKs, 0 late, maximum turnaround 174 µs. The coordinator ACKed our 5 of 5 at 193 µs. |
| Router join | The test network was PAN 0xE2E9 on channel 20 (zigbee-rs-hub with an EmberZNet NCP). With `permit 120` open, the Router joined as 0xB2B0 within 3 s. Commissioning (Trust Center link key) completed, and the interview went through. |
| reset and reflash | The Router resumed 0xB2B0 from the journals without rejoining, both after a reset and after reflashing the application. |

**Router log fix.** The first Router run joined, but the sniffer saw only
21 of 105 coordinator frames ACKed in time. The rest needed MAC
retransmissions, and their ACKs came 0.9–5 ms late. The cause was the
Router's UART logger: it wrote each line inside `interrupt::free`. A
blocking line takes about 9 ms at 115 200 Bd, and masking interrupts that
long delays the FRC interrupt that sends the Imm-Ack. The HAL cannot detect
this delay: it times frames from interrupt entry, so its `late` counter
stayed at 0.

The logger now borrows the UART in a short critical section and writes the
line with interrupts enabled. The run above was made with that fix. The
Router's 60 s summary now also prints the queue-full, ACK-failure and
maximum-turnaround counters.

### 10 dBm (MP) PA: EK2703A

The MP PA was captured and verified with the same method on an xG24
Explorer Kit EK2703A (BRD2703A A02, EFR32MG24B210F1536IM48, 39 MHz,
CTUNE 100). The kit has no current monitor, so the comparison with RAIL
uses the nRF sniffer only. The method, raw dumps and decoded summaries,
including a register-by-register comparison with the HP capture, are in
`tools/efr32s2-lab/captures/xg24-ek2703a`.

**What `efr32mg24-pa10` changes (`xg24::PaPhy`, MP `TX_POWER`)**

| item | MP (B210) | HP (B220) |
|---|---|---|
| RF path | path 1: `RFPATHEN1`, `LNAMIXTRIM1`, `AGCOVERWRITE1.MANLNAMIXRFATT1` | path 0 |
| power | `MODEM.RAMPLEV` per level (RAIL raw + 16..21) and `RAMPCTRL` 0x999/0x888/0x777/0x666; 20 levels from −30.0 to +10.0 dBm | `PACTRL` slice/power, `RAMPLEV` from DEVINFO |
| PA words | `RAC.TX` pre-stage 0xC001_3001, ramp 0xC029_F003, steady 0xC029_C001; `PADEBUG` 0 | 0xC401_3001 / 0xC3C1_F003 / 0xC3C1_C001, `PADEBUG` manual |
| PA trims | `PATRIM0/2/3/5`, `SYTRIM1` (no 20 dBm LO delay), `PATRIM4` unchanged in TX | HP values |
| IR correction | `IRCALCOEFWR0/1` = 0xFFFFFFFF; RAIL ignores the DEVINFO word | DEVINFO word |

The PHY, the channel values (identical on all 16 channels for both dies)
and the per-state values of the shared blocks are the same for both
variants. The power-up order is the same too: slices held in reset during
the pre-stage, then the ramp clock. The runtime calibrations
(`VCOGAIN`, `LPFCTRL2RX/TX`, `VCOCTRL`, `AUXADCTRIM`, `IFADCCAL`,
`LNAMIXTRIM3/4`, `PGATRIM`, `PGACAL`) are the EK2703A die's values in MP
builds, as the BRD4187A die's are in HP builds.

**TX output, sniffer RSSI on channel 26** (back to back, 125-byte frames):

| requested | RAILtest | zigbee-rs |
|---|---|---|
| −30 dBm | −73.3 dBm | −73.0 dBm |
| −10 dBm | −51.1 dBm | −51.0 dBm |
| 0 dBm | −42.0 dBm | −42.0 dBm |
| 5 dBm | −37.0 dBm | −36.5 dBm |
| 10 dBm | −32.0 dBm | −31.9 dBm |

**Other results**

Lab binary `s2-mg24-air --features ek2703`:

- 12 of 12 Imm-Acks to the nRF's ACK-wait, and none for another address;
- RX RSSI −49 dBm for the nRF at 0 dBm;
- energy detect on all 16 channels with the synthesizer locked;
- 44 beacons for one beacon request on channel 15.

Router `--features ek2703a`, running above a GSDK `bootloader-uart-xmodem`
built for `brd2703a`:

- **Join.** It joined PAN 0xE2E9 on channel 20 as 0x3A55, with the BRD4187A
  Router 0xB2B0 as its parent. This also exercises the xG24 HP Router as a
  parent. Commissioning completed.
- **Imm-Acks.** The sniffer saw 22 of 24 ACKed 158–169 µs after the frame
  end, and the HAL reported a maximum turnaround of 165 µs. The two
  misses were the first two copies of one frame right after the join,
  during the journal writes.
- **Coordinator ACKs.** The coordinator ACKed 30 of 30 of our frames.
- **Resume.** The Router resumed 0x3A55 from its journals after a reset.

**Not verified on hardware:**

- the 10 dBm tables on an RF-BM-MG24B1 (A410) or any other MP die;
- MP TX current (the EK2703A has no AEM);
- the 38.4 MHz tables (Dongle-M);
- CCA against a jammer (the Routers' CSMA ran without failure);
- EM2 and DC-DC;
- end-device children and Frame Pending through an xG24 Router (the EK2703A
  joined through the BRD4187A as a router child);
- OTA and GBL upload;
- long-duration runs;
- the NCP on an MP die (it ran on the BRD4187A, see below).

### ZRHP NCP on the BRD4187A (WSTK VCOM)

**Setup**

- NCP `examples/efr32mg24-ncp --features wstk-brd4187` on the BRD4187A,
  above the GSDK 4.5.0 `bootloader-uart-xmodem` (brd4187c).
- Host: zigbee-rs-hub `hubctl` (main `65cff43`, unmodified) on macOS,
  `--backend zrhp` over the WSTK VCOM, 115 200 8N1, no flow control.
- EK2703A Router as the joining device; nRF52840 sniffer on the test
  channel; WSTK AEM for current.
- Test network: formed by the hub on channel 25 (PAN 0x9BBF). The lab
  networks on channels 15, 17 and 20 were not touched; the EK2703A Router
  was first removed from the network it was in (Mgmt_Leave from that hub).

**Bring-up fixes**

- **Boot loop.** The first NCP image never ran: reset cause HardFault on
  every boot. A vector catch showed a precise BusFault on `TIMER0_CNT`
  (`0x4004_8024`). The session cookie reads `efr32s2_hal::delay::cycles()`
  before anything had clocked TIMER0. `start()` now calls
  `delay::enable_cycle_counter()` right after `init_clocks`. The fix applies
  to every MG24 NCP image; none had run before.
- **VCOM line rate.** The BRD4001A board controller runs its side of the
  VCOM UART at its own configured rate (default 115 200), not at the host's
  CDC rate. With the product default of 460 800 the NCP counted 1,443 framing
  errors in a few seconds and the host decoded garbage; at 921 600 the host
  received nothing. At 115 200 the link is clean. The `wstk-brd4187` NCP
  therefore starts at the board's `VCOM_BAUD` (115 200) and accepts only
  that rate in `SYS_SET_UART`, so a host cannot switch the link to a rate
  that strands it. hwlab cannot reach the board controller's admin console,
  so its configured rate was not read.

**Results**

| gate | result |
|---|---|
| session | `hubctl status`: session up at 115 200 in 68 ms; NCP fw 0.1.0, chip 12, board "BRD4187 on BRD4001A", capacities as `ncp-capacity` (256 TC devices). |
| formation | `network/form` channel 25: PAN 0x9BBF, ownership `owned`. |
| join | After a reboot, the EK2703A Router joined: Transport-Key (verified on the sniffer capture against ZigBeeAlliance09), unique TCLK (Request-Key → Verify-Key → Confirm-Key SUCCESS), Device_annce. |
| interview | Basic, active endpoint and simple descriptor (HA Range Extender, Basic + Identify). See the join note below. |
| ZCL | Read of 8 Basic attributes, Identify (10 s, Default Response SUCCESS), Configure / Read Reporting Configuration on Identify Time (SUCCESS). |
| backup / restore | `network/backup` (open coordinator backup, 1 device, 1 link key), then `network/restore` of the same backup with `force`: NWK frame counter start 31,504, APS 32,528 (backup limits + 10,000). The Router stayed joined; its Link Status continued through the restore. Unicast to it then failed (see the restore note); with the hub fix, a backup/restore cycle kept it reachable. |
| leave / rejoin | `devices/remove`: Mgmt_Leave, the Router left (`flags 0`) and factory-reset, re-armed steering after its 5-minute wait, joined again as a new address with permit open, and the hub interviewed it completely. |
| NCP reset | nRESET through the AEM connection (hwlab ADR 0042): the host saw `peer reset (Restarted)`, `NWK_INIT` with its floors and network state 2, resumed in 265 ms. Three APS requests in flight were reported as "link reset, outcome unknown". Ownership stayed `owned`. |
| frame counters | NCP NWK counter on the air: 20,590 before the restore, 31,504 after it; 31,527 before the reset, 41,744 after it. Never reused. |
| reflash | Reflashing the NCP application kept the network (journals). |
| link | `hubctl soak` (echoes) at 115 200, 10 min: 30,106 echoes, 0 failed, worst 117 ms, no resync, one session; the host retransmitted 66 of 60,304 frames. After the final reflash, 5 min: 15,033 echoes, 0 failed, worst 62 ms; host 29 retransmissions of 30,121 frames; the NCP counted 6 UART overruns (7 bad frames) and retransmitted 17. ARQ recovered every frame. |

**Current (AEM, 3.3 V, DC-DC bypassed).** The coordinator keeps its
receiver on: 10.2 mA mean with the network up and idle (7.0–7.6 mA minimum,
24.4 mA peaks at 0 dBm TX), the same before and after a reset. The nRESET
pulse read 0.48 mA and the NCP was back at 10 mA 23 ms after release.

**UART overruns: cause and the LDMA receiver.** An overrun means the
USART0 receive interrupt waited longer than about two character times
(~170 µs at 115 200). A temporary probe build timestamped every `RXOF`, every
`FRC` interrupt longer than 100 µs and every flash operation (TIMER0 cycles,
read over SWD). Under 90 s of ZCL reads (12 overruns), every overrun was
serviced 3 µs after an `FRC` interrupt that had run 646–661 µs:

- **The radio interrupt.** With `isr-ack` the `FRC` handler sends the
  Imm-Ack itself: turnaround, 352 µs of ACK airtime, then TX done, all inside
  the interrupt. The USART0 RX interrupt has the same NVIC priority and
  cannot preempt it. Each ZCL read brings about two acknowledged frames (APS
  ACK and response), about 1,300 such interrupts in 90 s.
- **Flash writes**, the second source. A page erase took 12.5 ms and
  stalls every interrupt handler fetched from flash. None fell inside that
  run, but frame-counter reservations and Trust Center records write the
  journals during traffic.

The receiver now runs on the LDMA, with no receive interrupt
(`efr32s2_hal::usart::host`). Channel 0 copies `RXDATA` into an 8 KiB ring of
four 2 KiB quarters (`XFERCNT` allows at most 2048 transfers per descriptor).
Each quarter sits behind a SYNC descriptor that closes the previous quarter's
`SYNCTRIG` gate and waits for its own. The reader reopens a quarter once it
has read the next one. The DMA therefore leads by at most 6 KiB, never
overwrites unread bytes, and needs no interrupt. A reader that falls behind
stalls the DMA, and the USART `RXOF` then counts it. The read position comes
from the channel's `DST`, or, at a gate, from `LINK`.

Same image and traffic, interrupt receiver (master) against LDMA, 90 s:

| counter | interrupt RX | LDMA RX |
|---|---|---|
| USART overruns (HAL) | 8 | 0 |
| NCP `rx_bad_frame` / `rx_overflow` / NAKs sent | 12 / 8 / 10 | 0 / 0 / 0 |
| host retransmissions (NAK-driven) | 13 (10) | 29 (0) |

**Soak with the LDMA receiver**: 35 min, `hubctl` v0.4.1, 115 200, the
EK2703A Router. Three phases:

1. 15 min of back-to-back Basic reads with bursts of 16 every 30 s
   (6,073 reads, 0 failed, p99 154 ms);
2. a leave and rejoin: `devices/remove`, the Router's 5-minute re-arm,
   permit join, a fresh join and a complete interview;
3. 15 min of reads with bursts of 32 every 20 s.

| counter | result |
|---|---|
| USART overruns and framing errors (HAL, since boot) | 0, 0 |
| ZRHP framing | NCP `rx_bad_frame` 0, `rx_overflow` 0; NAKs 0 both ways |
| traffic | host → NCP 50,857 frames (1.48 MB); NCP → host 50,529 frames (2.16 MB) |
| journal writes during traffic | 13 frame-counter reservations, plus the Trust Center writes of the leave and rejoin |
| retransmissions | NCP 724, host 464, each matched by duplicates on the other side (724, 461): timing, not loss |

The retransmissions already happen with the interrupt receiver (31 from
the NCP in the 90 s A/B run). Without flow control at 115 200, an ACK waits
behind the frames already queued (the 2 KiB TX ring drains in ~180 ms).
The adaptive RTO, 25 ms at its floor, expires first.

In phase 3, 219 of 6,465 reads failed with `not_delivered`:

- 204 `busy_responses`: 32 concurrent requests exceed the NCP's 16 APS
  in-flight slots;
- radio-side APS failures, `aps_tx_failed` 1,588 of 7,038.

The APS failure rate is the same with both receivers (27/638 and 32/641 in
the A/B run); it is not part of this change.

**RAM and stack.** The NCP's static RAM ends at `0x2002_A2D4`
(`.bss` 168,660 B, with the LDMA receiver); the linked stack is 89,388 B. The stack high-water mark
was not measured (the image has no stack paint).

**Join note: stale Trust Center address after a Leave.** The EK2703A Router
had been removed from another network with Mgmt_Leave (no rejoin); the
router app then factory-resets (other apps, such as the sleepy sensor, only
leave). On its next steering it joined our network, but rejected our
Transport-Key as unauthenticated, waited, reset and joined again in a loop.
The frame was valid (MIC verified on the host with the ZigBeeAlliance09
key-transport key). The cause is in the stack: neither a Leave nor a
factory reset cleared `aib.aps_trust_center_address`, so the old network's
Trust Center address made our Trust Center look like an impostor. A reboot
cleared it, and the Router then joined at once.

Every Leave without rejoin (local, Mgmt_Leave, Trust Center Remove-Device,
a failed TCLK exchange: the runtime's shared teardown), an abandoned or
failed initial commissioning in BDB steering, and a factory reset now clear
the address and the designated-coordinator flag (`Aib::forget_trust_center`;
the APS frame counter is kept). Regression tests:

- `mgmt_leave_without_rejoin_accepts_another_trust_center` (runtime:
  Mgmt_Leave, then a Transport-Key from another Trust Center is accepted);
- `factory_reset_forgets_the_trust_center_but_keeps_the_aps_counter`;
- `abandoned_initial_commissioning_forgets_the_trust_center`.

The fix is host-tested. On hardware, the factory-reset version let the
Router leave and rejoin the same Trust Center without a reboot. A Leave from
one Trust Center followed by a join to another, without a reboot, was not
repeated.

The hub interviewed the Router once, on its first (failed) join, and marked
the interview partial; later joins of the same device do not restart it.
`devices/reinterview` completed it.

**Restore note: a router child came back as a sleepy end device.** The
Router joined through the coordinator, so the NCP lists it as a child with
the FFD capability. The hub's backup (open coordinator backup) wrote it as
`is_child: true`, and that format has no capability, so the hub's restore
gave it an rx-off end-device child entry (capability 0x80). From then on the
NCP held every unicast to the Router for a data poll a router never sends:
the interview hung, ZCL requests got no APS ACK, and the Mgmt_Leave of a
`devices/remove` never went on air (sniffer), leaving the Router in the
network after the hub had forgotten it. Restoring with the Router marked
`is_child: false` brought unicast back. The NCP and the radio behaved as
told; the hub fix (zigbee-rs-hub PR #100: router children are exported as
plain devices) was verified on this setup with a backup, a restore
(`children: 0`, one link key) and a ZCL read right after. An rx-on end
device child still comes back rx-off: the backup format cannot say so.

**Not verified on hardware:**

- attribute reports through the NCP. Configure Reporting on the Router's
  Identify Time succeeded, but the Router never sent a report: its runtime
  only reports clusters the profile collects, and Basic and Identify are
  built in (open item in the runtime);
- the RF-BM-MG24B1 / ESP32-S3 host link at 460 800, and 921 600;
- `SYS_RESET` into the Gecko bootloader and GBL upload;
- children (end devices) of the NCP, many devices, long-duration runs;
- the NCP on an MP die or with a 38.4 MHz crystal.

### Boards

| | RF-BM-MG24B1 dev board | SONOFF Dongle-M | BRD4187A on BRD4001A (radio lab) |EK2703A (MP radio lab) |
|---|---|---|---|---|
| part | EFR32MG24A410F1536IM48-B (+10 dBm) | EFR32MG24A420F1536IM48 (+20 dBm), SONOFF spec page | EFR32MG24B220F1536IM48 (+20 dBm; DEVINFO, although GSDK lists BRD4187A as A020) |EFR32MG24B210F1536IM48 (+10 dBm; DEVINFO) |
| HFXO | 39 MHz on the module; CTUNE 92 (RF-star datasheet §4, **unmeasured**) | 38.4 MHz (stock image PHY); CTUNE: `MODXOCAL` → `MFG_CTUNE` → 100 (community manifests), unit value **not read** | 39 MHz; no `MFG_CTUNE` on this unit, so the BRD4187C default CTUNE 95 (**unmeasured**) |39 MHz; no `MFG_CTUNE`, so the GSDK board CTUNE 100 (**unmeasured**) |
| host UART | USART0 TX PA05 (pin 12), RX PA06 (pin 13) → ESP32-S3 SuperMini; RTS PA07 / CTS PA08 claimed but unused | USART0 routed to TX PC01, RX PC02 → ESP32 GPIO13/GPIO17, 115 200, no flow control (community builds use EUSART1 on the same pins; GSDK routes USART0 to PC01/PC02 on the BRD4187C; **unverified** on a unit) | USART0 to the WSTK VCOM, TX PA08, RX PA09, VCOM enable PB00; logs and the NCP's ZRHP link at 115 200 (the board controller's VCOM rate; 460 800 and 921 600 fail) |USART0 to the J-Link OB VCOM, TX PA05, RX PA06 (RTS PA08 / CTS PA09 unused); logs at 115 200 |
| reset / boot | RESET_N (pin 36) open-drain from S3 GPIO6 (hub profile `S3_MG24_DEV`) and the J-Link; no boot-entry line (Gecko via `SYS_RESET` mode 1 once installed) | ESP32 GPIO15 → RESET (active low); GPIO12 high during reset → stock Gecko Bootloader v2.04.02 (probed through the ESP32; the MG24 pin of that line is unknown) | WSTK J-Link (hwlab). GSDK `bootloader-uart-xmodem` (brd4187c); BTN0 (PB01) held at reset enters it |J-Link OB (hwlab). GSDK `bootloader-uart-xmodem` (brd2703a); BTN0 (PB02) held at reset enters it |
| debug | SWDIO PA02, SWCLK PA01, SWO PA03 (never claimed) | SWD pads unknown | WSTK J-Link OB, AEM current measurement |J-Link OB; no current monitor |
| other | 32.768 kHz LFXO, PCB antenna, no LED/button | no MG24 LED/button documented; RF path and LFXO unknown; stock PA configuration highest PA, 3300 mV, 10 µs (GSDK defaults); our default +10 dBm | LED0 PB02, LED1 PB04, BTN0 PB01, BTN1 PB03; 32.768 kHz LFXO; on-board 2.4 GHz antenna |LED0 PA04, LED1 PA07, BTN0 PB02, BTN1 PB03; 32.768 kHz LFXO; on-board 2.4 GHz antenna; mikroBUS, Qwiic |

A valid module calibration (`DEVINFO.MODXOCAL`) or a programmed
`MFG_CTUNE` token always wins over the board default (GSDK's order); the
DEVINFO module fields are erased on the BRD4187A and EK2703A dies, so their
images use the board defaults. Every MG24 image logs the CTUNE, its source
and the IEEE source at boot (the NCP on the ZRHP LOG endpoint).

### Memory map (every board, both roles)

```text
0x0800_0000..0x0800_6000  Gecko bootloader (kept, never written)      24 KiB
0x0800_6000..0x0815_8000  application = GBL payload                 1352 KiB
0x0815_8000..0x0818_0000  journals: identity, APS, GP, children,
                          Trust Center (NCP), security                160 KiB

0x2000_0000..0x2000_0004  Gecko BootloaderResetCause_t (outside the image)
0x2000_0004..0x2004_0000  image RAM, stack at the top             256 KiB - 4
```

On the BRD4187A the Gecko bootloader is the GSDK 4.5.0
`bootloader-uart-xmodem` sample built for `brd4187c`. It is 11.6 KiB and
uses its own reset-cause word at `0x2000_0000`. The lab binaries in
`tools/efr32s2-lab` (`wstk-brd4187`) are linked at `0x0800_0000` and replace
it; flash the bootloader again before going back to the Router.

The journals are those of the ESP32-H2 and CC2652P7 NCPs (spec 8.9). The
router uses the same security, child, APS and GP windows, so a reflash
between the NCP (role tag `0x5203`) and the router (`0x5201`) keeps the
frame-counter reservations and erases the other role's tables. A GBL only
erases the pages it programs, so the journals survive a firmware update.
The stock EmberZNet NVM3 (`0x0817_6000..0x0817_E000`) sits inside the
journal window; the first boot treats it as foreign and erases it, together
with the stock network key, frame counters and tables (host-tested, see
[SONOFF Dongle-M over the stock firmware](#sonoff-dongle-m-over-the-stock-firmware)).
An NVM3 page header at any page start of the window makes the whole window
foreign even when older records of ours survive above the stock NVM3 (after
a return from a stock image). **On a Dongle-M in service, the hub must take
an EZSP network backup before flashing**: nothing can be recovered from the
erased NVM3. Before going back to EmberZNet, erase the whole window
`0x0815_8000..0x0818_0000`: upload `efr32mg24-nv-clear.gbl` (CI artifact,
`efr32-gbl clear`) or erase it over SWD. NVM3 does not expect our journals
in its pages, and a clear of the stock NVM3 pages alone (the 32 KiB
`nvm3_clear` GBL, `0x0817_6000..0x0817_E000`) leaves our records in the
top page `0x0817_E000..0x0818_0000`. The 40 KiB `nvm3_clear` variant
(`0x0817_6000..0x0818_0000`) also covers that page.

### Firmware update path: Gecko bootloader kept

The NCP keeps the resident Gecko bootloader and does not build in `FW_*`:

- `SYS_RESET` mode 1 writes the BOOTLOAD reset cause and resets into Gecko
  (`gecko_bootloader::reboot_to_bootloader`); on the Dongle-M the ESP32 can
  also enter it through GPIO12 + RESET. Gecko speaks XMODEM at 115 200.
- `tools/efr32-gbl app --elf <ELF> --out <prefix>` writes the raw image
  (`.bin`, vector word 13 pointing at the `ApplicationProperties_t`, as GSDK
  startup files do), an Intel HEX for J-Link, and a plain GBL3. Its output is
  byte-identical to Simplicity Commander 1v22p1 `gbl create` (golden tests
  with Commander output, and the real NCP image compared locally).
- Every packaged image passes the start-up checks of the Gecko
  bootloader's `bootload_verifyApplication` (`efr32_gbl::gecko_boot_check`,
  also run by `efr32-gbl parse`): initial SP in SRAM, reset vector in the
  application space, vector word 13 → `ApplicationProperties_t` (magic,
  struct version 1.x), signature type NONE. The packager refuses an ELF
  with anything outside `0x0800_6000..0x0815_8000`, so no GBL writes the
  bootloader or the journals.
- Signing: the stock Dongle-M GBLs (EmberZNet 7.4.5, OpenThread, multi-PAN)
  and the community Dongle-M GBLs carry no signature or encryption tag, and
  SONOFF's flasher accepts the community images, so the stock bootloader does
  not enforce signed GBLs. Our image declares signature type NONE. A
  bootloader with secure boot enabled would refuse it; we did not inspect a
  Dongle-M bootloader's configuration.
- NCP GBLs carry the ZRHP metadata record (`"ZRHP"`, version 1, 8-byte build
  ID, firmware version; host-protocol 7.9, *Gecko bootloader path*). The
  packager stores the build ID (SHA-256 of the image with the field zeroed)
  in the image's `.zrhp_build_info` record, and the NCP reports it in
  `SYS_NEGOTIATE`, so the hub can check which build runs after an update.
  The hub refuses GBLs with bootloader or SE-upgrade records.
  The record is not the JSON that Nabu Casa's Universal Silabs Flasher (used
  by SONOFF's flasher) parses; whether those tools accept a non-JSON
  metadata record is untested.

**A/B on the radio chip (decision N6).** Flash would allow it: a 16 KiB
zrs-boot loader chained above Gecko (as on the ZBT-1) plus two 664 KiB slots
fits the 1352 KiB application window, and the NCP image is 441 KiB today.
It is not the default because the Dongle-M already has a host-driven,
unbrickable recovery path (ESP32 GPIO12 + RESET into Gecko), Gecko is the
path SONOFF's flasher and the community tools use, and a chained loader would
make every update depend on our loader's correctness on a device we cannot
yet test. The journal window is placed so that a later A/B layout below it
would not move it.

### SONOFF Dongle-M over the stock firmware

The Dongle-M NCP image (`efr32mg24-ncp-sonoff-dongle-m.gbl`, CI artifact
`efr32mg24-firmware` with `SHA256SUMS`) is meant to be uploaded through the
unit's stock Gecko bootloader: ESP32 GPIO12 high during a GPIO15 reset,
then XMODEM at 115 200 (menu `1. upload gbl`, then `2. run`). The table
is what the image is checked against; the first run on a unit is below it.

What the stock side is (stock GBL `zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`,
SHA-256 `44b81c1e…a311fa`, and a read-only probe of one unit through the
ESP32):

| item | stock | our image |
|---|---|---|
| bootloader | Gecko Bootloader v2.04.02 (GSDK 4.4.x), XMODEM menu | kept, never written |
| GBL | v3 header `0x0300_0000`, type 0; HEADER, APPLICATION, one ERASEPROG at `0x0800_6000`, END | the same records in the same order, plus the ZRHP METADATA record (a tag the GBL parser skips); `efr32_gbl::write` reproduces the stock file byte for byte from its records (CI) |
| application | `ApplicationProperties_t` v1.2 at `0x0803_C148`, type ZIGBEE, signature NONE, version 1, product ID zero | v1.2, type ZIGBEE, signature NONE, version `0x0001_0000`, zigbee-rs product ID |
| PHY | GSDK 4.5.0 38.4 MHz 802.15.4 configuration, byte for byte | the same (`hfxo-38m4hz`) with RAIL's per-channel words |
| PA | `2P4GIG_HIGHEST`, 3300 mV, 10 µs ramp; power set by the EZSP host | 20 dBm PA table of the BRD4187A; fixed +10 dBm (no `NWK_SET_TX_POWER` on the MG24 NCP yet) |
| NV | NVM3 at `0x0817_6000..0x0817_E000` | journals at `0x0815_8000..0x0818_0000`: the first boot finds NVM3 page headers (or no valid security record) and erases the whole window |
| IEEE | EZSP EUI-64 `B0E8E8FFFE5D4D03` on the probed unit | `MFG_CUSTOM_EUI_64` if programmed, else DEVINFO: the same IEEE as stock either way |
| CTUNE | `MODXOCAL` → `MFG_CTUNE` → 100 | the same order; value and source logged at boot |
| host UART | EUSART1 PC01/PC02, 115 200, no flow control | USART0 on PC01/PC02 at 115 200; the LDMA receiver uses the USART0 request, independent of the pins |

Checks behind the table:

- The GBL parser requires header major version 3 (`0x03xx_xxxx`), and
  signature or encryption flags only when the bootloader is built to
  require them; the stock file and ours carry the same plain header. The
  application-version and
  anti-rollback checks are optional bootloader components: with the
  version check built in, a different product ID is refused while the
  APPLICATION record is parsed, before any program record, so a rejected
  upload leaves the stock application in place. Whether the stock
  bootloader has it is unknown.
- `products/efr32mg24` `stores` host tests: NVM3 pages at the stock NVM3
  location, pseudo-random bytes (also with our record magic at every page
  start, which the security journal then rejects), and all-zero flash are
  all foreign (whole window erased, then a first record commits and
  survives the next open); a valid journal and the other
  windows are preserved; another role's journal keeps its counters.
- The return path "ours → 32 KiB `nvm3_clear` → stock EmberZNet → ours"
  leaves valid records of ours in the top page `0x0817_E000`. Resuming them
  would bring back the pre-stock network with frame-counter reservations
  behind what the network saw on the stock firmware (devices would drop the
  frames as replays). The opening policy therefore checks every 8 KiB page
  start of the window before the security journal. Each is a slot start of
  one of our journals (every slot size divides the page, asserted at
  compile time), so it holds erased flash or an ASCII `ZB..` record magic,
  possibly torn by a power loss while that word was programmed (a torn
  word keeps every 1 bit of the magic). Any other first word is foreign and
  the whole window is erased (`Opened::Foreign`), whatever the foreign
  format. The boot log names the first foreign page and its first word,
  and says `NVM3 (stock EmberZNet)` when a first word carries the GSDK
  NVM3 page magic `0xB29A` (a label only). A first version recognised NVM3
  by its page header (magic, version, Berger-coded erase counts as read
  from GSDK 4.5.0 `libnvm3`), and a Dongle Max run seemed to show that real
  pages did not match it (see the fourth run below: they do; that log came
  from a second boot). Host tests: the return path with NVM3-shaped and
  with unrecognised stock pages, a foreign first word at every page start
  beside a valid journal, a rolled-over journal set with a torn page-start
  magic that stays ours, and the hardware captures below.
- **Ground truth (BRD4187, SWD dumps through hwlab).** The board's flash
  above its Gecko bootloader was erased, SONOFF's EmberZNet 7.4.5 Dongle-M
  NCP (`zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`, SHA-256 `44b81c1e…`,
  converted to its `0x0800_6000` program image) was programmed and run
  for about 90 s, and `0x0804_4000..0x0818_0000` was read back. The stock
  NVM3 is exactly the instance the image declares (`nvm3_defaultInit` at
  `0x0804_2388`: base `0x0817_6000`, 32 KiB, cache of 200 objects): four
  8 KiB pages at `0x0817_6000`, `…8000`, `…A000` and `…C000`, each starting
  with `B29A0001 D0000001 0FFFFFFE FFFFFFFF FFFF903C` (magic and version,
  erase count 1 and its complement with Berger codes, page state good,
  format word), plus 1,856 bytes of objects in the first page. Nothing else
  above the image was written. It lies inside our window, so
  `efr32mg24-nv-clear.gbl` and the 40 KiB `nvm3_clear` both clear it; the
  32 KiB `nvm3_clear` leaves our top page, as above. Our current NCP image
  programmed over those pages logged `NVM3 (stock EmberZNet) at 0x08176000
  (first word 0xB29A0001): NV window erased` (read from its RAM log ring
  over SWD) and left the window erased. A commissioned NCP of ours on the
  same board starts its pages with `ZBAT` (`0x0815_C000`), `ZBCT`
  (`0x0816_6000`), `ZBTS` (`0x0817_0000`) and `ZBSS` (`0x0817_8000`), all
  other page starts erased. Both captures are host-test vectors
  (`products/efr32mg24` `stores`). The board was restored from its
  verified backup afterwards.

First run on a unit (one Dongle Max, image from `e96cd867`, build ID
`a603b22b88fc5c2b`, reported by the hub's HIL run): it booted from the stock
bootloader, kept the stock IEEE (DEVINFO EUI-64; the unit has neither
`MFG_CUSTOM_EUI_64` nor `MFG_CTUNE` programmed, so CTUNE came from
`MODXOCAL` or the default), ran 22 minutes at 460 800 Bd without a reset,
and took the hub's restore of the unit's EmberZNet backup (channel 20, the
network key, NWK counter from 153 428, one child, no link keys). An end
device's NWK-secured rejoins were received and decrypted, so reception at
38.4 MHz works. Its Trust Center then denied each rejoin and dropped the
child: an EmberZNet backup lists devices without keys (the stack derives
them from `hashed_tclk`), and the Trust Center refused secured rejoins of
devices missing from its table. Fixed in the shared stack (ZRHP spec 7.5):
secured rejoins are admitted, restored devices are registered (with their
EmberZNet-derived keys when the backup carries `hashed_tclk`; otherwise
with network-level membership only, under a private key with no APS
standing), and the child entry is kept. Transmission to a device, the output and the CTUNE
offset remain open for the next run.

Second run (build `0907cfddf5e63609` from `0607d319`, reported by the hub's
HIL run, with sniffer): boot log `NV journals preserved`, `HFXO 38400000 Hz
CTUNE 100 (BoardDefault), TX 10 dBm, IEEE B0E8E8FFFE5D4D03 (Devinfo)`. The
radio works both ways at 38.4 MHz: Link Status at −33 dBm / LQI 236 at the
sniffer, responses decrypted and MAC-ACKed by the device. The sleepy end
device's secured rejoin still failed, for a MAC reason:

1. the Rejoin Request is decrypted and admitted;
2. the device polls 4 and 6.5 ms later; the Rejoin Response is only queued
   about 15 ms after the request, so the interrupt acknowledges both polls
   with FP = 0;
3. the stack then serves the stale first poll and sends the response 12 ms
   after that ACK. The device's radio acknowledges it while idle and its
   stack drops it, as a sleepy receiver must after FP = 0. Later polls find
   nothing, and the device retries after ~0.45 s.

The only ACK with FP = 1 in the capture went to the Trust Center rejoin
that followed, whose unsecured response was queued before the poll; that
response was accepted. That rejoin then waited for a Transport-Key the
Trust Center cannot send under an unknown key (the restore had no
`hashed_tclk` root), and the child aged out as unauthenticated, as
designed. Fixed by the FP = 0 flag above (host tests: the HAL record
sequence with more children than the table holds, and the NCP serving the
capture's sequence). The device's side
behaves as specified.

Third run (same build, the EZSP backup restored with its `hashed_tclk`
root): the two secured rejoins failed exactly as in the second run, polls
acknowledged with FP = 0 before the response was queued. The device fell
back to a Trust Center rejoin; with the root the Trust Center derived the
device's EmberZNet key and sent the Transport-Key, and the device sent
Device_annce and its ED Timeout Request (answered, enum 14) and polled
every 5 s; the hub's interview (about 30 s), reads and reports followed,
and a 15-minute watch was clean (HIL report). The derived key therefore
works on hardware. The fallback is not needed: with the FP = 0 flag the
first secured rejoin completes, with or without a root. Two more findings:

- The NCP's log named the six Device_annce frames `UNKNOWN_ZDP 0x0013`.
  Only the log name was missing: the NCP forwarded each to the host as an
  APS data indication (spec 7.4; over that window the NCP's `aps_rx`
  counter rose by six, matching them), and a host test checks it. The log
  now names announcements.
- Every received frame reached the host with RSSI −128: the Series 2 PHY
  dropped the per-frame RSSI (the RX trailer byte after the RSSI offset)
  that the HAL reads and derives LQI from. The PHY now passes it on; the
  ESP32 and CC26x2 PHYs already did.

Fourth run (build `d99fb789c9f69dd4` from #23 at `163baae4`, restore
without a root, sniffer capture): the secured rejoin completed on the
first try. Two polls were acknowledged with FP = 0 and served nothing; the
next one got FP = 1 and the Rejoin Response 6 ms later. Device_annce and
the ED Timeout Request followed within 12 ms, with no Trust Center rejoin
and `joins_denied` 0, and indications carried RSSI −23 to −31 dBm. The
same run found two faults, both fixed in #23 and pending the next run:

- The boot log said `blank/foreign NV erased`, which looked as if the
  NVM3 page-header check had missed the EmberZNet 7.4.5 pages. It had not
  (ground truth above). The hub resets the NCP when it starts: the NCP's
  first boot after the GBL erased the stock NVM3, its log record waited in
  RAM for the hub's LOG credit and was lost with that reset, and the second
  boot found the window blank. Both logged boots of that run show an NCP
  uptime of 271 ms about 2.8 s after the hub's own boot. The opening policy
  is now independent of the foreign format anyway; to see the first boot's
  record, the hub must take the NCP's log before it resets the NCP.
- Every `network/restore` of the hub made the NCP broadcast a NWK Leave
  (rejoin 0) before installing the network: the wipe ran the BDB factory
  reset, which leaves the network first. Harmless for the sleepy device,
  but routers drop a coordinator they hear leave. Host wipes are now local
  (`BdbLayer::factory_reset_locally`): `NWK_LEAVE_RESET` without flags
  bit2, the restore rule 1 wipe, a restore over a stored network and
  `SYS_RESET` mode 2 transmit nothing; only `NWK_LEAVE_RESET` with bit2
  broadcasts the Leave (host test over ZRHP, which fails with the old
  wipe). This applies to every ZRHP NCP (ESP32, CC26x2, EFR32MG24).

Notes for flashing over the stock image:

- On first boot the stock network keys, frame counters and tables are gone.
  Take an EZSP network backup with the hub before flashing and restore it
  over ZRHP; because the IEEE is the stock one, the devices keep their Trust
  Center address. The MG24 NCP has no `SYS_SET_IEEE` (no `NcpPlatform`),
  so a backup from another coordinator cannot be restored with its IEEE.
- `SYS_RESET` mode 1 writes `0xF00F0202` (BOOTLOAD reason, valid signature)
  to `0x2000_0000` and resets, the EmberZNet mechanism; untested on any
  MG24 so far. The ESP32's GPIO12 + RESET path stays the recovery path.
- Going back to EmberZNet: upload `efr32mg24-nv-clear.gbl` (erases
  `0x0815_8000..0x0818_0000` only, no application record, so no version or
  product-ID check applies), then the stock GBL. Run on a Dongle Max
  (stock Gecko Bootloader v2.04.02, the hub's flasher: written and
  verified, the stock image then ran and took the EZSP restore).

First run on a unit (the hardware gate): the boot log line
`HFXO 38400000 Hz CTUNE … (source), TX 10 dBm, IEEE … (source)`, raw TX
and RX against the stock image with the nRF sniffer (frequency offset,
output, RSSI), ACK timing, formation and a join, then RAIL's per-state
values at 38.4 MHz against the shared analog states.

### Size and RAM (release, `nightly-2026-03-23`)

| image | file-backed flash | `.bss` | linked stack |
|---|---:|---:|---:|
| NCP, RF-BM-MG24B1 (full `ncp-capacity`, 256 TC devices) | 456,956 B | 168,660 B | 89,388 B |
| NCP, Dongle-M | 462,272 B | 168,660 B | 89,388 B |
| NCP, BRD4187 on a WSTK | 462,292 B | 168,660 B | 89,388 B |
| Router, RF-BM-MG24B1 (GP Proxy Basic) | 345,480 B | 37,140 B | 224,492 B |
| Router, Dongle-M | 345,804 B | 37,140 B | 224,492 B |
| Router, BRD4187 on a WSTK | 345,744 B | 37,136 B | 224,496 B |
| Router, EK2703A | 346,572 B | 37,136 B | 224,496 B |

The xG24 radio tables and PA sequence added 2.5–2.8 KB of flash to every
MG24 image compared with `15ee9550`, the build-only base without a radio.
Static RAM and stack did not change. The 10 dBm PA support added about
130 B (Router) and 220 B (NCP) to the RF-BM-MG24B1 images. The BRD4187 and
Dongle-M images are byte-identical with and without it. The EmberZNet
restore support (secured-rejoin admission with private unknown keys,
hashed-TCLK root and derivation, `SEC_DERIVE_EZSP_KEY`, the
application-key timeout) and the held boot log added about 3.7 KB to the
Dongle-M and BRD4187 NCPs against `e96cd867` (Dongle-M 457,108 → 460,784 B
file-backed), with the same `.bss` and linked stack; the Router changed by
4 B. #23 (`0607d319` → this change: the per-poll FP = 0 flag, the
per-frame RSSI, the ZDO log names, the NV page-start check with its boot
log line and the local wipe) adds 1,488 B of flash to the Dongle-M NCP and
1,096 B to the EK2703A Router, with the same `.bss` and stack. The flag
rides in bit 7 of the HAL RX queue record's and of `PhyRxFrame`'s length
byte, so it needs no RAM: the first version's 8-entry record table cost
16 B of `.bss` (in `.data` it had moved the MG22 Router's 512-byte-aligned
`.bss` past the end of SRAM), and a separate `PhyRxFrame` byte pushed the
TLSR8258 Router's `build_device` frame over its 5,120 B stack gate.

The NCP uses 33 % of the application window and 66 % of SRAM for static
data. The compact capacity tier is not needed. The recorded fatal resets
cost about 630 B of flash in the NCP and about 400 B in the Router (boot-time
record and log line, the poisoned-engine report, the panic/HardFault/
supervisor paths; the Router's SE check compiles away) and no static RAM or
stack: `.data` now starts at `0x2000_0008` (8-byte aligned), and the end of
static RAM and the linked stack are unchanged because alignment padding
absorbs the shift.

The NCP boot fix (TIMER0 enabled before the session cookie) added 48 B to
every NCP image. Forgetting the Trust Center on every Leave and factory
reset added 40 B to every image (56 B to the Routers, with the first
factory-reset version). Static RAM and stack did not change. The LDMA host
receiver added 368–492 B of flash and 124 B of static RAM to the NCP images
(the 128-byte descriptor chain; the 8 KiB ring replaces the interrupt ring),
taken from the linked stack; the Routers moved by 0 to −16 B.

Enabling the Dongle-M radio (`hfxo-38m4hz`), GSDK's CTUNE and EUI-64
precedence, and the boot log line with their sources added 596–684 B to
the NCP images and 292–416 B to the Routers compared with `8b91c3fc`
(Dongle-M: +608 B NCP, +416 B Router). The NVM3 page-header scan of the
opening policy and its log line added another 320–336 B. Static RAM and the
linked stack did not change.

### Hardware gates

Done on the BRD4187A (xG24 HP PA, 39 MHz):

- **Identification and backup.** The die was identified over SWD (J-Link
  device `EFR32MG24BXXXF1536`, through hwlab). DEVINFO, USERDATA and the
  whole flash were backed up and verified before any write.
- **Radio tables.** Captured from RAIL. Raw TX/RX, Imm-Ack timing, ED, RSSI
  offset and TX/RX current all match RAIL. See
  [Hardware validation](#hardware-validation-brd4187a).
- **Router.** Joined, interviewed and resumed in a zigbee-rs-hub test
  network with an EmberZNet coordinator. It ran above a GSDK Gecko
  bootloader, which started the application through its
  `ApplicationProperties`.

Done on the EK2703A (xG24 MP PA, 39 MHz):

- **Identification and backup.** Same as on the BRD4187A. The factory
  demo is kept in a verified backup.
- **MP PA.** RAIL's power table, transitions and register values were
  captured. TX output matches RAIL from −30 to +10 dBm. RX, Imm-Acks, ED
  and beacons work. See [10 dBm PA](#10-dbm-mp-pa-ek2703a). The current
  could not be compared: the kit has no AEM.
- **Router.** Joined through the BRD4187A Router as its parent and resumed
  after a reset.

Done on the BRD4187A as ZRHP NCP (WSTK VCOM, 115 200), against `hubctl`:

- session, formation, join with unique TCLK, interview, ZCL read and
  command, backup and restore of the same network, NCP reset and resume,
  frame counters monotonic across restore and reset, a reflash keeping the
  network, soaks. See [ZRHP NCP](#zrhp-ncp-on-the-brd4187a-wstk-vcom).

Open:

1. **RF-BM-MG24B1.** Identify the module (`EFR32MG24AxxxF1536`), read
   DEVINFO and `MODULEINFO`, and back up the flash. Then check HFXO start at
   CTUNE 92, the SYSRTC tick and USART0 at 115 200 / 460 800 / 921 600 on
   PA05/PA06. The lab binary for this is `s2-mg24-hello`
   (`tools/efr32s2-lab`, feature `rfbm`, linked at `0x0800_0000` for a
   module without a bootloader). It prints the reset cause, DEVINFO, CTUNE,
   the LFRCO-vs-HFXO ratio and SE TRNG bytes, then echoes the UART.
2. **MP (10 dBm) PA: current and another die.** The MP current is
   unmeasured. Measure the MP TX current against RAIL on a board with a
   current monitor.

   On the RF-BM-MG24B1 (enabled, same radio configuration as the EK2703A),
   the first run should measure three things against RAILtest:

   - the CTUNE frequency offset (datasheet CTUNE 92);
   - the output;
   - the current.

   That run also checks the board-level items: the A410 die, the RF path
   and the antenna.
3. **38.4 MHz crystal (enabled; reception seen on one unit).** On a
   Dongle-M, compare raw TX/RX, ACK timing, output and the CTUNE frequency offset
   with the stock EmberZNet image (nRF sniffer). Capture RAIL's per-state
   and per-channel values at 38.4 MHz (stock image over SWD, if the pads
   are found) and re-check the runtime trims on that die. If the radio
   misbehaves, take `hfxo-38m4hz` off the board crate: the radio is then
   `Unsupported` again.
4. **SE mailbox TRNG and the recorded fatal resets.** The record must
   survive the resident Gecko bootloader and come back as `reset_reason` 6.
   Test with a lab build that panics or faults on demand. For the poisoned
   mailbox, use an SE that stops answering, for example a debugger-held SE
   request.
5. **NCP on the product link.** The ZRHP session over the RF-BM-MG24B1's
   ESP32-S3 at 460 800 / 921 600, attribute reports from a device with a
   reportable cluster, end-device children, and a long run with many
   devices. The LDMA receiver ran only at 115 200 on the WSTK VCOM;
   460 800 / 921 600 need the S3 or the Dongle-M link.
6. **Gecko update path.** GBL upload over XMODEM and journal retention
   across an update.
7. **Dongle-M.** Upload `efr32mg24-ncp-sonoff-dongle-m.gbl` through the
   stock Gecko Bootloader v2.04.02 (ESP32 GPIO12 + RESET, XMODEM) after an
   EZSP backup; confirm the boot log (CTUNE and IEEE sources), the UART
   pins, the journal takeover of the stock NVM3, `SYS_RESET` mode 1 into
   Gecko, the restore of the EZSP backup under the same IEEE, the RF path
   and the TX power limit; then the return path (`efr32mg24-nv-clear.gbl`,
   stock GBL, ours again: logged `ForeignNvm3`).
8. **Power.** EM2, DC-DC, CCA against a jammer, children and Frame Pending
   through an xG24 Router, and long-duration runs.
