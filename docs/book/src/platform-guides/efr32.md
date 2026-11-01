# Silicon Labs EFR32

The repository supports these products:

- EFR32MG1P TRÅDFRI environmental sleepy end device (Series 1);
- EFR32MG22 BRD4183A development sensor (Series 2, hardware-proven);
- EFR32MG21 BRD4181A development sensor (Series 2, builds; no hardware run);
- Home Assistant Connect ZBT-1 always-on End Device (EFR32MG21, lab-tested).

All use the shared `SensorApp`; their boards, products, clocks, storage, and
power behavior are intentionally separate. Series 2 chips share one HAL
(`efr32s2-hal`, chip chosen by the `efr32mg21` / `efr32mg22` feature) and
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
table is the only Frame Pending owner, as on TLSR8258. The core still
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
1 KiB queue costs no extra RAM. A core change (receiving during NWK delays)
would grow every other router image, so it was not made.

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
