# Silicon Labs EFR32

The repository supports these products:

- EFR32MG1P TRÅDFRI environmental sleepy end device (Series 1);
- EFR32MG22 BRD4183A development sensor (Series 2, hardware-proven);
- EFR32MG21 BRD4181A development sensor (Series 2, builds; radio pending).

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
| PA | `RAC_PACTRL` guess | Per-chip PA tables. xG22: 0 dBm PA up to 6 dBm, ramping through `MODEM.RAMPCTRL`/`RAMPLEV`. xG21 20 dBm PA table is not captured yet |
| RSSI | raw `AGC_RSSI` | RAIL quarter-dB conversion; xG22 offset -50 dB (checked against `RAIL_GetRssi`) |
| EUI-64 | `0x0FE081A0` (TODO) | DEVINFO `EUI64L/H` (same offset on xG21/xG22) |
| IRQ numbers | Series 1 | GSDK vector tables per chip (xG21 61 IRQs, xG22 64): RTCC 10/12, RAC_RSM 37 |

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

### EFR32MG22 BRD4183A (hardware-proven)

| item | value |
|---|---|
| boards | BRD4183A on BRD4001A WSTK |
| MCU | EFR32MG22C224F512IM40, 512 KiB flash, 32 KiB RAM |
| HFXO | 38.4 MHz, CTUNE 120 |
| LED0 / BTN0 | PB1 active high / PB0 active low (EM2 wake) |
| VCOM | USART TX PA5, enable PA4, 115200 |
| layout | app `0x0..0x7C000` (no bootloader), journal `0x7C000..0x80000` |

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
| MCU | EFR32MG21A020F512IM32, 512 KiB flash, 64 KiB RAM |
| HFXO | 38.4 MHz, CTUNE 133 |
| LED0 / BTN0 | PB0 active high / PD2 active low (no EM2 wake on port D) |
| VCOM | USART0 TX PA5, enable PD4, 115200 |
| layout | Gecko bootloader `0x0..0x4000`, app `0x4000..0x7C000`, journal `0x7C000..0x80000` |

`examples/efr32mg21-sensor` is the same composition as MG22: RadioPhy and
SoftMac, RTCC time driver, EM2 and VCOM. It builds and passes the layout
checks.

The xG21 PHY, analog and PA tables (`efr32s2-hal/src/radio/xg21.rs`) are
still empty, so radio init returns `Unsupported` and the firmware halts with
a logged error. Filling them needs the same lab-only RAIL register capture
on an xG21 die.

| image (`tools/mac-footprint.py`) | flash | text | bss | task pool | static stack |
|---|---:|---:|---:|---:|---:|
| former scaffold MAC | 208,956 | 177,048 | 18,888 | 11,040 | 25,524 |
| efr32s2 RadioPhy + SoftMac | 229,084 | 187,048 | 14,600 | 2,744 | 20,652 |

### Home Assistant Connect ZBT-1 (prepared, not flashed)

`boards/ha-connect-zbt1` describes the ZBT-1:

- EFR32MG21A020F1024IM32 (1 MiB flash, 96 KiB RAM, SE firmware 0.1.6)
  behind a CP2102N;
- USART0 TX PA6, RX PA5, CTS PB1, RTS PB0, 115200 with hardware flow
  control, per the Nabu Casa firmware-builder manifests;
- HFXO CTUNE from the `MFG_CTUNE` token, otherwise a board default. On the
  inspected unit every MFG token is erased and DEVINFO has no CTUNE, so the
  default must match the stock firmware's compile-time value, read
  read-only from HFXO0 `XTALCTRL` before the first flash (140, the xG21
  device_init value, until then);
- EUI-64 from DEVINFO (the custom-EUI token is erased);
- no EFR-driven LED or button is confirmed.

The shipped Gecko Bootloader (2.4.2) occupies `0x0..0x4000`, which matches
the MG21 product layout, and accepts GBL files over XMODEM on that UART.
The stock flash image has the NCP at `0x4000..0x3E9D8`, NVM3 at
`0x74000..0x7E000`, and the upper 512 KiB erased. The NCP's Application
Properties use signature type NONE, so the bootloader does not enforce
secure boot and starts an application without an Application Properties
pointer. The MG21 product layout (app `0x4000..0x7C000`, journal
`0x7C000..0x80000`) fits the F1024 part unchanged; flashing it replaces
the stock NVM3.

**Recovery risk.** The bootloader has no GPIO activation. Once our
application replaces the NCP, the only ways back into the bootloader are:

- SWD pads;
- `efr32s2_hal::gecko_bootloader::reboot_to_bootloader()`, which writes
  `{0x0202, 0xF00F}` at `SRAM_BASE` and then triggers `SYSRESETREQ` (GSDK
  `btl_main.c`).

The application needs a trigger for this. That requires a UART receiver,
which the HAL does not have yet.

The bootloader accepts an application without an ApplicationProperties
struct when secure boot is off, which is the GSDK default for Series 2.

### Build

All EFR32 examples use `nightly-2026-03-23`:

```bash
cd examples/efr32mg1-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor

cd ../efr32mg22-sensor   # or efr32mg21-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg22-sensor
```
