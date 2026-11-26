# xG24 radio captures: BRD4187A on a BRD4001A WSTK

These are register dumps of Silicon Labs RAIL running on a real EFR32MG24.
`efr32s2-hal/src/radio/xg24.rs` was built from them. RAILtest images, the
GSDK and its register map are not included. Everything here is our own
measurement, captured 2026-11-26 with
[`../efr32_radio_capture.py`](../efr32_radio_capture.py) through hwlab
(J-Link OB of the WSTK, VCOM, AEM, nRF 802.15.4 sniffer).

## Target

| item | value |
|---|---|
| radio board | BRD4187A Rev A01 (xG24 +20 dBm), on a BRD4001A mainboard (J-Link 440206286) |
| die | DEVINFO `PART` 0x011804C4 = **EFR32MG24B220F1536IM48**, `INFO` PRODREV 0x19 (rev A1/1.1), 1536 KiB flash, 256 KiB SRAM |
| EUI-64 | C0:9B:9E:FF:FE:A8:45:C5 |
| crystal | 39 MHz HFXO. The board had no `MFG_CTUNE` (USERDATA erased) and DEVINFO has no module calibration (`MODXOCAL` erased), so RAILtest used the BRD4187C default CTUNE 95. It was not measured. |
| DEVINFO IR calibration | 0x0FE08248 / 0x0FE0824C = 0x80FBFFEF |
| DEVINFO HP-PA ramp levels | 0x0FE08250..0x0FE0825B = `D6 D5 D5 D5 D5 D4 D3 D3 D2 D2 D1 D0`, one byte per slice 1..12. RAIL programs this byte into `MODEM.RAMPLEV`. |
| `RAC.RFLOCK0` | 0x00060000 at boot, set by the SE (0/10 dBm PAs locked on this OPN); never written |

GSDK lists BRD4187A as an A020 part, but this unit carries a B220. The raw
DEVINFO page and the (erased) USERDATA page are in `raw/devinfo.tar.xz`.

Before anything was written, the board's flash was backed up and verified:
`~/.hwlab/artifacts/mg24-brd4187a/jlink-flash-EFR32MG24BxxxF1536-08000000-180000-852aad42795f.bin`.
The flash was fully erased as delivered.

## Reference program

- GSDK 4.5.0 `rail_soc_railtest`, generated with
  `slc generate … --with brd4187c`. BRD4187C is the B220 variant of the same
  board, 39 MHz, CTUNE 95.
- DCDC bypassed. EUSART VCOM restricted to EM1.
- `-Wl,--no-warn-rwx-segments` was removed for gcc 10.3.
- The PHY is `config2p4GHz802154`, RAIL's built-in IEEE 802.15.4 2.4 GHz PHY
  for 39 MHz.

For the PA transition samples (`sampler2`), the RAILtest `app.c` was replaced
by [`railtest-sampler/app.c`](railtest-sampler/app.c), our own lab-only code.
It starts one RAIL transmission and records every change of seven
PA-related registers. Sampling runs in a tight loop at 39 MHz, about one
sample per 11 µs.

## Method

1. A scenario JSON (`scenarios/*.json`) lists RAILtest CLI commands, sleeps,
   frame injections and register-dump sets.
2. `efr32_radio_capture.py experiment` turns it into an hwlab experiment.
   Run that experiment with `hwlab run --yes`.
3. Each dump is an SWD `MemoryAccess.read` of one block into
   `<out>/<state>/<block>.bin`.

Peripheral blocks are read through their **non-secure** aliases
(0x5xxx_xxxx). RAILtest marks the peripherals non-secure, and a debugger
read of the secure alias raises an SMU fault that hangs RAILtest. Radio
blocks (0xB8xx_xxxx, BUFC 0xBA00_0000) are read directly.

To reproduce the decoded tables:

```sh
mkdir -p /tmp/xg24 && for a in raw/*.tar.xz; do tar xJf "$a" -C /tmp/xg24; done
python3 ../efr32_radio_capture.py reduce --capture /tmp/xg24/cap1 \
    idle-ch26 rx-ch26 tone-ch26-p0 tone-ch26-p100 tone-ch26-p200
```

`reduce` needs a local GSDK 4.5.0 for the register map (`--gsdk`).

## Campaigns (`raw/*.tar.xz`, hashes in `manifest.json`)

| archive | content |
|---|---|
| `cap1` | Full dumps in each state: idle, RX ch26, RX ch15, CW tone at 0/10/20 dBm, PN9 stream at 20 dBm, idle again. |
| `cap2` | TX power sweep from −30 to +20 dBm (`PACTRL`, `RAMPLEV`, PA registers). RSSI/AGC dumps of frames injected by the nRF. |
| `cap3` | Fresh RX/idle dumps, plus RAC samples taken inside packet-TX windows. |
| `cap4` | RX on every channel 11..26 (`SRCCHF`, `DIGMIXCTRL`, SYNTH). Full RX dumps at ch11 and ch20. |
| `cap5` | Packet-TX samples with long RAIL transition timings (`setTimings`), for the TX entry and exit order. |
| `sampler2` | PA power-up and power-down transitions at `setPower` 0/50/100/150/200 (`railtest-sampler`). |
| `ours1`, `ours2` | The same blocks dumped from our driver during packet TX. Steady-state equality check against RAIL. |

Decoded summaries:

- [`summary/cap1-reduce.txt`](summary/cap1-reduce.txt): static table, per-state
  differences and registers outside the configurator map.
- [`summary/cap4-channels.md`](summary/cap4-channels.md): per-channel values.
- [`summary/pa-transitions.md`](summary/pa-transitions.md): RAIL's HP-PA
  power-up and power-down order. This sequence fixed the driver's PA current.
- [`summary/gsdk-phy-vs-silicon.txt`](summary/gsdk-phy-vs-silicon.txt): the
  GSDK 39 MHz PHY table compared with the captured RX state. 192 registers
  are equal and 16 differ; the 16 are per-state, per-channel, or runtime
  calibrations of this die. The file also lists the 21 registers in which
  the 38.4 MHz table differs from the 39 MHz one.

## Measurements

These numbers come from the WSTK AEM at 3.3 V with DCDC bypassed. The AEM
saturates at about 95.7 mA. RSSI is the nRF sniffer's reading at a fixed
bench geometry. RAIL and our driver were measured back to back with the
same frame (125-byte payload, channel 26, frames back to back).

| state | RAILtest | zigbee-rs `xg24.rs` |
|---|---|---|
| idle (radio off) | 4.1 mA | 3.6 mA |
| RX | 10.3 mA | 9.9–10.1 mA |
| TX 0 dBm (mean over the burst / sniffer RSSI) | 24.8 mA / −53.0 dBm | 23.7 mA / −52.9 dBm |
| TX 10 dBm | 57.3 mA / −43.0 dBm | 56.5 mA / −42.9 dBm |
| TX 15 dBm | ≥ 91.7 mA (AEM saturated) / −37.9 dBm | ≥ 92.4 mA (saturated) / −37.3 dBm |
| TX 20 dBm | ≥ 92.4 mA (saturated) / −33.0 dBm | ≥ 93.2 mA (saturated) / −32.9 dBm |

Sessions: `2026-11-26T160219Z-mg24-rail-pw` (RAIL) and
`2026-11-26T160108Z-mg24-ours-pw` (ours).

On channel 20, RAIL and our lab binary were measured alternately, 4 × 500
frames: −46, −47, −46 and −45 dBm. Sessions:
`2026-11-26T163255Z…163412Z-mg24-cmp-{rail,ours}`.

The RSSI offset is −50 dB. RAIL reports `floor(FRAMERSSI / 4) − 50`. The
driver's trailer RSSI matches this for frames injected by the nRF.

