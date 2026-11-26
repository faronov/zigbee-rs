# xG24 10 dBm (MP) PA captures: xG24 Explorer Kit EK2703A

These are register dumps of Silicon Labs RAIL on an EFR32MG24 with the
10 dBm medium-power PA. The MP-PA half of `efr32s2-hal/src/radio/xg24.rs`
was built from them: `PaPhy` for `efr32mg24-pa10`, the MP `TX_POWER` table
and the MP PA power-up words.

They were captured on 2026-11-26 with the same method, tool and sampler as
[`../xg24-brd4187a`](../xg24-brd4187a/README.md), through hwlab on the kit's
own J-Link OB and VCOM. The EK2703A has no current monitor: the hwlab
`wstk` device for the kit lists `PowerMeasurement`, but AEM start fails with
a protocol error. Output power is therefore compared with the nRF sniffer
only.

## Target

| item | value |
|---|---|
| kit | xG24 Explorer Kit EK2703A, board BRD2703A (J-Link OB 440378104) |
| die | DEVINFO `PART` 0x011804BA = **EFR32MG24B210F1536IM48** (10 dBm MP PA), PRODREV 0x19, 1536 KiB flash, 256 KiB SRAM |
| EUI-64 | 40:38:02:FF:FE:37:31:AE |
| crystal | 39 MHz HFXO. No `MFG_CTUNE`, so RAIL ran with the board's CTUNE 100. It was not measured. |
| `RAC.RFLOCK0` | 0x00080000 (`RACTX20DBM`: the 20 dBm PA is locked) |
| DEVINFO | IR word at 0x248/0x24C = 0x813A816E, but RAIL does not apply it. The HP ramp bytes at 0x250 are erased. |
| flash as delivered | Factory demo at 0x0800_0000..0x0801_8000, no bootloader. Verified backup: `~/.hwlab/artifacts/mg24-ek2703a/jlink-flash-EFR32MG24BxxxF1536-08000000-180000-6bddae16ccef.bin` |

The raw DEVINFO and USERDATA pages are in `raw/devinfo.tar.xz`.

## Reference program

GSDK 4.5.0 `rail_soc_railtest`, generated with `slc … --with brd2703a`
(revision A02). The settings differ from the BRD4187A build:

- RAIL selects `RAIL_TX_POWER_MODE_2P4GIG_HIGHEST`, which is the MP PA on
  this part.
- The ramp is 10 µs.
- DC-DC is bypassed (`SL_DEVICE_INIT_DCDC_BYPASS 1`), as in our firmware.
- `app.c` is the lab sampler.
- The VCOM is EUSART0 PA5/PA6 without flow control.

## Campaigns (`raw/*.tar.xz`, hashes in `manifest.json`)

| archive | content |
|---|---|
| `ek1` | Full dumps in each state: idle, RX ch26 and ch15, CW tone at −10/0/5/10 dBm, PN9 stream at 10 dBm, idle again. |
| `ek2` | `setPower` sweep from −30 to +10 dBm: `RAC0`, `RAMPCTRL`/`RAMPLEV`. |
| `ek3` | RX on every channel 11..26: `SRCCHF`, `DIGMIXCTRL`, SYNTH. All 16 channels are identical to the BRD4187A (both run at 39 MHz). |
| `sampler` | MP-PA power-up and power-down transitions at `setPower` 100/50/0/−100/−300. |

Decoded summaries:

- [`summary/ek1-reduce.txt`](summary/ek1-reduce.txt): static and per-state
  tables.
- [`summary/mp-vs-hp.txt`](summary/mp-vs-hp.txt): every register that differs
  from the BRD4187A (HP) capture.
- [`summary/mp-power-table.md`](summary/mp-power-table.md): RAIL's MP power
  programming.
- [`summary/mp-pa-transitions.md`](summary/mp-pa-transitions.md): the MP power-up
  order.

### What differs from the 20 dBm part

- **RF path.** RAIL drives RF path 1 (`RFPATHEN1`, `LNAMIXTRIM1`,
  `AGCOVERWRITE1.MANLNAMIXRFATT1`) where the B220 uses path 0. RAIL's
  `defaultPath` is the same in both builds, and it is ignored when one RF
  path is bonded out.
- **PA power programming.** The level is `MODEM.RAMPLEV` (raw + 16..21),
  with a per-band `RAMPCTRL` (0x999 / 0x888 / 0x777 / 0x666). `PACTRL` carries
  only `TXPASELPREDRVREGVDDPA` in TX, and `PADEBUG` stays 0.
- **PA words.** `RAC.TX` is 0xC001_3001 for the pre-stage, 0xC029_F003 for
  the ramp and 0xC029_C001 steady. The order matches the HP PA: slice reset
  and pre-stage, then ramp clock, then steady.
- **PA trims.** `PATRIM0/2/3/5`, `SYTRIM1` (no 20 dBm LO delay, TX LDO trim
  1) and `PATRIM4` (no TX change).
- **IR correction.** `IRCALCOEFWR0/1` = 0xFFFFFFFF.
- **Runtime calibrations of this die.** `VCOGAIN`, `LPFCTRL2RX/TX`,
  `VCOCTRL`, `AUXADCTRIM`, `IFADCCAL`, `LNAMIXTRIM3/4`, `PGATRIM`, `PGACAL`.

## Measurements

The tables below show the nRF sniffer's RSSI on channel 26, with 125-byte
frames sent back to back and the kit in the same position. RAIL and our
lab binary were measured back to back. The session names are
`2026-11-26T174737Z-mg24-ek-ours-pw` (ours) and
`2026-11-26T174832Z-mg24-ek-rail-pw` (RAIL).

| requested | RAIL level | RAILtest | zigbee-rs `xg24.rs` (MP) |
|---|---|---|---|
| −30 dBm | −30.0 dBm | −73.3 dBm | −73.0 dBm |
| −10 dBm | −11.8 dBm | −51.1 dBm | −51.0 dBm |
| 0 dBm | −0.2 dBm | −42.0 dBm | −42.0 dBm |
| 5 dBm | 4.9 dBm | −37.0 dBm | −36.5 dBm |
| 10 dBm | 10.0 dBm | −32.0 dBm | −31.9 dBm |

These results come from our lab binary `s2-mg24-air --features ek2703`:

- **Imm-Ack.** The nRF's ACK-wait got 12 of 12 ACKs for frames to the
  filter address, and 0 of 5 for another address.
- **RX.** RSSI read −49 dBm for the nRF at 0 dBm.
- **Energy detect.** Ran on all 16 channels with the synthesizer locked.
- **Beacons.** A beacon request on channel 15 brought back 44 beacons.

These results come from the Router `--features ek2703a`, running above a
GSDK `bootloader-uart-xmodem` built for `brd2703a`:

- **Join.** It joined the zigbee-rs-hub test network (PAN 0xE2E9, channel
  20) as 0x3A55, with the BRD4187A router 0xB2B0 as its parent.
  Commissioning completed.
- **Imm-Acks.** The sniffer saw 22 of 24 ACKed 158–169 µs after the frame
  end, and the HAL reported a maximum turnaround of 165 µs. The two
  misses were the first copies of one frame right after the join, while
  the journals were being written.
- **Coordinator ACKs.** The coordinator ACKed 30 of 30 of our frames.
- **Resume.** The Router resumed 0x3A55 from its journals after a reset.

Session: `2026-11-26T175540Z-mg24-ek-router-join`.
