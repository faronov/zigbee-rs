# SONOFF Dongle-M stock firmware: radio configuration (no hardware)

What the stock EmberZNet NCP of the SONOFF Dongle Max (EFR32MG24A420F1536IM48)
runs, read from its GBL file only. Nothing here was captured from a unit;
the per-channel words are computed, not measured. The stock image is not
in this repository.

Input: `donglem_mg24_zigbee_stable_1.0.0_115200_7.4.5.gbl` (SONOFF's
`zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`), 251 808 B, SHA-256
`44b81c1ecf81032b98d625388ae44c10d7c8a6abd1dcea10b2e2d934d7a311fa`, from
iHost-Open-Source-Project `hassio-ihost-sonoff-dongle-flasher` at commit
`136ab2fe`, `firmware-build/`. Reproduce with a local GSDK 4.5.0:

```sh
python3 ../efr32_radio_capture.py stock-gbl <path-to-the-gbl>
```

## Result

- The application (`0x0800_6000..0x0804_3754`, one ERASEPROG record) holds
  GSDK 4.5.0 `sl_rail_ieee802154_phy_2G4Hz_38M4Hz_modemConfigBase`
  (238/238 configuration words) and `_modemConfig` (66/66) byte for byte;
  the pointers differ. The FEM (61/66) and antenna-diversity (27/66)
  variants and the 39 and 40 MHz tables (218 and 214 of the 238 base
  words) do not match. So the stock radio runs GSDK's plain 802.15.4 PHY
  for a **38.4 MHz** HFXO.
- Its `phyInfo` (`0x0803_C770`): `freqOffsetFactor` `0x00444444`,
  `src2Denominator` 60 000 000, `synthCache` `0x02104911` (IFFREQ `0x4911`,
  LO above the channel, `DEC0` map 0 = decimation 3).
- RAIL computes `MODEM.SRCCHF.SRCRATIO2` and `MODEM.DIGMIXCTRL.DIGMIXFREQ`
  per channel at run time; the image holds only the channel-11 words of the
  configuration. With the expressions of `efr32s2-hal`
  `xg24::channel_words`, which reproduce all 16 channels captured from
  RAIL on the 39 MHz BRD4187A (`../xg24-brd4187a`, cap4), this `phyInfo`
  gives:

  | channel | SRCRATIO2 | DIGMIXFREQ |
  |---:|---|---|
  | 11 | 0x281B | 0x1BFBB |
  | 12 | 0x2831 | 0x1BECE |
  | 13 | 0x2846 | 0x1BDE1 |
  | 14 | 0x285B | 0x1BCF5 |
  | 15 | 0x2871 | 0x1BC0B |
  | 16 | 0x2886 | 0x1BB21 |
  | 17 | 0x289B | 0x1BA38 |
  | 18 | 0x28B1 | 0x1B950 |
  | 19 | 0x28C6 | 0x1B869 |
  | 20 | 0x28DB | 0x1B783 |
  | 21 | 0x28F1 | 0x1B69E |
  | 22 | 0x2906 | 0x1B5BA |
  | 23 | 0x291B | 0x1B4D7 |
  | 24 | 0x2931 | 0x1B3F5 |
  | 25 | 0x2946 | 0x1B313 |
  | 26 | 0x295B | 0x1B233 |

  Channel 11 equals the configuration's base words. The earlier
  proportional scaling differed from these by one LSB on some channels.
- `RAIL_TxPowerConfig_t` at `0x0803_D75C`: mode 4
  (`RAIL_TX_POWER_MODE_2P4GIG_HIGHEST`, the 20 dBm PA on the A420),
  3300 mV, 10 µs ramp: GSDK's `sl_rail_util_pa_config.h` defaults. The
  runtime power comes from the EZSP host and is not in the image.
- `ApplicationProperties_t` at `0x0803_C148`: struct 1.2, signature NONE,
  type ZIGBEE, version 1, product ID zero; initial SP `0x2000_1000`.

## Not known from the file

The per-state analog values and runtime calibrations RAIL writes at
38.4 MHz (`SYNTH.LPFCTRL2RX/TX`, `RAC.IFADCTRIM0`, `LNAMIXTRIM4`,
`SYTRIM1`, `SYNTHREGCTRL`, ...): the zigbee-rs 38.4 MHz build carries the
39 MHz B220 die's values. They need a capture on a 38.4 MHz unit.
