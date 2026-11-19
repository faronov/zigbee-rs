# Vendored TI sources

`rf_patches/rf_patch_cpe_ieee_802_15_4.{c,h}` — the RF core (CPE) patch for
the IEEE 802.15.4 command set on CC13x2/CC26x2.

| | |
|---|---|
| Origin | <https://github.com/TexasInstruments/simplelink-lowpower-f2-sdk> |
| Release | SimpleLink Low Power F2 SDK 8.33.00.16 |
| Commit | `3615ad5d3f4f271258d18a2f5ce1785064ff9a58` |
| Paths | `source/ti/devices/cc13x2_cc26x2/rf_patches/` and `source/ti/devices/cc13x2x7_cc26x2x7/rf_patches/` (byte-identical) |
| SHA-256 `.c` | `0fa9b643cafaf429534d7cf85691bcf466e3a5645c494afc5140cb0ec89389bf` |
| SHA-256 `.h` | `aaff403887d87b75f145fc094c3bc7715d25ca2a20210b66afa091ebadb9a3fa` |
| License | BSD-3-Clause, text in each file header (Copyright (c) 2015-2022 Texas Instruments Incorporated) |

The files are kept unmodified for provenance. They are not compiled:
`tools/gen_rf_patch.py` turns the patch image and its patch-table writes into
`src/rf/patch_ieee.rs` (`--check` in CI keeps the two in sync).

The IEEE 802.15.4 radio settings (override words, PA tables) in
`src/rf/settings.rs` are transcribed from the same SDK release,
`source/ti/devices/radioconfig/.meta/config/<device>_ieee_15_4_pg*/cmd_settings/setting_ieee_802_15_4.json`
and `.../pa_tables/cc13x2_cc26x2_r{3,7}/pasettings.json`; the override macro
encodings follow `driverlib/rf_mailbox.h` and the ANADIV computation
`driverlib/rfc.c` `RFCAnaDivTxOverride()`. Each table cites its source in
`settings.rs`.
