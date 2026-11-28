# ZRHP NCP 0.1.3

Changes since `ncp-v0.1.2`. ZRHP stays at protocol 1.0. Every addition comes
with a capability bit or a new flag, so hosts written for 0.1.2 keep working.

## Fixes

- **Sleepy children after a secured rejoin (#23).** A parent used to answer
  a child's poll with a frame queued after the radio had acknowledged that
  poll with "no data pending" (Frame Pending = 0). By then the child had
  switched its receiver off, so the frame was lost. The most visible effect:
  a sleepy end device's secured rejoin failed and fell back to a Trust
  Center rejoin.
  - Each poll now carries the radio's own decision, so there is no per-child
    table to fill up.
  - Applies to the EFR32 Series 2 parents (MG24 NCP; MG21/MG22/MG24 Routers)
    and the ESP32-H2/C6 parent.
  - On a SONOFF Dongle Max, a secured rejoin without restored keys now
    completes on the first try.
- **No Leave on air when the host wipes the NCP (#23).** The following used
  to broadcast a NWK Leave from the coordinator:
  - a backup restore;
  - a restore over a stored network;
  - `NWK_LEAVE_RESET`;
  - `SYS_RESET` mode 2.

  Routers that heard it dropped the coordinator. These wipes are now local.
  Only `NWK_LEAVE_RESET` with the new flags bit2 (explicit "leave network")
  sends the Leave. On a Dongle Max, a 17-minute capture across restores
  showed no Leave.
- **Networks restored without link keys (#22).** The Trust Center admits a
  NWK-secured rejoin from a device that holds the current network key, even
  when the restore carried no link key for it. The device gets network
  membership only: its private, never-shared key gives it no APS-layer
  trust. Restored children stay in the child table until they rejoin or
  poll, and a denied rejoin no longer removes them.
- **EmberZNet backups (#22).** A restore that carries EmberZNet's hashed
  Trust Center link key root (`hashed_tclk`) derives each device's unique
  link key the way EmberZNet does. Devices migrated from an EZSP coordinator
  keep working. Shown on a Dongle Max: a Transport-Key after a Trust Center
  rejoin, then a full interview.
- **ESP32-H2 RSSI and LQI (#24).** The ESP32-H2 PHY now applies ESP-IDF's
  RSSI compensation (`IEEE802154_RSSI_COMPENSATION_VALUE`, read from the PHY
  library at boot). On the parts we tested it is 10 dB; on the C6 it is 0.
  Before this, reported RSSI was 10 dB low and LQI about 32 low, which also
  skewed NWK link costs. Received RSSI, LQI and the `NWK_ACTIVE_SCAN`
  results now match ESP-IDF.
- **RSSI on EFR32 Series 2 (#23).** MG24 indications carried RSSI −128
  ("unknown"). They now carry the measured per-frame RSSI.
- **Logs.**
  - The ZDO log names Device_annce and Parent_annce instead of
    `UNKNOWN_ZDP` (#23). These announcements already reached the host.
  - The NCP now holds its first four log records after reset (boot log)
    until the host grants LOG credit (#22). Before, a host that opened with
    no LOG credit never received them.
  - The ZRHP link no longer stalls on an endpoint that the host opened with
    no credit and grants credit to later (R8) (#22).
- **Trust Center after a Leave (#19).** A node no longer keeps the previous
  network's Trust Center address after a Leave without rejoin or a factory
  reset. The stale address made the next network's Trust Center fail
  authentication.

## New hardware: EFR32MG24 and the SONOFF Dongle Max

- **xG24 radio (#17, #18).** 20 dBm (HP) and 10 dBm (MP) PA. The register
  values come from RAIL captures on a BRD4187A and an EK2703A with 39 MHz
  crystals, verified over the air.
- **38.4 MHz crystal (#21).** The PHY values for the SONOFF Dongle-M's
  38.4 MHz crystal are GSDK 4.5.0's, found byte for byte in the stock image.
  On a Dongle Max (host UART at 460 800 Bd) the radio both transmits and
  receives.
- **TX power.** The MG24 NCP transmits at a fixed +10 dBm on the Dongle-M.
  `NWK_SET_TX_POWER` is not supported yet. Output power has not been
  measured.
- **CTUNE and EUI-64 sources, in EmberZNet's order (#21).**
  - CTUNE: DEVINFO module calibration, then the `MFG_CTUNE` token, then the
    board default.
  - EUI-64: the custom EUI token if programmed, otherwise DEVINFO.

  The coordinator keeps the stock firmware's IEEE, so devices keep their
  Trust Center address. The boot log names both sources.
- **Taking over from stock EmberZNet (#21, #25).** On first boot the NCP
  erases the NV window `0x08158000..0x08180000` if any page start in it
  holds foreign data. That includes the stock NVM3, found on hardware at
  `0x08176000..0x0817E000` with page header `0xB29A0001`. Stale records of
  ours that the stock NVM3 left untouched therefore never resume an old
  network.
- **Gecko bootloader images (#21).** Unsigned GBL v3 images. A Dongle Max's
  stock Gecko Bootloader v2.04.02 accepted them through the hub's flasher.
  The app is linked at `0x08006000`; the bootloader itself is never
  written. `SYS_RESET` mode 1 (restart into the Gecko bootloader) is
  implemented but untested.
- **Host UART without overruns (#20).** The MG24 host UART receives through
  LDMA. A 35-minute soak at 115 200 Bd had 0 overruns and 0 framing errors.

## ZRHP protocol additions

| Addition | Use |
|---|---|
| `SEC_DERIVE_EZSP_KEY` (`0x0409`), capability bit 14 (`EZSP_HASHED_TCLK`) | the EmberZNet-derived link key of an IEEE; gated by key export like other keys |
| backup record `EZSP_HASHED_TCLK` (`0x06`) | restore input for the derived keys; exported with the keys |
| `NWK_LEAVE_RESET` flags bit2 | send the Leave over the air (explicit "leave network"); without it the wipe is local |
| `TBL_READ` / `TBL_STATS` table `0x0D` (`APS_GROUP`), `BKP_EXPORT_BEGIN` flags bit3, capability bit 24 (#12) | read and back up the coordinator's group memberships |

## Upgrade notes

- **Downgrading to 0.1.2.** The Trust Center table switches from record
  version 6 to version 7 only once it holds something 0.1.2 cannot store:
  an EZSP hashed root, or EZSP-derived or unknown keys. Firmware before
  0.1.3 skips version 7 records. It resumes an older Trust Center snapshot
  or an empty table, while the network itself resumes. Its APS counter can
  then start below values already used. Take a backup before a downgrade
  and restore it afterwards.
- **Hosts.**
  - Set `NWK_LEAVE_RESET` bit2 only when the user asks the coordinator to
    leave the network. 0.1.2 ignores the bit and sent the Leave on every
    wipe.
  - To read the boot log after an update, grant LOG credit before resetting
    the NCP again.
- **Removed devices.** A device deleted by the host is readmitted if it
  rejoins secured with the current network key, as with EmberZNet. To
  remove it for good, send it a Leave (rejoin 0) and rotate the network
  key.
- **Devices restored without keys.** If the restore carried no hashed root,
  such devices have network membership only. APS-secured exchanges with
  them fail until they are paired again.
- **ESP32-H2 RSSI and LQI.** Reported values rise by the chip's
  compensation (10 dB on the parts we tested). Hosts with RSSI or LQI
  thresholds may rate links differently.
- **SONOFF Dongle Max.**
  - Install `efr32mg24-ncp-sonoff-dongle-m.gbl` through the Gecko
    bootloader, for example with the hub's NCP update. The web flasher
    cannot reach the MG24 and does not offer it.
  - The first boot erases the stock network. Take an EZSP network backup
    with the hub before installing, then restore it over ZRHP.
  - To return to EmberZNet, install `efr32mg24-nv-clear.gbl` first, then
    the stock GBL.
- **Not in these images.** PTA coexistence for the ESP32-H2 (#13) is in the
  source but not enabled in the released images. Its hardware gate PTA1 is
  open.

## Assets added in this release

Two Gecko images join the ESP32-H2/C6 images:

- `efr32mg24-ncp-sonoff-dongle-m.gbl`: the ZRHP NCP for the SONOFF Dongle
  Max;
- `efr32mg24-nv-clear.gbl`: clears the NV window.

Both are listed under `gecko_images` in `release-manifest.json` and in
`SHA256SUMS`.
