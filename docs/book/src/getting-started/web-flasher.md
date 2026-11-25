# Web Flasher

**<https://faronov.github.io/zigbee-rs/flasher/>** installs released firmware
from the browser: pick the board, the version, keep or erase, click Install.
It needs Chrome or Edge on a desktop (Web Serial); Firefox and Safari cannot
flash. It uses [ESP Web Tools](https://esphome.github.io/esp-web-tools/),
which checks the chip before writing anything, so an image for the wrong chip
is refused.

Every image comes from a tagged GitHub release, never from a CI artifact:

| Board | Firmware | Release tag | Notes |
|---|---|---|---|
| ESP32-H2 DevKit | ZRHP NCP (`examples/esp32-ncp`, USB-Serial-JTAG) | `ncp-v*` (zigbee-rs) | the DevKit's "USB" port |
| ESP32-C6 DevKit | ZRHP NCP (USB-Serial-JTAG) | `ncp-v*` | the DevKit's "USB" port |
| Espressif Thread Border Router board, H2 | ZRHP NCP (UART 921 600, RTS/CTS on R19/R20) | `ncp-v*` | flash through the H2's own port USB1 |
| Espressif Thread Border Router board, S3 (4 MB or 8 MB, detected) | `tbr-hub`: A/B production layout on 8 MB, dev layout without OTA on 4 MB | `hub-v*` (zigbee-rs-hub) | one entry; the image follows the chip's flash size |
| SONOFF Dongle Max (experimental) | `zrh-donglem`: the hub on SONOFF's EmberZNet NCP, in `ota_1` of SONOFF's partition table | `hub-v*` (since 0.3.0) | **listed only, no install button** (below) |

There is no ESP32-C5 NCP image (the C5 port has router and sensor examples
only).

## Boards listed without an install button

A board whose release manifest says `"web_install": false` is shown with its
files, SHA-256, install steps and the esptool commands, but the page offers
no ESP Web Tools install for it and the site has no ESP Web Tools manifest
or copy of its images (the files link to the GitHub release). ESP Web Tools
cannot write a few regions *without* the option to erase the chip: for a
firmware that does not answer Improv Serial its install dialog either always
erases (`new_install_prompt_erase: false`) or asks with an *Erase device*
checkbox. On the **SONOFF Dongle Max** an erase removes SONOFF's bootloader,
partition table, `nvs` and factory data, and the two hub parts alone do not
boot. Its install is therefore SONOFF's own web console (*Upload & Flash
Firmware* with `zrh-donglem-<version>-app.bin`) or esptool over USB:
`zrh-donglem-<version>-app.bin` at 0x660000 (`ota_1`), then
`zrh-donglem-<version>-otadata-ota1.bin` at 0x5D000 (`otadata` sector 1,
selecting `ota_1`), each with its own `write-flash`, never an erase.

## Boards with several flash sizes

An image for a larger flash than the chip has does not boot at all (the
Thread Border Router board's S3 ships with 4 MB on early samples and 8 MB on
retail boards), and other S3 modules differ in PSRAM. Boards of one release
that differ only in flash size and/or PSRAM (same product, firmware, chip
and host link) therefore appear as **one entry**,
and installing takes one more step:

1. *Connect and read the flash size*: the page opens the port with
   esptool-js (vendored, pinned), checks the chip family, reads the SPI flash
   ID (`RDID`; size = 2^((ID >> 16) & 0xFF): `0x16` = 4 MB, `0x17` = 8 MB,
   `0x18` = 16 MB) and the partition table at `0x8000`.
2. It reads the embedded PSRAM from eFuse (`getChipFeatures()`:
   "Embedded PSRAM 2MB" / "8MB"; none listed = 0) and keeps only images whose
   `match.embedded_psram_mb` contains it (hub: quad build for 0/2/4 MB, octal
   `-opi` build for 8/16 MB; ESP-IDF fixes the PSRAM mode at build time and a
   wrong build runs without PSRAM). Of those it selects the largest whose
   flash size fits. Other images are disabled; a smaller fitting one can be
   chosen by hand. An unreadable ID
   (all 0 or all 1, implausible size) or a flash smaller than every image
   blocks the install.
3. If the chip's partition table differs from the chosen image's (another
   layout, e.g. 4 MB dev → 8 MB A/B, other firmware, or an erased chip), the
   install is forced to erase, with a warning to back up first (for the hub:
   its network backup in the web UI or the `network/backup` API call).
4. *Install* hands the same port to ESP Web Tools (no second port prompt), so
   the measured chip is the one written. After the install the chip reboots;
   the next install reads the flash again.

`tools/flasher_site.py` marks these groups (`size_group`,
`size_group_title`; a release manifest may set them itself). For checking
the selection without hardware, `?board=<group>&selftest=<MB>&table=<4MB|8MB|erased|other>`
simulates a detection, `&psram=<MB>` the embedded PSRAM (installing stays
blocked).

## Serial console and Wi-Fi setup

**<https://faronov.github.io/zigbee-rs/flasher/console/>** is a Web Serial
terminal (Chrome or Edge, no other dependencies) for the hub's USB console:

- **Port.** After tbr-hub starts, the S3's USB port shows two serial ports
  ("Thread BR H2 bridge", 303A:4002); the console is the **second** one
  (Windows: "Thread BR H2 bridge" or "USB Serial Device" / "Устройство с
  последовательным интерфейсом USB", usually the higher `COM`; macOS
  `cu.usbmodem…3`; Linux the higher `ttyACM`). The picker lists only
  Espressif devices unless *all ports* is ticked, which hides the PC's own
  "Communications Port (COM1)". "USB JTAG/serial debug unit" (303A:1001) is
  the S3's USB-Serial-JTAG: the chip is booting (ROM banner
  `ESP-ROM:esp32s3-…`), waits in its ROM loader, or the hub handed the port
  over; the page says so and asks to reconnect to the hub's second port once
  it appears.
- **Probe.** The page sends empty lines for about ten seconds and expects the
  prompt `> `, so a booting board has time; without a prompt it names the
  kind of port it got (hub's first port, USB-Serial-JTAG, other). It always
  opens at 115 200 baud: 1 200 baud on the console port is the touch that
  hands the USB port to USB-Serial-JTAG. It never toggles DTR/RTS on the
  hub's ports (the first one maps them to the H2's EN/BOOT in bridge mode);
  on USB-Serial-JTAG it releases RTS first and then DTR, since RTS without
  DTR resets the chip there (opening such a port may still reset it,
  depending on the OS driver). The NCP DevKits speak binary ZRHP and have no
  text console.
- **Terminal:** input line with history (↑/↓, session only), quick buttons
  (`help`, `mode`, `hub status`, `wifi`, *Read token* = `token`, *Reboot S3*
  = `s3 reset`), clear, download of the log.
- **Set up Wi-Fi:** sends `wifi set <ssid> <password>`; reads `mode`
  (e.g. `mode Bridge, persisted Some(Bridge)`) and, unless the hub runs in
  hub mode, explains and sends `mode hub` (persisted; the S3 reboots and the
  page reopens the console by itself, or asks to reconnect if the browser
  needs the port picked again); *Switch to hub mode* does the same alone; polls `wifi` until `up, ws://<ip>/api`; reads `token`; shows
  `http://<ip>/#token=<token>`, which the hub's web UI takes over. The
  console takes printable ASCII only and splits on spaces, so SSIDs with
  spaces and passwords with leading, trailing or double spaces are refused.
- **The password** stays in the form field until it is sent, then the field
  is cleared. It is never stored (no localStorage, no history) and is masked
  in the terminal and the downloaded log, including the console's echo of a
  hand-typed `wifi set`.

## Keep or erase

**Default: keep.** The flasher writes the bootloader, the partition table, a
blank `otadata` and the application; everything else stays:

- **NCP:** the `zbnv` partition (network key, PAN ID, channel, Trust Center
  link keys, frame counters, NCP identity, IEEE override). The blank
  `otadata` makes the new application in `ota_0` boot even if an earlier
  `FW_*` update over ZRHP had switched the NCP to `ota_1`. In the ESP Web
  Tools dialog, leave *Erase device* unticked.
- **Hub:** `nvs` (Wi-Fi credentials, boot mode, API token) and `hubdata`
  (device registry, ZRHP host store with the network backup).

Keep works only when the chip already runs the same firmware with the same
partition layout. For a first install over anything else (Espressif's stock
firmware, an older layout), and to start from scratch, tick **Erase the whole
chip**: ESP Web Tools then erases the flash before writing. On an NCP this
forgets the network; zigpy/ZHA and Zigbee2MQTT can restore their network
backup onto it, otherwise every device must be paired again. On the hub it
loses Wi-Fi, the token and the device registry; the H2 keeps its network.

Downgrading an NCP below the format of the stored network makes it start
factory-new (the journal version check fails closed), so the host restores
from its backup.

## Connecting

- **DevKits:** the "USB" port (native USB-Serial-JTAG), not the "UART" port.
  On Windows 10/11 the built-in driver handles it.
- **Thread Border Router board, H2:** its own port USB1.
- **Thread Border Router board, S3:** while tbr-hub or tbr-bridge runs, the
  S3's USB port shows the bridge's two serial ports. The flasher's *Switch
  the S3 to USB-Serial-JTAG* button opens the second (console) port at
  1200 bps, the same as the console command `s3 usj`; the S3's own
  USB-Serial-JTAG port appears and the flash-size detection can use it.
- If no port shows up or the connection fails: hold BOOT, press and release
  RESET, release BOOT, try again.
- Close other programs on the port first (Zigbee2MQTT, Home Assistant, serial
  monitors).

## Versions

The flasher lists the newest five releases of each repository; pre-releases
are marked and the newest stable release is preselected. Each version shows
its date, commit, `build_id`, ZRHP protocol version, the release notes
(changelog) and the SHA-256 of every image. Links can pin a board and a
version: `…/flasher/?board=esp32h2-devkit-ncp&version=ncp-v0.1.0` (for a size group
the group id, e.g. `thread-br-s3-hub`; a member id preselects that image if
it fits).

## Without a browser

Each release has the same images for esptool (v5 `write-flash`, v4
`write_flash`):

```sh
# factory-new NCP (wipes the stored network)
esptool --chip esp32h2 --port <PORT> write-flash 0x0 zrhp-ncp-h2-usj-<v>-merged.bin
# update keeping the network
esptool --chip esp32h2 --port <PORT> write-flash \
  0x0 zrhp-ncp-h2-usj-<v>-bootloader.bin 0x8000 zrhp-ncp-h2-usj-<v>-partition-table.bin \
  0x9000 zrhp-ncp-h2-usj-<v>-otadata-blank.bin 0x10000 zrhp-ncp-h2-usj-<v>-app.bin
```

Check downloads against the release's `SHA256SUMS`. Mixing esptool versions
or offsets from older instructions is the usual cause of a board that does
not boot; the release notes list the commands for that release.

## Release assets

`ncp-release.yml` (zigbee-rs, tag `ncp-v*`) and `hub-release.yml`
(zigbee-rs-hub, tag `hub-v*`) build from the tag and attach, per board:

| File | Use |
|---|---|
| `…-merged.bin` | full flash image at offset 0 (NCP: all 4 MB, so it erases `zbnv`; hub: bootloader to the end of the app) |
| `…-bootloader.bin`, `…-partition-table.bin`, `…-otadata-blank.bin`, `…-app.bin` | the parts the flasher writes |
| `…-fw-signed.bin` (NCP) | `FW_*` update over ZRHP, signed for the NCP's trusted key |
| `release-manifest.json` | machine-readable description (below) |
| `SHA256SUMS` | SHA-256 of every asset |

The version in the tag must equal the firmware's version
(`examples/esp32-ncp/Cargo.toml` and its `Cargo.lock` entry; for the hub
`firmware/hub/Cargo.toml` and `CONFIG_APP_PROJECT_VER`); the workflow also
reads the version back from the built image's app descriptor and fails on a
mismatch. A tag with a suffix (`ncp-v0.2.0-rc.1`) makes a pre-release.

Builds are reproducible from the tag: pinned toolchains and tools,
`--locked`, no build cache, `SOURCE_DATE_EPOCH` from the commit, and on the
NCP `ZRHP_BUILD_ID` = the first 8 bytes of the commit, which the NCP reports
as `SYS_GET_INFO.build_id` (zero in other builds, which stay byte-identical).

**NCP update files.** Shipped NCP firmware accepts only signed `FW_*`
images. The release signs them when the repository secret
`NCP_FW_SIGNING_KEY` holds the PEM private key behind
`bootloaders/keys/own-ec-p256.xy.hex` (the workflow refuses any other key).
Without the secret, the update file is the plain app image and the manifest
says `"signed": false`: only development builds (`dev-unsigned-fw`) accept it
over ZRHP; flashing over USB is not affected.

### `release-manifest.json`

```json
{
  "schema": 1,
  "product": "zrhp-ncp",                  // or "zigbee-rs-hub"
  "name": "zigbee-rs ZRHP NCP",
  "repository": "faronov/zigbee-rs",
  "tag": "ncp-v0.1.0", "version": "0.1.0", "prerelease": false,
  "commit": "<40 hex>", "build_id": "<16 hex>", "date": "<commit time, UTC>",
  "zrhp": {"protocol": "1.0"},
  "toolchain": {"rust": "nightly-2026-08-01", "espflash": "4.5.0"},
  "boards": [{
    "id": "esp32h2-devkit-ncp", "title": "…", "chip": "esp32h2", "chip_family": "ESP32-H2",
    "flash_size": "4MB", "layout": "esp32h2-ncp-4mb", "transport": "…", "connect": "…",
    "app_version": "0.1.0",
    "files": {"merged": {"name": "…", "offset": 0, "size": 4194304, "sha256": "…"},
              "bootloader": {…}, "partition_table": {…}, "otadata": {…}, "app": {…}},
    "flash": ["bootloader", "partition_table", "otadata", "app"],
    "preserved": [{"name": "zbnv", "offset": 4005888, "size": 196608, "what": "…"}],
    "fw_update": {"signed": true, "file": {"name": "…", "size": …, "sha256": "…"},
                  "version_word": "0x00010000"}
  }]
}
```

`flash` lists the parts the flasher writes, in order; their offsets must not
overlap. Hub manifests add `components` (e.g. the bundled bridge version)
and, once the hub speaks Improv Serial, `improv` (`firmware` name and wait
time) for ESP Web Tools. A board with `"web_install": false` (default
`true`) is never installed through ESP Web Tools; `flash` then lists the
parts in their USB write order, `install_steps` the instructions the page
shows, `experimental` and `hardware_status` its state, and
`stock_partition_table` (offset, size, SHA-256) the table the parts are
for.

## How the site gets the images

GitHub release downloads carry no CORS headers, so a page cannot fetch them.
The Pages build copies them onto the site instead:

- `.github/workflows/site.yml` builds the whole site: the book, the CI
  sensor flasher (`/flasher/sensor/`), and the release flasher.
  `tools/flasher_site.py fetch` takes the newest five releases of each
  repository that carry a `release-manifest.json`, downloads the parts,
  verifies size and SHA-256, and writes one ESP Web Tools manifest per board
  and mode plus `releases.json` for the page. ESP Web Tools itself is
  vendored from npm (pinned version, integrity-checked), and so is
  esptool-js for the flash-size detection.
- `ci.yml` calls it on every run: deployed on master pushes, a preview
  artifact (`github-pages`) on pull requests.
- `site-refresh.yml` rebuilds the live site's revision with the current
  release set. `ncp-release.yml` starts it after publishing; a release
  published by hand starts it through the `release` event; the hub's release
  workflow sends a `repository_dispatch` (`flasher-release`) when
  zigbee-rs-hub has the secret `FLASHER_DISPATCH_TOKEN` (a fine-grained token
  with *Contents: read and write* on zigbee-rs); without it, the six-hourly
  schedule picks the release up. The schedule compares a fingerprint of the
  release set with the live `releases.json` and does nothing when it is
  unchanged; *Run workflow* with *force* rebuilds by hand.
- A refresh deploys only while the live site still has the revision it
  started from, so it never replaces a newer CI deploy.
