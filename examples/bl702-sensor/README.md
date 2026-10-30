# BL702 pure-Rust Zigbee sensor

Environmental sleepy-end-device profile for the XT-ZB1 without Bouffalo radio
archives.

## Composition

```text
environmental Sleepy End Device profile
        ↓
products/bl702-xt-zb1
        ↓
boards/bl702-xt-zb1
        ↓
bl702-hal + Bl702RadioPhy + SoftMacCore
```

This example is the composition root.

The root constructs explicit `SensorSedParts`:

- `Bl702Wake` (`Active` and radio-quiesced `Idle`; `Retention` rejected);
- `NoStatus`;
- synthetic temperature/humidity;
- GPADC supply battery;
- `Bl702OtaLifecycle` (Zigbee OTA client, see [OTA](#ota));
- no user action;
- BL702 supervisor (software reset through the ROM `GLB_SW_POR_Reset`) and
  diagnostics.

The product selects `Idle` for both fast (250 ms) and slow (10 s) parent-poll
waits. An `Idle` wait:

1. disables M154 receive and waits (bounded) for the M154 state machine to
   report idle; if it does not, the wait returns `WakeReason::Activity`
   instead of sleeping;
2. awaits an Embassy timer, so the executor executes WFI;
3. wakes on the TIMER channel 0 comparator-0 interrupt (CLIC source 52,
   CLIC direct mode `mtvec = entry | 0b10`).

Clocks, the RF analog block, PLL, and per-die calibration stay powered, so
there is no restore step: the next MAC RX/TX starts from the same idle state
used between ordinary operations. The time driver arms the comparator for
the earliest deadline, capped at 30 minutes so the 1 MHz counter's 64-bit
extension never misses its ~71.6-minute wrap. A boot self-test proves alarm
interrupt delivery; on failure the driver reverts to the previous polling
executor and diagnostics report `wfi=false`.

`Retention` (PDS/HBN) is rejected by the wake controller and refused by
`bl702_hal::pm`; no PDS/HBN behavior is claimed. Idle sleep was validated on
one XT-ZB1 against ZHA: the alarm self-test passes (`wfi=true`), Idle waits
show no radio vetoes, and fresh join, interview/reconfigure over the 10 s long
poll, Mgmt_Leave and reset/resume all work. Current draw and behavior across
the ~71.6-minute counter wrap are not yet measured.

With `BL702_DIAGNOSTIC_LOG=1`, the UART (2 Mbaud) shows `alarm irq ready`
after `logger ready` (or `alarm irq FAILED: polling executor`), then an
`idle sleep: waits=…, idle_ms=…, radio_vetoes=…, alarm_irqs=…, wfi=…`
summary on the first and every 64th idle wait.

## Radio and Zigbee validation

Earlier XT-ZB1 images produced hardware evidence for:

- cold ACAL/KCAL/ROSCAL/RCCAL;
- channel selection, CCA/ED, RX, and TX;
- steering and association on channel 15;
- indirect polling and Transport-Key reception;
- ZHA interview, Configure Reporting, TCLK exchange, and live reports.

The production path installs token-owned SEC_ENG AES, runs two startup KATs,
and fails closed rather than using software AES.

## Storage and flash layout

The board exposes the physical XIP flash resource. The product
(`products/bl702-xt-zb1/layout`, the single source for the linker scripts,
the loader, the application and `tools/bl702_ota.py`) divides the 1 MiB
flash into 4 KiB sectors:

| physical | XIP | contents |
|---|---|---|
| `0x00000..0x01000` | | boot ROM header (BFNP) of the loader, `img_start` 0x2000 |
| `0x02000..0x0FFE0` | `0x23000000` | `bl702-xt-zb1-boot` loader |
| `0x0FFE0..0x10000` | `0x2300DFE0` | LoaderInfo record (key id, loader version) |
| `0x10000..0x87000` | `0x2300E000` | primary slot: MCUboot header 0x100, application at `0x2300E100` |
| `0x87000..0xFE000` | | secondary slot: OTA download (0x77000) |
| `0xFE000..0x100000` | | `SecurityStateJournal` (unchanged) |

The boot ROM maps XIP linearly from `img_start`, so physical `p` runs at
`0x23000000 + p - 0x2000`; no SF_CTRL remap is used. The last sector of each
slot holds the MCUboot trailer, so an image may be at most 0x76000 bytes and
the application at most 0x75C00.

The loader's ROM-hashed image ends before 0xFFE0: `bflb-mcu-tool` rounds
`img_len` up (0xE000 to 0xE010), so a loader image covering the whole region
would make the ROM hash the primary slot's header. The LoaderInfo record is
therefore placed separately by the release tool.

The product hands out two restricted views of the one flash device: the
journal sees only `0xFE000..0x100000`, the OTA writer can erase/program
only the secondary slot (and read below the journal). Neither can touch the
loader or the primary slot.

Persistence is integrated. Security-state and replay-counter journal writes
plus reset/resume were exercised on XT-ZB1 hardware; long-term journal
compaction still needs controlled validation.

## Bootloader

`products/bl702-xt-zb1/bootloader` (binary) uses `bootloaders/bl702-boot`
(core, host-tested) and the shared overwrite-only engine in
`mcuboot-image`, like the CC2340 loader:

1. the ROM boots the loader at `0x23000000` (32 MHz XTAL, same boot header
   clocks as before);
2. if the secondary slot carries an upgrade request (MCUboot trailer), its
   image is validated (SHA-256 and ECDSA P-256 against the key baked into
   the loader), rejected if it is a downgrade, then copied sector by sector
   into the primary slot through the ROM flash routines;
3. the copy is re-validated, then the request is cleared; an interrupted
   copy restarts from the still-valid secondary on the next boot;
4. the primary image is validated on every boot; its entry must be
   `0x2300E100`. The loader invalidates the L1C cache, masks interrupts,
   clears `mie` and jumps to the application's riscv-rt `_start`, which sets
   up its own `gp`, `sp`, trap vector, `.data` and `.bss`;
5. with no valid primary it prints the reason and halts (WFI, interrupts
   masked).

The loader is about 14 KiB (ROM image `img_len` 0x3700) and prints on the
application UART (GPIO14 TX, 2 Mbaud):

```text
bl702-boot: v1
bl702-boot: installing update ......      (only with a pending update)
bl702-boot: update installed
bl702-boot: app v0.1.0+0 @ 0x2300e100
```

The application cannot run without the loader: it is linked at
`0x2300E100` behind an MCUboot header.

## Build

```bash
python3 -m pip install --require-hashes --no-deps \
  -r tools/bl702-image-tool/build-requirements.txt
python3 -m pip install --require-hashes --no-deps --no-build-isolation \
  -r tools/bl702-image-tool/requirements.txt
python3 -m pip install imgtool==2.4.0
cd examples/bl702-sensor
./build-image.sh
```

Diagnostic logging:

```bash
BL702_DIAGNOSTIC_LOG=1 ./build-image.sh
```

`build-image.sh` builds the application and the loader, packages the
loader with `bflb-mcu-tool --build`, and runs `tools/bl702_ota.py release`,
which checks the boot header (magic, 32 MHz XTAL clocks, CRC, image hash,
`img_start`, end below 0xFFE0), the LoaderInfo key id, the application
entry and the embedded public key, then writes to
`target/riscv32imc-unknown-none-elf/release/ota/`:

| file | use |
|---|---|
| `bl702-xt-zb1-<label>-<ver>-full.bin` | first programming at `0x0` (covers `0x0..0x87000`) |
| `bl702-xt-zb1-<label>-<ver>-signed.bin` | MCUboot image (header 0x100, slot 0x77000) |
| `1234-0702-<file version>-bl702-xt-zb1-<label>-<ver>.zigbee` | Zigbee OTA file |

Signing key, as on the CC2340 flower product: `$BL702_OTA_KEY`, else
`~/.flower-keys/flower-own-ec-p256.pem`. The private key never enters the
repository. `build-image.sh` derives its public X‖Y (`bl702_ota.py key-xy`,
`imgtool getpub`) and builds both the loader and the application with it
through `BL702_OTA_KEY_XY`; without the variable they use
`products/bl702-xt-zb1/keys/own-ec-p256.xy.hex`. CI signs with a key
generated per run, so CI artifacts must not be installed on boards whose
loader trusts the real key.

Version: the Cargo version of `bl702-xt-zb1-product` (`0.1.0`) is the
MCUboot version, the Basic SW build id and the OTA file version
`0x25MMmmpp` (`0x25000100`). For a bench OTA test only,
`BL702_OTA_VERSION_OVERRIDE=0.1.1 ./build-image.sh` builds an otherwise
identical image with a higher version; it logs `TEST-ONLY version override`
and must never be published.

Current sizes (production / diagnostic, signed image against the 0x76000
image limit):

| artifact | production | diagnostic |
|---|---:|---:|
| raw application | 228,208 | 298,584 |
| signed image | 228,615 | 298,992 |
| margin to 0x76000 | 254,713 | 184,336 |
| loader ROM image (header + loader) | 22,272 | 22,272 |

OTA adds about 28 KiB of code against the `NoOta` image (P-256, SHA-256,
OTA client and writer); `.data`/`.bss` are unchanged.

## Flash and monitor

The first programming writes the full image at `0x0` once; later updates
arrive over Zigbee. The image covers `0x0..0x87000` only, so the security
journal at `0xFE000` is kept. Erase the secondary slot (`0x87000..0xFE000`)
as well when the board previously ran something else there.

With a USB/UART ISP programmer that writes raw files (for example the
lab's `FlashProgramming.program address=0 path=…-full.bin`), program the
`-full.bin` at address 0. Or connect the CH340 port, hold BOOT/GPIO28,
then:

```bash
python3 -m pip install bflb-iot-tool==1.10.0
./flash.sh
./monitor.sh
```

`flash.sh` writes the full image unchanged with `bflb-iot-tool --single`
(`bflb-mcu-tool` would wrap it in another boot header). It has not yet been
run on hardware. Do not infer persistence validation from a successful
program operation.

## OTA

The product's OTA identity is manufacturer `0x1234`, image type `0x0702`
(unused by the other products here and by the CC2340 flower), file version
`0x25MMmmpp`. The client queries the OTA server (coordinator, endpoint 1)
300 s after boot and then every 24 h, downloads 48-byte blocks into the
secondary slot, keeps polling fast for 120 s around OTA activity, verifies
the image (SHA-256 and signature against the key in the loader), marks the
upgrade, checkpoints the security state and resets into the loader. It
declines offers when no loader or an unknown loader key is detected (the
`OTA channel …` log line).

Serving an image with ZHA (zigpy local index provider):

```bash
python3 tools/bl702_ota.py index --output /tmp/zigpy_ota/index.json \
  /tmp/zigpy_ota/1234-0702-25000101-bl702-xt-zb1-diagnostic-0.1.1.zigbee
# copy /tmp/zigpy_ota/ to /config/zigpy_ota/ on Home Assistant
```

```yaml
zha:
  zigpy_config:
    ota:
      extra_providers:
        - type: zigpy_local
          index_file: /config/zigpy_ota/index.json
```

Index paths are relative to the index file. After a Home Assistant restart
the sensor's update entity offers the image; install it there, or wait for
the device's own query. `python3 tools/bl702_ota.py check FILE.zigbee`
validates a file (header, identity, version, MCUboot hash and signature
presence, slot fit). Downgrades are refused by the loader: to return to an
older image, program its full image again.

## Remaining gates

- loader and OTA on hardware (pending): first boot through the loader,
  a full OTA v0.1.0 → v0.1.1 with reboot into the new image, retained
  network state, power loss during the copy, P-256 verify time at 32 MHz,
  ROM erase/program with interrupts masked during the download, and the
  software reset;
- `flash.sh` raw programming with `bflb-iot-tool`;
- security-journal erase/program, power interruption, and reset recovery;
- GPADC battery accuracy;
- `Idle` sleep: WFI wake on the TIMER alarm, sleep current, and long-poll
  stability across a 71.6-minute counter wrap;
- PDS/HBN low-power modes (not implemented);
- RF output/spectral qualification;
- hardware address filtering and auto-ACK.
