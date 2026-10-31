# ESP32-C6 / ESP32-H2

The ESP products share a board flash resource, product profile/storage/OTA
implementation, and the platform-independent sleepy-sensor lifecycle.

## Layering

```text
environmental OTA profile
        ↓
products/esp32-zigbee-devkit (identity, policy, partitions, OTA)
        ↓
boards/esp32-zigbee-devkit (physical flash resource)
        ↓
esp-hal + esp-radio + EspRadioPhy
        ↓
zigbee-mac SoftMacCore (EspSoftMac)
```

The example root owns chip startup, radio resources and button/LED/temperature
adapters. The product supplies the shared interrupt-driven time driver and
single-future executor, button wake and optional retained PMU sleep.
`SensorApp` owns commissioning, parent polling, reporting, persistence
checkpoints, and OTA-first event routing.

Both targets use the Rust `esp-radio` IEEE 802.15.4 backend, with the existing
Espressif PHY binaries underneath; the Zigbee stack is not ESP-IDF/ZBOSS.
Local, version-pinned HAL/radio sources live in `vendor/`; their
`LOCAL-PATCHES.md` files record provenance and sleep changes.

The MAC is the shared `SoftMacCore` over `EspRadioPhy` (`EspSoftMac`):
scan, association, polling, CSMA-CA backoff and retries are the common
implementation. The radio supplies raw TX, hardware auto-ACK with frame-pending
control and a software Imm-Ack wait (`type Tx = AckWaitTx`; esp-radio has no
usable CCA/retry engine), and 10 dBm through
`RadioPhy::DEFAULT_TX_POWER_DBM`. This migration is build-, layout-, symbol-
and host-tested only; the hardware evidence below predates it.

## Application parts and policy

Both images provide a concrete
`esp32_zigbee_devkit_product::ota_transport::OtaTransport` paired with an
`WithOta` profile.

| build selection (both chips) | fast wait | slow wait | slow parent poll |
|---|---|---|---|
| default | `Active` | `Active` | 30 s |
| `--features light-sleep` (experimental) | `Active` | retained `Idle` | 5 s |

C6 has no status LED adapter; H2 keeps its active-low fitted LED.

`Active`/`Active` describes the radio/SoC policy, not a continuously spinning
CPU. The executor uses `WFI` while its root future is pending. SYSTIMER alarm 0
wakes expired Embassy timers and is rearmed for the next deadline; the
check-to-idle transition masks interrupts to avoid losing a wakeup. The HAL
monotonic counter is not reset at startup.

The default is **CPU idle only**, with radio RX enabled. Button and long-press
waits now use GPIO interrupts rather than a 10 ms polling timer.

The opt-in path stops a quiet MAC, disables the PHY and enters actual PMU
light sleep, retaining CPU state, RAM, flash power and digital/register
domains. GPIO9 uses digital low-level wake, not RTC EXT1. After wake, measured
LP-timer time corrects only the missing SYSTIMER ticks, overdue Embassy alarms
are serviced, and PHY/RX are restored before the application continues.
This is not reset-on-wake deep sleep, and it does not promise minimum-current
operation while so many domains remain retained.

Queued/raced RX, active TX/auto-ACK and pending interrupts veto suspension.
The application services that activity and checks the idle policy again.
Commissioning/TCLK, APS acknowledgements, queued protocol responses, rejoin,
interview/identify and active OTA keep short, radio-on waits. Sub-20 ms waits
remain active; a rejected PMU request is reported as rejection, not as sleep.
Pending UART transmission also vetoes entry. A native USB session observed
since boot keeps the device radio-on with an explicit warning: the existing
auto console does not own a sleep-safe USB suspend/restore protocol. For power
measurements, cold-boot from a power-only supply (or use the separate UART
console). Unplugging native USB without resetting is not enough; its console
selection latch is deliberately preserved.
Unexpected wake/restoration failures are fatal rather than silently falling
back to WFI. Five-second polling leaves margin under a typical parent queue
lifetime, but still needs verification with the actual coordinator/parent.

**Hardware acceptance is pending:** timer/button and simultaneous wakes,
rejected sleep, repeated sleep/wake, parent loss/rejoin, downlink commands,
reports, OTA, long-press reset, and measured current/temperature on both chips.
No hardware was flashed by this change. Defaults and the web-flasher image
remain radio-on until qualification; do not infer battery suitability from
successful compilation.

The reported temperature is the on-chip sensor reading, not ambient
temperature. Real self-heating is possible. No guessed temperature offset is
applied. Both chips select ESP-IDF's default −10…80 °C TSENS range (DAC 15,
offset 0) before every sample, verify it by read-back, and subtract the
per-chip sign-magnitude `TEMP_CALIB` eFuse delta exactly as ESP-IDF v5.5
does (`products/esp32-zigbee-devkit/src/chip_temperature.rs`). Earlier C6
builds used esp-hal's hard-coded offset −1 without programming the DAC,
which over-reports by 27.88 °C when the DAC is in the default range. C6,
like H2, powers its temperature sensor only around samples. Accuracy against
a reference thermometer is still a hardware gate, separate from measuring the
effect of sleep.

OTA cluster events reach `OtaTransport` before generic application handling.
When the image is ready, `SensorApp` checkpoints Zigbee security state before
the product writer changes `otadata` and resets.

## Product-owned flash layout

The checked 4 MiB table is:

```text
0x000000..0x008000  second-stage bootloader
0x008000..0x008C00  partition table
0x009000..0x00B000  redundant otadata sectors
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3FE000..0x400000  security journal within zbnv
```

The board crate exposes raw physical flash. The product checks the table,
bounds writes, selects the inactive slot, owns `otadata`, and constructs the
security journal.

If the on-device table is missing or incompatible, or the running application
cannot be identified safely, startup fails explicitly.
These OTA-capable images always advertise the OTA client cluster and do not
pretend an unsafe staging path exists.

The writer:

- identifies the executing slot from the live flash MMU translation, not the
  preferred `otadata` entry, which can remain stale after bootloader fallback;
- stages only in the other slot and rejects missing or changed running-image
  evidence;
- erases staging sectors lazily;
- pads the final 4-byte write with `0xFF`;
- validates segment count, lengths, load ranges, mapping alignment, descriptor,
  chip/revision compatibility, entry address, XOR checksum, exact image length,
  and appended SHA-256;
- rechecks staged flash before activation;
- writes one redundant `otadata` entry in the `ESP_OTA_IMG_NEW` state for
  activation, and refuses to stage while the running image is unconfirmed,
  because the other slot is then the rollback target.

### Pending verification and rollback

The espflash-bundled ESP-IDF bootloader is built without
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. It still honours the entry states
and sequence numbers, so rollback is application-driven
(`esp32_zigbee_devkit_product::ota::{check_boot, confirm_boot, roll_back}` and
the `ota_boot` policy):

1. Before persistence opens, `ota_boot::begin()` checks `otadata`. The first
   boot of a `NEW` image programs a boot-attempt mark into the entry's unused
   `seq_label` bytes, which only clears bits and needs no erase.
2. A fresh join, a secured rejoin, or an inbound decrypted ZCL frame confirms
   the image: its entry is rewritten as `VALID`.
3. A silent resume has no radio exchange. It confirms only after the image has
   run for 15 minutes without a failed rejoin, commissioning or wake.
4. A boot that finds its own mark without confirmation (crash, watchdog or
   power loss), the 15-minute deadline without evidence, a reached
   secure-rejoin limit (before the application wipes network state), or a
   security-store failure selects the previous slot. A `VALID` entry with a
   higher sequence is written to the spare sector first, then the abandoned
   entry is marked `ABORTED`, and the chip resets.
5. If the bootloader itself fell back from an invalid new image, the running
   slot is recorded as `VALID`.

Confirmation erases and rewrites the active sector. A power loss inside that
window boots the previous image. Any deliberate reset before confirmation also
rolls back.

`EspFirmwareWriter::new()` returns an initialization error if its flash
implementation cannot supply running-slot and image-compatibility evidence.
The standard sensor profile propagates this error instead of guessing a slot.

These checks cover unsigned plaintext image structure and integrity, not
secure-boot signatures, anti-rollback (version downgrade) policy, or program
correctness. The
offline size checker and OTA packager share the same Python structural
validator; on-device revision compatibility is checked by the writer rather
than guessed by offline tooling.

## Build and flash

Use the fixed `nightly-2026-08-01` and `espflash 4.5.0`:

```bash
cd examples/esp32c6-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc

cd ../esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
```

To build the retained-sleep variant in either example directory, add
`--features light-sleep`. This feature does not change partitions, OTA identity
or network persistence. CI links both opt-in images and runs the real-image
OTA corpus separately from the default images. Size reports measure the
artifacts without enforcing artificial regression budgets; physical OTA-slot
and other layout checks remain mandatory.

Documentation can be published after its core/code checks pass even when a
required firmware job fails a build, physical limit, or other check. In that
case the site includes no prebuilt ESP binaries, its firmware manifest has no
builds, and one-click installation is
disabled with a link to the failing CI run. The existing ESP32-C6, ESP32-H2 and
BL702 jobs must all succeed before prebuilt ESP firmware is published.
Exceeding a former regression budget no longer fails those jobs; binaries and
one-click installation can become available when all remaining checks pass.
The flasher's custom-file mode remains an explicit user-supplied operation,
not a qualification of that file or automatic flashing.

The configured runner installs the product partition table, writes `ota_0`,
and clears `otadata`:

```bash
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Back up commissioned state before the first partition-table migration.

Local integrated snapshot (2026-09-10), not remote CI or hardware results:

| image | application | former regression budget | merged flash | Zigbee OTA v1 | static `.data + .bss` |
|---|---:|---:|---:|---:|---:|
| ESP32-C6 default | 381,056 | 368,640 | 446,592 | 381,122 | 53,796 |
| ESP32-C6 `light-sleep` | 392,080 | 368,640 | 457,616 | 392,146 | 53,992 |
| ESP32-H2 default | 365,920 | 356,352 | 431,456 | 365,986 | 53,448 |
| ESP32-H2 `light-sleep` | 376,640 | 356,352 | 442,176 | 376,706 | 53,696 |

Each physical OTA slot is 2,031,616 B.
The RAM totals include `.data.wifi` (80 B on C6 and 84 B on H2), but exclude
RAM-resident code. Linked default/light stack regions are 390,696/390,504 B
for C6 and 201,312/200,640 B for H2, not measured high-water marks.
The applications passed structural/layout and host flash-mock staging,
verification, and activation checks. The hardware results below are earlier
path evidence, not exact-image reruns.
All four snapshots exceeded their then-enforced regression budgets:
default/light C6 by 12,416/23,440 B and H2 by 9,568/20,288 B. Those historical
failures remain recorded, but the former budgets are no longer build blockers.
The physical slot and other image/layout checks remain mandatory.
These sizes include the combined idle, OTA, reporting, and runtime changes;
their differences from earlier snapshots are not isolated per-fix costs.

## OTA packaging

From the selected example:

```bash
tools/create-ota.py 2 --toolchain nightly-2026-08-01
```

For the retained-sleep variant, use a separate output directory and explicitly
pass its feature again; a previous feature-selected build does not select the
OTA build's features:

```bash
tools/create-ota.py 2 target/ota-light-sleep \
  --toolchain nightly-2026-08-01 --features light-sleep
```

The tool builds the locked application with `ESP32_OTA_VERSION=2`, creates the Zigbee
OTA container, and updates a `zigpy_local` index. C6 and H2 use distinct image
types, so a server cannot offer one chip's image to the other.
Default and retained-sleep variants share the same chip-specific OTA identity:
do not publish both variants with the same version to one server index.
`--elf` packages an already selected image without rebuilding and cannot be
combined with `--features`.

Ordinary product host tests leave the artifact-dependent tests ignored.
Each CI firmware build explicitly enables them with `--include-ignored`,
using its freshly generated application and the shared Python/Rust validation
corpus alongside size reporting and physical image/layout checks. These
exercise flash mocks, not the physical device.

## Hardware validation

### ESP32-H2

An earlier 4 MiB ESP32-H2 revision 1.2 image demonstrated:

- migration to the security journal;
- secure reset/resume and reporting;
- complete v1→v2 ZHA OTA transfer;
- verification, activation, reboot into `ota_1`;
- retained IEEE address, PAN, parent, network credentials, and counters.

Fresh factory-reset commissioning and long-duration power behavior remain
separate gates. Pending verification, confirmation and rollback are host-tested
only; no OTA boot of either chip has exercised them. Neither current H2 variant above has hardware execution
evidence. Removing artificial size budgets does not close those hardware gates.

### ESP32-C6

Earlier C6 hardware runs demonstrated commissioning/reporting and OTA transfer
to 18.3% before intentional cancellation. Complete C6
verification/activation/reboot remains open. Neither current C6 variant above
has hardware execution evidence. The former regression budgets are not live
build gates.
