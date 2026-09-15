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
esp-hal + esp-radio + EspMac
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
applied. C6, like H2, now powers its temperature sensor only around samples.
Validating TSENS range and calibration is separate from measuring the effect
of sleep.

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
- writes one redundant `otadata` entry for activation.

`EspFirmwareWriter::new()` returns an initialization error if its flash
implementation cannot supply running-slot and image-compatibility evidence.
The standard sensor profile propagates this error instead of guessing a slot.

These checks cover unsigned plaintext image structure and integrity, not
secure-boot signatures, anti-rollback policy, or program correctness. The
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
OTA corpus separately from the default images, with unchanged size budgets.

Documentation can be published after its core/code checks pass even when a
firmware size gate fails. In that case the site includes no prebuilt ESP
binaries, its firmware manifest has no builds, and one-click installation is
disabled with a link to the failing CI run. The existing ESP32-C6, ESP32-H2 and
BL702 job gates must all pass before prebuilt ESP firmware is published.
The flasher's custom-file mode remains an explicit user-supplied operation,
not a qualification of that file or automatic flashing.

The configured runner installs the product partition table, writes `ota_0`,
and clears `otadata`:

```bash
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Back up commissioned state before the first partition-table migration.

Local integrated snapshot (2026-09-10), not remote CI or hardware results:

| image | application | regression gate | merged flash | Zigbee OTA v1 | static `.data + .bss` |
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
All four images exceed their unchanged regression budgets: default/light C6
by 12,416/23,440 B and H2 by 9,568/20,288 B. Physical slot fit does not clear
those release blockers.
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
corpus before applying the size gate. These exercise flash mocks, not the
physical device.

## Hardware validation

### ESP32-H2

An earlier 4 MiB ESP32-H2 revision 1.2 image demonstrated:

- migration to the security journal;
- secure reset/resume and reporting;
- complete v1→v2 ZHA OTA transfer;
- verification, activation, reboot into `ota_1`;
- retained IEEE address, PAN, parent, network credentials, and counters.

Fresh factory-reset commissioning and long-duration power behavior remain
separate gates. Neither current H2 variant above has hardware execution
evidence, and both exceed their regression budget.

### ESP32-C6

Earlier C6 hardware runs demonstrated commissioning/reporting and OTA transfer
to 18.3% before intentional cancellation. Complete C6
verification/activation/reboot remains open. Neither current C6 variant above
has hardware execution evidence, and both fail their regression size gate.
