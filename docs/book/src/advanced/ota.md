# OTA Updates

OTA is split into independently owned concerns:

```text
Zigbee OTA cluster transport and retry policy   zigbee-runtime
image identity/version/profile composition      product
staging storage and verification                product FirmwareWriter
bootloader activation                           product
security/persistence checkpoint                 shared application/runtime
physical flash or bootloader connection         board/HAL
```

The board never chooses image identity, partitions, or upgrade policy.

## OTA-first sensor routing

`SensorApp` gives every stack event to the selected `OtaLifecycle` before
generic application matching:

```rust,ignore
match ota.handle_event(&mut node, event).await {
    OtaEventOutcome::NotHandled => { /* generic event match */ }
    OtaEventOutcome::Handled { activation_pending } => { /* ... */ }
    OtaEventOutcome::Unexpected => { /* diagnose invalid pairing */ }
}
```

The OTA implementation reports `activation_pending`; it must not reset from
`handle_event` or `service`. The shared application:

1. checkpoints network keys and security counters;
2. calls `OtaLifecycle::activate`.

This keeps a reset-causing bootloader transition behind the durable security
checkpoint.

OTA has no timed keep-awake window. The fast poll (with the CPU awake) is
held exactly while `OtaLifecycle::is_active` reports a session: query sent,
download, verification or activation wait. The session itself is bounded by
`RecoveryPolicy::max_ota_awake_ms` (abort plus anomaly reason 2) and by the
runtime's response timeout. A Query Next Image Response with
NO_IMAGE_AVAILABLE or any other non-success status, an Image Notify declined
by QueryJitter, a response timeout, a failure or an abort ends the session,
and the device returns to its slow poll on the next service iteration. An
OTA event never opens, extends or shortens the generic commissioning or
interview window. Periodic OTA service uses the same checkpoint/activation
path.
If reading or writing the security checkpoint fails, the application records
the persistence fault and stops before invoking activation.

The shared sensor host regressions cover both event-driven and service-driven
activation, failed checkpoint reads and counter-reservation writes, recovery
with non-reused counters, an Image Notify answered with NO_IMAGE_AVAILABLE
(no fast-poll tail), and OTA events that leave the interview window intact. These use a real `ZigbeeNode` with `MockMac` and an
instrumented store; they do not qualify a physical flash writer or bootloader.

## Profile pairing

`NoOta` implements `OtaLifecycle` only for `NonOtaProfile`. A profile owner
must explicitly declare that its component does not advertise the OTA client.
An OTA-decorated profile therefore cannot accidentally compile with `NoOta`.

ESP OTA images use `WithOta<BaseProfile, FirmwareWriter>`, so cluster `0x0019`
is part of the compile-time endpoint descriptor. A missing or incompatible
checked partition table is an explicit startup failure; it cannot turn an
OTA image into a differently described non-OTA device.

## Shared runtime pieces

| API | responsibility |
|---|---|
| `OtaManager` | Zigbee OTA client state machine and image-header policy |
| `OtaSession` | server lock, request/retry bookkeeping, APS delivery, session cleanup |
| `OtaLifecycle` | application-facing event/service/activation contract |
| `FirmwareWriter` | erase, write, verify, activate, abort, and slot bounds |

The product writer owns format validation beyond the Zigbee header, staging
geometry, boot selection, and reset.

## Lost blocks and interrupted downloads

`OtaManager` re-sends the outstanding Image Block Request until a response
arrives or the 120 s response timeout expires. The first retry is after 2 s,
the interval grows by 1 s per retry, and it is capped at 5 s, so a server that
aborts after 30 s without progress (zigpy) sees several retries first. Late or
mismatched blocks never consume the retry copy.

A response timeout or a server ABORT during a download that has already staged
data suspends the session instead of discarding it: the writer is not aborted,
and the server is not sent an Upgrade End. When the next Query Next Image
response offers the same manufacturer code, image type, file version and size,
the download continues at the saved offset without erasing the slot. Any other
offer, an explicit abort, or a verification failure discards the staged data
and starts again at offset 0. One BL702 hardware run resumed after a ZHA
server abort at offset 12,720 (commit `8aeee49`); the other products have
host tests only.

### Resume after a reset

The suspended state lives in RAM. A writer that reports a non-zero
`FirmwareWriter::checkpoint_interval` also keeps progress across a reset or
power cut:

- `OtaManager` calls `FirmwareWriter::checkpoint` with the offer identity
  and the staged length each time the staged payload crosses a multiple of
  the interval. It never records per block, and not for the block that
  completes the payload.
- `OtaManager::new` asks the writer for its record
  (`FirmwareWriter::resume`). A record for this client's manufacturer code and
  image type restores a suspended download. The next offer of the same file
  version and size continues at the offset the writer reports, which may be
  earlier than the last checkpoint. Any other offer erases the slot as usual.
- The default trait methods keep no progress and compile away, so products
  without a resumable writer are unchanged; a reset restarts at offset 0.

`mcuboot-image::SlotWriter::with_resume_checkpoints` (overwrite mode only)
keeps the records at the start of the secondary slot's trailer sector: one
32-byte header (identity and CRC-32) and append-only 8-byte entries
(`staged`, `!staged`). A torn entry fails its complement and is skipped.
The writer resumes at the last entry rounded down to an erase sector and
erases that sector again before writing it. The loader reads only the
request magic at the end of that sector and erases the whole sector after an
install or a dropped request; `erase_slot` and `abort` erase it too. The
records therefore cost no erase of their own, never touch the NV journal,
and disappear with the download they describe. The final signature check
still covers the whole image.

| product | interval | status |
|---|---|---|
| BL702 XT-ZB1 | 8 KiB (`OTA_RESUME_CHECKPOINT_BYTES`, about 30 entries per image) | host-tested on the product layout |
| ESP32-C6/H2, EFR32MG1 | none (writer defaults) | a reset restarts the download |

## Reaching a sleepy device

ZHA (zigpy) serves an image only while `update.install` is running. That
call starts with a unicast Image Notify and fails immediately if the notify
gets no APS acknowledgement. Outside an install, zigpy answers a device's own
Query Next Image with NO_IMAGE_AVAILABLE, so periodic queries cannot start
an update. A sleepy device is therefore reachable for OTA only when:

- its slow poll fetches the queued notify before the parent drops it
  (`SensorPolicy::slow_poll_within_parent_hold`: the slow poll is at most
  `PARENT_INDIRECT_HOLD_MIN_MS`, the shortest common parent indirect hold,
  6 s; the IEEE 802.15.4 default `macTransactionPersistenceTime` is 7.68 s);
  or
- a user action opens a fast-poll window (`ForceReportAction`) before the
  install is started.

`SensorPolicy::ota_reachable` combines both; a product can assert it at
compile time. A product that fails it can still be updated: the sensor polls
fast for a bounded window after a boot with a restored network
(`restored_fast_ms`) and after a join or rejoin (`fresh_join_fast_ms`), so
start the install inside that window.

| product | slow poll | short press | reachable while idle |
|---|---|---|---|
| BL702 XT-ZB1 | 10 s | none | no: only in the 120 s window after a reset or rejoin |
| EFR32MG1 TRADFRI | 30 s | force report (fast-poll window) | after a button press |
| ESP32-C6/H2 devkit | 30 s | join toggle | no (open) |

## Platform implementations

| product | staging/activation | status |
|---|---|---|
| ESP32-C6 | inactive `ota_0`/`ota_1`, appended SHA-256, redundant `otadata` | prior C6 run reached 18.3%; current exact image has build/layout evidence only |
| ESP32-H2 | same product writer/layout | prior H2 run demonstrated v1→v2 activation/reboot/network retention; current exact image has build/layout evidence only |
| EFR32MG1 | Gecko Bootloader storage slot 0 through `BootloaderFlashAccess` | writer implemented; real Zigbee download/install/reboot open |
| nRF52840/52833 | no writer in current products | `NoOta` |
| BL702 | overwrite-only `bl702-xt-zb1-boot` loader (`bootloaders/bl702-boot` shell of the [bootloader core](./bootloader.md), SHA-256 + ECDSA P-256, downgrades refused); download into the secondary slot `0x87000..0xFE000` | one XT-ZB1 upgraded 0.1.0→0.1.1 over ZHA (299 KB, ~33 min), resumed after a server abort, booted the new image and kept its network (commit `8aeee49`). The v2 loader (trial boot and confirm) upgraded 0.2.0→0.2.1 over ZHA and survived 46 manual power cuts in a lab soak ([results](./bootloader.md#results-2026-11-08)) |
| CC2340R5 | `bootloaders/cc2340r5-boot` shell of the [bootloader core](./bootloader.md); no product loader binary or writer | `NoOta`; loader build/host-tested only |
| PHY6222/PB-03F/EFR32MG21/TLSR8258 | no product writer | `NoOta` |

## ESP32 layout

```text
0x009000..0x00B000  otadata
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3FE000..0x400000  security journal
```

The writer stages only into the inactive slot, erases sectors lazily, pads a
ragged final word with `0xFF`, and re-reads the image to verify magic, chip ID,
and SHA-256. OTA writes cannot enter `zbnv`.

Package a new image from the selected example:

```bash
tools/create-ota.py 2
```

The build-time version is shared by the Basic cluster and OTA container. C6
and H2 use distinct image types. The prior 2026-09-06 containers are
368,098 B and 352,994 B respectively; these packages are build/package-tested,
not exact-image HIL reruns, and have not been regenerated for the
2026-09-08 application images.

## EFR32MG1 ownership

The board external-flash token can be consumed for either:

- direct USART0 SPI access; or
- `BootloaderFlashAccess` for the resident Gecko Bootloader.

The product OTA writer retains the bootloader-access marker. Direct SPI and
bootloader-managed access cannot be constructed concurrently through the typed
resource path.

## Completion criteria

An OTA path is complete only after hardware proves:

1. a real newer version is offered and downloaded;
2. staging stays inside its partition;
3. image identity/version and integrity checks pass;
4. activation selects the new image;
5. the device reboots into the new version;
6. commissioned network state and non-reused counters survive;
7. protected NV, factory, and bootloader regions remain intact;
8. interrupted download/activation has a defined recovery or rollback path.

A compiled `FirmwareWriter`, successful transfer fragment, or bootloader API
call is not by itself complete OTA support.

The BL702 run above covers items 1–5, network retention from item 6, and the
download half of item 8 on one device; a counter-reuse check, region integrity
(item 7) and copy-time power loss (item 8) remain open. No current exact ESP32 or EFR32 image is documented as
rerunning its hardware OTA demonstration.
