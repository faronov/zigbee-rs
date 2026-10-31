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
    OtaEventOutcome::Handled { keep_awake_ms, activation_pending } => { /* ... */ }
    OtaEventOutcome::Unexpected => { /* diagnose invalid pairing */ }
}
```

The OTA implementation reports `activation_pending`; it must not reset from
`handle_event` or `service`. The shared application:

1. extends the active/fast-poll window;
2. checkpoints network keys and security counters;
3. calls `OtaLifecycle::activate`.

This keeps a reset-causing bootloader transition behind the durable security
checkpoint.

An OTA-handled event retains the lifecycle's explicit `keep_awake_ms` window;
generic commissioning activity must not replace it with a shorter or longer
deadline. Periodic OTA service uses the same checkpoint/activation path.
If reading or writing the security checkpoint fails, the application records
the persistence fault and stops before invoking activation.

The shared sensor host regressions cover both event-driven and service-driven
activation, failed checkpoint reads and counter-reservation writes, recovery
with non-reused counters, and OTA windows shorter and longer than the normal
commissioning window. These use a real `ZigbeeNode` with `MockMac` and an
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
and starts again at offset 0. Resume state lives in RAM only, so a reset still
restarts the download. One BL702 hardware run resumed after a ZHA server abort
at offset 12,720 (commit `8aeee49`); the other products have host tests
only.

## Platform implementations

| product | staging/activation | status |
|---|---|---|
| ESP32-C6 | inactive `ota_0`/`ota_1`, appended SHA-256, redundant `otadata` | prior C6 run reached 18.3%; current exact image has build/layout evidence only |
| ESP32-H2 | same product writer/layout | prior H2 run demonstrated v1→v2 activation/reboot/network retention; current exact image has build/layout evidence only |
| EFR32MG1 | Gecko Bootloader storage slot 0 through `BootloaderFlashAccess` | writer implemented; real Zigbee download/install/reboot open |
| nRF52840/52833 | no writer in current products | `NoOta` |
| BL702 | overwrite-only `bl702-xt-zb1-boot` loader (`bootloaders/bl702-boot` + `mcuboot-image`, SHA-256 + ECDSA P-256, downgrades refused); download into the secondary slot `0x87000..0xFE000` | one XT-ZB1 upgraded 0.1.0→0.1.1 over ZHA (299 KB, ~33 min), resumed after a server abort, booted the new image and kept its network (commit `8aeee49`); power loss during the copy untested |
| CC2340R5 | `bootloaders/cc2340r5-boot` loader core on `mcuboot-image`; no product writer | `NoOta`; loader build/host-tested only |
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
