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

## Platform implementations

| product | staging/activation | status |
|---|---|---|
| ESP32-C6 | inactive `ota_0`/`ota_1`, appended SHA-256, redundant `otadata` | transfer hardware-tested through 18.3%; complete activation open |
| ESP32-H2 | same product writer/layout | full v1→v2 activation/reboot/network retention proven |
| EFR32MG1 | Gecko Bootloader storage slot 0 through `BootloaderFlashAccess` | writer implemented; real Zigbee download/install/reboot open |
| nRF52840/52833 | no writer in current products | `NoOta` |
| BL702 | no OTA partition/writer | `NoOta`; raw flash validation still open |
| PHY6222/CC2340/EFR32MG21/TLSR8258 | no product writer | `NoOta` |

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
and H2 use distinct image types.

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
