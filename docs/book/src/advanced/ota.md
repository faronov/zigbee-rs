# OTA Updates

The Over-the-Air (OTA) Upgrade cluster (cluster ID `0x0019`) lets you update
device firmware over the Zigbee network. zigbee-rs implements the OTA client
state machine and provides a `FirmwareWriter` trait that platform backends
implement to write the downloaded image to flash.

---

## OTA Upgrade Cluster Overview

The OTA cluster is defined in `zigbee_zcl::clusters::ota`. It defines
attributes that track upgrade state and commands that drive the download
protocol.

### Attributes

```rust
pub const ATTR_UPGRADE_SERVER_ID:       AttributeId = AttributeId(0x0000);
pub const ATTR_FILE_OFFSET:             AttributeId = AttributeId(0x0001);
pub const ATTR_CURRENT_FILE_VERSION:    AttributeId = AttributeId(0x0002);
pub const ATTR_CURRENT_STACK_VERSION:   AttributeId = AttributeId(0x0003);
pub const ATTR_DOWNLOADED_FILE_VERSION: AttributeId = AttributeId(0x0004);
pub const ATTR_DOWNLOADED_STACK_VERSION: AttributeId = AttributeId(0x0005);
pub const ATTR_IMAGE_UPGRADE_STATUS:    AttributeId = AttributeId(0x0006);
pub const ATTR_MANUFACTURER_ID:         AttributeId = AttributeId(0x0007);
pub const ATTR_IMAGE_TYPE_ID:           AttributeId = AttributeId(0x0008);
pub const ATTR_MIN_BLOCK_PERIOD:        AttributeId = AttributeId(0x0009);
```

### Commands

| Direction | Command | ID | Purpose |
|-----------|---------|---:|---------|
| Client → Server | `QueryNextImageRequest` | 0x01 | Ask if a new image is available |
| Server → Client | `QueryNextImageResponse` | 0x02 | Respond with image info or "no update" |
| Client → Server | `ImageBlockRequest` | 0x03 | Request a data block at a given offset |
| Server → Client | `ImageBlockResponse` | 0x05 | Deliver a block (or tell client to wait) |
| Server → Client | `ImageNotify` | 0x00 | Proactively tell client an update exists |
| Client → Server | `UpgradeEndRequest` | 0x06 | Report download success or failure |
| Server → Client | `UpgradeEndResponse` | 0x07 | Tell client when to activate |

### Image Upgrade Status Values

```rust
pub const STATUS_NORMAL:               u8 = 0x00;  // idle
pub const STATUS_DOWNLOAD_IN_PROGRESS: u8 = 0x01;
pub const STATUS_DOWNLOAD_COMPLETE:    u8 = 0x02;
pub const STATUS_WAITING_TO_UPGRADE:   u8 = 0x03;
pub const STATUS_COUNT_DOWN:           u8 = 0x04;
pub const STATUS_WAIT_FOR_MORE:        u8 = 0x05;
```

---

## OTA Image Format

OTA images use a standard header defined in `zigbee_zcl::clusters::ota_image`.
The file starts with a fixed header, followed by optional fields, followed by
one or more sub-elements (the actual firmware binary, signatures, etc.).

### Header Structure

```rust
pub struct OtaImageHeader {
    pub magic: u32,                    // must be 0x0BEEF11E
    pub header_version: u16,           // 0x0100 for ZCL 7+
    pub header_length: u16,            // total header size in bytes
    pub field_control: OtaHeaderFieldControl,
    pub manufacturer_code: u16,
    pub image_type: u16,               // manufacturer-specific
    pub file_version: u32,             // new firmware version
    pub stack_version: u16,
    pub header_string: [u8; 32],       // human-readable description
    pub total_image_size: u32,         // header + payload

    // Optional fields (controlled by field_control bits)
    pub security_credential_version: Option<u8>,
    pub min_hardware_version: Option<u16>,
    pub max_hardware_version: Option<u16>,
}
```

The minimum header size is **56 bytes** (`OTA_HEADER_MIN_SIZE`). The magic
number `0x0BEEF11E` is checked during parsing to reject corrupt or non-OTA
files.

### Field Control Bits

```rust
pub struct OtaHeaderFieldControl {
    pub security_credential: bool,  // bit 0: credential version present
    pub device_specific: bool,      // bit 1: device-specific file
    pub hardware_versions: bool,    // bit 2: HW version range present
}
```

### Sub-Elements

After the header, the image contains sub-elements, each with a 6-byte header
(2-byte tag + 4-byte length):

```rust
pub struct OtaSubElement {
    pub tag: OtaTagId,
    pub length: u32,
}

pub enum OtaTagId {
    UpgradeImage   = 0x0000,  // the actual firmware binary
    EcdsaCert      = 0x0001,  // signing certificate
    EcdsaSignature = 0x0002,  // ECDSA signature
    ImageIntegrity = 0x0003,  // hash for integrity check
    PictureData    = 0x0004,  // optional picture data
}
```

The `UpgradeImage` sub-element contains the raw firmware binary that gets
written to the device's update flash slot.

---

## Upgrade Flow

The OTA client state machine (`OtaState`) drives the entire process:

```text
                    ┌───────┐
                    │ Idle  │
                    └───┬───┘
                        │ QueryNextImageRequest
                        ▼
                  ┌───────────┐
                  │ QuerySent │
                  └─────┬─────┘
           server has    │    no update
           new image     │    available
              ┌──────────┴──────────┐
              ▼                     ▼
     ┌──────────────┐          (back to Idle)
     │ Downloading  │
     │  offset=0    │◄─────────────┐
     │  total=N     │              │
     └──────┬───────┘    ┌─────────────────┐
            │            │  WaitForData    │
            │ block resp │  (server busy)  │
            ├───────────►│  delay N secs   │
            │            └─────────────────┘
            │ all blocks
            ▼
     ┌───────────┐
     │ Verifying │  ── verify hash/size
     └─────┬─────┘
            │
            ▼
  ┌────────────────────┐
  │ WaitingActivate    │  ── UpgradeEndRequest sent
  └────────┬───────────┘
           │ UpgradeEndResponse (activate now)
           ▼
     ┌──────────┐
     │   Done   │  ── reboot and run new firmware
     └──────────┘
```

### OtaState Enum

```rust
pub enum OtaState {
    Idle,
    QuerySent,
    Downloading { offset: u32, total_size: u32 },
    Verifying,
    WaitingActivate,
    WaitForData {
        delay_secs: u32,
        elapsed: u32,
        download_offset: u32,
        download_total: u32,
    },
    Done,
    Failed,
}
```

### OtaAction — What the Runtime Should Do Next

After processing each OTA command, the engine returns an `OtaAction`:

```rust
pub enum OtaAction {
    SendQuery(QueryNextImageRequest),
    SendBlockRequest(ImageBlockRequest),
    WriteBlock { offset: u32, data: heapless::Vec<u8, 64> },
    SendEndRequest(UpgradeEndRequest),
    ActivateImage,
    Wait(u32),
    None,
}
```

The runtime event loop dispatches these actions to the MAC layer (for sending
ZCL commands) or to the `FirmwareWriter` (for writing blocks to flash).

### Block Size

The default block size is **48 bytes** (`DEFAULT_BLOCK_SIZE`), chosen to fit
within a single MAC frame without requiring APS fragmentation. On networks with
reliable links, this can be tuned up to ~64 bytes.

### Rate Limiting (WaitForData)

If the OTA server is busy or needs to throttle downloads, it responds with a
`WaitForData` status instead of image data. The client pauses for the specified
number of seconds, then resumes from the saved offset.

---

## Shared Runtime Session Transport

`zigbee_runtime::ota::OtaManager` owns the Zigbee OTA protocol state machine
and image-header checks. `zigbee_runtime::ota_transport::OtaSession` owns the
common network bookkeeping used by EFR32MG1 and ESP32-C6: locking a transfer
to one server, forwarding OTA commands, retrying queued APS requests, cleaning
up failed sessions, and deferring activation until the application has
checkpointed security state.

Storage access, image-format validation beyond the Zigbee header, version
policy, boot-slot selection, activation, and reset remain in each product's
`FirmwareWriter`. `OptionalOta` lets ESP32-C6 omit the OTA client cluster when
the checked on-device partition table is incompatible; normal commissioning
and reporting continue. ESP32-H2 has no OTA backend or OTA client in the
current product.

---

## FirmwareWriter Trait

The `FirmwareWriter` trait (`zigbee_runtime::firmware_writer`) abstracts the
platform-specific flash operations needed to store a downloaded firmware image:

```rust
pub trait FirmwareWriter {
    /// Erase the firmware update slot, preparing it for writes.
    fn erase_slot(&mut self) -> Result<(), FirmwareError>;

    /// Write a block of data at the given offset within the update slot.
    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError>;

    /// Verify the written image (size check + optional hash).
    fn verify(
        &mut self,
        expected_size: u32,
        expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError>;

    /// Mark the new image as pending activation (bootloader swap on reboot).
    fn activate(&mut self) -> Result<(), FirmwareError>;

    /// Return the maximum image size this slot can hold.
    fn slot_size(&self) -> u32;

    /// Abort an in-progress update and revert.
    fn abort(&mut self) -> Result<(), FirmwareError>;
}
```

### FirmwareError

```rust
pub enum FirmwareError {
    EraseFailed,
    WriteFailed,
    VerifyFailed,   // hash mismatch or size mismatch
    OutOfRange,     // offset beyond slot boundary
    ImageTooLarge,  // image exceeds slot_size()
    ActivateFailed, // boot flag not set
    HardwareError,
}
```

### Platform Implementations

| Platform | Writer | Slot Location | Status |
|----------|--------|---------------|--------|
| EFR32MG1 | `efr32mg1_tradfri_product::ota::Efr32FirmwareWriter` | Gecko Bootloader storage slot 0 (external SPI flash) | Implemented |
| ESP32-C6 | `esp32_zigbee_devkit_product::ota::EspFirmwareWriter` | Inactive `ota_0`/`ota_1` partition, selected through `otadata` | Implemented and host-tested; real upgrade still pending |
| ESP32-H2 | — | — | Not implemented; product intentionally omits OTA |
| Mock | `zigbee_runtime::firmware_writer::MockFirmwareWriter` | RAM buffer (`heapless::Vec<u8, 262144>`) | For host testing — 256 KB max |
| nRF52840 | — | Secondary flash bank via NVMC | Not implemented |
| BL702 | — | XIP flash via `bl702-pac` | Not implemented |

> Only the platforms marked *Implemented* have a `FirmwareWriter`; the others
> are design sketches for future backends.

The EFR32MG1 writer can only be constructed with the
`BootloaderFlashAccess` marker obtained by consuming the board's external-flash
token. The product profile retains that marker for the writer's lifetime, so
the direct USART0 SPI path and Gecko Bootloader storage API cannot both be
selected through the typed board API.

#### ESP32 specifics

The ESP writer stages into whichever of `ota_0`/`ota_1` is *not* running,
erases 4 KiB sectors lazily as data reaches them (a whole-slot erase would mask
interrupts for seconds), pads the ragged final Zigbee block up to the 4-byte
flash write granularity with `0xFF`, and verifies the staged slot by re-reading
it: image magic, chip ID and the SHA-256 appended by `espflash save-image` must
all match. Activation writes a single 32-byte `otadata` entry into the
redundant sector and resets. See the
[ESP32 platform guide](../platform-guides/esp32.md) for the partition table and
the packaging tool.

### MockFirmwareWriter (for Testing)

```rust
use zigbee_runtime::firmware_writer::MockFirmwareWriter;

let mut writer = MockFirmwareWriter::new(128_000);  // 128 KB slot

writer.erase_slot().unwrap();
writer.write_block(0, &firmware_chunk_0).unwrap();
writer.write_block(chunk_0_len, &firmware_chunk_1).unwrap();
writer.verify(total_size, None).unwrap();
writer.activate().unwrap();

assert!(writer.is_activated());
assert_eq!(writer.bytes_written(), total_size);
```

The mock writer enforces sequential writes (offset must equal the number of
bytes already written) and requires `erase_slot()` before any writes, just
like real flash hardware.

---

## Integration with Bootloaders

OTA is a two-part process: the Zigbee stack downloads and writes the image,
then the **bootloader** handles the swap and boot.

### Typical Flow

1. `FirmwareWriter::erase_slot()` — erase the secondary/staging flash area.
2. `FirmwareWriter::write_block()` — called once per OTA block (48 bytes
   each, potentially thousands of calls for a large image).
3. `FirmwareWriter::verify()` — check the written size and optional hash.
4. `FirmwareWriter::activate()` — set a boot flag or swap marker telling the
   bootloader to run the new image on next boot.
5. **Reboot** — the runtime triggers a system reset.
6. **Bootloader** — detects the pending update flag, validates the new image
   (CRC, signature), and swaps it into the primary slot.

### Bootloader Examples

| Platform | Bootloader | Swap Method |
|----------|-----------|-------------|
| EFR32MG1 | Silicon Labs Gecko Bootloader | GBL install from storage slot |
| ESP32 | ESP-IDF second stage bootloader | `otadata` sequence number selects `ota_0`/`ota_1` |
| nRF52840 | MCUboot / nRF Bootloader | Dual-bank swap (planned) |
| BL702 | BL702 ROM bootloader | XIP remap (planned) |

> **Rollback:** If the new firmware fails to start (e.g., crashes in a loop),
> most bootloaders support automatic rollback — they detect that the new image
> never confirmed itself and revert to the previous working image.
