# Error & Status Types

This chapter catalogues every error and status enum in zigbee-rs, organised by
stack layer from bottom (MAC) to top (BDB). Use the hex codes to correlate
with Zigbee specification tables and on-the-wire values.

---

## MAC Layer

### `MacError`

**Crate:** `zigbee-mac` · **Spec ref:** IEEE 802.15.4 status codes

General-purpose error returned by MAC primitives. No numeric discriminants —
these are Rust-only symbolic values.

| Variant | Meaning |
|---------|---------|
| `NoBeacon` | No beacon received during an active or passive scan |
| `InvalidParameter` | A primitive was called with an out-of-range parameter |
| `RadioError` | The radio hardware reported an unrecoverable error |
| `ChannelAccessFailure` | CSMA-CA failed — the channel remained busy for all back-off attempts |
| `NoAck` | No acknowledgement frame was received after transmission |
| `FrameTooLong` | The assembled MPDU exceeds the PHY maximum frame size |
| `Unsupported` | The requested operation is not supported by this radio backend |
| `SecurityError` | Frame security processing (encryption / MIC) failed |
| `TransactionOverflow` | The indirect-transmit queue is full |
| `TransactionExpired` | An indirect frame was not collected before the persistence timer expired |
| `ScanInProgress` | A scan request was issued while another scan is already active |
| `TrackingOff` | Superframe tracking was lost (beacon-enabled networks) |
| `AssociationDenied` | The coordinator denied the association request |
| `PanAtCapacity` | The coordinator indicated the PAN is at capacity |
| `Other` | Catch-all for unmapped / unknown errors |
| `NoData` | A data request (poll) returned no pending frame within the timeout |

### `AssociationStatus`

**Crate:** `zigbee-mac` · **File:** `primitives.rs` · **Spec ref:** IEEE 802.15.4 Table 83

Returned by the coordinator in an Association Response command.

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Association was successful |
| `PanAtCapacity` | `0x01` | PAN is at capacity — no room for new devices |
| `PanAccessDenied` | `0x02` | Access to the PAN is denied |

### `RadioError` (platform-specific)

Each radio backend defines its own `RadioError` enum. The variants are similar
but not identical across platforms.

#### CC2340 backend

| Variant | Meaning |
|---------|---------|
| `TxFailed` | Transmission failed (generic) |
| `ChannelBusy` | CCA indicated a busy channel |
| `NoAck` | No acknowledgement from the receiver |
| `Timeout` | Operation timed out |
| `HardwareError` | Radio hardware fault |

#### BL702 backend

| Variant | Meaning |
|---------|---------|
| `CcaFailure` | Clear Channel Assessment failure — channel is busy |
| `TxAborted` | TX was aborted by hardware |
| `HardwareError` | Radio hardware error during TX |
| `InvalidFrame` | Frame too long or too short |
| `CrcError` | Received frame failed CRC check |
| `NotInitialized` | Radio driver has not been initialized |

#### Telink backend

| Variant | Meaning |
|---------|---------|
| `CcaFailure` | CCA failure — channel is busy |
| `TxAborted` | TX was aborted |
| `HardwareError` | Radio hardware error |
| `InvalidFrame` | Frame too long or too short |
| `CrcError` | Received frame failed CRC check |
| `NotInitialized` | Radio not initialized |

### `RclCommandStatus` (CC2340 only)

**Crate:** `zigbee-mac` · **File:** `cc2340/driver.rs`

Low-level Radio Control Layer status on the CC2340.

| Variant | Code | Meaning |
|---------|------|---------|
| `Idle` | `0x0000` | Command is idle |
| `Active` | `0x0001` | Command is currently executing |
| `Finished` | `0x0101` | Command completed successfully |
| `ChannelBusy` | `0x0801` | CCA failed — channel busy |
| `NoAck` | `0x0802` | No acknowledgement received |
| `RxErr` | `0x0803` | Receive error |
| `Error` | `0x0F00` | Generic hardware error |

### `Lmac154TxStatus` (BL702 only)

**Crate:** `zigbee-mac` · **File:** `bl702/driver.rs`

TX completion status from the BL702 lower-MAC.

| Variant | Code | Meaning |
|---------|------|---------|
| `TxFinished` | `0` | Transmission completed successfully |
| `CsmaFailed` | `1` | CSMA-CA procedure failed |
| `TxAborted` | `2` | TX was aborted |
| `HwError` | `3` | Hardware error |

---

## NWK Layer

### `NwkStatus`

**Crate:** `zigbee-nwk` · **Spec ref:** Zigbee spec Table 3-70

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Operation completed successfully |
| `InvalidParameter` | `0xC1` | A parameter was out of range or invalid |
| `InvalidRequest` | `0xC2` | The request is invalid in the current state |
| `NotPermitted` | `0xC3` | Operation not permitted (e.g. security policy) |
| `StartupFailure` | `0xC4` | Network startup (formation or join) failed |
| `AlreadyPresent` | `0xC5` | An entry already exists (e.g. duplicate address) |
| `SyncFailure` | `0xC6` | Synchronisation with the parent lost |
| `NeighborTableFull` | `0xC7` | The neighbour table has no room for a new entry |
| `UnknownDevice` | `0xC8` | The specified device is not in the neighbour table |
| `UnsupportedAttribute` | `0xC9` | NIB attribute identifier is not recognized |
| `NoNetworks` | `0xCA` | No networks were found during the scan |
| `MaxFrmCounterReached` | `0xCC` | The outgoing frame counter has reached its maximum |
| `NoKey` | `0xCD` | No matching network key found for decryption |
| `BadCcmOutput` | `0xCE` | CCM* encryption / decryption produced invalid output |
| `RouteDiscoveryFailed` | `0xD0` | Route discovery did not find a path to the destination |
| `RouteError` | `0xD1` | A routing error occurred (e.g. link failure) |
| `BtTableFull` | `0xD2` | The broadcast transaction table is full |
| `FrameNotBuffered` | `0xD3` | A frame could not be buffered for later transmission |
| `FrameTooLong` | `0xD4` | The NWK frame exceeds the maximum allowed size |

### `RouteStatus`

**Crate:** `zigbee-nwk` · **File:** `routing.rs`

Internal status of a routing table entry.

| Variant | Meaning |
|---------|---------|
| `Active` | Route is valid and in use |
| `DiscoveryUnderway` | Route discovery has been initiated |
| `DiscoveryFailed` | Route discovery completed without finding a route |
| `Inactive` | Route exists but is not currently active |
| `ValidationUnderway` | Route is being validated (e.g. many-to-one route) |

---

## APS Layer

### `ApsStatus`

**Crate:** `zigbee-aps` · **Spec ref:** Zigbee spec Table 2-27

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Request executed successfully |
| `AsduTooLong` | `0xA0` | ASDU is too large and fragmentation is not supported |
| `DefragDeferred` | `0xA1` | A fragmented frame could not be defragmented at this time |
| `DefragUnsupported` | `0xA2` | Device does not support fragmentation / defragmentation |
| `IllegalRequest` | `0xA3` | A parameter value was out of range |
| `InvalidBinding` | `0xA4` | UNBIND request failed — binding table entry not found |
| `InvalidParameter` | `0xA5` | GET/SET request used an unknown attribute identifier |
| `NoAck` | `0xA6` | APS-level acknowledged transmission received no ACK |
| `NoBoundDevice` | `0xA7` | Indirect (binding) transmission found no bound devices |
| `NoShortAddress` | `0xA8` | Group-addressed transmission found no matching group entry |
| `TableFull` | `0xA9` | Binding table or group table is full |
| `UnsecuredKey` | `0xAA` | Frame was secured with a link key not in the key table |
| `UnsupportedAttribute` | `0xAB` | GET/SET request used an unsupported attribute identifier |
| `SecurityFail` | `0xAD` | An unsecured frame was received when security was required |
| `DecryptionError` | `0xAE` | APS frame decryption or authentication failed |
| `InsufficientSpace` | `0xAF` | Not enough buffer space for the requested operation |
| `NotFound` | `0xB0` | No matching entry in the binding table |

---

## ZCL Layer

### `ZclStatus`

**Crate:** `zigbee-zcl` · **Spec ref:** ZCL Rev 8, Table 2-12

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Operation completed successfully |
| `Failure` | `0x01` | Generic failure |
| `NotAuthorized` | `0x7E` | Sender is not authorized for this operation |
| `ReservedFieldNotZero` | `0x7F` | A reserved field in the frame was non-zero |
| `MalformedCommand` | `0x80` | The command frame is malformed |
| `UnsupClusterCommand` | `0x81` | Cluster-specific command is not supported |
| `UnsupGeneralCommand` | `0x82` | General ZCL command is not supported |
| `UnsupManufacturerClusterCommand` | `0x83` | Manufacturer-specific cluster command is not supported |
| `UnsupManufacturerGeneralCommand` | `0x84` | Manufacturer-specific general command is not supported |
| `InvalidField` | `0x85` | A field in the command contains an invalid value |
| `UnsupportedAttribute` | `0x86` | The specified attribute is not supported on this cluster |
| `InvalidValue` | `0x87` | The attribute value is out of range or otherwise invalid |
| `ReadOnly` | `0x88` | Attribute is read-only and cannot be written |
| `InsufficientSpace` | `0x89` | Not enough space to fulfil the request |
| `DuplicateExists` | `0x8A` | A duplicate entry already exists |
| `NotFound` | `0x8B` | The requested element was not found |
| `UnreportableAttribute` | `0x8C` | The attribute does not support reporting |
| `InvalidDataType` | `0x8D` | The data type does not match the attribute's type |
| `InvalidSelector` | `0x8E` | The selector (index) for a structured attribute is invalid |
| `WriteOnly` | `0x8F` | Attribute is write-only and cannot be read |
| `InconsistentStartupState` | `0x90` | Startup attribute set is inconsistent |
| `DefinedOutOfBand` | `0x91` | Value was already defined by an out-of-band mechanism |
| `Inconsistent` | `0x92` | Supplied values are inconsistent |
| `ActionDenied` | `0x93` | The requested action has been denied |
| `Timeout` | `0x94` | The operation timed out |
| `Abort` | `0x95` | Operation was aborted |
| `InvalidImage` | `0x96` | OTA image is invalid |
| `WaitForData` | `0x97` | Server is not ready — try again later |
| `NoImageAvailable` | `0x98` | No OTA image is available for this device |
| `RequireMoreImage` | `0x99` | More image data is required to continue |
| `NotificationPending` | `0x9A` | A notification is pending delivery |
| `HardwareFailure` | `0xC0` | Hardware failure on the device |
| `SoftwareFailure` | `0xC1` | Software failure on the device |
| `CalibrationError` | `0xC2` | Calibration error |
| `UnsupportedCluster` | `0xC3` | The cluster is not supported |

### `ZclFrameError`

**Crate:** `zigbee-zcl` · **File:** `frame.rs`

Errors during ZCL frame parsing.

| Variant | Meaning |
|---------|---------|
| `TooShort` | Buffer too short to contain a valid ZCL header |
| `PayloadTooLarge` | Payload exceeds maximum buffer size |
| `InvalidFrameType` | Frame type bits are invalid / reserved |

### `OtaImageError`

**Crate:** `zigbee-zcl` · **File:** `clusters/ota_image.rs`

Errors when parsing an OTA Upgrade image header.

| Variant | Meaning |
|---------|---------|
| `TooShort` | Data too short for the OTA header |
| `BadMagic` | Magic number does not match the OTA file identifier |
| `UnsupportedVersion` | Header version is not supported |
| `BadHeaderLength` | Header length field does not match actual data |
| `ImageTooLarge` | Image size exceeds available storage |

---

## ZDO Layer

### `ZdpStatus`

**Crate:** `zigbee-zdo` · **Spec ref:** Zigbee spec Table 2-96

Returned in every ZDP response frame. Also aliased as `ZdoStatus`.

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Request completed successfully |
| `InvRequestType` | `0x80` | The request type is invalid |
| `DeviceNotFound` | `0x81` | The addressed device could not be found |
| `InvalidEp` | `0x82` | The endpoint is invalid or not active |
| `NotActive` | `0x83` | The endpoint is not in the active state |
| `NotSupported` | `0x84` | The requested operation is not supported |
| `Timeout` | `0x85` | The operation timed out |
| `NoMatch` | `0x86` | No descriptor matched the request |
| `TableFull` | `0x87` | The internal table (binding, etc.) is full |
| `NoEntry` | `0x88` | No matching entry was found |
| `NoDescriptor` | `0x89` | The requested descriptor is not available |

### `ZdoError`

**Crate:** `zigbee-zdo` · **File:** `lib.rs`

Errors originating from ZDO internal processing.

| Variant | Meaning |
|---------|---------|
| `BufferTooSmall` | Serialisation buffer is too small for the frame |
| `InvalidLength` | Input data is shorter than the frame format requires |
| `InvalidData` | A parsed field contains a reserved or invalid value |
| `ApsError(ApsStatus)` | The underlying APS layer returned an error (wraps `ApsStatus`) |
| `TableFull` | An internal fixed-capacity table is full |

---

## BDB Layer

### `BdbStatus`

**Crate:** `zigbee-bdb` · **Spec ref:** BDB spec Table 4

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Commissioning completed successfully |
| `InProgress` | `0x01` | Commissioning is currently in progress |
| `NotOnNetwork` | `0x02` | Node is not on a network (required for this operation) |
| `NotPermitted` | `0x03` | Operation is not supported by this device type |
| `NoScanResponse` | `0x04` | No beacons received during network steering |
| `FormationFailure` | `0x05` | Network formation failed |
| `SteeringFailure` | `0x06` | Network steering failed after all retries |
| `NoIdentifyResponse` | `0x07` | No Identify Query response during Finding & Binding |
| `BindingTableFull` | `0x08` | Binding table full or cluster matching failed |
| `TouchlinkFailure` | `0x09` | Touchlink commissioning failed or is not supported |
| `TargetFailure` | `0x0A` | Target device is not in identifying mode |
| `Timeout` | `0x0B` | The operation timed out |

### `BdbCommissioningStatus`

**Crate:** `zigbee-bdb` · **File:** `attributes.rs` · **Spec ref:** BDB spec Table 4

Attribute value recording the result of the last commissioning attempt.
Similar to `BdbStatus` but used as a persistent attribute rather than a
one-shot return value.

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Last commissioning attempt succeeded (default) |
| `InProgress` | `0x01` | Commissioning is in progress |
| `NoNetwork` | `0x02` | Device is not on a network |
| `TlTargetFailure` | `0x03` | Touchlink target failure |
| `TlNotAddressAssignment` | `0x04` | Touchlink address assignment failure |
| `TlNoScanResponse` | `0x05` | Touchlink scan received no response |
| `NotPermitted` | `0x06` | Operation not permitted for this device type |
| `SteeringFormationFailure` | `0x07` | Network steering or formation failed |
| `NoIdentifyQueryResponse` | `0x08` | Finding & Binding received no Identify response |
| `BindingTableFull` | `0x09` | Binding table is full |
| `NoScanResponse` | `0x0A` | No scan response received |

---

## Runtime / Support

### `StartError`

**Crate:** `zigbee-runtime` · **File:** `event_loop.rs`

High-level errors from device start / join / leave operations.

| Variant | Meaning |
|---------|---------|
| `InitFailed` | BDB initialization failed |
| `CommissioningFailed(BdbStatus)` | BDB commissioning failed with a specific `BdbStatus` cause |

### `FirmwareError`

**Crate:** `zigbee-runtime` · **File:** `firmware_writer.rs`

Errors during OTA firmware write operations.

| Variant | Meaning |
|---------|---------|
| `EraseFailed` | Flash erase operation failed |
| `WriteFailed` | Flash write operation failed |
| `VerifyFailed` | Verification failed (hash or size mismatch) |
| `OutOfRange` | Offset is out of range for the firmware slot |
| `ImageTooLarge` | Firmware slot is not large enough for the image |
| `ActivateFailed` | Activation failed (e.g. boot flag not set) |
| `HardwareError` | Flash hardware error |

### `NvError`

**Crate:** `zigbee-runtime` · **File:** `nv_storage.rs`

Non-volatile storage errors.

| Variant | Meaning |
|---------|---------|
| `NotFound` | Requested item was not found |
| `Full` | Storage is full |
| `BufferTooSmall` | Item is too large for the provided buffer |
| `HardwareError` | Hardware error during read or write |
| `Corrupt` | Data corruption detected |
