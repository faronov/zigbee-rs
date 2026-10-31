# Error & Status Types

This chapter catalogues the public error and status enums in zigbee-rs, organised by
stack layer from bottom (MAC) to top (BDB / runtime). Use the hex codes to correlate
with Zigbee specification tables and on-the-wire values.

---

## MAC Layer

### `MacError`

**Crate:** `zigbee-mac` · **File:** `lib.rs` · **Spec ref:** IEEE 802.15.4 status codes

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

### `PhyError`

**Crate:** `zigbee-mac` · **File:** `phy.rs`

Error type of the `RadioPhy` trait — the hardware boundary under the shared
`SoftMacCore` (`soft_mac.rs`). The software MAC maps these to `MacError`
(e.g. a busy CCA drives CSMA-CA back-off).

| Variant | Meaning |
|---------|---------|
| `ChannelBusy` | A single clear-channel assessment found the channel busy |
| `InvalidChannel` | The requested channel is invalid or unsupported |
| `FrameTooLong` | The frame exceeds the IEEE 802.15.4 PSDU limit (125 bytes without FCS) |
| `CrcFailed` | The received frame failed its FCS check |
| `Hardware` | The radio peripheral, DMA engine, or calibration failed |
| `Unsupported` | The operation is not implemented by this PHY |

### `PibError`

**Crate:** `zigbee-mac` · **File:** `pib.rs`

| Variant | Meaning |
|---------|---------|
| `InvalidValue` | The value is out of range or has the wrong type for the attribute |
| `ReadOnly` | The attribute cannot be set |

### `FrameBuildError`

**Crate:** `zigbee-mac` · **File:** `frames.rs`

Returned by the beacon frame builder.

| Variant | Meaning |
|---------|---------|
| `FrameTooLong` | The frame would not fit the 125-byte MAC frame buffer |
| `InvalidParameter` | Invalid source address / broadcast PAN, or too many pending addresses |

### `RadioError` (platform-specific)

Some radio backends keep a driver-level `RadioError` enum below their MAC
adapter. The nRF52, BL702 and Telink `RadioPhy` implementations
(`nrf/radio_phy.rs`, `bl702/radio_phy.rs`, `telink/radio_phy.rs`) report
`PhyError` / `MacError` directly and have no public backend error enum.

#### CC2340 backend (`cc2340/driver.rs`)

| Variant | Meaning |
|---------|---------|
| `InvalidChannel` | Channel is outside IEEE 802.15.4 channels 11–26 |
| `InvalidFrame` | Frame is empty or exceeds the 125-byte FCS-free limit |
| `InvalidTxPower` | Requested TX power is outside the TI TX power table range |
| `FirmwareUnavailable` | `CC2340_SDK_DIR` was not available at build time |
| `ClockTimeout` | Radio clocks or the HFXT oscillator did not become ready |
| `FirmwareImageTooLarge` | A radio firmware image does not fit the TOPsm RAM |
| `RadioConfigUnavailable` | TI IEEE PHY configuration was not imported |
| `FactoryTrimUnavailable` | Required per-die FCFG trim data is unavailable |
| `SynthConfigInvalid` | Synthesizer configuration is invalid (e.g. zero frequency) |
| `SynthTimeout` | Synthesizer divider did not complete |
| `RadioReadyTimeout` | PBE/MCE/RFE TOPsm startup did not complete |
| `ChannelBusy` | CCA indicated a busy channel |
| `NoAck` | No acknowledgement from the receiver |
| `RxTimeout` | Receive operation ended without a frame |
| `CrcError` | Received frame failed CRC check |
| `FifoOverflow` | PBE reported an RX/TX FIFO error or insufficient space |
| `MalformedFrame` | RX FIFO entry or metadata layout was invalid |
| `LinkQualityUnavailable` | RX entry carried no usable RSSI, so no LQI can be derived |
| `OperationTimeout` | A PBE operation or hard stop did not complete in time |
| `PbeError(code)` | PBE returned an unclassified `ENDCAUSE` value |
| `HardwareError` | Radio hardware fault |

#### EFR32MG1 / EFR32MG21 / PHY6222 backends

`efr32/driver.rs`, `efr32s2/driver.rs` and `phy6222/driver.rs` share the same
variant set:

| Variant | Meaning |
|---------|---------|
| `CcaFailure` | Clear Channel Assessment failure — channel is busy |
| `HardwareError` | TX/RX hardware error or inconsistent engine state |
| `InvalidFrame` | Frame too long or too short |
| `CrcError` | Received frame failed CRC check |
| `NotInitialized` | Radio driver has not been initialized |
| `RxTimeout` | RX timeout — no frame received |

#### ESP32 backend (`esp/driver.rs`)

The ESP backend's driver error is named `Error`:

| Variant | Meaning |
|---------|---------|
| `Radio(esp_radio::ieee802154::Error)` | Error from the `esp-radio` 802.15.4 driver |
| `InvalidLength` | Invalid IEEE 802.15.4 frame length |
| `TransmitTimeout` | Transmission completion timed out |

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

### `WriteAttributesParseError`

**Crate:** `zigbee-zcl` · **File:** `foundation/write_attributes.rs`

Returned by `WriteAttributesRequest::parse_checked()`.

| Variant | Meaning |
|---------|---------|
| `Malformed` | Payload is empty, a record is truncated, has an unknown data type, or exceeds the record limit |

---

## ZDO Layer

### `ZdpStatus`

**Crate:** `zigbee-zdo` · **File:** `lib.rs` · **Spec ref:** Zigbee R22 Table 2-138

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
| `NoEntry` | `0x88` | No matching entry was found |
| `NoDescriptor` | `0x89` | The requested descriptor is not available |
| `TableFull` | `0x8C` | The internal table (binding, etc.) is full (`0x87` is reserved) |
| `IllegalRequest` | `0xA3` | Remote Trust Center policy changes are disabled |
| `TrustCenterPolicyNotSupported` | `0xAA` | TC-significant request while whitelist mode is active (R22 §4.7.3.2) |

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
| `BindingTableFull` | `0x08` | Binding table full during Finding & Binding |
| `TouchlinkFailure` | `0x09` | Touchlink commissioning failed or is not supported |
| `TargetFailure` | `0x0A` | Target device is not in identifying mode |
| `Timeout` | `0x0B` | The operation timed out |
| `TrustCenterLinkKeyExchangeFailure` | `0x0C` | Trust Center link-key exchange failed |
| `PersistenceFailure` | `0x0D` | Security state could not be durably persisted |

### `BdbCommissioningStatus`

**Crate:** `zigbee-bdb` · **File:** `attributes.rs` · **Spec ref:** BDB spec Table 4

Attribute value recording the result of the last commissioning attempt.
Similar to `BdbStatus` but used as a persistent attribute rather than a
one-shot return value.

| Variant | Code | Meaning |
|---------|------|---------|
| `Success` | `0x00` | Last commissioning attempt succeeded (default) |
| `InProgress` | `0x01` | Commissioning is in progress |
| `NotAaCapable` | `0x02` | Device is not address-assignment capable |
| `NoNetwork` | `0x03` | No suitable network was found |
| `TargetFailure` | `0x04` | Touchlink target procedure failed |
| `FormationFailure` | `0x05` | Network formation failed |
| `NoIdentifyQueryResponse` | `0x06` | Finding & Binding received no Identify response |
| `BindingTableFull` | `0x07` | Binding table is full |
| `NoScanResponse` | `0x08` | No scan response received |
| `NotPermitted` | `0x09` | Operation is not permitted |
| `TcLinkKeyExchangeFailure` | `0x0A` | Trust Center link-key exchange failed |
| `NotOnANetwork` | `0x0B` | Procedure required the node to be on a network |
| `OnANetwork` | `0x0C` | Procedure required the node to be off-network |

### `SecurityPersistenceError`

**Crate:** `zigbee-bdb` · **File:** `security_persistence.rs`

Persistence failure reported synchronously to commissioning (surfaces as
`BdbStatus::PersistenceFailure`).

| Variant | Meaning |
|---------|---------|
| `Storage` | The backing store failed |
| `CounterExhausted` | An outgoing frame counter cannot be advanced further |
| `InvalidState` | The security state to persist is inconsistent |

### `TrustCenterError`

**Crate:** `zigbee-bdb` · **File:** `trust_center.rs`

| Variant | Meaning |
|---------|---------|
| `InvalidAddress` | The device IEEE address is invalid |
| `InvalidInstallCode(InstallCodeError)` | The install code failed validation |
| `DeviceTableFull` | The Trust Center device table is full |
| `DeviceNotFound` | No entry for the device |
| `RequestNotPermitted` | The request is not allowed by the current TC policy |

---

## Crypto

### `AesMmoError` / `InstallCodeError`

**Crate:** `zigbee-crypto` · **File:** `lib.rs`

| Enum | Variant | Meaning |
|------|---------|---------|
| `AesMmoError` | `InputTooLong` | Input length does not fit the 16-bit bit-length field |
| `AesMmoError` | `CipherFailure` | The selected AES backend failed |
| `InstallCodeError` | `InvalidLength` | Code is not 6, 8, 12 or 16 bytes plus a 2-byte CRC |
| `InstallCodeError` | `InvalidCrc` | The appended CRC-16/X-25 does not match |
| `InstallCodeError` | `CipherFailure` | AES failed while computing the AES-MMO hash |

---

## Runtime / Support

### `StartError`

**Crate:** `zigbee-runtime` · **File:** `event_loop.rs`

High-level errors from device start / join / leave operations.

| Variant | Meaning |
|---------|---------|
| `InitFailed` | BDB initialization failed |
| `CommissioningFailed(BdbStatus)` | BDB commissioning failed with a specific `BdbStatus` cause |
| `PersistenceFailed(SecurityStoreError)` | Durable security-state storage failed |

### `SendError`

**Crate:** `zigbee-runtime` · **File:** `event_loop.rs`

Errors while sending application ZCL traffic.

| Variant | Meaning |
|---------|---------|
| `NotJoined` | The device has not joined a network yet |
| `Serialization` | The ZCL frame could not be serialized |
| `PayloadTooLong` | The payload exceeded the fixed ZCL frame capacity |
| `Aps(ApsStatus)` | APS rejected or failed the data request |

### `BuildError`

**Crate:** `zigbee-runtime` · **File:** `builder.rs`

Returned by the `DeviceBuilder::try_build*` methods (the plain `build*`
methods panic instead).

| Variant | Meaning |
|---------|---------|
| `RoleRejectsDeviceType { role, device_type }` | The build role does not accept the configured `DeviceType` |

### `NodeError` / `ProfileError`

**Crate:** `zigbee-runtime` · **Files:** `node.rs`, `profile/mod.rs`

`NodeError` is returned by `ZigbeeNode` operations such as `process_incoming`; `ProfileError` comes from application profiles.

| Enum | Variant | Meaning |
|------|---------|---------|
| `NodeError` | `Profile(ProfileError)` | The application profile was rejected |
| `NodeError` | `Persistence(SecurityStoreError)` | Durable security-state storage failed |
| `ProfileError` | `TooManyClusters` | The profile's clusters do not fit the `ClusterRef` list |
| `ProfileError` | `EndpointMismatch { profile_endpoint, component_endpoint }` | A wrapped component (e.g. `WithOta`) uses a different endpoint than the profile |
| `ProfileError` | `Reporting(ZclStatus)` | Installing a default reporting configuration failed |

### `SecurityStoreError`

**Crate:** `zigbee-runtime` · **File:** `security_store.rs`

| Variant | Meaning |
|---------|---------|
| `NotFound` | No persisted security state |
| `Corrupt` | Persisted record failed validation |
| `Full` | Storage is full |
| `Hardware` | Flash/NV hardware error |
| `CounterExhausted` | A persisted frame counter cannot be advanced further |
| `GenerationExhausted` | The record generation counter is exhausted |

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
