# MAC Layer & Backends

The MAC (Medium Access Control) layer is the boundary between the
platform-independent Zigbee stack and the hardware-specific 802.15.4 radio.
In zigbee-rs, this boundary is defined by a single trait: **`MacDriver`**.

```text
┌──────────────────────────────────────────┐
│  Zigbee Stack (NWK / APS / ZCL / BDB)    │  platform-independent
└───────────────┬──────────────────────────┘
                │ MacDriver trait
┌───────────────┴──────────────────────────┐
│  MAC backends: esp / nrf / bl702 / …      │  platform-specific
└──────────────────────────────────────────┘
```

Each hardware platform implements `MacDriver` once (~500 lines of
platform-specific code).  The entire upper stack is built against this trait and
never touches hardware directly.

## The `MacDriver` Trait

`MacDriver` is an async trait covering the minimal complete set of MLME/MCPS
primitives needed for Zigbee PRO R22 operation.  All methods are async to
support interrupt-driven radios with Embassy or other async executors.

```rust,ignore
pub trait MacDriver {
    // Scanning
    async fn mlme_scan(&mut self, req: MlmeScanRequest)
        -> Result<MlmeScanConfirm, MacError>;

    // Association
    async fn mlme_associate(&mut self, req: MlmeAssociateRequest)
        -> Result<MlmeAssociateConfirm, MacError>;
    async fn mlme_associate_response(&mut self, rsp: MlmeAssociateResponse)
        -> Result<(), MacError>;
    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest)
        -> Result<(), MacError>;

    // Management
    async fn mlme_reset(&mut self, set_default_pib: bool)
        -> Result<(), MacError>;
    async fn mlme_start(&mut self, req: MlmeStartRequest)
        -> Result<(), MacError>;

    // PIB access
    async fn mlme_get(&self, attr: PibAttribute)
        -> Result<PibValue, MacError>;
    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue)
        -> Result<(), MacError>;

    // Polling (sleepy devices)
    async fn mlme_poll(&mut self)
        -> Result<Option<MacFrame>, MacError>;

    // Data service
    async fn mcps_data(&mut self, req: McpsDataRequest<'_>)
        -> Result<McpsDataConfirm, MacError>;
    async fn mcps_data_indication(&mut self)
        -> Result<McpsDataIndication, MacError>;

    // Capability query
    fn capabilities(&self) -> MacCapabilities;
}
```

### Method Reference

#### Scanning

| Method | Description |
|--------|-------------|
| `mlme_scan(req)` | Perform an ED, Active, Passive, or Orphan scan.  Scans channels in `req.channel_mask` for `req.scan_duration`.  Returns discovered PAN descriptors or energy measurements. |

#### Association

| Method | Description |
|--------|-------------|
| `mlme_associate(req)` | Request association with a coordinator on `req.channel`.  Returns the assigned short address on success. |
| `mlme_associate_response(rsp)` | Coordinator/Router only.  Respond to an association request with an assigned address or denial. |
| `mlme_disassociate(req)` | Send a disassociation notification to leave the PAN. |

#### Management

| Method | Description |
|--------|-------------|
| `mlme_reset(set_default_pib)` | Reset the MAC to its default state.  If `true`, all PIB attributes are reset to defaults. |
| `mlme_start(req)` | Start a PAN (coordinator) or begin beacon transmission (router).  End devices do not use this. |

#### PIB Access

| Method | Description |
|--------|-------------|
| `mlme_get(attr)` | Read a MAC PIB attribute (e.g., short address, PAN ID, channel). |
| `mlme_set(attr, value)` | Write a MAC PIB attribute. |

#### Polling

| Method | Description |
|--------|-------------|
| `mlme_poll()` | Send a MAC Data Request to the coordinator and return any pending indirect frame.  Used by Sleepy End Devices to retrieve queued data. |

#### Data Service

| Method | Description |
|--------|-------------|
| `mcps_data(req)` | Transmit a MAC frame to `req.dst_address` with the specified options (ACK, security, indirect). |
| `mcps_data_indication()` | Block until an incoming MAC frame is received from the radio.  The caller filters by frame type and addressing. |

#### Capability Query

| Method | Description |
|--------|-------------|
| `capabilities()` | Returns a `MacCapabilities` struct describing what this backend supports. |

### `MacCapabilities`

```rust,ignore
pub struct MacCapabilities {
    /// Can act as PAN coordinator
    pub coordinator: bool,
    /// Can act as router (relay frames)
    pub router: bool,
    /// Supports MAC-level hardware encryption
    pub hardware_security: bool,
    /// Maximum frame payload size (typically 102 bytes)
    pub max_payload: u16,
    /// Minimum supported TX power
    pub tx_power_min: TxPower,
    /// Maximum supported TX power
    pub tx_power_max: TxPower,
}
```

## Available Backends

zigbee-rs ships with MAC backends for a wide range of 802.15.4 radios.  Each
backend is behind a Cargo feature flag:

| Backend | Feature Flag | Chip(s) | Notes |
|---------|-------------|---------|-------|
| **ESP32** | `esp32c6`, `esp32h2` | ESP32-C6, ESP32-H2 | Espressif IEEE 802.15.4 radio, `esp-ieee802154` HAL |
| **nRF** | `nrf52840`, `nrf52833` | nRF52840, nRF52833 | Nordic 802.15.4 radio, `embassy-nrf` peripherals |
| **BL702** | `bl702` | BL702, BL706 | Bouffalo Lab 802.15.4 radio |
| **CC2340** | `cc2340` | CC2340R5 | TI SimpleLink, Cortex-M0+ with 802.15.4 |
| **Telink** | `telink` | B91 (TLSR9518), TLSR8258 | Telink 802.15.4 radios — TLSR8258 is pure Rust (direct register access), B91 uses FFI |
| **PHY6222** | `phy6222` | PHY6222 | Phyplus BLE+802.15.4 combo SoC |
| **EFR32MG1** | `efr32` | EFR32MG1P | Silicon Labs Series 1, Cortex-M4F — pure Rust (direct register access) |
| **EFR32MG21** | `efr32s2` | EFR32MG21 | Silicon Labs Series 2, Cortex-M33 — pure Rust (direct register access) |
| **Mock** | `mock` | — | In-memory mock for unit tests and CI |

### Choosing a Backend

Enable exactly one backend in your `Cargo.toml`:

```toml
[dependencies]
zigbee-mac = { path = "../zigbee-mac", features = ["esp32c6"] }
```

The rest of the stack is completely platform-independent — changing backends is
a one-line Cargo feature change plus updating the MAC initialization code.

**Decision guide:**

- **ESP32-C6 / ESP32-H2** — Best for Wi-Fi+Zigbee combo (C6) or pure Zigbee
  (H2).  Great ESP-IDF ecosystem and tooling.
- **nRF52840** — Best for ultra-low-power battery sensors.  Excellent BLE+Zigbee
  combo.  Mature Embassy async support.
- **BL702** — Low cost, good for high-volume products.
- **CC2340R5** — TI ecosystem, good for industrial applications.
- **Telink B91 / TLSR8258** — Very low cost, widely used in commercial Zigbee
  products (IKEA TRÅDFRI, etc.). TLSR8258 has a pure-Rust radio driver (no
  vendor SDK needed); B91 requires Telink SDK.
- **PHY6222** — Budget BLE+802.15.4 combo, pure-Rust radio driver, suitable for simple sensors.
- **EFR32MG1** — Silicon Labs Series 1 (Cortex-M4F), used in IKEA TRÅDFRI modules.
  Pure-Rust radio driver — no GSDK/RAIL required. Great for repurposing existing hardware.
- **EFR32MG21** — Silicon Labs Series 2 (Cortex-M33), used in Sonoff ZBDongle-E.
  Pure-Rust radio driver with separate `efr32s2` module (different register map from Series 1).
- **Mock** — Use for testing your application logic without hardware.

## The Mock Backend

The `mock` feature provides `MockMac` — a fully functional in-memory MAC
implementation for testing:

```rust,ignore
use zigbee_mac::mock::MockMac;

// Create with a specific IEEE address
let mac = MockMac::new([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]);

let device = ZigbeeDevice::builder(mac)
    .device_type(DeviceType::EndDevice)
    .build();
```

`MockMac` simulates scan responses, association, and data transfer — useful for
integration tests and CI pipelines where no radio hardware is available.

## Writing Your Own `MacDriver`

To port zigbee-rs to a new 802.15.4 radio, implement the `MacDriver` trait.
Here's the skeleton:

```rust,ignore
use zigbee_mac::*;

pub struct MyRadioMac {
    // Your radio peripheral handle, state, buffers, etc.
}

impl MacDriver for MyRadioMac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        // 1. Configure radio for the requested scan type
        // 2. For each channel in req.channel_mask:
        //    - Set radio to channel
        //    - Send beacon request (active scan) or listen (passive/ED)
        //    - Collect responses for scan_duration
        // 3. Return collected PAN descriptors or ED values
        todo!()
    }

    async fn mlme_associate(&mut self, req: MlmeAssociateRequest) -> Result<MlmeAssociateConfirm, MacError> {
        // 1. Set radio to req.channel
        // 2. Send Association Request command frame to req.coord_address
        // 3. Wait for Association Response (with timeout)
        // 4. Return assigned short address
        todo!()
    }

    async fn mlme_associate_response(&mut self, rsp: MlmeAssociateResponse) -> Result<(), MacError> {
        // Coordinator/Router: send Association Response frame
        todo!()
    }

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        // Send Disassociation Notification frame
        todo!()
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        // Reset radio hardware, optionally reset PIB to defaults
        todo!()
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        // Configure radio as PAN coordinator/router on the given channel
        todo!()
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        // Read PIB attribute from your stored state or hardware registers
        todo!()
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        // Write PIB attribute to your stored state and configure hardware
        todo!()
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Send Data Request to coordinator, wait for response
        todo!()
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        // Build 802.15.4 frame from req, transmit via radio with CSMA-CA
        // If req.tx_options.ack_tx, wait for ACK
        todo!()
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        // Wait for incoming frame from radio (interrupt-driven)
        // Parse 802.15.4 header, return payload + addressing
        todo!()
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(8),
        }
    }
}
```

> **Tip:** Study the existing `esp` or `nrf` backends for reference — they
> handle all the edge cases (scan timing, CSMA-CA retry, ACK waiting, indirect
> TX for sleepy devices).

## MAC Primitives Reference

The MAC layer uses structured request/confirm/indication types that map to IEEE
802.15.4 service primitives.

### Scan Primitives

```rust,ignore
pub struct MlmeScanRequest {
    pub scan_type: ScanType,        // Ed, Active, Passive, Orphan
    pub channel_mask: ChannelMask,  // Which channels to scan
    pub scan_duration: u8,          // Duration exponent (0-14)
}

pub struct MlmeScanConfirm {
    pub scan_type: ScanType,
    pub pan_descriptors: PanDescriptorList,   // Up to 27 discovered PANs
    pub energy_list: EdList,                  // ED scan results
}
```

**Scan types:**

| Type | Description |
|------|-------------|
| `ScanType::Ed` | Measure noise energy on each channel |
| `ScanType::Active` | Send beacon requests, collect responses |
| `ScanType::Passive` | Listen for beacons without transmitting |
| `ScanType::Orphan` | Search for our coordinator after losing sync |

**Scan duration:** The time spent on each channel is
`aBaseSuperframeDuration × (2^n + 1)` symbols.  Typical values:

| Exponent | Time per channel | Use case |
|----------|-----------------|----------|
| 3 | ~138 ms | Fast scan |
| 5 | ~530 ms | Normal scan |
| 7 | ~2.1 s | Thorough scan |

### Association Primitives

```rust,ignore
pub struct MlmeAssociateRequest {
    pub channel: u8,                    // Channel to associate on
    pub coord_address: MacAddress,      // Coordinator address
    pub capability_info: CapabilityInfo, // Our capabilities
}

pub struct CapabilityInfo {
    pub device_type_ffd: bool,     // Full Function Device
    pub mains_powered: bool,
    pub rx_on_when_idle: bool,     // false = sleepy
    pub security_capable: bool,
    pub allocate_address: bool,    // Request short address
}

pub struct MlmeAssociateConfirm {
    pub short_address: ShortAddress,   // Assigned address (or 0xFFFF on failure)
    pub status: AssociationStatus,     // Success, PanAtCapacity, PanAccessDenied
}
```

### Data Primitives

```rust,ignore
pub struct McpsDataRequest<'a> {
    pub src_addr_mode: AddressMode,    // None, Short, Extended
    pub dst_address: MacAddress,
    pub payload: &'a [u8],            // Up to MAX_MAC_PAYLOAD (127) bytes
    pub msdu_handle: u8,               // Handle for TX confirmation
    pub tx_options: TxOptions,
}

pub struct TxOptions {
    pub ack_tx: bool,          // Request ACK from receiver
    pub indirect: bool,        // Indirect TX (for sleepy children)
    pub security_enabled: bool, // MAC-level security
}

pub struct McpsDataIndication {
    pub src_address: MacAddress,
    pub dst_address: MacAddress,
    pub lqi: u8,                // Link Quality (0-255)
    pub payload: MacFrame,      // Received frame data
    pub security_use: bool,
}
```

### `MacFrame` — Zero-Allocation Frame Buffer

`MacFrame` is a fixed-size buffer (127 bytes) that holds received frame data
without heap allocation:

```rust,ignore
pub struct MacFrame {
    buf: [u8; MAX_MAC_PAYLOAD],  // 127 bytes
    len: usize,
}

impl MacFrame {
    pub fn from_slice(data: &[u8]) -> Option<Self>;
    pub fn as_slice(&self) -> &[u8];
    pub fn len(&self) -> usize;
    pub fn is_empty(&self) -> bool;
}
```

## `MacError` — Error Types

All `MacDriver` methods return `Result<_, MacError>`:

```rust,ignore
pub enum MacError {
    NoBeacon,             // No beacon received during scan
    InvalidParameter,     // Invalid parameters supplied
    RadioError,           // Hardware radio failure
    ChannelAccessFailure, // CSMA-CA failed (channel busy)
    NoAck,                // No acknowledgement received
    FrameTooLong,         // Frame exceeds PHY maximum
    Unsupported,          // Operation not supported by backend
    SecurityError,        // MAC security processing failed
    TransactionOverflow,  // Indirect queue full
    TransactionExpired,   // Indirect frame expired before delivery
    ScanInProgress,       // Another scan is already running
    TrackingOff,          // Lost superframe tracking
    AssociationDenied,    // Coordinator denied association
    PanAtCapacity,        // PAN has no room for more devices
    NoData,               // No data frame within timeout (poll)
    Other,                // Unclassified error
}
```

## PibAttribute & PibValue — The MAC Configuration Interface

The NWK layer configures the MAC through PIB (PAN Information Base) get/set
operations.  This is the standard IEEE 802.15.4 configuration mechanism.

### PibAttribute Reference

#### Addressing (set during join)

| Attribute | ID | Description | Default |
|-----------|-----|-------------|---------|
| `MacShortAddress` | 0x53 | Own 16-bit NWK address | 0xFFFF (unassigned) |
| `MacPanId` | 0x50 | PAN ID of our network | 0xFFFF (not associated) |
| `MacExtendedAddress` | 0x6F | Own 64-bit IEEE address | From hardware |
| `MacCoordShortAddress` | 0x4B | Parent's short address | — |
| `MacCoordExtendedAddress` | 0x4A | Parent's extended address | — |

#### Network Configuration

| Attribute | ID | Description |
|-----------|-----|-------------|
| `MacAssociatedPanCoord` | 0x56 | Is this the PAN coordinator? |
| `MacRxOnWhenIdle` | 0x52 | Receive during idle (false = sleepy) |
| `MacAssociationPermit` | 0x41 | Accepting join requests? |

#### Beacon (always 15/15 for Zigbee PRO non-beacon mode)

| Attribute | ID | Description |
|-----------|-----|-------------|
| `MacBeaconOrder` | 0x47 | Beacon interval (always 15) |
| `MacSuperframeOrder` | 0x54 | Superframe duration (always 15) |
| `MacBeaconPayload` | 0x45 | Beacon payload bytes |
| `MacBeaconPayloadLength` | 0x46 | Length of beacon payload |

#### TX/RX Tuning

| Attribute | ID | Description | Default |
|-----------|-----|-------------|---------|
| `MacMaxCsmaBackoffs` | 0x4E | Max CSMA-CA retries | 4 |
| `MacMinBe` | 0x4F | Min backoff exponent | 3 |
| `MacMaxBe` | 0x57 | Max backoff exponent | 5 |
| `MacMaxFrameRetries` | 0x59 | Max ACK retries | 3 |

#### PHY Attributes (accessed via MAC GET/SET)

| Attribute | ID | Description |
|-----------|-----|-------------|
| `PhyCurrentChannel` | 0x00 | Operating channel (11-26) |
| `PhyChannelsSupported` | 0x01 | Supported channels bitmask |
| `PhyTransmitPower` | 0x02 | TX power in dBm |
| `PhyCcaMode` | 0x03 | Clear Channel Assessment mode |
| `PhyCurrentPage` | 0x04 | Channel page (always 0 for 2.4 GHz) |

### PibValue

`PibValue` is a tagged union for PIB get/set operations:

```rust,ignore
pub enum PibValue {
    Bool(bool),
    U8(u8),
    U16(u16),
    U32(u32),
    I8(i8),
    ShortAddress(ShortAddress),
    PanId(PanId),
    ExtendedAddress(IeeeAddress),
    Payload(PibPayload),           // Variable-length beacon payload (max 52 bytes)
}
```

Convenience accessors (`as_bool()`, `as_u8()`, `as_short_address()`, etc.) are
provided for safe downcasting.
