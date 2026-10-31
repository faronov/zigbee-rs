# PIB Attributes Reference

The **PAN Information Base (PIB)** is the MAC-layer configuration interface defined by IEEE 802.15.4. The `zigbee-mac` crate exposes PIB attributes through the `MacDriver` trait's `mlme_get()` and `mlme_set()` methods (both return `Result<_, MacError>`). Types live in `zigbee-mac/src/pib.rs`.

Backends built on the shared `SoftMacCore` (nRF52, BL702, Telink sensors and router, PHY6222, CC2340, ESP32-C6/H2 — over the `RadioPhy` trait) store PIB state in the portable `MacPib` struct; the defaults and validation rules below are those of `MacPib`. Backends with their own `MacDriver` implementation (EFR32 and the mock MAC) keep their own PIB state and may differ in defaults.

```rust
use zigbee_mac::{PibAttribute, PibValue, MacDriver};

// Read current channel
let channel = mac.mlme_get(PibAttribute::PhyCurrentChannel).await?;

// Set short address after joining
mac.mlme_set(
    PibAttribute::MacShortAddress,
    PibValue::ShortAddress(ShortAddress(0x1234)),
).await?;
```

---

## PibAttribute Variants

### Addressing (set during join)

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacShortAddress` | `0x53` | `ShortAddress` | `0xFFFF` (unassigned) | Own 16-bit network short address. Set by the coordinator during association. |
| `MacPanId` | `0x50` | `PanId` | `0xFFFF` (not associated) | PAN identifier of the network. Set during join or network formation. |
| `MacExtendedAddress` | `0x6F` | `ExtendedAddress` | Passed to `MacPib::new` (backend reads it from hardware) | Own 64-bit IEEE address. Writable in `MacPib` (updates the RX address filter). |
| `MacCoordShortAddress` | `0x4B` | `ShortAddress` | `0xFFFF` | Short address of the parent coordinator or router. Set during association. |
| `MacCoordExtendedAddress` | `0x4A` | `ExtendedAddress` | `[0; 8]` | Extended address of the parent coordinator or router. |

### Network Configuration

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacAssociatedPanCoord` | `0x56` | `Bool` | `false` | `true` if this device is the PAN coordinator. Set during network formation. |
| `MacRxOnWhenIdle` | `0x52` | `Bool` | `false` | Whether the radio receiver is on during idle. Set `true` for coordinators and routers, `false` for sleepy end devices. Controls how the device is addressed in network discovery. |
| `MacAssociationPermit` | `0x41` | `Bool` | `false` | Whether the device is accepting association requests (join permit open). Set by `permit_joining` commands. |

### Beacon (Non-Beacon Mode)

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacBeaconOrder` | `0x47` | `U8` | `15` | Beacon order. **Always 15** for Zigbee PRO (non-beacon mode). Values > 15 are rejected. |
| `MacSuperframeOrder` | `0x54` | `U8` | `15` | Superframe order. **Always 15** for Zigbee PRO. Values > 15 are rejected. |
| `MacBeaconPayload` | `0x45` | `Payload` | Empty | Beacon payload bytes. Contains NWK beacon content for coordinators and routers. Max 52 bytes. |
| `MacBeaconPayloadLength` | `0x46` | `U8` | `0` | Length of the beacon payload in bytes. Derived from `MacBeaconPayload`; a SET only succeeds if it equals the current length. |

### TX/RX Tuning

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacAutoRequest` | `0x42` | `Bool` | `true` | Automatically send data request after receiving a beacon with the pending bit set. Used by end devices to retrieve buffered data from their parent. |
| `MacMaxCsmaBackoffs` | `0x4E` | `U8` | `4` | Maximum number of CSMA-CA backoff attempts before declaring channel access failure. Range: 0–5. |
| `MacMinBe` | `0x4F` | `U8` | `3` | Minimum backoff exponent for CSMA-CA (2.4 GHz default: 3). Must be ≤ `MacMaxBe` and ≤ 8. Lower values mean more aggressive channel access. |
| `MacMaxBe` | `0x57` | `U8` | `5` | Maximum backoff exponent for CSMA-CA. Range: `MacMinBe`–8. |
| `MacMaxFrameRetries` | `0x59` | `U8` | `3` | Number of retransmission attempts after an ACK failure. Range: 0–7. |
| `MacMaxFrameTotalWaitTime` | `0x58` | `U32` | `0` | Maximum time (in symbols) to wait for an indirect transmission frame. Used by end devices polling their parent. |
| `MacResponseWaitTime` | `0x5A` | `U8` | `32` | Maximum time to wait for a response frame (in units of `aBaseSuperframeDuration`). Used during association. |

### Sequence Numbers

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacDsn` | `0x4C` | `U8` | Backend-supplied seed | Data/command frame sequence number. Incremented automatically per transmission. |
| `MacBsn` | `0x49` | `U8` | Backend-supplied seed | Beacon sequence number. Incremented per beacon transmission. |

### Indirect TX (Coordinator/Router)

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacTransactionPersistenceTime` | `0x55` | `U16` | `0x01F4` | How long (in unit periods) a coordinator stores indirect frames for sleepy children before discarding. |

### Debug / Special

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `MacPromiscuousMode` | `0x51` | `Bool` | `false` | When `true`, the radio receives all frames regardless of addressing. Used for sniffing/debugging. |

### PHY Attributes (via MAC GET/SET)

| Attribute | ID | PibValue Type | Default | Description |
|-----------|----|---------------|---------|-------------|
| `PhyCurrentChannel` | `0x00` | `U8` | `11` | Current 2.4 GHz channel (11–26; other values are rejected). Set during network formation or join. |
| `PhyChannelsSupported` | `0x01` | `U32` | `0x07FFF800` | Bitmask of supported channels (`ChannelMask::ALL_2_4GHZ`, bits 11–26). Read-only: SET returns `PibError::ReadOnly`. |
| `PhyTransmitPower` | `0x02` | `I8` | `0` | TX power in dBm, applied via `RadioPhy::set_tx_power`. Range depends on radio hardware. |
| `PhyCcaMode` | `0x03` | `U8` | `1` | Clear Channel Assessment mode. Mode 1 = energy above threshold. Fixed: only `1` is accepted. |
| `PhyCurrentPage` | `0x04` | `U8` | `0` | Channel page. **Always 0** for 2.4 GHz Zigbee; only `0` is accepted. |

---

## PibValue Variants

The `PibValue` enum is the value container for all PIB GET/SET operations:

| Variant | Contained Type | Used By |
|---------|---------------|---------|
| `Bool(bool)` | `bool` | `MacAssociatedPanCoord`, `MacRxOnWhenIdle`, `MacAssociationPermit`, `MacAutoRequest`, `MacPromiscuousMode` |
| `U8(u8)` | `u8` | `MacBeaconOrder`, `MacSuperframeOrder`, `MacBeaconPayloadLength`, `MacMaxCsmaBackoffs`, `MacMinBe`, `MacMaxBe`, `MacMaxFrameRetries`, `MacResponseWaitTime`, `MacDsn`, `MacBsn`, `PhyCurrentChannel`, `PhyCcaMode`, `PhyCurrentPage` |
| `U16(u16)` | `u16` | `MacTransactionPersistenceTime` |
| `U32(u32)` | `u32` | `MacMaxFrameTotalWaitTime`, `PhyChannelsSupported` |
| `I8(i8)` | `i8` | `PhyTransmitPower` |
| `ShortAddress(ShortAddress)` | `ShortAddress` (newtype over `u16`) | `MacShortAddress`, `MacCoordShortAddress` |
| `PanId(PanId)` | `PanId` (newtype over `u16`) | `MacPanId` |
| `ExtendedAddress(IeeeAddress)` | `[u8; 8]` | `MacExtendedAddress`, `MacCoordExtendedAddress` |
| `Payload(PibPayload)` | `PibPayload` (max 52 bytes) | `MacBeaconPayload` |

### PibPayload

Fixed-capacity beacon payload buffer:

```rust
pub struct PibPayload {
    buf: [u8; 52],
    len: usize,
}

impl PibPayload {
    pub fn new() -> Self;                       // Empty payload
    pub fn from_slice(data: &[u8]) -> Option<Self>;  // None if > 52 bytes
    pub fn as_slice(&self) -> &[u8];            // Current content
}
```

### Convenience Accessors on PibValue

Each variant has a corresponding accessor that returns `Option`:

| Method | Returns |
|--------|---------|
| `as_bool()` | `Option<bool>` |
| `as_u8()` | `Option<u8>` |
| `as_u16()` | `Option<u16>` |
| `as_u32()` | `Option<u32>` |
| `as_i8()` | `Option<i8>` |
| `as_short_address()` | `Option<ShortAddress>` |
| `as_pan_id()` | `Option<PanId>` |
| `as_extended_address()` | `Option<IeeeAddress>` |

---

## MacPib and Validation

```rust
impl MacPib {
    pub fn new(extended_address: IeeeAddress, dsn: u8, bsn: u8) -> Self;
    pub fn reset(&mut self, dsn: u8, bsn: u8);  // defaults, keeps the IEEE address
    pub fn get(&self, attr: PibAttribute) -> PibValue;
    pub fn set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), PibError>;
    pub fn next_dsn(&mut self) -> u8;            // returns current, then wraps +1
    pub fn next_bsn(&mut self) -> u8;
}
```

`set()` returns `PibError::ReadOnly` for `PhyChannelsSupported` and
`PibError::InvalidValue` for a mismatched `PibValue` variant or an
out-of-range value. `SoftMacCore::set_pib()` validates on a copy, applies
hardware side effects through `RadioPhy` (channel, TX power, address filter,
RX-on-when-idle), and only then commits; both `PibError` variants surface as
`MacError::InvalidParameter`.

---

## PHY Constants & Helpers

```rust
/// Base superframe duration in symbols (960)
pub const A_BASE_SUPERFRAME_DURATION: u32 = 960;

/// Symbol rate at 2.4 GHz in symbols/second
pub const SYMBOL_RATE_2_4GHZ: u32 = 62_500;

/// Calculate scan duration per channel in symbols
pub fn scan_duration_symbols(exponent: u8) -> u32;

/// Calculate scan duration per channel in microseconds
pub fn scan_duration_us(exponent: u8) -> u64;
```

| Scan Duration (exponent) | Symbols | Milliseconds | Typical Use |
|--------------------------|---------|--------------|-------------|
| 0 | 1,920 | 30.7 | Ultra-fast scan |
| 2 | 4,800 | 76.8 | Quick scan |
| 3 | 8,640 | 138 | BDB default (`scan_duration: 3`) |
| 4 | 16,320 | 261 | Standard scan |
| 5 | 31,680 | 507 | Extended scan |
| 8 | 246,720 | 3,948 | Deep scan |

---

## Usage Patterns

### Reading current network state

```rust
let short = mac.mlme_get(PibAttribute::MacShortAddress).await?
    .as_short_address().unwrap();
let pan = mac.mlme_get(PibAttribute::MacPanId).await?
    .as_pan_id().unwrap();
let channel = mac.mlme_get(PibAttribute::PhyCurrentChannel).await?
    .as_u8().unwrap();
```

### Configuring TX performance

```rust
// Fewer backoffs, more retries, lower min BE
mac.mlme_set(PibAttribute::MacMaxCsmaBackoffs, PibValue::U8(3)).await?;
mac.mlme_set(PibAttribute::MacMaxFrameRetries, PibValue::U8(5)).await?;
mac.mlme_set(PibAttribute::MacMinBe, PibValue::U8(2)).await?;
```

### Setting up a sleepy end device

```rust
// Disable RX during idle for battery saving
mac.mlme_set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(false)).await?;
// Enable auto data request after beacon
mac.mlme_set(PibAttribute::MacAutoRequest, PibValue::Bool(true)).await?;
```

### Adjusting TX power

```rust
// Set to +4 dBm
mac.mlme_set(PibAttribute::PhyTransmitPower, PibValue::I8(4)).await?;
// Read back actual power (may be clamped by hardware)
let actual = mac.mlme_get(PibAttribute::PhyTransmitPower).await?;
```
