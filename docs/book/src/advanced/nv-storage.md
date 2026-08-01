# NV Storage

The storage stack separates portable persistence algorithms from flash
controllers, physical board wiring, and product-specific partition layouts:

```text
zigbee-runtime      persistence algorithms and Zigbee security semantics
embedded-storage    common raw NOR flash traits
<chip>-hal          flash controller implementation
boards/<board>      physical flash chip/bus wiring only
products/<product>  bounded partitions, linker layout, bootloader/OTA policy
examples/<role>     application behavior and scheduling
```

Application/NV partition addresses must not be placed in examples or generic
chip HALs. They depend on the board, bootloader, OTA layout, and linked
firmware region, so the product owns them. Vendor-defined factory metadata
locations are different: a chip HAL may model them when they are derived from
the verified fitted flash geometry.

## Generic application NV

`NvStorage` is an item-oriented interface for ordinary application and stack
state:

```rust
pub trait NvStorage {
    fn read(
        &mut self,
        id: NvItemId,
        buf: &mut [u8],
    ) -> Result<usize, NvError>;
    fn write(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError>;
    fn delete(&mut self, id: NvItemId) -> Result<(), NvError>;
    fn exists(&mut self, id: NvItemId) -> Result<bool, NvError>;
    fn item_length(&mut self, id: NvItemId) -> Result<usize, NvError>;
    fn compact(&mut self) -> Result<(), NvError>;
}
```

`RamNvStorage` implements this interface for host tests.
`LogStructuredNv<F>` provides flash-backed storage when `F` implements
`embedded_storage::nor_flash::NorFlash`. It uses two erase sectors, appends
new item versions, and copies live values during compaction.

```rust
use zigbee_runtime::log_nv::LogStructuredNv;

let nv = LogStructuredNv::new(flash_partition, 0, erase_size)?;
```

The flash value passed here is already bounded to the product-owned partition.
Offsets are relative to that partition, not absolute chip addresses.

## Security state

Zigbee network keys and outgoing frame counters require stronger guarantees
than generic item storage. `SecurityStateJournal<F>` is a separate two-sector
atomic journal with CRC, generations, read-back verification, and a final
commit marker. It also supports crash-safe outgoing-counter reservations.

Do not replace the security journal with `LogStructuredNv`. Both use the same
raw NOR traits, but they intentionally provide different semantics.

### Record versions

| Version | Encoded state | CRC offset | Added |
|---|---|---|---|
| 1 | 80 bytes | 92 | initial layout |
| 2 | 97 bytes | 112 | staged network key |
| 3 | 98 bytes | 112 | R22 End Device Timeout negotiation |

Version 3 reuses flags bit 6 for `parent_information_valid`, the previously
unused encoded byte 11 for `parent_information` and the new byte 97 for
`end_device_timeout`. Nothing else moved: the 128-byte slot size, 116-byte
record prefix and byte-124 commit marker are unchanged, so the crash-safety
scheme and every existing partition layout are unaffected. Version 1 and 2
records are still decoded through their own explicit paths and migrate to
"never negotiated, default enumeration 8".

> **Downgrade is not supported.** Once this firmware has written a version 3
> record, older firmware will skip it while scanning and select the newest
> record it *can* decode — an older generation with stale counters, a stale
> parent and possibly a stale network key. Reusing those reservations would
> replay NWK/APS frame counters. Recommission the device instead of
> downgrading.

## Child table (router/coordinator)

A router persists its authenticated child table through a **separate** durable
store (`zigbee_runtime::child_store`), kept independent of the security journal
on purpose: the security record is rewritten on every frame-counter reservation
and must stay small, whereas the child table is a larger, lower-frequency
snapshot. The child store never reads or writes NWK/APS frame counters, so
restoring or discarding it can never replay a secured frame.

Each child persists its IEEE identity, short address, capability/configuration
(rx-on-when-idle, security-capable, router vs end device) and accepted End
Device Timeout enumeration. The live aging countdown is **not** persisted — on
restore each end-device child's deadline is re-armed to the full window of its
enumeration, granting it a fresh window to prove liveness rather than being
evicted immediately after a reboot, and avoiding a flash write per countdown
tick.

| Backend | Use |
|---|---|
| `RamChildTableStore` | Complete generic store for a router **without** a configured child-table partition — keeps the table for the current power cycle only and never claims durable persistence. |
| `ChildTableJournal<S: NorFlash>` | Two-sector crash-safe journal (its own sectors, not the security journal's) with generation numbers, a CRC over the record prefix and a trailing commit marker written last. |

The record is versioned (`ZBCT`, version 1). An unrecognised version is skipped
like a corrupt record, so downgrading simply drops the persisted child table:
children re-appear through normal association/rejoin and keepalive, and no
security counter is affected. Wire it through `ZigbeeDevice::save_child_table`
and `restore_child_table`; a successful restore schedules a Parent Announce.

> **Partition not yet allocated.** No product linker script currently reserves
> child-table sectors, so a router opts in with `RamChildTableStore` today.
> Durable child persistence on hardware requires allocating two dedicated
> sectors (distinct from the security journal's) in the product memory map and
> constructing a `ChildTableJournal` over them — a remaining hardware gate.

## Platform ownership

| Platform | Flash controller | Current partition owner | Store |
|---|---|---|---|
| BL702 | `bl702-hal` mask-ROM XIP flash | `products/bl702-xt-zb1` | Security journal; destructive silicon validation pending |
| TLSR8258 | `tlsr8258-hal` | `products/tlsr8258-tb04` | Security journal; geometry-aware identity and fail-closed Zbit voltage guard |
| nRF52840 | Embassy NVMC | `products/nrf52840-sensor` | Security journal |
| PHY6222/PHY6252 | `phy6222-hal` | `boards/phy62x2-evk` | Security journal |
| ESP32-C6/H2 | `esp_storage::FlashStorage` | `products/esp32-zigbee-devkit` | Security journal |
| EFR32MG1P | `efr32mg1-hal` MSC | `products/efr32mg1-tradfri` | Security journal + separate generic NV |
| EFR32MG21 | `efr32mg21-hal` MSC | `boards/efr32mg21-devkit` | Generic NV |

BL702, TLSR8258, EFR32MG1P, ESP32-C6/H2, and nRF52840 are product-split platforms:
`boards/<board>` exposes only the physical chip flash resource (raw
whole-chip read/write/erase, no partition or NV knowledge, no
`zigbee-runtime` dependency), and `products/<product>` bounds it to a window
and constructs the store. The remaining board-owned rows are legacy
boundaries that will migrate after the API is proven across more platforms.
Partition wrappers validate bounds and translate relative offsets to
physical addresses. Product linker scripts (or, on ESP32, the product-owned
partition table) reserve the same regions so application code cannot overlap
persistent storage.

### TLSR8258 geometry and write safety

TLSR8258 factory EUI-64 and ADC-calibration sectors move with flash capacity.
The HAL supports the Telink 512 KiB, 1 MiB, 2 MiB, and 4 MiB layouts.
Non-512-KiB products must verify the JEDEC capacity before reading those
sectors; otherwise ordinary application data at another geometry's address
could be accepted as a plausible device identity.

The TB-04 product remains a 512 KiB layout. Its security journal occupies
`0x74000..0x76000`, immediately below the factory EUI sector, and its
production examples preserve their deployed EUI offsets.

Zbit `ZB25WD40B`/`ZB25WD80B` flash writes have an additional hardware
requirement. Before each physical page program or sector erase, the HAL calls
the owned ADC/PC5 guard and requires a measured voltage above 2200 mV with
less than 500 mV fluctuation. Missing, busy, or failed ADC readings return a
flash error; there is no constant-voltage fallback. Multi-page writes repeat
the check for every page rather than trusting one stale measurement.

## Adding a platform

1. Implement `ReadNorFlash` and `NorFlash` in the chip HAL.
2. Expose only physical flash wiring from the board crate.
3. Define a bounded partition wrapper in the product crate.
4. Reserve that partition in the product linker layout.
5. Construct `SecurityStateJournal` or `LogStructuredNv` in the product.
6. Pass the resulting store to the application without exposing addresses.

Flash errors must be returned to the storage algorithm. Reads, writes, and
erases must never be treated as successful after a controller failure.
