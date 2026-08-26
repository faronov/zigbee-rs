# NV Storage

Persistence separates physical flash, product layout, and Zigbee durability
semantics:

```text
chip HAL            flash controller and raw NOR operations
board               physical flash resource
product             bounded partitions, linker map, migration/reset policy
zigbee-runtime      journals and security/child semantics
application         when to restore, checkpoint, clear, or activate OTA
```

Partition addresses belong to the product, not a generic HAL or example loop.

## Security state

`SecurityStateJournal<F>` is a two-sector atomic journal for:

- network identity and keys;
- outgoing NWK/APS counter reservations;
- parent and commissioned state;
- staged key/update state;
- End Device Timeout client state.

Each record has a version, generation, CRC, read-back verification, and a
commit marker written last. The scanner tolerates erased, torn, corrupt, and
unknown-version slots and selects the newest valid generation.

Outgoing counters are reserved ahead in durable storage. After a crash, the
runtime resumes above the reservation rather than reusing a secured frame
counter.

Do not replace the security journal with generic `LogStructuredNv`; their
durability contracts are different.

The default journal sector is 4 KiB. A product with another physical erase
size fixes that size in the type while retaining the same journal logic:

```rust,ignore
type SecurityStore =
    SecurityStateJournal<PartitionFlash, { PRODUCT_SECURITY_SECTOR_SIZE }>;

let store = SecurityStore::new_with_sector_size(
    partition_flash,
    0,
    PRODUCT_SECURITY_SECTOR_SIZE as u32,
);
```

`PartitionFlash` translates those relative offsets into the product-owned
protected address range. Boards never supply partition addresses or a
product-specific journal wrapper.

Current records use version 4. Older supported records decode through explicit
migration paths. Downgrading to firmware that cannot understand the current
record version is not supported because selecting an older counter reservation
can create replay risk.

## Generic application NV

`NvStorage` is the item API for non-security state. `LogStructuredNv<F>`
implements it over two NOR sectors. EFR32MG1 uses a separate application-NV
partition in addition to its security journal.

## Parent child table

Child persistence is intentionally separate from security state:

- `PersistentChildren<C>` is used by `ParentRouterApp` and `CoordinatorApp`;
- `ChildTableJournal<F>` stores bounded child snapshots;
- `NoChildren` is used by `RelayRouterApp`.

The child journal stores the extended PAN ID and each admitted child's
identity/configuration/timeout enumeration. It does not store NWK/APS outgoing
counters. Restoring a foreign-network or corrupt snapshot is an explicit
error and the stale record is cleared before fresh parent operation.

The runtime writes only when the child table fingerprint changes. Factory
reset clears child persistence before recommissioning, even if the next
network has the same extended PAN ID.

## Current product partitions

| product | security state | other protected storage |
|---|---|---|
| nRF52840 DK sensor/router | `0xFE000..0x100000` | UF2 variants use board-specific product maps |
| nRF52833 sensor | `0x7E000..0x80000` | — |
| ESP32-C6/H2 | `0x3FE000..0x400000` | OTA slots and `otadata` are separate product partitions |
| BL702 XT-ZB1 | `0xFE000..0x100000` | — |
| CC2340R5 | `0x7E000..0x80000` | — |
| PHY6222 | `0x7E000..0x80000` | — |
| PHY6252 | `0x3E000..0x40000` | — |
| EFR32MG1 | `0x37000..0x39000` | application NV `0x39000..0x3A000`; bootloader/native regions preserved |
| EFR32MG21 | `0x7C000..0x80000` | bootloader `0x00000..0x04000` |
| TLSR8258 TB-04 | `0x74000..0x76000` | child `0x72000..0x74000`, factory EUI/config `0x76000..0x78000` |

Linker scripts and Rust partition wrappers independently assert the same
boundaries.

## Ownership examples

### ESP32

The board exposes raw chip flash. The product checks the 4 MiB partition table,
constructs a bounded security journal, performs legacy migration, and owns the
OTA writer. Neither example hard-codes the final 8 KiB address.

### TLSR8258

The product consumes one board flash token and splits it into distinct
child/security/factory capabilities. The sensor drops the child token; the
router consumes it. Zbit page program/erase requires a fresh, stable ADC/PC5
voltage check and fails closed.

### EFR32MG1

The product reserves security and generic application NV separately. Direct
USART0 access to external OTA storage and Gecko Bootloader-managed access are
alternative owners of the same physical path.

### EFR32MG21

The product bounds raw board flash to `0x7C000..0x80000` and instantiates
`SecurityStateJournal<PartitionFlash, { 8 * 1024 }>`. Its linker script keeps
the same region out of the application image.

## Identity and reset order

Before resuming persisted state, compare it with the factory/device EUI-64.
If identity changed, clear incompatible membership before constructing a
running node.

A durable factory reset must:

1. stop new secured work;
2. clear security state;
3. clear child state for parent products;
4. preserve factory identity/calibration and bootloader/OTA regions;
5. reset or start fresh commissioning.

OTA activation follows the same safety rule: checkpoint security before the
reset-causing activation call.

## Hardware status

- nRF, ESP32, EFR32MG1, and deployed TLSR8258 security persistence have
  hardware evidence.
- BL702 journal integration builds, but destructive erase/program and
  reset/resume remain open.
- PHY62x2, CC2340, and EFR32MG21 flash paths remain hardware-unverified.
- Telink child-table persistence exists and passes host/target checks; complete
  corrected-image child acceptance remains a router HIL gate.

## Adding a backend

1. Implement `ReadNorFlash` and `NorFlash` in the chip HAL.
2. Expose the physical flash token from the board.
3. Bound it to product partitions.
4. reserve the same regions in the product linker layout;
5. construct the correct journal in the product;
6. pass the store to `ZigbeeNode`/the shared application.

Return controller failures. Never turn a failed erase/program/read-back into
success.
