# telink-tlsr8258-radio

Standalone, pure-Rust TLSR8258 802.15.4 radio bring-up firmware. **No
Zigbee-stack crate dependency** (`zigbee-mac`/`zigbee-nwk`/... are not used):
this crate exists to prove out PHY/DMA bring-up in isolation, independent of
the generic `zigbee-mac` Telink backend (which is known invalid for this
chip) and without touching the 7000-line `examples/telink-tlsr8258-sensor`
bring-up lab, which remains the read-only source of the proven low-level
sequences transcribed here.

## What it does

The build variant selects one explicit hardware gate:

| Variant | Gate |
|---|---|
| `band` | Repeated Beacon Requests on channels 11, 18, and 26 |
| `control` | Repeated TX/RX against the local channel-15 coordinator |
| `scan` | Standards-timed active scan across channels 11 through 26, including Zigbee beacon-payload parsing |
| `association` | Scan, MAC association, indirect Data Request polling, software ACK, and short-addressed unicast |
| `association-fresh` | Association with an isolated UID-derived locally administered EUI |
| `association-stress` | Association followed by 100 poll cycles and ten unicast frames |

All normal outbound frames use unslotted IEEE 802.15.4 CSMA-CA
(`macMinBE=3`, `macMaxBE=5`, `macMaxCSMABackoffs=4`), a 128 us averaged RSSI
CCA at the official Telink `-70 dBm` threshold, and up to three MAC retries
when an ACK is requested.

RX DMA is armed before TX and RX is selected synchronously at TX-done, before
returning to caller code. This is required for the 12-symbol (192 us)
802.15.4 ACK turnaround and mirrors the official Telink SDK
`rf_tx_irq_handler()` sequence. A previous implementation started RX from a
later task-level receive call; small images happened to meet the deadline,
while larger images missed every ACK.

All waits are bounded (fixed tick timeouts, never an infinite spin on radio
status); interrupts stay globally disabled the entire time (the firmware is
fully polled; nothing is logged from an ISR, because there is no ISR body
beyond a `bx lr` stub — see [Design notes](#design-notes)).

## Module map

| Module | Role |
|---|---|
| [`platform`](src/platform/mod.rs) | Boot vectors/startup asm, MMIO register helpers, clocks, Timer0-based bounded waits, GPIO/LED, linker-symbol accessors + layout self-checks |
| [`platform::flash`](src/platform/flash.rs) | Factory EUI / flash UID reads and stable UID-derived locally administered EUI fallback |
| [`radio`](src/radio/mod.rs) | PHY init, DMA TX/RX, immediate TX-done→RX transition, CCA/CSMA-CA, software ACK, bounded waits, and frame classification |
| [`radio::frame`](src/radio/frame.rs) | **Pure, host-testable** Beacon, Association Request/Response, Data Request, data, ACK, and DMA framing/parsing |
| [`mac_test`](src/mac_test.rs) | The explicit raw test state machine: channel cycle, TX, bounded RX window, outcome recording |
| [`association`](src/association.rs) | Active scan, parent selection, association, polling, unicast, retries, and stress state machines |
| [`diag`](src/diag.rs) | Fixed-address SRAM diagnostic record, checksum, cache-boundary canary |

### Host tests vs. embedded build

Hardware-only code (assembly, MMIO, linker externs, register tables) is
gated on `#[cfg(target_arch = "tc32")]`, true only for the real
`tc32-unknown-none-elf` target. `radio::frame`'s frame encode/decode logic
and `diag`'s checksum/canary/record logic have **no** such gate and are
unit-tested on the host via `cargo test` (`#![cfg_attr(not(test), no_std)]` /
`#![cfg_attr(not(test), no_main)]`). See [Testing](#testing).

> **Important — target arch note:** the tc32-45/tc32-43 forked toolchains'
> *built-in* `tc32-unknown-none-elf` target reports `target_arch = "tc32"`
> (confirmed via `rustc --print target-spec-json`), **not** `"arm"` — despite
> the standalone `targets/tc32-none-eabi*.json` target-spec files in this
> repository (unused by this crate) declaring `"arch": "arm"`. All cfg gates
> in this crate use `"tc32"`; double-check this if you ever switch to a
> JSON-file target instead of the toolchain's built-in one.

## Building

Requires one of the tc32-stage2 forked toolchains at
`.toolchains/tc32-stage2-tc32-45` (default) or `tc32-stage2-tc32-43`
(comparison), same as `examples/telink-tlsr8258-sensor`.

```sh
# Default toolchain (tc32-45)
scripts/tlsr8258.sh build

# Local ZHA RX control on channel 15
scripts/tlsr8258.sh build control

# Active scan / association / stress
scripts/tlsr8258.sh build scan
scripts/tlsr8258.sh build association
scripts/tlsr8258.sh build association-stress

# Explicit tc32-43 for comparison
TC32_TOOLCHAIN=/path/to/.toolchains/tc32-stage2-tc32-43 scripts/tlsr8258.sh build

# cargo check only, no link
scripts/tlsr8258.sh check
```

Equivalent raw `cargo` invocation (what the script wraps):

```sh
TC32=.toolchains/tc32-stage2-tc32-45
env CARGO_HOME="$HOME/.cargo" "$TC32/bin/cargo" rustc --release \
    --target tc32-unknown-none-elf \
    -Z build-std=core -Z build-std-features=compiler-builtins-mem \
    -- -C lto=no -C opt-level=1
```

`build` always runs `verify_layout` after linking (see
[Post-link verification](#post-link-verification)) and never passes
`--noinhibit-exec` to the linker — a memory.x `ASSERT()` violation must fail
the build, not silently produce an image.

### Flashing / dumping

```sh
scripts/tlsr8258.sh flash [variant]       # build, then TlsrPgm.py flash
scripts/tlsr8258.sh dump-diag [words]     # diagnostic record (default 64 words)
scripts/tlsr8258.sh dump <addr> [words]   # dump an arbitrary address
```

These require `TlsrPgm.py`/`tlsr_debug.py` and a connected board
(`TELINK_PORT`, default `/dev/cu.usbserial-1410`) — **not exercised here**;
hardware flashing is done by the parent session.

## Testing

Host-side unit/golden-vector tests (`radio::frame`, `diag`) run against the
ambient host `cargo`/`rustc`, not the tc32 toolchain (there is no host `std`
for the `tc32-unknown-none-elf` target):

```sh
scripts/tlsr8258.sh test
# equivalent to:
cargo test --target "$(rustc -vV | awk '/^host:/ {print $2}')"
```

27 tests cover Beacon Request, Association Request/Response, pre- and
post-association Data Request, short-addressed data and ACK golden vectors;
DMA/length/CRC/RSSI transcriptions; short/extended Zigbee Beacon parsing;
RSSI→LQI; diagnostics; and Timer0 conversion.

## Diagnostic record

Fixed address **`0x0084FE00`** (512 bytes, ending at `0x00850000`, the top
of the 64 KiB SRAM window), defined as a `NOLOAD` linker section (`.diag` in
`memory.x`) so the assembly startup's `.bss`-zero loop (which only touches
`[_sbss, _ebss)`) never touches it. Both stacks sit **below** this address
and grow downward (SP descending), so a stack overflow moves away from the
diagnostics rather than toward them.

`diag::init()` reads the existing record at boot, validates
`magic`/`version`/checksum, and either increments `boot_count` (record
survived intact) or resets it deterministically via `DiagRecord::fresh()`
(cold power-on, corrupted record, or firmware/layout version bump). Fields
include raw scan and beacon data; association status and assigned short
address; ACK latency; pending/non-pending polls; software ACK; unicast and
stress results; CCA attempts/busy/access failures; frame retries; and the
cache/`.data`/`.bss` invariants. CPU address `0x84FE00` is read through the
SWire alias **`0x04FE00`**.

### Cache-boundary canary

[`diag::CACHE_CANARY`](src/diag.rs) is forced to be the **first word of
`.data`** (`.data.canary_first` input section, placed ahead of the generic
`.data`/`.data.*` wildcard in `memory.x`), i.e. it sits at exactly
`_icache_data_end_` — immediately after the TLSR8258 I-cache tag/data
reservation. A cache write overrun (the historical TLSR8258 bug class this
bring-up guards against) corrupts this word before any other static,
making the overrun externally visible via `diag::verify_cache_canary()`
(called periodically from `mac_test::run()`) instead of silently corrupting
program state.

`CACHE_CANARY` is declared `static mut` (never accessed through a `&`/`&mut`
reference, only `addr_of_mut!` + `read_volatile`) rather than a plain
immutable `static` — see the code comment in `diag.rs` for why: LLVM treats
a never-mutated, no-interior-mutability `static` as a true constant
regardless of `#[link_section]`, which was empirically observed to strip
the ELF `.data` output section's `SHF_WRITE` flag (`llvm-readelf -S` showed
`AR` instead of `WA`) even though address placement was already correct.

`.data`/`.bss` initialization is independently spot-checked at boot
(`platform::linker::data_init_ok()` re-reads the canary;
`platform::linker::bss_zero_ok()` samples a handful of words across
`[_sbss, _ebss)`), recorded into the diagnostic record's
`data_init_ok`/`bss_zero_ok` fields.

## Post-link verification

`scripts/tlsr8258.sh build` always runs `verify_layout`, which re-derives
every `memory.x` invariant from the **linked ELF's** symbol table (some lld
configurations silently swallow in-script `ASSERT()` failures, so this is
not redundant):

1. I-cache tag (`_ictag_start_`/`_ictag_end_`, 256 B) and data
   (`_icache_data_end_`, next 0x800 B) reservation sizes.
2. `.data` (`_sdata`) starts at exactly `_icache_data_end_` (so the cache
   canary really is the first `.data` word).
3. `.rf_dma` (`RF_RX_BUF`/`RF_TX_BUF`) placement **outside** the cache
   reservation, **below** the IRQ stack, and **4-byte aligned** — checked
   directly against the linked addresses, not just `#[repr(align(4))]` at
   compile time.
4. `.bss` stays below the IRQ stack; IRQ stack top abuts SVC stack bottom;
   SVC stack top stays at/below the diagnostic record start.
5. Diagnostic record address (`== 0x0084FE00`) and reserved size (warns if
   `!= 512`, fails if `< 64`).
6. `.ram_code` fits under the absolute `.text` base (`<= 0x8000`).
7. Image size: **warns** past the 256 KiB production/OTA slot (`0x40000`),
   **fails** if it reaches the factory-data region (`0x76000`).

## Design notes / limitations

- **RF PHY table "6+28" vs. the task's "5+28":** the proven source
  (`examples/telink-tlsr8258-sensor/src/main.rs`, `tbl_rf_init`) documents
  **6** common-init register writes (`0x8012D2`, `0x8012D3`, `0x80127B`,
  `0x801276`, `0x801277`, `0x800430`) followed by the 28-entry Zigbee-250K
  table. This crate transcribes that faithfully as 6+28
  (`radio::phy::rf_phy_init_zigbee`) with an explicit code comment flagging
  the discrepancy from the task description's "5+28" — the hardware-proven
  sensor lab is treated as authoritative.
- **Fully polled test harness:** the IRQ vector remains a bare `bx lr` stub;
  the same mandatory TX-done→RX transition that the SDK performs in its TX
  ISR is performed synchronously in the polled send function. All waits are
  deadline-bounded.
- **Software ACK is address-filtered:** only CRC-valid frames addressed to
  the configured PAN and local extended/short address are ACKed. It runs
  before the full 144-byte DMA snapshot so the coordinator accepts it within
  the MAC turnaround budget.
- **One RX DMA buffer:** sufficient for these gates, but the reusable HAL
  should use two buffers so higher-layer processing cannot hold DMA
  ownership.
- **No NWK/APS/security/ZDO yet:** raw MAC association does not create a
  Home Assistant device. Network-key transport, Device Announcement, ZDO
  interview, APS security, and ZCL remain the next integration layer.
- **No flash program/erase routines:** unlike the sensor lab (which needs
  `.ram_code`-resident flash erase/program for its own OTA-adjacent
  experiments), this crate's `.ram_code` only holds `platform::clocks::init`
  and `radio::phy::set_channel` (channel-switch register writes must not
  execute from flash mid-switch) — deliberately minimal.
- **Independent capture still required on 11/18/26:** TX-done is validated
  on all three default channels and RX is validated against the live channel
  15 coordinator, but an external sniffer/transmitter is still needed to
  prove actual over-the-air frequency/payload reception on 11/18/26.
- **Intentional bad-FCS injection is still missing:** CRC rejection is
  implemented and exercised on received status, but a controlled on-air
  malformed-FCS source is required for independent proof.

## Validation performed

- Host golden-vector suite: **27/27 passed**.
- tc32-45 and tc32-43 builds pass all post-link SRAM/cache/DMA/stack/image
  checks. TLSR8258 SRAM is 64 KiB; current writable layout starts at
  `0x840900 + align256(ram_code_size)` after the 0x100-byte cache-tag and
  0x800-byte cache-data reservations.
- tc32-45 band gate: 296 TX completions, zero TX timeout, 98 complete
  11/18/26 cycles, 300 CCA checks, four busy results, zero access failures.
- tc32-45 channel-15 control: 415 TX completions and 437 valid coordinator
  Beacons in ten seconds. CCA observed 64 busy periods; two Beacon Requests
  exhausted channel access while the channel was genuinely occupied.
- Active scan:
  - tc32-45: 22 valid Zigbee descriptors in three complete scans;
  - tc32-43: 18 valid Zigbee descriptors in three complete scans;
  - measured dwell remains within a few hundred 24 MHz ticks of
    `0x32A000`.
- tc32-45 association: first logical attempt succeeded, status `0`, assigned
  short address `0x575E`, 16/16 post-association polls ACKed, and 1/1
  short-addressed unicast ACKed.
- Large-image stress after the TX→RX and CSMA fixes:
  - association succeeded on the first logical attempt;
  - **100/100** stress polls ACKed;
  - **10/10** stress unicasts ACKed;
  - **0** logical failures;
  - 182 CCA checks, 13 busy results, zero channel-access failures, and 24
    successful MAC retries.
- Reset stress: 100/100 consecutive hardware resets passed with monotonic
  boot count, valid diagnostic checksum, zero TX timeouts, and intact
  cache/data/BSS canaries.

The earlier small/large binary split is not evidence of a tc32 compiler bug.
Disassembly showed that inlining moved substantial state initialization
between TX completion and RX activation in the larger image. The official
Telink SDK switches to RX as the first action in `rf_tx_irq_handler()`;
moving that transition into the radio send path removed the size/layout
dependency without a compiler patch.
