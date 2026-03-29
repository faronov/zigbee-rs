# Telink B91 & TLSR8258

Telink's B91 (RISC-V) and TLSR8258 (tc32 ISA) are popular SoCs in commercial
Zigbee products. The zigbee-rs Telink backend uses FFI bindings to the Telink
radio driver library for 802.15.4 radio access. Both chips share a single
`TelinkMac` driver.

## Hardware Overview

### Telink B91 (TLSR9218)

| Spec | Value |
|------|-------|
| **Core** | RISC-V 32-bit, up to 96 MHz |
| **Flash** | 512 KB |
| **SRAM** | 256 KB |
| **Radio** | BLE 5.0 + IEEE 802.15.4 |
| **Target** | `riscv32imc-unknown-none-elf` |
| **I/O** | UART ×2, SPI, I2C, ADC, PWM, USB |

### Telink TLSR8258

| Spec | Value |
|------|-------|
| **Core** | tc32 (Telink custom ISA) |
| **Flash** | 512 KB |
| **SRAM** | 64 KB |
| **Radio** | BLE + IEEE 802.15.4 |
| **Cargo target** | `thumbv6m-none-eabi` (stand-in for tc32) |
| **Real toolchain** | Telink tc32 GCC |

> The TLSR8258 uses Telink's proprietary **tc32** instruction set. There is no
> official Rust target for tc32. For `cargo check`/`cargo build`, we use
> `thumbv6m-none-eabi` as a compilation stand-in. Real production builds require
> the Telink tc32 GCC toolchain.

### Common Products Using These Chips

- **TLSR8258:** Sonoff SNZB-02/SNZB-03/SNZB-04, many Tuya Zigbee sensors,
  IKEA TRÅDFRI devices
- **B91:** Next-generation Telink Zigbee 3.0 modules, TL321x/TL721x variants

### Memory Maps

**B91:**
```
FLASH : ORIGIN = 0x20000000, LENGTH = 512K
RAM   : ORIGIN = 0x00000000, LENGTH = 256K
```

**TLSR8258:**
```
FLASH : ORIGIN = 0x00000000, LENGTH = 512K
RAM   : ORIGIN = 0x00840000, LENGTH = 64K
```

## Current Status

> ⚡ **Stubs mode** — Both Telink backends compile with stub FFI symbols in CI.
> Full radio operation requires linking the Telink SDK driver libraries.

The backends are architecturally complete:
- Full `MacDriver` trait implementation
- FFI bindings to Telink RF driver library
- Frame construction and PIB management
- Example firmware with GPIO, LED, and button handling

What's needed for real RF operation:
- Telink Zigbee SDK (`tl_zigbee_sdk`)
- Driver libraries (`libdrivers_b91.a` or `libdrivers_8258.a`)
- Proper Embassy time driver using the Telink system timer
- For TLSR8258: the tc32 GCC toolchain for real builds

## Prerequisites

### Rust Toolchain

**For B91:**
```bash
rustup default nightly
rustup update nightly
rustup target add riscv32imc-unknown-none-elf
rustup component add rust-src
```

**For TLSR8258:**
```bash
rustup default nightly
rustup update nightly
rustup target add thumbv6m-none-eabi
rustup component add rust-src
```

### Telink SDK (for real RF)

Download the [Telink Zigbee SDK](http://wiki.telink-semi.cn/wiki/IDE-and-Tools/):

```bash
export TELINK_SDK_DIR=/path/to/tl_zigbee_sdk
```

## Building

### Telink B91

**With stubs (CI mode):**
```bash
cd examples/telink-b91-sensor
cargo build --release --features stubs
```

**With Telink SDK (real radio):**
```bash
cd examples/telink-b91-sensor
TELINK_SDK_DIR=/path/to/sdk cargo build --release
```

### Telink TLSR8258

**With stubs (CI mode / cargo check):**
```bash
cd examples/telink-tlsr8258-sensor
cargo build --release --features stubs
```

> **Note:** This builds for `thumbv6m-none-eabi` as a stand-in. Real TLSR8258
> firmware requires the Telink tc32 GCC toolchain.

### CI Build Commands

From `.github/workflows/ci.yml`:

**B91:**
```bash
# Toolchain: nightly with riscv32imc-unknown-none-elf + rust-src + llvm-tools
cd examples/telink-b91-sensor
cargo build --release --features stubs

# Firmware artifacts
OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary $ELF ${ELF}.bin
$OBJCOPY -O ihex   $ELF ${ELF}.hex
```

**TLSR8258:**
```bash
# Toolchain: nightly with thumbv6m-none-eabi + rust-src + llvm-tools
cd examples/telink-tlsr8258-sensor
cargo build --release --features stubs

OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary $ELF ${ELF}.bin
$OBJCOPY -O ihex   $ELF ${ELF}.hex
```

### Build Scripts

**B91 `build.rs`:**
```rust
// Links libdrivers_b91.a when TELINK_SDK_DIR is set
if let Ok(sdk_dir) = std::env::var("TELINK_SDK_DIR") {
    let lib_path = format!("{}/platform/lib", sdk_dir);
    println!("cargo:rustc-link-search=native={}", lib_path);
    println!("cargo:rustc-link-lib=static=drivers_b91");
}
```

**TLSR8258 `build.rs`:**
```rust
// Links libdrivers_8258.a when TELINK_SDK_DIR is set
if let Ok(sdk_dir) = std::env::var("TELINK_SDK_DIR") {
    let lib_path = format!("{}/platform/lib", sdk_dir);
    println!("cargo:rustc-link-search=native={}", lib_path);
    println!("cargo:rustc-link-lib=static=drivers_8258");
}
```

### `.cargo/config.toml`

**B91:**
```toml
[build]
target = "riscv32imc-unknown-none-elf"

[unstable]
build-std = ["core", "alloc"]
```

**TLSR8258:**
```toml
[build]
# tc32 stand-in — real builds use Telink tc32 GCC toolchain
target = "thumbv6m-none-eabi"

[unstable]
build-std = ["core", "alloc"]
```

## Flashing

### B91 — Telink BDT (Burning & Debug Tool)

1. Connect via Telink's Swire debug interface
2. Use the Telink BDT GUI to flash the `.bin` file
3. Alternatively, use Telink's command-line `tl_check_fw` + `tl_bulk_pgm` tools

### TLSR8258 — Telink BDT or OTA

For commercial products (Sonoff SNZB-02 etc.), OTA updates through Zigbee
are the typical approach. For development:

1. Use Telink BDT via Swire debug pins
2. Flash the `.bin` to address 0x0000

### J-Link (B91 only)

Some B91 development boards support SWD debug via J-Link:

```bash
# If supported by your board:
probe-rs run --chip TLSR9218 target/riscv32imc-unknown-none-elf/release/telink-b91-sensor
```

## MAC Backend Notes

Both B91 and TLSR8258 share a single MAC backend in `zigbee-mac/src/telink/`:

```
zigbee-mac/src/telink/
├── mod.rs      # TelinkMac struct, MacDriver trait impl
└── driver.rs   # TelinkDriver — FFI bindings to Telink RF driver
```

### Feature Flag

```toml
# Same feature for both B91 and TLSR8258
zigbee-mac = { features = ["telink"] }
```

### Architecture

```
MacDriver trait methods
       │
       ▼
TelinkMac (mod.rs)
  ├── PIB state (addresses, channel, config)
  ├── Frame construction
  └── TelinkDriver (driver.rs)
         ├── FFI → tl_zigbee_sdk MAC PHY
         │     ├── rf_setChannel / rf_setTxPower / rf_setTrxState
         │     ├── rf802154_tx_ready + rf802154_tx / rf_setRxBuf
         │     └── rf_performCCA / rf_startEDScan / rf_getLqi
         ├── TX completion: rf_tx_irq_handler() → TX_SIGNAL
         └── RX completion: rf_rx_irq_handler() → RX_SIGNAL
```

### Packet Format

**TX buffer layout:**
```
[0..3]  dmaLen   (u32, LE — DMA header)
[4]     rfLen    (payload length + 2 for CRC)
[5..]   payload  (802.15.4 MAC frame)
```

**RX buffer layout:**
```
[0..3]  dmaLen      (u32, DMA transfer length)
[4]     rssi        (raw RSSI byte)
[5..11] reserved    (7 bytes)
[12]    payloadLen  (802.15.4 PSDU length)
[13..]  payload     (MAC frame)
```

### Radio Features

- 2.4 GHz IEEE 802.15.4 compliant
- Hardware CRC generation and checking
- Configurable TX power (chip-dependent power table)
- RSSI / LQI measurement
- Energy Detection (ED) scan
- CCA (Clear Channel Assessment) with configurable threshold
- DMA-based TX/RX with hardware packet format

## Example Walkthrough

### B91 Sensor

The `telink-b91-sensor` example is a Zigbee 3.0 end device for the B91
development board with GPIO-based button and LED control.

**Pin assignments (B91 devboard):**
- GPIO2 — Button (join/leave)
- GPIO3 — Green LED
- GPIO4 — Blue LED

**Device setup:**
```rust
let mac = TelinkMac::new();

let mut device = ZigbeeDevice::builder(mac)
    .device_type(DeviceType::EndDevice)
    .manufacturer("Zigbee-RS")
    .model("B91-Sensor")
    .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
        ep.cluster_server(0x0000)
            .cluster_server(0x0402)
            .cluster_server(0x0405)
    })
    .build();
```

### TLSR8258 Sensor

The `telink-tlsr8258-sensor` example targets TLSR8258-based products (Sonoff
SNZB-02 etc.). The code structure is identical to the B91 example — only
the hardware constants (GPIO addresses, pin assignments) differ.

**Time driver note:** Both examples include a minimal Embassy time driver
stub. A production firmware would implement a proper driver using the
Telink system timer.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Linker error: undefined `rf_*` | Telink SDK not linked | Set `TELINK_SDK_DIR` or use `--features stubs` |
| `portable-atomic` errors | Missing feature flag | Ensure `features = ["unsafe-assume-single-core"]` |
| TLSR8258 real build fails | tc32 toolchain needed | Use Telink tc32 GCC for production builds |
| B91 wrong target | Using `riscv32imac` | B91 CI uses `riscv32imc-unknown-none-elf` (no atomics) |
| No debug output | No logger registered | Use Telink UART or BDT for debug output |
| BDT can't connect | Swire not connected | Check debug interface wiring |

## Roadmap

To bring the Telink backends to full RF operation:

1. **Embassy time driver** — implement using Telink system timer
2. **Link real SDK** — test with `tl_zigbee_sdk` driver libraries
3. **Interrupt wiring** — connect RF IRQ handler to Embassy signals
4. **B91 HAL crate** — community `embassy-telink-b91` effort
5. **TLSR8258 Rust target** — explore custom target JSON for tc32 ISA
