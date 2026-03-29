# nRF52840 / nRF52833

Nordic's nRF52840 and nRF52833 are ARM Cortex-M4F SoCs with a built-in
IEEE 802.15.4 radio. The zigbee-rs nRF backend uses Embassy's radio driver
for interrupt-driven, DMA-based TX/RX — **no SoftDevice required**.

## Hardware Overview

| | nRF52840 | nRF52833 |
|---|----------|----------|
| **Core** | ARM Cortex-M4F, 64 MHz | ARM Cortex-M4F, 64 MHz |
| **Flash** | 1024 KB | 512 KB |
| **RAM** | 256 KB | 128 KB |
| **Radio** | BLE 5.3 + 802.15.4 + NFC | BLE 5.3 + 802.15.4 + NFC |
| **Target** | `thumbv7em-none-eabihf` | `thumbv7em-none-eabihf` |

### Hardware Radio Features

- Auto-CRC generation and checking
- Hardware address filtering (PAN ID + short address)
- Auto-ACK for frames with ACK request bit set
- Energy Detection (ED) via EDREQ task
- RSSI measurement per packet
- DMA-driven TX/RX buffers
- Factory-programmed IEEE address in FICR registers

### Common Development Boards

- **nRF52840-DK (PCA10056)** — J-Link debugger, 4 buttons, 4 LEDs
- **nRF52840 USB Dongle (PCA10059)** — USB bootloader, compact form
- **nice!nano v2** — Pro Micro form factor, UF2 bootloader
- **Seeed XIAO nRF52840** — compact, USB-C
- **Makerdiary nRF52840 MDK USB Dongle** — UF2 bootloader
- **nRF52833-DK (PCA10100)** — J-Link debugger, 4 buttons, 4 LEDs

### No SoftDevice Needed

Unlike BLE-only projects, zigbee-rs accesses the 802.15.4 radio peripheral
directly through Embassy's `embassy-nrf` radio driver. There is no dependency
on Nordic's SoftDevice. This gives full control over the radio and avoids the
SoftDevice's RAM/Flash overhead.

> **UF2 variant note:** If your board has a SoftDevice-based UF2 bootloader
> (e.g., nice!nano with Adafruit bootloader), the `nrf52840-sensor-uf2`
> example disables the SoftDevice at startup via an SVC call. See the
> [UF2 section](#uf2-drag-and-drop-flash) below.

## Prerequisites

### Rust Toolchain

```bash
rustup default nightly
rustup update nightly

# Add the ARM Cortex-M4F target
rustup target add thumbv7em-none-eabihf
```

### Debug Probe (for DK boards)

```bash
# probe-rs handles flashing + defmt log viewing
cargo install probe-rs-tools
```

Supported probes:
- On-board J-Link (nRF52840-DK, nRF52833-DK)
- Any CMSIS-DAP probe
- Segger J-Link (external)

### For UF2 boards (no probe needed)

```bash
pip install intelhex   # for uf2conv.py
```

## Building

### nRF52840-DK (probe-rs)

```bash
cd examples/nrf52840-sensor
cargo build --release
```

### nRF52833-DK (probe-rs)

```bash
cd examples/nrf52833-sensor
cargo build --release
```

### nRF52840 UF2 (nice!nano / ProMicro / MDK Dongle)

```bash
cd examples/nrf52840-sensor-uf2
cargo build --release                                    # ProMicro (default)
cargo build --release --no-default-features --features board-mdk         # MDK Dongle
cargo build --release --no-default-features --features board-nrf-dongle  # PCA10059
cargo build --release --no-default-features --features board-nrf-dk      # DK (J-Link)
```

### nRF52840 Bridge (coordinator)

```bash
cd examples/nrf52840-bridge
cargo build --release
```

### What `.cargo/config.toml` Sets

```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip nRF52840_xxAA"

[env]
DEFMT_LOG = "info"
```

### CI Build Commands

From `.github/workflows/ci.yml`:

```bash
# nRF52840 sensor
cd examples/nrf52840-sensor
cargo build --release

# nRF52833 sensor
cd examples/nrf52833-sensor
cargo build --release

# UF2 variant (includes .uf2 conversion)
cd examples/nrf52840-sensor-uf2
cargo build --release

# Firmware artifact extraction
OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary $ELF ${ELF}.bin
$OBJCOPY -O ihex   $ELF ${ELF}.hex

# UF2 conversion (CI uses uf2conv.py from Microsoft's UF2 repo)
python uf2conv.py -c -f 0xADA52840 ${ELF}.hex -o ${ELF}.uf2
```

### Memory Layout

The `memory.x` linker script defines the memory regions:

**nRF52840** (full chip, no bootloader):
```
FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
RAM   : ORIGIN = 0x20000000, LENGTH = 256K
```

**nRF52833**:
```
FLASH : ORIGIN = 0x00000000, LENGTH = 512K
RAM   : ORIGIN = 0x20000000, LENGTH = 128K
```

**nRF52840 UF2 (with SoftDevice S140 bootloader)**:
```
FLASH : ORIGIN = 0x00026000, LENGTH = 808K    ← app starts after SoftDevice
RAM   : ORIGIN = 0x20002000, LENGTH = 248K
```

The UF2 example's `build.rs` selects the memory layout based on the board feature.

## Flashing

### probe-rs (DK boards)

```bash
cd examples/nrf52840-sensor

# Flash + live defmt log output
cargo run --release

# Or flash only
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

> **Tip:** Plug in the DK before running `cargo run`. probe-rs auto-detects
> the probe. Check with `probe-rs list` if detection fails.

### UF2 Drag-and-Drop Flash

For boards with UF2 bootloaders (nice!nano, ProMicro, MDK Dongle):

1. **Build the firmware:**
   ```bash
   cd examples/nrf52840-sensor-uf2
   cargo build --release
   ```

2. **Convert to UF2:**
   ```bash
   # Extract binary
   OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
   $OBJCOPY -O ihex target/thumbv7em-none-eabihf/release/nrf52840-sensor-uf2 fw.hex

   # Convert to UF2 (download uf2conv.py from Microsoft's UF2 repo)
   python uf2conv.py -c -f 0xADA52840 fw.hex -o fw.uf2
   ```

3. **Enter bootloader mode:** Double-tap the RESET button on the board.
   A USB mass storage device appears (e.g., `NICENANO`).

4. **Copy the `.uf2` file** to the USB drive. The board flashes automatically
   and reboots into your firmware.

### J-Link Commander (alternative)

```bash
nrfjprog --program target/thumbv7em-none-eabihf/release/nrf52840-sensor.hex --chiperase --verify
nrfjprog --reset
```

## MAC Backend Notes

The nRF MAC backend lives in `zigbee-mac/src/nrf/mod.rs` (single file — no
separate driver module needed since Embassy provides the radio abstraction).

### Feature Flags

| Feature | Chip | Cargo.toml dependency |
|---------|------|----------------------|
| `nrf52840` | nRF52840 | `zigbee-mac = { features = ["nrf52840"] }` |
| `nrf52833` | nRF52833 | `zigbee-mac = { features = ["nrf52833"] }` |

### Key Dependencies

```toml
embassy-nrf = { version = "0.3", features = ["nrf52840", "time-driver-rtc1", "gpiote"] }
embassy-executor = { version = "0.7", features = ["arch-cortex-m", "executor-thread"] }
```

### How It Works

1. **`NrfMac<T: Instance>`** wraps Embassy's `Radio<T>` and implements `MacDriver`
2. Radio TX/RX is fully interrupt-driven with DMA — no polling needed
3. Hardware auto-ACK is enabled for frames with the ACK request bit
4. Hardware address filtering is configured through the radio peripheral
5. The factory-programmed IEEE address is read from FICR registers
6. Embassy's `time-driver-rtc1` provides async timers via RTC1

### Embassy Integration

The nRF examples use Embassy's cooperative async executor:

```rust
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mac = zigbee_mac::nrf::NrfMac::new(radio);
    // ...
}
```

The `select3` combinator handles concurrent events:

```rust
match select3(
    device.receive(),                                    // Radio RX
    button.wait_for_falling_edge(),                      // Button press
    Timer::after(Duration::from_secs(REPORT_INTERVAL)),  // Periodic report
).await {
    Either3::First(event)  => { /* handle stack event */ }
    Either3::Second(_)     => { /* handle button press */ }
    Either3::Third(_)      => { /* read sensor, update clusters */ }
}
```

## Example Walkthrough

### nrf52840-sensor

The flagship example: an Embassy-based Zigbee 3.0 end device that reads the
on-chip temperature sensor and reports simulated humidity.

**Initialization:**

```rust
let p = embassy_nrf::init(Default::default());

// On-chip temperature sensor (real hardware reading)
let mut temp_sensor = Temp::new(p.TEMP, Irqs);

// Button 1 on nRF52840-DK (P0.11, active low)
let mut button = gpio::Input::new(p.P0_11, gpio::Pull::Up);

// IEEE 802.15.4 MAC driver (interrupt-driven, DMA-based)
let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
let mac = zigbee_mac::nrf::NrfMac::new(radio);
```

**Real temperature reading:**

```rust
// Read actual die temperature (°C with 0.25° resolution)
let temp_c = temp_sensor.read().await;
let temp_hundredths = (temp_c.to_num::<f32>() * 100.0) as i16;
temp_cluster.set_temperature(temp_hundredths);
```

### nrf52840-sensor-uf2

The UF2 variant supports multiple boards via cargo features:

| Feature | Board | LED | Flash Origin |
|---------|-------|-----|-------------|
| `board-promicro` | ProMicro / nice!nano | P0.15 (HIGH) | 0x26000 |
| `board-mdk` | Makerdiary MDK Dongle | P0.22 (LOW) | 0x1000 |
| `board-nrf-dongle` | Nordic PCA10059 | P0.06 (LOW) | 0x1000 |
| `board-nrf-dk` | Nordic DK (PCA10056) | P0.13 (LOW) | 0x0000 |

This variant auto-joins on boot (no button press needed) and includes a
`log` → `defmt` bridge so internal stack log messages appear in RTT output.

### nrf52840-bridge

A coordinator/bridge example that exposes the Zigbee network over USB serial.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `probe-rs` can't find device | Probe not connected | Check USB; run `probe-rs list` |
| `probe-rs` permission denied | Missing udev rules (Linux) | See [probe-rs setup](https://probe.rs/docs/getting-started/probe-setup/) |
| `292` / RAM overflow | Too many features enabled | Check Embassy feature flags, reduce arena size |
| defmt output garbled | Version mismatch | Ensure `defmt`, `defmt-rtt`, `panic-probe` versions match |
| UF2 board not appearing | Not in bootloader | Double-tap RESET quickly; look for USB drive |
| Device doesn't join | Coordinator not permitting | Enable permit-join on coordinator |
| No temperature reading | TEMP interrupt not bound | Ensure `bind_interrupts!` includes TEMP handler |

### Adjusting Log Level

```bash
# Via environment variable
DEFMT_LOG=trace cargo run --release

# Or set in .cargo/config.toml
[env]
DEFMT_LOG = "debug"
```

### Expected Serial Output (via RTT)

```
INFO  Zigbee-RS nRF52840 sensor starting…
INFO  Radio ready
INFO  Device ready — press Button 1 to join/leave
INFO  [btn] Joining network…
INFO  [scan] Scanning channels 11-26…
INFO  [scan] Found network: ch=15, PAN=0x1AAA
INFO  [join] Association successful, addr=0x1234
INFO  [sensor] T=23.75°C  H=52.30%
```
