#!/usr/bin/env bash
# build-tc32.sh — Build the TLSR8258 sensor example for real tc32 hardware
#
# This script builds the Rust code targeting Thumb-1 (which produces
# binary-compatible tc32 instructions) and then links with the Telink
# tc32-elf-ld linker to produce a flashable firmware.
#
# Prerequisites:
#   - Rust nightly with thumbv6m-none-eabi target
#   - Telink tc32-elf-gcc toolchain (tc32-elf-ld, tc32-elf-objcopy)
#   - TELINK_SDK_DIR pointing to tl_zigbee_sdk
#
# Usage:
#   cd examples/telink-tlsr8258-sensor
#   ./build-tc32.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TARGET_JSON="$REPO_ROOT/targets/tc32-none-eabi.json"

# ── Check prerequisites ─────────────────────────────────────────

if ! command -v tc32-elf-ld &>/dev/null; then
    echo "ERROR: tc32-elf-ld not found in PATH"
    echo ""
    echo "Install the Telink tc32-elf-gcc toolchain from:"
    echo "  https://github.com/nicufmurphy/tc32"
    echo "  or the Telink IDE (Eclipse-based) which bundles it."
    echo ""
    echo "Alternatively, build as a static library for C integration:"
    echo "  cargo build --release --target thumbv6m-none-eabi --crate-type staticlib"
    exit 1
fi

if [ -z "${TELINK_SDK_DIR:-}" ]; then
    echo "ERROR: TELINK_SDK_DIR not set"
    echo "  export TELINK_SDK_DIR=/path/to/tl_zigbee_sdk"
    exit 1
fi

# ── Step 1: Build Rust code as static library ────────────────────

echo "[1/4] Building Rust code with thumbv6m (tc32-compatible)..."
cd "$SCRIPT_DIR"
cargo +nightly build \
    --release \
    -Z build-std=core,alloc \
    --target "$TARGET_JSON"

# Find the output library
LIB_DIR="$SCRIPT_DIR/target/tc32-none-eabi/release"
RUST_LIB="$LIB_DIR/libtelink_tlsr8258_sensor.a"

if [ ! -f "$RUST_LIB" ]; then
    # Try the rlib
    RUST_LIB=$(find "$LIB_DIR" -name "*.a" | head -1)
fi

echo "  Rust library: $RUST_LIB"

# ── Step 2: Assemble tc32 startup code ───────────────────────────

echo "[2/4] Assembling tc32 startup code..."
BOOT_DIR="$TELINK_SDK_DIR/platform/boot/8258"
BOOT_S="$BOOT_DIR/cstartup_8258.S"

if [ ! -f "$BOOT_S" ]; then
    echo "  WARNING: $BOOT_S not found"
    echo "  Skipping startup assembly — using Rust entry point only"
    BOOT_OBJ=""
else
    tc32-elf-gcc -c -o "$LIB_DIR/cstartup.o" "$BOOT_S"
    BOOT_OBJ="$LIB_DIR/cstartup.o"
fi

# ── Step 3: Link with tc32-elf-ld ────────────────────────────────

echo "[3/4] Linking with tc32-elf-ld..."
LINK_SCRIPT="$SCRIPT_DIR/memory-tc32.x"
TELINK_LIB="$TELINK_SDK_DIR/platform/lib"
OUTPUT_ELF="$LIB_DIR/firmware.elf"

tc32-elf-ld \
    -T "$LINK_SCRIPT" \
    -nostdlib \
    ${BOOT_OBJ:+"$BOOT_OBJ"} \
    --whole-archive "$RUST_LIB" --no-whole-archive \
    -L "$TELINK_LIB" -ldrivers_8258 \
    -o "$OUTPUT_ELF"

echo "  ELF: $OUTPUT_ELF"

# ── Step 4: Create flashable binary ─────────────────────────────

echo "[4/4] Creating flashable binary..."
OUTPUT_BIN="$LIB_DIR/firmware.bin"
tc32-elf-objcopy -O binary "$OUTPUT_ELF" "$OUTPUT_BIN"

SIZE=$(wc -c < "$OUTPUT_BIN")
echo ""
echo "=== Build complete ==="
echo "  ELF:    $OUTPUT_ELF"
echo "  Binary: $OUTPUT_BIN ($SIZE bytes)"
echo ""
echo "Flash with: TelinkBDT --chip 8258 --firmware $OUTPUT_BIN"
