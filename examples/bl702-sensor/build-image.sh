#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
REPO="$(cd ../.. && pwd)"

# Builds the XT-ZB1 sensor for the bootloader layout
# (products/bl702-xt-zb1/layout/src/lib.rs) and packages it:
#
#   ota/bl702-xt-zb1-<label>-<version>-full.bin    program once at 0x0 (ISP)
#   ota/bl702-xt-zb1-<label>-<version>-signed.bin  MCUboot image (slot 0x77000)
#   ota/1234-0702-<file version>-...zigbee         Zigbee OTA file
#
# The application only runs under the loader: it is linked at XIP
# 0x2300E100 (physical 0x10100) behind an MCUboot header.
#
# Every build uses the hardware-proven SEC_ENG AES backend. Set
# BL702_DIAGNOSTIC_LOG=1 only to retain the full UART trace.
#
# Signing key: $BL702_OTA_KEY, else ~/.flower-keys/flower-own-ec-p256.pem
# (never committed). Its public X || Y is baked into the loader and the
# application through BL702_OTA_KEY_XY.
#
# TEST ONLY: BL702_OTA_VERSION_OVERRIDE=MM.mm.pp builds an otherwise
# identical image with another version (e.g. a "v2" for a bench OTA test).

PYTHON="${PYTHON:-python3}"
OTA_TOOL="$REPO/tools/bl702_ota.py"
TARGET_DIR=target/riscv32imc-unknown-none-elf/release
ELF="$TARGET_DIR/bl702-sensor"
LOADER_DIR="$REPO/products/bl702-xt-zb1/bootloader"
LOADER_ELF="$LOADER_DIR/target/riscv32imc-unknown-none-elf/release/bl702-xt-zb1-boot"
OUT_DIR="$TARGET_DIR/ota"

BL702_OTA_KEY_XY="$("$PYTHON" "$OTA_TOOL" key-xy)"
export BL702_OTA_KEY_XY
echo "Signing key public X||Y: ${BL702_OTA_KEY_XY:0:16}..."
if [[ -n "${BL702_OTA_VERSION_OVERRIDE:-}" ]]; then
    echo "WARNING: TEST-ONLY version override ${BL702_OTA_VERSION_OVERRIDE}; never publish this build" >&2
fi

if [[ "${BL702_DIAGNOSTIC_LOG:-0}" == "1" ]]; then
    LABEL=diagnostic
    cargo build --release --locked --no-default-features --features diagnostic-logging
else
    LABEL=production
    cargo build --release --locked
fi
(cd "$LOADER_DIR" && cargo build --release --locked)

OBJCOPY="${OBJCOPY:-$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)}"
if [[ -z "$OBJCOPY" || ! -x "$OBJCOPY" ]]; then
    echo "llvm-objcopy not found; install it with: rustup component add llvm-tools-preview" >&2
    exit 1
fi

# Raw application (size report; the release step signs its own copy).
RAW_IMAGE="$ELF.bin"
"$OBJCOPY" -O binary "$ELF" "$RAW_IMAGE"

# The loader's boot ROM image. .loader_info stays out of the ROM-hashed
# range (see products/bl702-xt-zb1/bootloader/memory.x); the release step
# places it at physical 0xFFE0.
LOADER_RAW="$LOADER_ELF.rom.bin"
LOADER_IMAGE="$LOADER_ELF.boot.bin"
"$OBJCOPY" --gap-fill 0xff --remove-section .loader_info -O binary "$LOADER_ELF" "$LOADER_RAW"

# Manual keeps the CPU on the 32 MHz XTAL; the tool default selects 144 MHz.
rm -f firmware.bin
./run-bflb-mcu-tool.sh \
    --chipname=bl702 \
    --xtal=32M \
    --pllclk=Manual \
    --flashclk=XCLK \
    --firmware="$LOADER_RAW" \
    --build

if [[ ! -f firmware.bin ]]; then
    echo "bflb-mcu-tool did not produce firmware.bin" >&2
    exit 1
fi
mv -f firmware.bin "$LOADER_IMAGE"

# Signs the application, checks the boot header (magic, 32 MHz XTAL clocks,
# CRC, image hash, img_start 0x2000, end below 0xFFE0), the LoaderInfo key
# and the application entry, then writes the OTA file and the full image.
# One build per directory: flash.sh programs the only full image there.
rm -rf "$OUT_DIR"
"$PYTHON" "$OTA_TOOL" release \
    --app-elf "$ELF" \
    --loader-elf "$LOADER_ELF" \
    --loader-image "$LOADER_IMAGE" \
    --label "$LABEL" \
    --out "$OUT_DIR"

echo "Raw application: $RAW_IMAGE"
echo "Loader image:    $LOADER_IMAGE"
echo "Release files:   $OUT_DIR"
