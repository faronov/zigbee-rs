#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Build mode selection (mutually exclusive):
#   (default)                 standard software-AES production image
#   BL702_DIAGNOSTIC_LOG=1    diagnostic-logging build (no production profile)
#   BL702_HARDWARE_AES=1      opt-in SEC_ENG hardware-AES production image
#
# The hardware-AES variant is named with a `.hardware-aes` infix so it never
# overwrites the standard recovery artifacts; the standard `cargo build
# --release` output remains the default recovery image.
TARGET_DIR=target/riscv32imc-unknown-none-elf/release
ELF="$TARGET_DIR/bl702-sensor"

if [[ "${BL702_DIAGNOSTIC_LOG:-0}" == "1" && "${BL702_HARDWARE_AES:-0}" == "1" ]]; then
    echo "BL702_DIAGNOSTIC_LOG and BL702_HARDWARE_AES are mutually exclusive" >&2
    exit 1
fi

ARTIFACT_INFIX=""
if [[ "${BL702_DIAGNOSTIC_LOG:-0}" == "1" ]]; then
    cargo build --release --no-default-features --features diagnostic-logging
elif [[ "${BL702_HARDWARE_AES:-0}" == "1" ]]; then
    cargo build --release --features hardware-aes
    ARTIFACT_INFIX=".hardware-aes"
else
    cargo build --release
fi

RAW_IMAGE="$ELF$ARTIFACT_INFIX.bin"
FLASH_IMAGE="$ELF$ARTIFACT_INFIX.flash.bin"

OBJCOPY="${OBJCOPY:-$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)}"
if [[ -z "$OBJCOPY" || ! -x "$OBJCOPY" ]]; then
    echo "llvm-objcopy not found; install it with: rustup component add llvm-tools-preview" >&2
    exit 1
fi
"$OBJCOPY" -O binary "$ELF" "$RAW_IMAGE"

BFLB_MCU_TOOL="${BFLB_MCU_TOOL:-bflb-mcu-tool}"
if ! command -v "$BFLB_MCU_TOOL" >/dev/null 2>&1; then
    echo "bflb-mcu-tool not found; install version 1.10.0 with pip" >&2
    exit 1
fi

# Manual keeps the CPU on the 32 MHz XTAL; the tool default selects 144 MHz.
"$BFLB_MCU_TOOL" \
    --chipname=bl702 \
    --xtal=32M \
    --pllclk=Manual \
    --flashclk=XCLK \
    --firmware="$RAW_IMAGE" \
    --build

if [[ ! -f firmware.bin ]]; then
    echo "bflb-mcu-tool did not produce firmware.bin" >&2
    exit 1
fi

MAGIC="$(od -An -tx4 -N4 firmware.bin | tr -d '[:space:]')"
if [[ "$MAGIC" != "504e4642" ]]; then
    echo "invalid BL702 boot image magic: $MAGIC" >&2
    exit 1
fi

mv -f firmware.bin "$FLASH_IMAGE"
echo "Raw image:   $RAW_IMAGE"
echo "Flash image: $FLASH_IMAGE"
