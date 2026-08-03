#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Every build uses the hardware-proven SEC_ENG AES backend. Set
# BL702_DIAGNOSTIC_LOG=1 only to retain the full UART trace.
TARGET_DIR=target/riscv32imc-unknown-none-elf/release
ELF="$TARGET_DIR/bl702-sensor"

if [[ "${BL702_DIAGNOSTIC_LOG:-0}" == "1" ]]; then
    cargo build --release --no-default-features --features diagnostic-logging
else
    cargo build --release
fi

RAW_IMAGE="$ELF.bin"
FLASH_IMAGE="$ELF.flash.bin"

OBJCOPY="${OBJCOPY:-$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)}"
if [[ -z "$OBJCOPY" || ! -x "$OBJCOPY" ]]; then
    echo "llvm-objcopy not found; install it with: rustup component add llvm-tools-preview" >&2
    exit 1
fi
"$OBJCOPY" -O binary "$ELF" "$RAW_IMAGE"

# Manual keeps the CPU on the 32 MHz XTAL; the tool default selects 144 MHz.
./run-bflb-mcu-tool.sh \
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

python3 - "$FLASH_IMAGE" <<'PY'
from pathlib import Path
import sys

image = Path(sys.argv[1]).read_bytes()
if image[:4] != b"BFNP":
    raise SystemExit("invalid BL702 boot image magic")

# BL702 boot-header clock fields:
# xtal_type=1 (32 MHz), pll_clk=1 (XTAL), hclk_div=0, bclk_div=0,
# flash_clk_type=1 (XCLK).
expected = bytes((1, 1, 0, 0, 1))
actual = image[0x68:0x6D]
if actual != expected:
    raise SystemExit(
        "invalid BL702 boot clocks at 0x68: "
        f"expected {expected.hex(' ')}, got {actual.hex(' ')}"
    )
PY

echo "Raw image:   $RAW_IMAGE"
echo "Flash image: $FLASH_IMAGE"
