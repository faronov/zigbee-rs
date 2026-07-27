#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
BFLB_MCU_TOOL="${BFLB_MCU_TOOL:-bflb-mcu-tool}"
PORT="${1:-${BL702_PORT:-}}"
RAW_IMAGE=target/riscv32imc-unknown-none-elf/release/bl702-sensor.bin

if [[ -z "$PORT" ]]; then
    PORT="$("$PYTHON" - <<'PY'
from serial.tools import list_ports

ports = [
    port.device
    for port in list_ports.comports()
    if (port.vid, port.pid) == (0x1A86, 0x7523)
]
if len(ports) == 1:
    print(ports[0])
PY
)"
fi

if [[ -z "$PORT" ]]; then
    echo "XT-ZB1 CH340 port not found; pass it as the first argument" >&2
    exit 1
fi

./build-image.sh

printf 'Hold BOOT on the XT-ZB1, then press Enter to flash via %s: ' "$PORT"
read -r

"$PYTHON" - "$PORT" <<'PY'
import serial
import sys
import time

with serial.Serial(sys.argv[1], 115200) as port:
    port.dtr = False
    port.rts = False
    time.sleep(0.1)
    port.rts = True
    time.sleep(0.25)
    port.rts = False
    time.sleep(0.25)
PY

LOG_FILE="$(mktemp -t bl702-flash.XXXXXX)"
trap 'rm -f "$LOG_FILE"' EXIT

"$BFLB_MCU_TOOL" \
    --chipname=bl702 \
    --interface=uart \
    --port="$PORT" \
    --baudrate=115200 \
    --xtal=32M \
    --pllclk=Manual \
    --flashclk=XCLK \
    --firmware="$RAW_IMAGE" 2>&1 | tee "$LOG_FILE"

if ! grep -Fq '[All Successful]' "$LOG_FILE"; then
    echo "BL702 flashing did not complete successfully" >&2
    exit 1
fi

echo "Flash verified. Release BOOT, then run: ./monitor.sh $PORT"
