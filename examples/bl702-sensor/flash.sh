#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
PORT="${1:-${BL702_PORT:-}}"
BFLB_IOT_TOOL="${BFLB_IOT_TOOL:-bflb-iot-tool}"
OTA_DIR="target/riscv32imc-unknown-none-elf/release/ota"

# Programs the full-flash image (boot header, loader, LoaderInfo and the
# signed application; physical 0x0..0x87000) unmodified at 0x0. The
# secondary slot (0x87000..0xFE000) and the security journal
# (0xFE000..0x100000) are outside the written range and keep their contents.
#
# bflb-mcu-tool always wraps --firmware in a new boot header, so this raw
# write uses bflb-iot-tool --single (pip install bflb-iot-tool==1.10.0).
# Not yet run on hardware: the bench programs the same file with hwlab.

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

if ! command -v "$BFLB_IOT_TOOL" >/dev/null; then
    echo "bflb-iot-tool not found; install it with: pip install bflb-iot-tool==1.10.0" >&2
    exit 1
fi

./build-image.sh

shopt -s nullglob
FULL_IMAGES=("$OTA_DIR"/bl702-xt-zb1-*-full.bin)
if [[ ${#FULL_IMAGES[@]} -ne 1 ]]; then
    echo "expected one full-flash image in $OTA_DIR, found ${#FULL_IMAGES[@]}" >&2
    exit 1
fi
FULL_IMAGE="${FULL_IMAGES[0]}"

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

"$BFLB_IOT_TOOL" \
    --chipname=bl702 \
    --interface=uart \
    --port="$PORT" \
    --baudrate=115200 \
    --addr=0x0 \
    --firmware="$FULL_IMAGE" \
    --single 2>&1 | tee "$LOG_FILE"

if ! grep -Fq '[All Successful]' "$LOG_FILE"; then
    echo "BL702 flashing did not complete successfully" >&2
    exit 1
fi

echo "Flash verified: $FULL_IMAGE at 0x0"
echo "Release BOOT, then run: ./monitor.sh $PORT"
