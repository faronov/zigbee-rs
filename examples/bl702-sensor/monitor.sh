#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
PORT="${1:-${BL702_PORT:-}}"

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

exec "$PYTHON" -u - "$PORT" <<'PY'
import serial
import sys
import time

with serial.Serial(sys.argv[1], 2_000_000, timeout=0.05) as port:
    port.dtr = False
    port.rts = False
    time.sleep(0.1)
    port.reset_input_buffer()
    port.rts = True
    time.sleep(0.25)
    port.rts = False

    try:
        while True:
            data = port.read(4096)
            if data:
                print(data.decode("utf-8", "backslashreplace"), end="", flush=True)
    except KeyboardInterrupt:
        pass
PY
