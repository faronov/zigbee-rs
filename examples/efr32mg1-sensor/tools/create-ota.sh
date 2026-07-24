#!/bin/sh
set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "usage: $0 <firmware-version> [output-directory]" >&2
    exit 2
fi

VERSION_INPUT=$1
OUTPUT_DIR=${2:-target/ota}
DEVICE=EFR32MG1P132F256IM32
MANUFACTURER_ID=0x1049
IMAGE_TYPE=0x0002

if [ -n "${COMMANDER:-}" ]; then
    COMMANDER_BIN=$COMMANDER
elif [ -x "/Applications/Commander-cli.app/Contents/MacOS/commander-cli" ]; then
    COMMANDER_BIN=/Applications/Commander-cli.app/Contents/MacOS/commander-cli
else
    COMMANDER_BIN=$(command -v commander)
fi

VERSION_DEC=$(python3 - "$VERSION_INPUT" <<'PY'
import sys

value = int(sys.argv[1], 0)
if not 0 <= value < 0xFFFFFFFF:
    raise SystemExit("firmware version must be in 0..0xFFFFFFFE")
print(value)
PY
)

mkdir -p "$OUTPUT_DIR"
EFR32_OTA_VERSION=$VERSION_DEC cargo build --release
ELF=target/thumbv7em-none-eabi/release/efr32mg1-sensor
tools/verify-layout.py "$ELF"

BASE=$OUTPUT_DIR/efr32mg1-sensor-v$VERSION_DEC
S37=$BASE.s37
GBL=$BASE.gbl
OTA=$BASE.ota
PARSED_APP=$OUTPUT_DIR/.parsed-app.s37

arm-none-eabi-objcopy -O srec "$ELF" "$S37"
APP_PROPERTIES_ADDR=$(arm-none-eabi-nm -g --defined-only "$ELF" |
    awk '$3 == "APP_PROPERTIES" { print "0x" $1 }')
python3 - "$S37" "$APP_PROPERTIES_ADDR" "$VERSION_DEC" <<'PY'
import struct
import sys

memory = {}
for line in open(sys.argv[1], encoding="ascii"):
    line = line.strip()
    if not line.startswith(("S1", "S2", "S3")):
        continue
    address_bytes = {"S1": 2, "S2": 3, "S3": 4}[line[:2]]
    record = bytes.fromhex(line[2:])
    count = record[0]
    address = int.from_bytes(record[1:1 + address_bytes], "big")
    data = record[1 + address_bytes:count]
    for offset, value in enumerate(data):
        memory[address + offset] = value

address = int(sys.argv[2], 0)
raw = bytes(memory[address + offset] for offset in range(36))
actual = struct.unpack_from("<I", raw, 32)[0]
expected = int(sys.argv[3])
if actual != expected:
    raise SystemExit(f"APP_PROPERTIES version mismatch: {actual} != {expected}")
PY

"$COMMANDER_BIN" gbl create "$GBL" --app "$S37" --device "$DEVICE"

GBL_SIZE=$(wc -c < "$GBL" | tr -d ' ')
if [ "$GBL_SIZE" -gt 262144 ]; then
    echo "GBL exceeds the 256 KiB external OTA slot: $GBL_SIZE bytes" >&2
    exit 1
fi

"$COMMANDER_BIN" ota create \
    --type zigbee \
    --upgrade-image "$GBL" \
    --firmware-version "$VERSION_DEC" \
    --manufacturer-id "$MANUFACTURER_ID" \
    --image-type "$IMAGE_TYPE" \
    --min-hw 1 \
    --max-hw 1 \
    --string "zigbee-rs EFR32MG1 sensor" \
    --outfile "$OTA"

"$COMMANDER_BIN" gbl parse "$GBL" --app "$PARSED_APP"
"$COMMANDER_BIN" ota parse "$OTA"
rm -f "$PARSED_APP"
printf 'Created %s (%s-byte GBL)\n' "$OTA" "$GBL_SIZE"
