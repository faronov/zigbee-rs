#!/bin/sh
# Convert the checked-in partition CSV into the 3072-byte ESP-IDF binary that
# is flashed at 0x8000.
#
# Usage: partitions/build-partition-table.sh [output-file]
#
# Requires `espflash` (already required to flash these examples). The generated
# binary is verified twice: its size must be exactly 3072 bytes and converting
# it back to CSV must reproduce the same partitions.
set -eu

HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CSV=$HERE/esp32-4mb-ota.csv
OUT=${1:-$HERE/esp32-4mb-ota.bin}
EXPECTED_SIZE=3072

espflash partition-table --to-binary --output "$OUT" "$CSV"

SIZE=$(wc -c < "$OUT" | tr -d ' ')
if [ "$SIZE" -ne "$EXPECTED_SIZE" ]; then
    echo "partition table is $SIZE bytes, expected $EXPECTED_SIZE" >&2
    exit 1
fi

espflash partition-table --to-csv "$OUT" > "$OUT.csv"
python3 - "$CSV" "$OUT.csv" <<'PY'
import sys


def load(path):
    rows = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        cells = [cell.strip() for cell in line.split(",")]
        if cells[0].lower() == "name":
            continue
        name, kind, subtype, offset, size = cells[:5]
        rows.append((name, kind, subtype, int(offset, 0), int(size, 0)))
    return rows


source, roundtrip = load(sys.argv[1]), load(sys.argv[2])
if source != roundtrip:
    raise SystemExit(f"round trip mismatch:\n  {source}\n  {roundtrip}")
print("partition table round trip OK:")
for name, kind, subtype, offset, size in source:
    print(f"  {name:<9} {kind}/{subtype:<10} 0x{offset:06X} + 0x{size:06X}")
PY
rm -f "$OUT.csv"

printf 'Wrote %s (%s bytes)\n' "$OUT" "$SIZE"
