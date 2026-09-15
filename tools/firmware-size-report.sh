#!/usr/bin/env sh
set -eu

if [ "$#" -ne 3 ]; then
    echo "usage: $0 <name> <binary> <output-json>" >&2
    exit 2
fi

name=$1
binary=$2
output=$3

case "$name" in
    *[!A-Za-z0-9._-]* | "")
        echo "invalid firmware name: $name" >&2
        exit 2
        ;;
esac

if [ ! -f "$binary" ]; then
    echo "firmware binary not found: $binary" >&2
    exit 2
fi

bytes=$(wc -c < "$binary")
bytes=$(printf '%s' "$bytes" | tr -d '[:space:]')
cat > "$output" <<EOF
{
  "name": "$name",
  "bytes": $bytes
}
EOF

printf '%s: %s bytes\n' "$name" "$bytes"
