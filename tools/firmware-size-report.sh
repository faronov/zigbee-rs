#!/usr/bin/env sh
set -eu

if [ "$#" -ne 4 ]; then
    echo "usage: $0 <name> <binary> <budget-bytes> <output-json>" >&2
    exit 2
fi

name=$1
binary=$2
budget=$3
output=$4

case "$name" in
    *[!A-Za-z0-9._-]* | "")
        echo "invalid firmware name: $name" >&2
        exit 2
        ;;
esac

case "$budget" in
    *[!0-9]* | "")
        echo "invalid byte budget: $budget" >&2
        exit 2
        ;;
esac

if [ ! -f "$binary" ]; then
    echo "firmware binary not found: $binary" >&2
    exit 2
fi

bytes=$(wc -c < "$binary" | tr -d '[:space:]')
remaining=$((budget - bytes))
status=within_budget
if [ "$remaining" -lt 0 ]; then
    status=exceeded
fi

cat > "$output" <<EOF
{
  "name": "$name",
  "bytes": $bytes,
  "budget_bytes": $budget,
  "remaining_bytes": $remaining,
  "status": "$status"
}
EOF

printf '%s: %s / %s bytes (%s)\n' "$name" "$bytes" "$budget" "$status"
if [ "$status" = exceeded ]; then
    exit 1
fi
