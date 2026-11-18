#!/usr/bin/env bash
# The ESP32-C5 802.15.4 driver (vendor/esp-radio-0.18/src/ieee802154) is the
# C6/H2 driver (vendor/esp-radio/src/ieee802154) adapted to esp-radio 0.18's
# internals (vendor/esp-radio-0.18/LOCAL-PATCHES.md). Fail when the two drift
# apart: the shared files must be identical and mod.rs/raw.rs may differ only
# by the recorded port diff. After changing the C6/H2 driver, port the change
# and regenerate the record with `--update`.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OLD="$ROOT/vendor/esp-radio/src/ieee802154"
NEW="$ROOT/vendor/esp-radio-0.18/src/ieee802154"
RECORD="$ROOT/vendor/esp-radio-0.18/ieee802154-port.diff"

for file in hal.rs frame.rs fcf.rs suspend.rs; do
  cmp -s "$OLD/$file" "$NEW/$file" ||
    { echo "vendor/esp-radio-0.18/src/ieee802154/$file differs from vendor/esp-radio" >&2; exit 1; }
done

port_diff() {
  for file in mod.rs raw.rs; do
    diff -u --label "a/$file" --label "b/$file" "$OLD/$file" "$NEW/$file" || true
  done
}

if [[ "${1:-}" == "--update" ]]; then
  port_diff >"$RECORD"
  echo "updated $RECORD"
  exit 0
fi
if ! diff -u "$RECORD" <(port_diff); then
  echo "the C5 driver port diff changed; see vendor/esp-radio-0.18/LOCAL-PATCHES.md" >&2
  exit 1
fi
echo "esp-radio C5 driver in sync with the C6/H2 driver"
