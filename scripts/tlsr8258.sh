#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-}"
mode="${2:-sensor}"

usage() {
    cat >&2 <<'EOF'
usage: scripts/tlsr8258.sh <check|build|flash> [sensor|sensor-retention|sensor-retention-10s|router]
       scripts/tlsr8258.sh <dump|dump-activate> <address> [word-count]

Production firmware:
  sensor       polling end-device sensor with hardware AES
  sensor-retention       feature-gated 250 ms LOW32K proof
  sensor-retention-10s   feature-gated 10 s steady-state LOW32K proof
  router       always-on join/relay router with hardware AES

SWire memory dumps are served by the standalone radio harness script
(examples/telink-tlsr8258-radio/scripts/tlsr8258.sh), which also builds the
raw-radio, association and runtime-join bring-up images.
EOF
    exit 2
}

[[ -n "$action" ]] || usage

case "$action:$mode" in
    check:sensor|build:sensor|flash:sensor|\
    check:runtime-sensor|build:runtime-sensor|flash:runtime-sensor)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor
        ;;
    check:sensor-retention|build:sensor-retention|flash:sensor-retention)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor retention-proof
        ;;
    check:sensor-retention-10s|build:sensor-retention-10s|flash:sensor-retention-10s)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor retention-proof-10s
        ;;
    check:router|build:router|flash:router|\
    check:runtime-router|build:runtime-router|flash:runtime-router)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-router telink-tlsr8258-router
        ;;
    dump:*|dump-activate:*)
        exec "$ROOT_DIR/examples/telink-tlsr8258-radio/scripts/tlsr8258.sh" "$@"
        ;;
    *)
        usage
        ;;
esac
