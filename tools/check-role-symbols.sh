#!/usr/bin/env bash
# Check that a linked Zigbee image contains parent/router code exactly when
# its role needs it.
#
#   tools/check-role-symbols.sh leaf   <elf> [<elf>...]  # ED / SED images
#   tools/check-role-symbols.sh parent <elf> [<elf>...]  # router images
#
# `leaf` fails if any parent-MAC, router-lifecycle or NWK parent-maintenance
# symbol is linked. `parent` fails if none is: it is the positive control
# that keeps the pattern in step with the code it is meant to find.
#
# NM defaults to llvm-nm from the active Rust sysroot (rustup component
# llvm-tools), then rust-nm, then llvm-nm on PATH.
set -euo pipefail

if [[ $# -lt 2 || ( "$1" != leaf && "$1" != parent ) ]]; then
  echo "usage: $0 leaf|parent <elf>..." >&2
  exit 2
fi
expect=$1
shift

if [[ -z "${NM:-}" ]]; then
  NM=$(find "$(rustc --print sysroot)" -name llvm-nm -print -quit 2>/dev/null || true)
  [[ -n "$NM" ]] || NM=$(command -v rust-nm || command -v llvm-nm || true)
fi
[[ -n "$NM" ]] || { echo "$0: no llvm-nm found (set NM)" >&2; exit 2; }

# Parent MAC (`soft_mac::parent`), router app lifecycle, and the NWK
# parent-maintenance entry points.
PARENT_SYMS='soft_mac::parent|ParentRouterApp|ParentCore|parent_beacon_response|handle_parent_command|handle_child_rejoin_request|queue_pending_child_update|process_pending_routing|send_link_status'

status=0
for elf in "$@"; do
  count=$("$NM" --demangle "$elf" | grep -cE "($PARENT_SYMS)" || true)
  name=$(basename "$elf")
  if [[ "$expect" == leaf && "$count" -ne 0 ]]; then
    echo "$name: links $count parent/router symbols, expected none:" >&2
    "$NM" --demangle "$elf" | grep -E "($PARENT_SYMS)" | head -20 >&2 || true
    status=1
  elif [[ "$expect" == parent && "$count" -eq 0 ]]; then
    echo "$name: links no parent/router symbols; the router image or the pattern is wrong" >&2
    status=1
  else
    echo "$name: $expect ok ($count parent/router symbols)"
  fi
done
exit $status
