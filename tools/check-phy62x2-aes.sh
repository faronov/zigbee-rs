#!/usr/bin/env bash
# Every PHY62x2 Zigbee image must run CCM*/AES-MMO on the AES-128 block
# (`zigbee_mac::phy6222::Phy6222Aes`) and link no software AES.
set -euo pipefail

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <elf>..." >&2
    exit 2
fi

NM=${NM:-$(find "$(rustc --print sysroot)" -name llvm-nm -print -quit)}
for ELF in "$@"; do
    SYMS="$("$NM" --demangle "$ELF")"
    if grep -Eq 'aes::soft|SoftwareAes128' <<<"$SYMS"; then
        echo "$(basename "$ELF") still links software AES:" >&2
        grep -E 'aes::soft|SoftwareAes128' <<<"$SYMS" | head -5 >&2
        exit 1
    fi
    if ! grep -Eq 'Phy6222Aes>?::encrypt_block' <<<"$SYMS"; then
        echo "$(basename "$ELF") is missing the PHY62x2 AES block backend" >&2
        exit 1
    fi
    echo "$(basename "$ELF"): PHY62x2 AES block backend, no software AES"
done
