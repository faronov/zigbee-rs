#!/bin/sh
# Regenerate the imgtool test vectors used by `src/tests.rs`.
#
#   IMGTOOL=/path/to/imgtool MCUBOOT_DIR=/path/to/mcuboot ./generate.sh
#
# TEST-ONLY-ec-p256.pem is a throwaway key committed only so the vectors can
# be reproduced. It must never sign anything installed on a device.
# root-ec-p256.pem is MCUboot's public sample key, which TI's SDK (and
# koptserg's flower bootloader) use; it is read from an MCUboot checkout and
# not committed here.
set -eu
cd "$(dirname "$0")"
IMGTOOL=${IMGTOOL:-imgtool}
MCUBOOT_DIR=${MCUBOOT_DIR:-/tmp/mcuboot}
PY=$(head -n1 "$(command -v "$IMGTOOL")" | sed 's/^#!//')

[ -f TEST-ONLY-ec-p256.pem ] || "$IMGTOOL" keygen -t ecdsa-p256 -k TEST-ONLY-ec-p256.pem

# 3000-byte deterministic payload.
"$PY" -c 'import sys; sys.stdout.buffer.write(bytes((i * 7 + 3) & 0xFF for i in range(3000)))' > payload.bin

COMMON="--header-size 0x100 --pad-header --align 4 --slot-size 0x4000"
"$IMGTOOL" sign $COMMON --version 1.0.0 --key TEST-ONLY-ec-p256.pem payload.bin test-v1.bin
"$IMGTOOL" sign $COMMON --version 2.0.0 --key TEST-ONLY-ec-p256.pem payload.bin test-v2.bin
"$IMGTOOL" sign $COMMON --version 2.0.0+7 --key TEST-ONLY-ec-p256.pem --pad payload.bin test-v2-pad.bin
"$IMGTOOL" sign $COMMON --version 1.2.3+4 --security-counter auto --key TEST-ONLY-ec-p256.pem payload.bin test-v1-protected.bin
"$IMGTOOL" sign $COMMON --version 2.0.0 --key "$MCUBOOT_DIR/root-ec-p256.pem" payload.bin ti-v2.bin

# Raw big-endian X || Y of each public key.
for pair in "TEST-ONLY-ec-p256.pem test-key.xy" "$MCUBOOT_DIR/root-ec-p256.pem ti-key.xy"; do
    set -- $pair
    "$PY" - "$1" "$2" <<'PYEOF'
import sys
from cryptography.hazmat.primitives import serialization
key = serialization.load_pem_private_key(open(sys.argv[1], "rb").read(), None)
numbers = key.public_key().public_numbers()
open(sys.argv[2], "wb").write(numbers.x.to_bytes(32, "big") + numbers.y.to_bytes(32, "big"))
PYEOF
done
rm payload.bin
