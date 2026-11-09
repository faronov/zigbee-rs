#!/bin/sh
# Regenerate the imgtool test vectors used by `src/tests.rs`.
#
#   IMGTOOL=/path/to/imgtool MCUBOOT_DIR=/path/to/mcuboot ./generate.sh
#
# TEST-ONLY-ec-p256.pem is a throwaway key committed only so the vectors can
# be reproduced. It must never sign anything installed on a device.
# TEST-ONLY-ec-p256-b.pem and TEST-ONLY-ed25519.pem are throwaway keys for
# the key-rotation and Ed25519 tests, under the same rule.
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
# Rotation and Ed25519 vectors (added later; regenerated only when missing so
# the vectors above stay byte-identical).
[ -f TEST-ONLY-ec-p256-b.pem ] || "$IMGTOOL" keygen -t ecdsa-p256 -k TEST-ONLY-ec-p256-b.pem
[ -f TEST-ONLY-ed25519.pem ] || "$IMGTOOL" keygen -t ed25519 -k TEST-ONLY-ed25519.pem
[ -f test-b-v2.bin ] || "$IMGTOOL" sign $COMMON --version 2.0.0 --key TEST-ONLY-ec-p256-b.pem payload.bin test-b-v2.bin
[ -f test-ed-v2.bin ] || "$IMGTOOL" sign $COMMON --version 2.0.0 --key TEST-ONLY-ed25519.pem payload.bin test-ed-v2.bin
"$PY" - TEST-ONLY-ec-p256-b.pem test-key-b.xy TEST-ONLY-ed25519.pem test-ed-key.pub <<'PYEOF'
import sys
from cryptography.hazmat.primitives import serialization
def load(path):
    return serialization.load_pem_private_key(open(path, "rb").read(), None).public_key()
numbers = load(sys.argv[1]).public_numbers()
open(sys.argv[2], "wb").write(numbers.x.to_bytes(32, "big") + numbers.y.to_bytes(32, "big"))
raw = load(sys.argv[3]).public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)
open(sys.argv[4], "wb").write(raw)
PYEOF
rm payload.bin
# Cortex-M vector for the Cortex-M loader shell (zrs-boot-cortex-m): a
# 0x200-byte header and a vector table for a primary slot at 0x800 —
# initial SP 0x20010000, reset handler 0xA41 (Thumb) — then a pattern.
if [ ! -f test-cm-v1.bin ]; then
    "$PY" -c 'import struct, sys; sys.stdout.buffer.write(struct.pack("<II", 0x20010000, 0xA41) + bytes((i * 5 + 1) & 0xFF for i in range(504)))' > cm-payload.bin
    "$IMGTOOL" sign --header-size 0x200 --pad-header --align 4 --slot-size 0x4000 \
        --version 1.0.0 --key TEST-ONLY-ec-p256.pem cm-payload.bin test-cm-v1.bin
    rm cm-payload.bin
fi
# Envelope vectors (app-side verification on ROM/SDK-booted platforms): the
# 32-byte MCUboot header directly precedes the native image, which is staged
# without it (see `src/envelope.rs`).
if [ ! -f test-env-v2.bin ] || [ ! -f test-env-v1-protected.bin ]; then
    "$PY" -c 'import sys; sys.stdout.buffer.write(bytes((i * 11 + 5) & 0xFF for i in range(2999)))' > env-payload.bin
    ENV="--header-size 0x20 --pad-header --align 4 --slot-size 0x4000"
    "$IMGTOOL" sign $ENV --version 2.0.0 --key TEST-ONLY-ec-p256.pem env-payload.bin test-env-v2.bin
    "$IMGTOOL" sign $ENV --version 1.2.3+4 --security-counter auto --key TEST-ONLY-ec-p256.pem env-payload.bin test-env-v1-protected.bin
    rm env-payload.bin
fi
