#!/usr/bin/env python3
"""Build, wrap and verify a Zigbee OTA image for an ESP32-C6/H2 sensor.

    tools/create-esp32-ota.py <firmware-version> [output-directory] --chip <chip>
        [--variant thermometer] [--sign-key KEY.pem]

The pipeline is deliberately end-to-end so a version can never be stamped onto
an image that does not contain it:

1. `cargo build --release` with `ESP32_OTA_VERSION` set to the requested
   version, which is what the firmware reports in `QueryNextImageRequest` and
   in `Basic::SWBuildID`.
2. `espflash save-image` turns the ELF into an ESP application image — the
   exact bytes the second stage bootloader expects in `ota_0`/`ota_1`. An ELF
   or a merged flash image would be rejected by the device. Shared preflight
   checks segments, descriptor, mapping, checksum and hash for both OTA slots.
3. The application image becomes the `UpgradeImage` sub-element of a Zigbee OTA
   container.
4. The container is parsed back and compared byte for byte with the image, and
   its size is checked against the 0x1F0000-byte application slot.
5. A `zigpy_local` index is regenerated from every `.ota` file in the output
   directory, with the sha3-256 checksum ZHA verifies after downloading.

`--sign-key` builds with the `signed-ota` feature and wraps the application
image in an `imgtool sign --header-size 0x20 --pad-header` envelope, which is
what that firmware accepts (`products/esp32-zigbee-devkit/src/signed_ota.rs`).
The envelope version is the file version `0xMMmmrrrr` read as `MM.mm.rrrr`.
The signature is checked with `imgtool verify`, and the key must be one the
firmware trusts (`products/esp32-zigbee-devkit/boot/app-verify.conf`, or its
`ESP32_DEVKIT_OTA_KEY*_XY` overrides).

`--variant thermometer` packages the ESP32-H2 `esp32h2-thermometer` binary;
its model and OTA image type come from `examples/esp32h2-sensor/thermometer.toml`.

The container steps use only the Python standard library; signing needs
imgtool (`pip install imgtool==2.4.0`).
Preflight does not authenticate firmware or prove bootability. Actual silicon
and eFuse revision compatibility is checked by the receiving device.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import struct
import subprocess
import sys
from dataclasses import dataclass

# Zigbee OTA container -------------------------------------------------------
OTA_MAGIC = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_BASE_LEN = 56
FIELD_CONTROL_HARDWARE_VERSIONS = 0x0004
TAG_UPGRADE_IMAGE = 0x0000
SUB_ELEMENT_HEADER_LEN = 6
ZIGBEE_PRO_STACK_VERSION = 0x0002

# Must match products/esp32-zigbee-devkit/src/lib.rs and the product profile.
MANUFACTURER_CODE = 0x1234
HARDWARE_VERSION = 1
MANUFACTURER_NAME = "Zigbee-RS"

# Must match products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv.
OTA_SLOT_SIZE = 0x001F_0000

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_TARGET = "riscv32imac-unknown-none-elf"

sys.path.insert(0, str(REPO_ROOT / "products/esp32-zigbee-devkit/partitions"))
sys.path.insert(0, str(REPO_ROOT / "tools"))
from esp_app_image import CHIP_IDS, validate_application
from bl702_ota import IMAGE_MAGIC, ToolError, find_imgtool, imgtool_public_xy, parse_mcuboot
from zrs_boot_ota import parse_xy, trusted_keys

# The product's app-side trust anchor for `signed-ota` builds.
APP_VERIFY_CONF = REPO_ROOT / "products/esp32-zigbee-devkit/boot/app-verify.conf"
# `imgtool sign --header-size` the firmware's envelope parser expects.
ENVELOPE_HEADER_SIZE = 0x20


@dataclass(frozen=True)
class ChipConfig:
    chip_id: int
    image_type: int
    model_name: str
    header_string: str
    example_name: str
    bin_name: str = ""
    features: str = ""

    @property
    def example_dir(self) -> pathlib.Path:
        return REPO_ROOT / "examples" / self.example_name

    @property
    def binary(self) -> str:
        return self.bin_name or self.example_name


def read_flat_toml(path: pathlib.Path) -> dict[str, str]:
    """`key = value` lines of a flat TOML file (Python 3.9 has no tomllib)."""
    values = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        key, sep, value = line.partition("=")
        if sep:
            values[key.strip()] = value.strip().strip('"')
    return values


def thermometer_config() -> ChipConfig:
    """The ESP32-H2 thermometer; identity from its `thermometer.toml`."""
    example = "esp32h2-sensor"
    values = read_flat_toml(REPO_ROOT / "examples" / example / "thermometer.toml")
    try:
        model = values["model"]
        image_type = int(values["ota_image_type"], 0)
    except (KeyError, ValueError) as error:
        raise SystemExit(f"thermometer.toml: bad or missing model/ota_image_type: {error}")
    return ChipConfig(
        chip_id=CHIP_IDS["esp32h2"],
        image_type=image_type,
        model_name=model,
        header_string="zigbee-rs ESP32-H2 thermometer",
        example_name=example,
        bin_name="esp32h2-thermometer",
        features="thermometer,light-sleep",
    )


CHIPS = {
    "esp32c6": ChipConfig(
        chip_id=CHIP_IDS["esp32c6"],
        image_type=0x0001,
        model_name="ESP32-C6-Sensor",
        header_string="zigbee-rs ESP32-C6 sensor",
        example_name="esp32c6-sensor",
    ),
    "esp32h2": ChipConfig(
        chip_id=CHIP_IDS["esp32h2"],
        image_type=0x0002,
        model_name="ESP32-H2-Sensor",
        header_string="zigbee-rs ESP32-H2 sensor",
        example_name="esp32h2-sensor",
    ),
}
VARIANTS = {"sensor": None, "thermometer": ("esp32h2", thermometer_config)}


def known_configs() -> dict[int, ChipConfig]:
    configs = {config.image_type: config for config in CHIPS.values()}
    thermometer = thermometer_config()
    configs[thermometer.image_type] = thermometer
    return configs


def select_config(chip: str, variant: str) -> ChipConfig:
    if VARIANTS[variant] is None:
        return CHIPS[chip]
    variant_chip, load = VARIANTS[variant]
    if chip != variant_chip:
        raise SystemExit(f"--variant {variant} is {variant_chip} only")
    return load()


def parse_version(raw: str) -> int:
    value = int(raw, 0)
    if not 0 <= value < 0xFFFFFFFF:
        raise SystemExit("firmware version must be in 0..0xFFFFFFFE")
    return value


def run(command: list[str], *, cwd: pathlib.Path, env: dict | None = None) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def build_elf(
    version: int, example_dir: pathlib.Path, toolchain: str, features: str = "",
    bin_name: str = "",
) -> pathlib.Path:
    env = dict(os.environ, ESP32_OTA_VERSION=str(version))
    command = ["cargo"]
    if toolchain:
        command.append(f"+{toolchain}")
    command += ["build", "--release", "--locked", "-Z", "build-std=core,alloc"]
    if bin_name:
        command += ["--bin", bin_name]
    if features:
        command += ["--features", features]
    run(command, cwd=example_dir, env=env)
    elf = example_dir / "target" / DEFAULT_TARGET / "release" / (bin_name or example_dir.name)
    if not elf.is_file():
        raise SystemExit(f"expected ELF at {elf}")
    return elf


def save_image(
    elf: pathlib.Path, chip: str, example_dir: pathlib.Path, output: pathlib.Path
) -> bytes:
    run(
        ["espflash", "save-image", "--chip", chip, str(elf), str(output)],
        cwd=example_dir,
    )
    return output.read_bytes()


def check_esp_image(image: bytes, config: ChipConfig) -> None:
    """Check structure/integrity at either OTA destination, without guessing revisions."""
    try:
        for offset in (0x00010000, 0x00200000):
            validate_application(image, config.chip_id, offset, OTA_SLOT_SIZE)
    except ValueError as error:
        raise SystemExit(f"{config.example_name}: {error}") from error


def envelope_version(file_version: int) -> str:
    """imgtool `--version` for file version `0xMMmmrrrr` (`MM.mm.rrrr`), as
    `signed_ota::image_version` maps it."""
    return (f"{file_version >> 24}.{(file_version >> 16) & 0xFF}."
            f"{file_version & 0xFFFF}+0")


def firmware_trusted_keys() -> list[bytes]:
    """The P-256 keys a `signed-ota` build trusts, resolved as its build
    script resolves them."""
    values = {}
    for line in APP_VERIFY_CONF.read_text().splitlines():
        key, sep, value = line.split("#", 1)[0].partition("=")
        if sep:
            values[key.strip()] = value.strip()

    class AppVerify:
        env_prefix = values["env-prefix"]
        key_file = APP_VERIFY_CONF.parent / values["key-file"] if "key-file" in values else None

    return trusted_keys(AppVerify)


def envelope_image(payload: bytes) -> bytes | None:
    """The ESP image inside a signed envelope, or None for a plain image."""
    if len(payload) < 4 or struct.unpack_from("<I", payload)[0] != IMAGE_MAGIC:
        return None
    try:
        parsed = parse_mcuboot(payload)
    except ToolError as error:
        raise SystemExit(f"bad signed envelope: {error}") from error
    if parsed.hdr_size != ENVELOPE_HEADER_SIZE:
        raise SystemExit(f"envelope header is 0x{parsed.hdr_size:X} bytes, "
                         f"expected 0x{ENVELOPE_HEADER_SIZE:X}")
    if not parsed.sha256_ok or not parsed.has_signature:
        raise SystemExit("envelope has a bad digest or no signature")
    if parsed.end != len(payload):
        raise SystemExit("trailing bytes after the envelope's TLVs")
    return payload[parsed.hdr_size:parsed.hdr_size + parsed.img_size]


def sign_image(image_path: pathlib.Path, version: int, key: pathlib.Path,
               imgtool: str, output: pathlib.Path, *, untrusted_key: bool) -> bytes:
    xy = imgtool_public_xy(imgtool, key)
    if not untrusted_key and xy not in firmware_trusted_keys():
        raise SystemExit(f"{key} is not a key the firmware trusts "
                         f"({APP_VERIFY_CONF.relative_to(REPO_ROOT)})")
    run([imgtool, "sign", "--header-size", hex(ENVELOPE_HEADER_SIZE), "--pad-header",
         "--align", "4", "--slot-size", hex(OTA_SLOT_SIZE),
         "--version", envelope_version(version), "--key", str(key),
         str(image_path), str(output)], cwd=REPO_ROOT)
    result = subprocess.run([imgtool, "verify", "--key", str(key), str(output)],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f"imgtool verify failed: {result.stdout}{result.stderr}".strip())
    envelope = output.read_bytes()
    parsed = parse_mcuboot(envelope)
    major, minor, revision, _build = parsed.version
    if (major << 24 | minor << 16 | revision) != version:
        raise SystemExit("envelope version does not match the file version")
    if envelope_image(envelope) != image_path.read_bytes():
        raise SystemExit("envelope does not carry the application image unchanged")
    return envelope


def build_container(image: bytes, version: int, config: ChipConfig) -> bytes:
    header_length = OTA_HEADER_BASE_LEN + 4  # + min/max hardware version
    total = header_length + SUB_ELEMENT_HEADER_LEN + len(image)
    header_string = config.header_string.encode("utf-8")[:32].ljust(32, b"\x00")

    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_MAGIC,
        OTA_HEADER_VERSION,
        header_length,
        FIELD_CONTROL_HARDWARE_VERSIONS,
        MANUFACTURER_CODE,
        config.image_type,
        version,
        ZIGBEE_PRO_STACK_VERSION,
        header_string,
        total,
    )
    header += struct.pack("<HH", HARDWARE_VERSION, HARDWARE_VERSION)
    assert len(header) == header_length, len(header)

    sub_element = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(image)) + image
    return header + sub_element


def parse_container(blob: bytes) -> dict:
    """Independent parser used to verify what was just written."""
    if len(blob) < OTA_HEADER_BASE_LEN:
        raise SystemExit("OTA container is shorter than its header")
    (
        magic,
        header_version,
        header_length,
        field_control,
        manufacturer_code,
        image_type,
        file_version,
        stack_version,
        header_string,
        total_image_size,
    ) = struct.unpack_from("<IHHHHHIH32sI", blob, 0)

    if magic != OTA_MAGIC:
        raise SystemExit(f"bad OTA magic 0x{magic:08X}")
    if header_version != OTA_HEADER_VERSION:
        raise SystemExit(f"unsupported OTA header version 0x{header_version:04X}")
    if total_image_size != len(blob):
        raise SystemExit(
            f"header says {total_image_size} bytes, file is {len(blob)} bytes"
        )

    offset = OTA_HEADER_BASE_LEN
    minimum_hardware = maximum_hardware = None
    if field_control & 0x0001:
        offset += 1
    if field_control & 0x0002:
        offset += 8
    if field_control & FIELD_CONTROL_HARDWARE_VERSIONS:
        minimum_hardware, maximum_hardware = struct.unpack_from("<HH", blob, offset)
        offset += 4
    if offset != header_length:
        raise SystemExit(f"header length {header_length} != parsed {offset}")

    payload = None
    while offset + SUB_ELEMENT_HEADER_LEN <= len(blob):
        tag, length = struct.unpack_from("<HI", blob, offset)
        offset += SUB_ELEMENT_HEADER_LEN
        if offset + length > len(blob):
            raise SystemExit(f"sub-element 0x{tag:04X} runs past the end of the file")
        if tag == TAG_UPGRADE_IMAGE:
            payload = blob[offset : offset + length]
        offset += length
    if offset != len(blob):
        raise SystemExit("trailing bytes after the last sub-element")
    if payload is None:
        raise SystemExit("no UpgradeImage sub-element")

    return {
        "manufacturer_code": manufacturer_code,
        "image_type": image_type,
        "file_version": file_version,
        "stack_version": stack_version,
        "header_string": header_string.rstrip(b"\x00").decode("utf-8"),
        "minimum_hardware_version": minimum_hardware,
        "maximum_hardware_version": maximum_hardware,
        "payload": payload,
    }


def verify_container(
    blob: bytes, image: bytes, version: int, config: ChipConfig
) -> None:
    parsed = parse_container(blob)
    if parsed["payload"] != image:
        raise SystemExit("UpgradeImage payload differs from the application image")
    if parsed["file_version"] != version:
        raise SystemExit("container file version does not match the requested version")
    if parsed["manufacturer_code"] != MANUFACTURER_CODE:
        raise SystemExit("container manufacturer code mismatch")
    if parsed["image_type"] != config.image_type:
        raise SystemExit("container image type mismatch")
    expected_size = 60 + SUB_ELEMENT_HEADER_LEN + len(image)
    if len(blob) != expected_size:
        raise SystemExit(f"container is {len(blob)} bytes, expected {expected_size}")
    # Recheck application structure and integrity after the round trip.
    check_payload(parsed["payload"], config)


def check_payload(payload: bytes, config: ChipConfig) -> None:
    """Check a plain ESP image, or the one inside a signed envelope."""
    inner = envelope_image(payload)
    check_esp_image(payload if inner is None else inner, config)


def write_index(output_dir: pathlib.Path) -> pathlib.Path:
    firmwares = []
    for path in sorted(output_dir.glob("*.ota")):
        blob = path.read_bytes()
        parsed = parse_container(blob)
        if parsed["manufacturer_code"] != MANUFACTURER_CODE:
            raise SystemExit(
                f"{path} uses unknown manufacturer 0x"
                f"{parsed['manufacturer_code']:04X}"
            )
        config = known_configs().get(parsed["image_type"])
        if config is None:
            raise SystemExit(
                f"{path} uses unknown image type 0x{parsed['image_type']:04X}"
            )
        check_payload(parsed["payload"], config)
        firmwares.append(
            {
                "path": path.name,
                "file_version": parsed["file_version"],
                "file_size": len(blob),
                "image_type": parsed["image_type"],
                "manufacturer_id": parsed["manufacturer_code"],
                "manufacturer_names": [MANUFACTURER_NAME],
                "model_names": [config.model_name],
                "checksum": "sha3-256:" + hashlib.sha3_256(blob).hexdigest(),
                "min_hardware_version": parsed["minimum_hardware_version"],
                "max_hardware_version": parsed["maximum_hardware_version"],
                "changelog": (
                    f"zigbee-rs {config.model_name} v{parsed['file_version']}"
                ),
            }
        )
    index = output_dir / "index.json"
    index.write_text(json.dumps({"firmwares": firmwares}, indent=2) + "\n")
    return index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("version", help="OTA file version (decimal or 0x-prefixed)")
    parser.add_argument(
        "output_dir",
        nargs="?",
        default="target/ota",
        help="output directory (default: target/ota)",
    )
    parser.add_argument("--chip", default="esp32c6", choices=sorted(CHIPS))
    parser.add_argument("--variant", default="sensor", choices=sorted(VARIANTS),
                        help="image to package (thermometer: ESP32-H2 only)")
    parser.add_argument("--sign-key", help="P-256 PEM: build `signed-ota` and sign")
    parser.add_argument("--imgtool", help="imgtool executable (default: on PATH)")
    parser.add_argument(
        "--test-only-untrusted-key", action="store_true",
        help="TEST-ONLY: allow a signing key the firmware does not trust "
             "(builds an image the device must refuse)",
    )
    parser.add_argument(
        "--toolchain", default="nightly", help="cargo toolchain (default: nightly)"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--elf", help="use an existing ELF instead of building one (skips cargo)"
    )
    source.add_argument(
        "--features", choices=["light-sleep"], default="",
        help="build the experimental retained-sleep variant",
    )
    args = parser.parse_args()

    version = parse_version(args.version)
    config = select_config(args.chip, args.variant)
    example_dir = config.example_dir
    if args.test_only_untrusted_key and not args.sign_key:
        parser.error("--test-only-untrusted-key needs --sign-key")
    features = ",".join(f for f in (config.features, args.features,
                                    "signed-ota" if args.sign_key else "") if f)
    output_dir = pathlib.Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = example_dir / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    elf = (
        pathlib.Path(args.elf).resolve()
        if args.elf
        else build_elf(version, example_dir, args.toolchain, features, config.bin_name)
    )

    signed = "-signed" if args.sign_key else ""
    base = output_dir / f"{config.binary}-v{version}{signed}"
    app_bin = base.with_suffix(".app.bin")
    ota_file = base.with_suffix(".ota")

    image = save_image(elf, args.chip, example_dir, app_bin)
    check_esp_image(image, config)
    payload = image
    if args.sign_key:
        try:
            payload = sign_image(
                app_bin, version, pathlib.Path(args.sign_key).resolve(),
                find_imgtool(args.imgtool), base.with_suffix(".signed.bin"),
                untrusted_key=args.test_only_untrusted_key,
            )
        except ToolError as error:
            raise SystemExit(str(error)) from error

    container = build_container(payload, version, config)
    ota_file.write_bytes(container)
    verify_container(ota_file.read_bytes(), payload, version, config)

    index = write_index(output_dir)

    print()
    print(f"application image : {app_bin} ({len(image)} bytes)")
    if args.sign_key:
        print(f"signed envelope   : {base.with_suffix('.signed.bin')} "
              f"({len(payload)} bytes, version {envelope_version(version)})")
    print(f"OTA container     : {ota_file} ({len(container)} bytes)")
    print(f"  manufacturer    : 0x{MANUFACTURER_CODE:04X}")
    print(f"  image type      : 0x{config.image_type:04X}")
    print(f"  file version    : {version} (0x{version:08X})")
    print(f"  hardware        : {HARDWARE_VERSION}..{HARDWARE_VERSION}")
    print(f"  slot usage      : {len(image)}/{OTA_SLOT_SIZE} bytes "
          f"({100 * len(image) / OTA_SLOT_SIZE:.1f}%)")
    print(f"  sha3-256        : {hashlib.sha3_256(container).hexdigest()}")
    print(f"zigpy_local index : {index}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
