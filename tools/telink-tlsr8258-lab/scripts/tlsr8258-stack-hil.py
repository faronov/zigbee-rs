#!/usr/bin/env python3
"""TLSR8258 stack high-water HIL harness (HW-04 / STACK-01).

Flow for `run`:

1. halt the CPU, read back product NV and the protected sectors (read-only);
2. program only the firmware image span and the requested product NV
   partitions (`--aps/--child/--security keep|erase|FILE`);
3. reset and halt at PC 0, paint `.rf_dma`, the unassigned gap, the SVC stack
   guard, the SVC stack and the IRQ stack over SWS (`__reset`/`_start` do not
   clear them; the firmware re-arms the guard first thing);
4. run for `--settle` seconds, then take `--samples` RAM dumps over SWS while
   the CPU runs (`sample` repeats this later without reset or repaint);
5. derive SVC and IRQ high-water separately from the paint, compare against
   the linker `_svc_stack_bottom` / `_irq_stack_bottom`, check every guard
   word, and look for stack data below the guard and in `.rf_dma`.

The paint high-water is the lowest word written. Untouched parts of a large
frame keep the paint, so it is a lower bound of the real SP excursion.

Every flash erase/program is computed with TlsrPgm's real erase span and
refused when it overlaps a protected range (default 0x76000..0x80000).
There is no full-chip write path.
"""

import argparse
import hashlib
import importlib.util
import json
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

SECTOR = 0x1000
FLASH_END = 0x80000
PROTECTED = ((0x76000, 0x78000), (0x78000, 0x80000))
PARTITIONS = {"aps": (0x70000, 0x72000), "child": (0x72000, 0x74000),
              "security": (0x74000, 0x76000)}
PRODUCT_NV = (0x70000, 0x76000)
PAINT_WORD = 0xA5A55A5A
GUARD_WORD = 0x5AC33CA5
RUNTIME_HEADROOM_BYTES = 2048
IRQ_REGS = 0x800640
PAINT = struct.pack("<I", PAINT_WORD)
RAM = (0x840000, 0x850000)


class ProtectedRangeError(RuntimeError):
    pass


def we_span(offset, size):
    """Sectors TlsrPgm `we` erases for a file of `size` bytes at `offset`."""
    start = offset & ~(SECTOR - 1)
    end = (offset + size + SECTOR - 1) & ~(SECTOR - 1)
    return start, end


def es_span(offset, size):
    """Sectors TlsrPgm `es` erases: ceil(size / 4K) from align_down(offset)."""
    start = offset & ~(SECTOR - 1)
    return start, start + ((size + SECTOR - 1) // SECTOR) * SECTOR


def check_span(span, protected=PROTECTED):
    start, end = span
    if start < 0 or end > FLASH_END or start >= end:
        raise ProtectedRangeError(f"erase span {start:#x}..{end:#x} is outside flash")
    for low, high in protected:
        if start < high and low < end:
            raise ProtectedRangeError(
                f"erase span {start:#x}..{end:#x} overlaps protected {low:#x}..{high:#x}")
    return span


def load_symbols(elf):
    report = Path(__file__).resolve().parents[2] / "tlsr8258-stack-report.py"
    spec = importlib.util.spec_from_file_location("tlsr8258_stack_report", report)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    symbols = module.Elf(elf).symbols
    names = ("_rf_dma_start_", "_rf_dma_end_", "_svc_stack_bottom", "_svc_stack_top",
             "_irq_stack_bottom", "_irq_stack_top", "_end_bss_")
    missing = [name for name in names if name not in symbols]
    if missing:
        raise SystemExit(f"{elf}: missing linker symbols {missing}")
    layout = {name: symbols[name] for name in names}
    for name in ("_svc_stack_guard_start", "_svc_stack_guard_end"):
        if name in symbols:
            layout[name] = symbols[name]
    return layout


def lowest_dirty(data, base, start, end):
    for address in range(start, end, 4):
        offset = address - base
        if data[offset:offset + 4] != PAINT:
            return address
    return None


def analyze(layout, svc_dump, svc_base, irq_dump, irq_base):
    """Separate SVC and IRQ high-water from painted dumps."""
    rf_start, rf_end = layout["_rf_dma_start_"], layout["_rf_dma_end_"]
    svc_bottom, svc_top = layout["_svc_stack_bottom"], layout["_svc_stack_top"]
    irq_bottom, irq_top = layout["_irq_stack_bottom"], layout["_irq_stack_top"]
    guard_start = layout.get("_svc_stack_guard_start", svc_bottom)
    guard_end = layout.get("_svc_stack_guard_end", svc_bottom)
    declared = svc_top - svc_bottom
    result = {"declared_svc_stack_bytes": declared,
              "declared_irq_stack_bytes": irq_top - irq_bottom,
              "physical_bytes_to_rf_dma": svc_top - rf_end,
              "guard_bytes": guard_end - guard_start}

    def word(address):
        return struct.unpack_from("<I", svc_dump, address - svc_base)[0]

    # The firmware arms the guard over the paint, so guard words are judged
    # against GUARD_WORD, never against the paint.
    guard = [word(a) for a in range(guard_start, guard_end, 4)]
    guard_damaged = [a for a, w in zip(range(guard_start, guard_end, 4), guard)
                     if w != GUARD_WORD]
    svc_dirty = lowest_dirty(svc_dump, svc_base, guard_end, svc_top)
    below_guard_dirty = lowest_dirty(svc_dump, svc_base, rf_end, guard_start)
    # RF DMA legitimately overwrites paint inside .rf_dma, so plain "dirty"
    # is not stack evidence there.  A word holding an address inside the
    # descending SVC stack range is: radio payloads do not carry 0x0084xxxx
    # stack pointers, while spilled frame pointers/return slots do.  Large
    # TC32 frames leave unused slots painted, so this is a lower bound.
    stack_pointers = [
        (a, w) for a in range(rf_start, rf_end, 4)
        for w in [word(a)] if rf_end <= w < svc_top]
    reached_rf_dma = bool(stack_pointers)
    candidates = [a for a in (svc_dirty, below_guard_dirty) if a is not None]
    candidates += [a for a, _ in stack_pointers[:1]]
    if guard and len(guard_damaged) < len(guard):
        candidates += guard_damaged[:1]
    lowest = min(candidates) if candidates else None
    high_water = svc_top - lowest if lowest is not None else 0
    guard_armed = len(guard_damaged) < len(guard) or not guard
    result.update({
        "svc_lowest_touched": f"{lowest:#x}" if lowest is not None else None,
        # Every stack write lands at or above SP, so SP reached at least this
        # low; untouched slots of the deepest frame may hide a lower SP.
        "svc_min_sp_at_most": f"{lowest:#x}" if lowest is not None else None,
        "svc_high_water_bytes": high_water,
        "svc_headroom_bytes": declared - high_water,
        "svc_runtime_headroom_ok": declared - high_water >= RUNTIME_HEADROOM_BYTES,
        "svc_high_water_is_lower_bound": reached_rf_dma,
        "svc_crossed_declared_bottom": lowest is not None and lowest < svc_bottom,
        "svc_bytes_below_declared_bottom": max(0, svc_bottom - lowest) if lowest else 0,
        "guard_armed": guard_armed,
        "guard_intact": guard_armed and not guard_damaged,
        "guard_damaged_words": [f"{a:#x}" for a in guard_damaged] if guard_armed else [],
        "below_guard_words_written": sum(
            svc_dump[a - svc_base:a - svc_base + 4] != PAINT for a in range(rf_end, guard_start, 4)),
        "rf_dma_reached_by_stack": reached_rf_dma,
        "rf_dma_intact": not reached_rf_dma,
        "rf_dma_stack_pointer_words": [f"{a:#x}={w:#x}" for a, w in stack_pointers],
    })
    irq_dirty = lowest_dirty(irq_dump, irq_base, irq_bottom, irq_top)
    result.update({
        "irq_lowest_touched": f"{irq_dirty:#x}" if irq_dirty is not None else None,
        "irq_high_water_bytes": irq_top - irq_dirty if irq_dirty is not None else 0,
        "irq_whole_stack_dirty": irq_dirty == irq_bottom,
    })
    result["svc_stack_safe"] = (not result["svc_crossed_declared_bottom"]
                                and result["below_guard_words_written"] == 0
                                and not reached_rf_dma
                                and (result["guard_intact"] or not guard))
    result["irq_stack_safe"] = not result["irq_whole_stack_dirty"]
    return result


class Programmer:
    def __init__(self, tlsrpgm, port, dry_run, log):
        self.tlsrpgm, self.port, self.dry_run, self.log = tlsrpgm, port, dry_run, log

    def call(self, *args, halt=False, reset=False):
        command = ["python3", self.tlsrpgm, "-p", self.port]
        if reset:
            command += ["-t", "500", "-a", "200"]
        if halt:
            command += ["-s"]
        command += [str(a) for a in args]
        self.log.append(" ".join(command))
        print("+", " ".join(command), flush=True)
        if self.dry_run:
            return
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)

    def read_flash(self, start, end, path):
        self.call("rf", hex(start), hex(end - start), path, halt=True, reset=True)

    def write_flash(self, offset, path):
        size = Path(path).stat().st_size
        check_span(we_span(offset, size))
        self.call("we", hex(offset), path, halt=True, reset=True)

    def erase_flash(self, offset, size):
        check_span(es_span(offset, size))
        self.call("es", hex(offset), hex(size), halt=True, reset=True)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def partition_plan(args):
    """Validate every requested NV action before touching the target."""
    plan = {}
    for name, (start, end) in PARTITIONS.items():
        action = getattr(args, name)
        if action in ("keep", "erase"):
            if action == "erase":
                check_span(es_span(start, end - start))
            plan[name] = action
            continue
        path = Path(action)
        if path.stat().st_size > end - start:
            raise SystemExit(f"{name} image larger than its {end - start:#x} B partition")
        check_span(we_span(start, path.stat().st_size))
        plan[name] = path
    return plan


def dump_and_analyze(pgm, layout, out, tag):
    svc_base = layout["_rf_dma_start_"]
    svc_size = layout["_svc_stack_top"] - svc_base
    irq_base = layout["_irq_stack_bottom"]
    irq_size = layout["_irq_stack_top"] - irq_base
    svc_path, irq_path = out / f"ram-svc{tag}.bin", out / f"ram-irq{tag}.bin"
    regs_path = out / f"irq-regs{tag}.bin"
    pgm.call("rs", hex(svc_base), hex(svc_size), svc_path)
    pgm.call("rs", hex(irq_base), hex(irq_size), irq_path)
    # Live REG_IRQ_MASK (0x640, u32) / REG_IRQ_EN (0x643, u8) distinguish
    # "IRQ never enabled" from "enabled but no source fired".
    pgm.call("rs", hex(IRQ_REGS), "0x8", regs_path)
    if pgm.dry_run:
        return {}
    result = analyze(layout, svc_path.read_bytes(), svc_base, irq_path.read_bytes(), irq_base)
    regs = regs_path.read_bytes()
    result["live_irq_mask"] = f"{int.from_bytes(regs[0:4], 'little'):#010x}"
    result["live_irq_global_enable"] = regs[3]
    result["sampled_at"] = time.strftime("%H:%M:%S")
    return result


def summary(result):
    keys = ("sampled_at", "svc_high_water_bytes", "svc_headroom_bytes", "svc_min_sp_at_most",
            "guard_intact", "below_guard_words_written", "rf_dma_intact",
            "irq_high_water_bytes", "live_irq_mask", "svc_stack_safe")
    return {k: result.get(k) for k in keys}


def partition_report(out, plan, before):
    after = (out / "product-nv-after.bin").read_bytes()
    report = {}
    for name, (start, end) in PARTITIONS.items():
        lo, hi = start - PRODUCT_NV[0], end - PRODUCT_NV[0]
        action = plan.get(name, "keep")
        if action == "keep":
            expected = before[lo:hi]
        elif action == "erase":
            expected = b"\xff" * (end - start)
        else:
            expected = action.read_bytes().ljust(end - start, b"\xff")
        report[name] = {"action": str(action),
                        "changed_by_firmware": after[lo:hi] != expected,
                        "sha256_after": hashlib.sha256(after[lo:hi]).hexdigest()}
    return report


def read_after(pgm, out):
    # Resets the target: TlsrPgm halts the CPU for flash access.
    pgm.read_flash(*PRODUCT_NV, out / "product-nv-after.bin")
    pgm.read_flash(PROTECTED[0][0], PROTECTED[-1][1], out / "protected-after.bin")


def flash_report(out, plan):
    before = (out / "product-nv-before.bin").read_bytes()
    report = {"partitions": partition_report(out, plan, before),
              "product_nv_before_sha256": sha256(out / "product-nv-before.bin"),
              "protected_sha256_before": sha256(out / "protected-before.bin"),
              "protected_sha256_after": sha256(out / "protected-after.bin")}
    report["protected_unchanged"] = (
        report["protected_sha256_before"] == report["protected_sha256_after"])
    return report


def command_finish(args):
    """Read NV/protected flash after a `run --leave-running` session."""
    out = Path(args.out)
    result = json.loads((out / "result.json").read_text())
    pgm = Programmer(args.tlsrpgm, args.port, args.dry_run, [])
    read_after(pgm, out)
    if args.dry_run:
        return 0
    plan = {k: (v if v in ("keep", "erase") else Path(v)) for k, v in result["nv_plan"].items()}
    result.update(flash_report(out, plan))
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("partitions", "protected_unchanged")}, indent=2))
    if args.resume:
        pgm.call("-g", "i")
    return 0 if result["protected_unchanged"] else 2


def command_run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    layout = load_symbols(args.elf)
    image = Path(args.bin)
    check_span(we_span(0, image.stat().st_size))
    plan = partition_plan(args)
    log = []
    pgm = Programmer(args.tlsrpgm, args.port, args.dry_run, log)

    pgm.read_flash(PROTECTED[0][0], PROTECTED[-1][1], out / "protected-before.bin")
    pgm.read_flash(*PRODUCT_NV, out / "product-nv-before.bin")
    if not args.keep_image:
        pgm.write_flash(0, image)
    for name, action in plan.items():
        start, end = PARTITIONS[name]
        if action == "erase":
            pgm.erase_flash(start, end - start)
        elif action != "keep":
            pgm.write_flash(start, action)

    svc_base = layout["_rf_dma_start_"]
    svc_size = layout["_svc_stack_top"] - svc_base
    irq_base = layout["_irq_stack_bottom"]
    irq_size = layout["_irq_stack_top"] - irq_base
    with tempfile.TemporaryDirectory() as tmp:
        svc_paint, irq_paint = Path(tmp) / "svc-paint.bin", Path(tmp) / "irq-paint.bin"
        svc_paint.write_bytes(PAINT * (svc_size // 4))
        irq_paint.write_bytes(PAINT * (irq_size // 4))
        pgm.call("i", halt=True, reset=True)
        pgm.call("ws", hex(svc_base), svc_paint)
        pgm.call("ws", hex(irq_base), irq_paint)
        pgm.call("-g", "i")
    if not args.dry_run:
        time.sleep(args.settle)
    samples = []
    for index in range(args.samples):
        if index and not args.dry_run:
            time.sleep(args.interval)
        sample = dump_and_analyze(pgm, layout, out, "" if index + 1 == args.samples else f"-{index}")
        samples.append(sample)
        if sample:
            print(json.dumps(summary(sample)), flush=True)
    if not args.leave_running:
        read_after(pgm, out)

    result = {"scenario": args.scenario, "elf": str(args.elf), "image_sha256": sha256(image),
              "layout": {k: f"{v:#x}" for k, v in layout.items()},
              "nv_plan": {k: str(v) for k, v in plan.items()}, "commands": log}
    if not args.dry_run:
        result.update(samples[-1])
        result["samples"] = [summary(sample) for sample in samples]
        if not args.leave_running:
            result.update(flash_report(out, plan))
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in ("commands", "samples")}, indent=2))
    return 0 if args.dry_run or (result["svc_stack_safe"] and result["irq_stack_safe"]) else 2


def command_sample(args):
    """Dump and analyze again without reset or repaint (cumulative since `run`)."""
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    layout = load_symbols(args.elf)
    pgm = Programmer(args.tlsrpgm, args.port, args.dry_run, [])
    result = dump_and_analyze(pgm, layout, out, f"-{args.tag}")
    result["scenario"] = args.tag
    (out / f"sample-{args.tag}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(summary(result)))
    return 0 if args.dry_run or (result["svc_stack_safe"] and result["irq_stack_safe"]) else 2


def command_restore(args):
    """Write file[start:end] back to flash[start:end], never a full chip."""
    data = Path(args.file).read_bytes()
    start, end = int(args.start, 0), int(args.end, 0)
    if start % SECTOR or end % SECTOR or end > len(data):
        raise SystemExit("restore range must be sector aligned and inside the file")
    check_span(we_span(start, end - start))
    log = []
    pgm = Programmer(args.tlsrpgm, args.port, args.dry_run, log)
    with tempfile.TemporaryDirectory() as tmp:
        chunk = Path(tmp) / "restore.bin"
        chunk.write_bytes(data[start:end])
        pgm.write_flash(start, chunk)
        readback = Path(tmp) / "readback.bin"
        pgm.read_flash(start, end, readback)
        if not args.dry_run and readback.read_bytes() != data[start:end]:
            raise SystemExit(f"readback mismatch in {start:#x}..{end:#x}")
    print(f"restored {start:#x}..{end:#x} from {args.file}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/cu.usbserial-1430")
    parser.add_argument("--tlsrpgm", default=str(Path.home() / "TLSRPGM" / "TlsrPgm.py"))
    parser.add_argument("--dry-run", action="store_true")
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--elf", required=True)
    run.add_argument("--bin", required=True)
    run.add_argument("--scenario", required=True, help="free-form label stored in result.json")
    for name in PARTITIONS:
        run.add_argument(f"--{name}", default="keep",
                         help=f"{name} NV partition: keep | erase | image file")
    run.add_argument("--keep-image", action="store_true", help="do not reprogram the image span")
    run.add_argument("--settle", type=float, default=20.0)
    run.add_argument("--samples", type=int, default=1)
    run.add_argument("--interval", type=float, default=10.0)
    run.add_argument("--leave-running", action="store_true",
                     help="skip the resetting NV/protected readback; use `finish` later")
    run.add_argument("--out", required=True)
    finish = commands.add_parser("finish")
    finish.add_argument("--out", required=True)
    finish.add_argument("--resume", action="store_true", help="restart the firmware afterwards")
    sample = commands.add_parser("sample")
    sample.add_argument("--elf", required=True)
    sample.add_argument("--tag", required=True)
    sample.add_argument("--out", required=True)
    restore = commands.add_parser("restore")
    restore.add_argument("--file", required=True)
    restore.add_argument("--start", required=True)
    restore.add_argument("--end", required=True)
    args = parser.parse_args()
    handlers = {"run": command_run, "sample": command_sample, "finish": command_finish, "restore": command_restore}
    try:
        return handlers[args.command](args)
    except ProtectedRangeError as error:
        print(f"REFUSED: {error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
