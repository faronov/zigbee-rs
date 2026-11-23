#!/usr/bin/env python3
"""Web flasher content from tagged GitHub releases.

The flasher (https://faronov.github.io/zigbee-rs/flasher/) serves firmware
only from GitHub releases that carry a `release-manifest.json` (written by
`tools/ncp-release/esp_release.py` here and by zigbee-rs-hub's release
workflow). Release downloads have no CORS headers, so the browser cannot
fetch them: the Pages workflow copies the images onto the site.

    flasher_site.py fetch   [--repo OWNER/NAME:TAG_PREFIX ...] [--keep N] --out DIR
    flasher_site.py check   DIR
    flasher_site.py resolve [--site URL] [--repo ...] [--keep N] [--force] [--revision SHA]
    flasher_site.py live    [--site URL]

Defaults: faronov/zigbee-rs:ncp-v and faronov/zigbee-rs-hub:hub-v, the newest
5 releases of each, site https://faronov.github.io/zigbee-rs/.

`fetch` downloads the newest `--keep` releases per repository, verifies every
image against the release manifest (size and SHA-256), and writes
`DIR/releases.json` plus `DIR/releases/<tag>/` with the images and one ESP
Web Tools manifest per board and mode (`<board>.update.json` keeps the data
partitions, `<board>.erase.json` erases the chip first). `resolve` decides
whether the site must be rebuilt (the release set changed) and on which base
(the revision and CI run of the live site); `live` prints the live base.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.request

SCHEMA = 1
API = "https://api.github.com"
SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._+-]*$")
CHIP_FAMILIES = {"ESP32", "ESP32-C2", "ESP32-C3", "ESP32-C5", "ESP32-C6", "ESP32-H2", "ESP32-S2", "ESP32-S3"}
SENSOR_JOBS = ("esp32c6", "esp32h2", "bl702")
# Repositories and tag prefixes the flasher serves, and how many releases of each.
DEFAULT_REPOS = ["faronov/zigbee-rs:ncp-v", "faronov/zigbee-rs-hub:hub-v"]
DEFAULT_KEEP = 5
SITE = "https://faronov.github.io/zigbee-rs/"


class SiteError(Exception):
    pass


def http_get(url: str, *, api: bool = False, attempts: int = 4) -> bytes:
    headers = {"User-Agent": "zigbee-rs-flasher-site"}
    if api:
        headers["Accept"] = "application/vnd.github+json"
        token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
        if token:
            headers["Authorization"] = f"Bearer {token}"
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=60) as rsp:
                return rsp.read()
        except urllib.error.HTTPError as error:
            if error.code < 500 or attempt == attempts - 1:
                raise SiteError(f"GET {url}: HTTP {error.code}") from None
        except (urllib.error.URLError, TimeoutError) as error:
            if attempt == attempts - 1:
                raise SiteError(f"GET {url}: {error}") from None
        time.sleep(2 ** attempt)
    raise AssertionError


def parse_repo(spec: str) -> tuple[str, str]:
    repo, sep, prefix = spec.partition(":")
    if not sep or not re.fullmatch(r"[\w.-]+/[\w.-]+", repo) or not prefix:
        raise SiteError(f"--repo expects OWNER/NAME:TAG_PREFIX, got {spec!r}")
    return repo, prefix


def list_releases(repo: str, prefix: str, keep: int) -> list[dict]:
    releases = json.loads(http_get(f"{API}/repos/{repo}/releases?per_page=100", api=True))
    picked = [r for r in releases if not r["draft"] and r["tag_name"].startswith(prefix)]
    picked.sort(key=lambda r: r["published_at"] or "", reverse=True)
    return picked[:keep]


def asset(release: dict, name: str) -> dict | None:
    return next((a for a in release["assets"] if a["name"] == name), None)


def check_file(item: dict, what: str, *, offset: bool) -> None:
    if not isinstance(item, dict) or not SAFE_NAME.match(str(item.get("name", ""))):
        raise SiteError(f"{what}: bad file entry {item!r}")
    if not re.fullmatch(r"[0-9a-f]{64}", str(item.get("sha256", ""))) or not isinstance(item.get("size"), int):
        raise SiteError(f"{what}: {item['name']} needs sha256 and size")
    if offset and (not isinstance(item.get("offset"), int) or item["offset"] < 0 or item["offset"] % 0x1000):
        raise SiteError(f"{what}: {item['name']} needs a 4 KiB aligned offset")


def validate_manifest(manifest: dict, tag: str) -> None:
    if manifest.get("schema") != SCHEMA:
        raise SiteError(f"{tag}: release-manifest.json schema {manifest.get('schema')!r}, expected {SCHEMA}")
    if manifest.get("tag") != tag:
        raise SiteError(f"{tag}: release-manifest.json is for {manifest.get('tag')!r}")
    for key in ("product", "name", "version", "commit"):
        if not isinstance(manifest.get(key), str) or not manifest[key]:
            raise SiteError(f"{tag}: release-manifest.json lacks {key}")
    if not manifest.get("boards"):
        raise SiteError(f"{tag}: no boards")
    for board in manifest["boards"]:
        bid = board.get("id", "")
        if not SAFE_NAME.match(bid):
            raise SiteError(f"{tag}: bad board id {bid!r}")
        if board.get("chip_family") not in CHIP_FAMILIES:
            raise SiteError(f"{tag}/{bid}: unknown chip family {board.get('chip_family')!r}")
        files = board.get("files", {})
        for role, item in files.items():
            check_file(item, f"{tag}/{bid}/{role}", offset="offset" in item)
        if not board.get("flash") or any(role not in files or "offset" not in files[role] for role in board["flash"]):
            raise SiteError(f"{tag}/{bid}: `flash` must list file roles with offsets")
        offsets = sorted((files[r]["offset"], files[r]["offset"] + files[r]["size"], r) for r in board["flash"])
        for (_, end, a), (start, _, b) in zip(offsets, offsets[1:]):
            if start < end:
                raise SiteError(f"{tag}/{bid}: {a} overlaps {b}")
        fw = board.get("fw_update")
        if fw is not None:
            check_file(fw.get("file"), f"{tag}/{bid}/fw_update", offset=False)


def esp_web_tools_manifest(manifest: dict, board: dict, erase: bool) -> dict:
    improv = board.get("improv") or {}
    name = improv.get("firmware") or manifest["name"]
    return {
        # A different name than the running Improv firmware reports makes ESP
        # Web Tools treat the install as new, so the erase manifest always
        # erases; the update manifest asks (unticked by default) instead.
        "name": f"{name} (erase)" if erase else name,
        "version": manifest["version"],
        "new_install_prompt_erase": not erase,
        "new_install_improv_wait_time": improv.get("wait_time", 0),
        "builds": [{
            "chipFamily": board["chip_family"],
            "parts": [{"path": board["files"][role]["name"], "offset": board["files"][role]["offset"]}
                      for role in board["flash"]],
        }],
    }


def flash_bytes(size: str) -> int:
    m = re.fullmatch(r"(\d+)\s*([KM])B", str(size).strip().upper())
    if m is None:
        raise SiteError(f"bad flash size {size!r}")
    return int(m.group(1)) << (10 if m.group(2) == "K" else 20)


def psram_match(board: dict) -> list[int] | None:
    match = (board.get("match") or {}).get("embedded_psram_mb")
    if match is not None and (not isinstance(match, list) or not all(isinstance(n, int) for n in match)):
        raise SiteError(f"{board.get('id')}: match.embedded_psram_mb must be a list of MB")
    return match


def mark_size_groups(boards: list[dict], product_name: str) -> None:
    """Boards of one release that differ only in flash size and/or PSRAM
    (same product, firmware, chip and host link) form a size group: the page
    reads the chip's flash ID and embedded PSRAM and offers only images that
    fit, the largest flash first. A board's `match.embedded_psram_mb` lists
    the PSRAM sizes it is for (absent: any). A release manifest may name the
    group itself (`size_group`, `size_group_title`)."""
    keyed: dict[str, list[dict]] = {}
    for board in boards:
        # A feature *string* is a different build (the NCP's Cargo features per transport); a feature
        # *list* (hub: ["mqtt"] on 8 MB only) describes what an image of the group includes.
        features = board.get("features") if isinstance(board.get("features"), str) else ""
        key = board.get("size_group") or "|".join(
            [str(board.get(k) or "") for k in ("product", "firmware", "chip_family", "transport")] + [features])
        keyed.setdefault(key, []).append(board)
    for members in keyed.values():
        sizes = {flash_bytes(b["flash_size"]) for b in members}
        variants = {(flash_bytes(b["flash_size"]), tuple(sorted(psram_match(b) or ()))) for b in members}
        if len(members) < 2 or len(variants) != len(members):
            continue
        ids = os.path.commonprefix([b["id"] for b in members]).rstrip("-")
        titles = os.path.commonprefix([b["title"] for b in members]).rstrip(" (:,")
        label = " / ".join(f"{n >> 20} MB" for n in sorted(sizes))
        psram = sorted({b["psram"] for b in members if b.get("psram")})
        if psram:
            label += ", " + "/".join(psram) + " PSRAM"
        group = members[0].get("size_group") or ids or members[0]["id"]
        title = members[0].get("size_group_title") or f"{titles} ({label}, detected): {product_name}"
        for board in members:
            board["size_group"] = group
            board["size_group_title"] = title
    for board in boards:
        board["flash_bytes"] = flash_bytes(board["flash_size"])


def fingerprint(entries: list[tuple[str, str, str]]) -> str:
    """Identity of the published release set: (repo, tag, manifest SHA-256)."""
    return hashlib.sha256(json.dumps(sorted(entries)).encode()).hexdigest()


def collect(repos: list[str], keep: int) -> list[tuple[str, dict, dict | None, bytes | None]]:
    out = []
    for spec in repos:
        repo, prefix = parse_repo(spec)
        for release in list_releases(repo, prefix, keep):
            item = asset(release, "release-manifest.json")
            raw = http_get(item["browser_download_url"]) if item else None
            manifest = json.loads(raw) if raw else None
            if manifest is not None:
                validate_manifest(manifest, release["tag_name"])
            out.append((repo, release, manifest, raw))
    return out


def cmd_fetch(args: argparse.Namespace) -> None:
    out = Path(args.out)
    if out.exists():
        raise SiteError(f"{out} exists; refusing to mix stale images in")
    (out / "releases").mkdir(parents=True)
    products: dict[str, dict] = {}
    entries = []
    for repo, release, manifest, raw in collect(args.repo, args.keep):
        tag = release["tag_name"]
        if manifest is None:
            print(f"skip {repo} {tag}: no release-manifest.json", file=sys.stderr)
            continue
        entries.append((repo, tag, hashlib.sha256(raw).hexdigest()))
        directory = out / "releases" / tag
        directory.mkdir()
        boards = []
        for board in manifest["boards"]:
            for role in board["flash"]:
                item = board["files"][role]
                target = directory / item["name"]
                if not target.exists():
                    source = asset(release, item["name"])
                    if source is None:
                        raise SiteError(f"{tag}: asset {item['name']} missing")
                    data = http_get(source["browser_download_url"])
                    if len(data) != item["size"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
                        raise SiteError(f"{tag}: {item['name']} does not match release-manifest.json")
                    target.write_bytes(data)
            manifests = {}
            for mode in ("update", "erase"):
                name = f"{board['id']}.{mode}.json"
                (directory / name).write_text(json.dumps(
                    esp_web_tools_manifest(manifest, board, mode == "erase"), indent=2) + "\n")
                manifests[mode] = f"releases/{tag}/{name}"
            downloads = {a["name"]: a["browser_download_url"] for a in release["assets"]}
            files = [{"role": role, **item, "download": downloads.get(item["name"])}
                     for role, item in board["files"].items()]
            fw = board.get("fw_update")
            boards.append({
                **{k: v for k, v in board.items() if k not in ("files", "fw_update")},
                "files": files,
                "fw_update": fw and {**fw, "download": downloads.get(fw["file"]["name"])},
                "manifests": manifests,
            })
        mark_size_groups(boards, manifest["name"])
        product = products.setdefault(manifest["product"], {
            "product": manifest["product"], "name": manifest["name"], "repository": repo, "releases": []})
        product["releases"].append({
            "tag": tag,
            "version": manifest["version"],
            "prerelease": bool(release["prerelease"]),
            "date": manifest.get("date") or release["published_at"],
            "published": release["published_at"],
            "commit": manifest["commit"],
            "build_id": manifest.get("build_id"),
            "zrhp": manifest.get("zrhp"),
            "components": manifest.get("components"),
            "url": release["html_url"],
            "manifest_download": downloads.get("release-manifest.json"),
            "sha256sums_download": downloads.get("SHA256SUMS"),
            "boards": boards,
        })
    index = {
        "schema": SCHEMA,
        "generated": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "keep": args.keep,
        "fingerprint": fingerprint(entries),
        "products": list(products.values()),
    }
    (out / "releases.json").write_text(json.dumps(index, indent=2) + "\n")
    print(f"{len(entries)} release(s): " + ", ".join(tag for _, tag, _ in entries) if entries else "no releases yet")


def cmd_check(args: argparse.Namespace) -> None:
    """Every ESP Web Tools manifest resolves to files whose SHA-256 is the
    one the release manifest promised."""
    root = Path(args.dir)
    index = json.loads((root / "releases.json").read_text())
    count = 0
    for product in index["products"]:
        for release in product["releases"]:
            for board in release["boards"]:
                sums = {f["name"]: f["sha256"] for f in board["files"]}
                for mode, rel in board["manifests"].items():
                    path = root / rel
                    manifest = json.loads(path.read_text())
                    if not isinstance(manifest.get("name"), str) or not manifest.get("builds"):
                        raise SiteError(f"{rel}: not an ESP Web Tools manifest")
                    if manifest["new_install_prompt_erase"] != (mode == "update"):
                        raise SiteError(f"{rel}: wrong erase policy")
                    for build in manifest["builds"]:
                        if build["chipFamily"] not in CHIP_FAMILIES or not build["parts"]:
                            raise SiteError(f"{rel}: bad build")
                        for part in build["parts"]:
                            data = (path.parent / part["path"]).read_bytes()
                            if hashlib.sha256(data).hexdigest() != sums[part["path"]]:
                                raise SiteError(f"{rel}: {part['path']} SHA-256 mismatch")
                            if not isinstance(part["offset"], int):
                                raise SiteError(f"{rel}: offset must be a number")
                    count += 1
    print(f"{count} ESP Web Tools manifest(s) verified")


def live_status(site: str) -> dict | None:
    for rel in ("flasher/sensor/status.json", "flasher/status.json"):
        try:
            return json.loads(http_get(f"{site.rstrip('/')}/{rel}?t={int(time.time())}", attempts=2))
        except (SiteError, json.JSONDecodeError):
            continue
    return None


def live_fingerprint(site: str) -> str | None:
    try:
        return json.loads(http_get(f"{site.rstrip('/')}/flasher/releases.json?t={int(time.time())}", attempts=2))["fingerprint"]
    except (SiteError, json.JSONDecodeError, KeyError):
        return None


def cmd_live(args: argparse.Namespace) -> None:
    status = live_status(args.site) or {}
    print(status.get("revision", ""))


def cmd_resolve(args: argparse.Namespace) -> None:
    status = live_status(args.site)
    revision, run_id, jobs = "", "0", dict.fromkeys(SENSOR_JOBS, "skipped")
    if status and re.fullmatch(r"[0-9a-f]{40}", status.get("revision", "")):
        revision = status["revision"]
        match = re.search(r"/actions/runs/(\d+)", status.get("workflow_run", ""))
        if match and status.get("firmware_available"):
            run_id = match.group(1)
            jobs.update({k: v for k, v in status.get("firmware_jobs", {}).items() if k in jobs})
    if args.revision:
        revision = args.revision
    if not revision:
        raise SiteError("no live site status and no --revision")
    entries = [(repo, release["tag_name"], hashlib.sha256(raw).hexdigest())
               for repo, release, manifest, raw in collect(args.repo, args.keep) if manifest is not None]
    wanted = fingerprint(entries)
    live = live_fingerprint(args.site)
    changed = args.force or live != wanted
    outputs = {"revision": revision, "sensor_run_id": run_id, **{f"{k}_result": v for k, v in jobs.items()},
               "changed": str(changed).lower(), "fingerprint": wanted}
    print(json.dumps({**outputs, "live_fingerprint": live}, indent=2))
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as f:
            f.writelines(f"{k}={v}\n" for k, v in outputs.items())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("fetch")
    p.add_argument("--repo", action="append")
    p.add_argument("--keep", type=int, default=DEFAULT_KEEP)
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_fetch)
    p = sub.add_parser("check")
    p.add_argument("dir")
    p.set_defaults(func=cmd_check)
    p = sub.add_parser("resolve")
    p.add_argument("--site", default=SITE)
    p.add_argument("--repo", action="append")
    p.add_argument("--keep", type=int, default=DEFAULT_KEEP)
    p.add_argument("--force", action="store_true")
    p.add_argument("--revision", default="")
    p.set_defaults(func=cmd_resolve)
    p = sub.add_parser("live")
    p.add_argument("--site", default=SITE)
    p.set_defaults(func=cmd_live)
    args = parser.parse_args()
    if getattr(args, "repo", False) is None:
        args.repo = DEFAULT_REPOS
    try:
        args.func(args)
    except (SiteError, OSError, KeyError, ValueError) as error:
        sys.exit(f"flasher_site: {error}")


if __name__ == "__main__":
    main()
