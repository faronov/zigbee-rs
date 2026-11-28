#!/usr/bin/env python3
"""Generate the zigbee-rs host protocol (ZRHP) v1 golden vectors.

This script is a vector generator, not an implementation. It encodes the
byte layouts from docs/book/src/advanced/host-protocol.md with the smallest
amount of code that makes every vector reproducible:

    python3 generate.py          # rewrite the JSON files next to this script
    python3 generate.py --check  # fail if a committed JSON file differs

Only the Python standard library is required, except for the AES-MMO
install-code vector, which uses `cryptography` (or `pycryptodome`) when
available and otherwise keeps the committed value.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# --------------------------------------------------------------------------
# CRC-32C (Castagnoli), reflected polynomial 0x82F63B78, init/xorout 0xFFFFFFFF
# --------------------------------------------------------------------------


def crc32c(data: bytes, crc: int = 0) -> int:
    crc ^= 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


assert crc32c(b"123456789") == 0xE3069283

# --------------------------------------------------------------------------
# COBS (Cheshire/Baker), delimiter 0x00 appended by the framer
# --------------------------------------------------------------------------


def cobs_encode(data: bytes) -> bytes:
    """Minimal COBS: a full 254-byte block at the very end is closed by its
    0xFF code without a trailing empty 0x01 block (n + ceil(n/254) bytes)."""
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if code == 0xFF:  # previous block full and more data follows
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS")
        out += data[i + 1 : i + code]
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


# --------------------------------------------------------------------------
# Link layer
# --------------------------------------------------------------------------

LINK_VERSION = 1
T_DATA, T_ACK, T_NAK, T_RESET_REQ, T_RESET_ACK, T_REJECT = 0, 1, 2, 3, 4, 5
F_POLL, F_RETX = 0x20, 0x10
HEADER_LEN, CRC_LEN, MAX_PAYLOAD = 8, 4, 640


def link_frame(ftype, *, epoch=0, endpoint=0, seq=0, ack=0, ep_seq=0, flags=0, payload=b""):
    assert len(payload) <= MAX_PAYLOAD
    ctrl = (LINK_VERSION << 6) | flags | ftype
    header = struct.pack("<BBBBBBH", ctrl, epoch, endpoint, seq, ack, ep_seq, len(payload))
    body = header + payload
    crc = crc32c(body)
    decoded = body + struct.pack("<I", crc)
    wire = cobs_encode(decoded) + b"\x00"
    return decoded, crc, wire


def session_epoch(host_cookie: int, ncp_cookie: int) -> int:
    return 1 + crc32c(struct.pack("<II", host_cookie, ncp_cookie)) % 255


def reset_payload(cookie, peer_cookie, role, rx_window, reason, rx_mtu, ack_delay_us,
                  keepalive_ms, link_timeout_ms, credits):
    out = struct.pack("<IIBBBBHHHHB", cookie, peer_cookie, role, LINK_VERSION, rx_window,
                      reason, rx_mtu, ack_delay_us, keepalive_ms, link_timeout_ms, len(credits))
    for ep, credit in credits:
        out += struct.pack("<BB", ep, credit)
    return out


def ack_payload(sack, nbad=0, credits=()):
    out = struct.pack("<IBB", sack, nbad, len(credits))
    for ep, edge in credits:
        out += struct.pack("<BB", ep, edge)
    return out


# --------------------------------------------------------------------------
# Message layer
# --------------------------------------------------------------------------

K_REQUEST, K_RESPONSE, K_EVENT = 0, 1, 2


def message(kind, msg_id, tid, payload, retry=False):
    flags = (kind << 6) | (0x20 if retry else 0)
    return struct.pack("<BHHH", flags, msg_id, tid, len(payload)) + payload


def ieee_wire(text: str) -> bytes:
    """OCB/zigpy big-endian hex -> on-air little-endian bytes."""
    return bytes.fromhex(text)[::-1]


def ieee_text(raw: bytes) -> str:
    return raw[::-1].hex()


EP_LINK, EP_SYSTEM, EP_ZIGBEE, EP_TABLES, EP_BULK, EP_LOG = 0, 1, 2, 3, 4, 5

SYS_NEGOTIATE, SYS_GET_STATUS, SYS_ECHO, SYS_SET_UART = 0x0001, 0x0004, 0x0007, 0x0009
EVT_NCP_READY = 0x0080
NWK_PERMIT_JOIN, EVT_DEVICE_JOINED, EVT_ROUTE_RECORD = 0x0105, 0x0181, 0x0185
EVT_CHILD_POLL = 0x018C
APS_DATA_REQUEST, EVT_APS_DATA_INDICATION, EVT_APS_DATA_CONFIRM = 0x0201, 0x0280, 0x0281
SEC_ADD_INSTALL_CODE, SEC_LOCK_KEY_EXPORT, EVT_FC_CHECKPOINT = 0x0403, 0x040A, 0x0483
GP_SINK_CONFIG, GP_SINK_COMMISSIONING = 0x0A01, 0x0A03
GP_PAIRING_CONFIGURATION, GP_SEND_PAIRING, GP_TRANSLATION_UPDATE = 0x0A08, 0x0A09, 0x0A0A
GP_TRANSLATION_REQUEST, GP_REMOTE_SINK_TABLE_REQUEST = 0x0A0B, 0x0A0C
EVT_GP_FRAME, EVT_GP_COMMISSIONED = 0x0A80, 0x0A82
EVT_GP_REMOTE_STATUS, EVT_GP_REMOTE_TRANSLATION_TABLE = 0x0A85, 0x0A88
EVT_GP_APP_DESCRIPTION = 0x0A89
LOG_RECORD = 0x0980
TBL_LOOKUP, TBL_WRITEBACK, TBL_READ = 0x0501, 0x0502, 0x0506
BKP_RESTORE_BEGIN, BKP_RESTORE_PUT, BKP_RESTORE_COMMIT = 0x0611, 0x0612, 0x0613
BKP_EXPORT_BEGIN, BKP_EXPORT_READ = 0x0601, 0x0602

HOST_COOKIE, NCP_COOKIE = 0x1A2B3C4D, 0xC0FFEE01
EPOCH = session_epoch(HOST_COOKIE, NCP_COOKIE)


def sys_negotiate_req():
    name = b"zigpy-zrs/0.1"
    return struct.pack("<BBQB", 1, 1, 0, len(name)) + name


def sys_negotiate_rsp():
    # Bit 14 (EZSP hashed TCLK) stays clear until gate H5; bit 11 is reserved.
    caps = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) \
        | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) \
        | (1 << 20)
    capacities = [(1, 16), (2, 64), (3, 256), (4, 512), (5, 128), (6, 100), (7, 1024),
                  (8, 8), (13, 16), (14, 8), (15, 16), (16, 8), (17, 8), (18, 256)]
    out = struct.pack("<BBBQB", 0x00, 1, 0, caps, len(capacities))
    for cid, value in capacities:
        out += struct.pack("<BI", cid, value)
    fw = b"0.1.0"
    out += struct.pack("<B", len(fw)) + fw
    out += bytes.fromhex("0123456789abcdef")  # build id: first 8 bytes of the source commit
    out += struct.pack("<B", 1)  # chip: 1 = ESP32-H2
    return out, caps, capacities


def ncp_ready_payload():
    return struct.pack("<IBBIIIHI", NCP_COOKIE, 0x01, 0x02, 7, 2011024, 14024, 0, 0)


def aps_data_request(dst_nwk, dst_ep, src_ep, profile, cluster, asdu, tx_options=0x01,
                     radius=0, relays=None, priority=1):
    if relays is not None:
        tx_options |= 0x40
    out = struct.pack("<BH", 0x02, dst_nwk)
    out += struct.pack("<BBHHBBBB", dst_ep, src_ep, profile, cluster, tx_options, radius, 0, priority)
    if relays is not None:
        out += struct.pack("<B", len(relays)) + b"".join(struct.pack("<H", r) for r in relays)
    out += struct.pack("<H", len(asdu)) + asdu
    return out


def aps_data_indication(src_nwk, src_ieee, src_ep, dst_ep, profile, cluster, asdu, lqi, rssi,
                        last_hop, aps_counter, rx_time_ms, flags=0x06, key_source=1):
    out = struct.pack("<BHB", 0x02, 0x0000, dst_ep)
    out += struct.pack("<H", src_nwk) + ieee_wire(src_ieee)
    out += struct.pack("<BHHBBbHBBI", src_ep, profile, cluster, flags, lqi, rssi, last_hop,
                       aps_counter, key_source, rx_time_ms)
    out += struct.pack("<H", len(asdu)) + asdu
    return out


def aps_data_confirm(req_tid, status, zb_status, aps_counter, dst_nwk, flags, elapsed_ms):
    return struct.pack("<HBBBHBH", req_tid, status, zb_status, aps_counter, dst_nwk, flags, elapsed_ms)


# --------------------------------------------------------------------------
# AES-MMO install-code key (Zigbee spec B.6), verified against zigbee-crypto
# --------------------------------------------------------------------------

SILABS_INSTALL_CODE = bytes.fromhex("88776655443322111122334455667788D490")
SILABS_INSTALL_KEY = bytes.fromhex("FA8081CAAA41D5ADE9B5658799268B88")


def _aes_ecb(key: bytes, block: bytes) -> bytes | None:
    try:
        from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

        enc = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
        return enc.update(block) + enc.finalize()
    except ImportError:
        pass
    try:
        from Crypto.Cipher import AES  # type: ignore

        return AES.new(key, AES.MODE_ECB).encrypt(block)
    except ImportError:
        return None


def aes_mmo(data: bytes) -> bytes | None:
    bits = len(data) * 8
    padded = bytearray(data) + b"\x80"
    while (len(padded) + 2) % 16:
        padded.append(0)
    padded += struct.pack(">H", bits)
    digest = bytes(16)
    for i in range(0, len(padded), 16):
        block = bytes(padded[i : i + 16])
        enc = _aes_ecb(digest, block)
        if enc is None:
            return None
        digest = bytes(a ^ b for a, b in zip(enc, block))
    return digest


def hmac_mmo(key: bytes, message: bytes) -> bytes | None:
    """Zigbee keyed hash (HMAC with the AES-MMO hash, block size 16)."""
    assert len(key) == 16
    inner = aes_mmo(bytes(k ^ 0x36 for k in key) + message)
    if inner is None:
        return None
    return aes_mmo(bytes(k ^ 0x5C for k in key) + inner)


# --------------------------------------------------------------------------
# Backup records (BKP_* and TBL_* share them)
# --------------------------------------------------------------------------

R_NODE, R_NETWORK, R_NETWORK_KEY, R_TC_LINK_KEY, R_DEVICE = 0x01, 0x02, 0x03, 0x04, 0x05
R_EZSP_HASHED_TCLK, R_SOURCE_ROUTE, R_GP_PROXY, R_RS_PARAMS, R_END = 0x06, 0x08, 0x09, 0x0A, 0xFF
R_GP_SINK, R_GP_TRANSLATION, R_APS_GROUP = 0x0B, 0x0C, 0x0D
APS_UNKNOWN_TCLK_MARGIN = 1 << 20  # host rule when a backup has no TCLK counter (9.4)
DEV_IS_CHILD, DEV_HAS_LINK_KEY, DEV_HAS_CHILD_INFO, DEV_NWK_VALID = 0x01, 0x02, 0x04, 0x08
KEY_ATTR_VERIFIED, KEY_ORIGIN_IMPORTED = 2, 3
FC_STRIDE = 0x400
TCLK_DEFAULT = b"ZigBeeAlliance09"


def record(rtype: int, value: bytes) -> bytes:
    return struct.pack("<BH", rtype, len(value)) + value


def gp_sink_record(src_id, device_id, group, key, frame_counter, security_level=3, key_type=4):
    value = struct.pack("<BIBBHHBBB", 0, src_id, device_id, 2, group, 0xFFFF,
                        0x01 | (0x08 if key else 0), security_level, key_type)
    value += (key or b"") + struct.pack("<IB", frame_counter, 0)
    return record(R_GP_SINK, value)


def gp_translation(gpd_command, endpoint, cluster, zcl_command, payload=b"", payload_len=None,
                   profile=0x0104):
    """One translation (7.11): payload_len 0xFE derived, 0xFF copied, else the fixed bytes."""
    if payload_len is None:
        payload_len = len(payload)
    return struct.pack("<BBHHBB", gpd_command, endpoint, profile, cluster, zcl_command,
                       payload_len) + (payload if payload_len < 0xFE else b"")


def gp_translation_record(src_id, translation):
    return record(R_GP_TRANSLATION, struct.pack("<BI", 0, src_id) + translation)


def end_record(records: list[bytes]) -> bytes:
    body = b"".join(records)
    return record(R_END, struct.pack("<II", len(records), crc32c(body)))


def channels_to_mask(channels):
    mask = 0
    for channel in channels:
        assert 11 <= channel <= 26
        mask |= 1 << channel
    return mask


def mask_to_channels(mask):
    return [c for c in range(11, 27) if mask & (1 << c)]


def device_record(ieee, nwk, is_child, link_key=None, key_seq=0, capability=0x80, timeout_exp=8):
    flags = (DEV_IS_CHILD if is_child else 0) | (DEV_NWK_VALID if nwk is not None else 0)
    flags |= DEV_HAS_CHILD_INFO if is_child else 0
    flags |= DEV_HAS_LINK_KEY if link_key else 0
    value = ieee_wire(ieee) + struct.pack("<HB", 0xFFFF if nwk is None else nwk, flags)
    if is_child:
        value += struct.pack("<BB", capability, timeout_exp)
    if link_key:
        value += bytes.fromhex(link_key["key"]) + struct.pack(
            "<BBBII", key_seq, KEY_ATTR_VERIFIED, KEY_ORIGIN_IMPORTED,
            link_key["tx_counter"], link_key["rx_counter"])
    return record(R_DEVICE, value)


def ocb_to_records(ocb: dict) -> list[bytes]:
    """Host-side conversion of an Open Coordinator Backup into restore records."""
    internal = ocb["metadata"].get("internal", {})
    node = internal.get("node", {})
    network = internal.get("network", {})
    coordinator = ieee_wire(ocb["coordinator_ieee"])
    node_nwk = int(node.get("nwk", "0000"), 16)
    recs = [record(R_NODE, coordinator + struct.pack("<HBB", node_nwk, 0, 1))]
    tx_power = internal.get("tx_power")
    recs.append(record(R_NETWORK, struct.pack(
        "<H", int(ocb["pan_id"], 16)) + ieee_wire(ocb["extended_pan_id"]) + struct.pack(
        "<BIBHBb", ocb["channel"], channels_to_mask(ocb["channel_mask"]), ocb["nwk_update_id"],
        int(network.get("nwk_manager", "0000"), 16), ocb["security_level"],
        0x7F if tx_power is None else tx_power)))
    key = ocb["network_key"]
    recs.append(record(R_NETWORK_KEY, bytes.fromhex(key["key"]) + struct.pack(
        "<BIB", key["sequence_number"], key["frame_counter"], 0x01)))
    tclk = network.get("tc_link_key")
    tc_key = bytes.fromhex(tclk["key"]) if tclk else TCLK_DEFAULT
    tc_counter = tclk.get("frame_counter", 0) if tclk else 0
    tc_address = ieee_wire(network["tc_address"]) if "tc_address" in network else coordinator
    hashed = ocb.get("stack_specific", {}).get("ezsp", {}).get("hashed_tclk")
    recs.append(record(R_TC_LINK_KEY, tc_key + tc_address + struct.pack(
        "<IB", tc_counter, 0x01 if hashed else 0x00)))
    if hashed:
        recs.append(record(R_EZSP_HASHED_TCLK, bytes.fromhex(hashed)))
    zrs = ocb.get("stack_specific", {}).get("zigbee_rs")
    if zrs:
        recs.append(record(R_RS_PARAMS, struct.pack(
            "<IIIIB", zrs["frame_counter_stride"], zrs["table_epoch"],
            zrs["nwk_frame_counter_limit"], zrs["aps_frame_counter_limit"],
            0x01 if zrs.get("host_backed_tables") else 0x00)))
    seqs = internal.get("link_key_seqs", {})
    for dev in sorted(ocb["devices"], key=lambda d: d["ieee_address"].lower()):
        ieee = dev["ieee_address"].lower()
        nwk = None if dev["nwk_address"] is None else int(dev["nwk_address"].zfill(4), 16)
        recs.append(device_record(ieee, nwk, dev.get("is_child", True), dev.get("link_key"),
                                  seqs.get(ieee, 0)))
    recs.append(end_record(recs))
    return recs


def parse_records(stream: bytes) -> list[tuple[int, bytes]]:
    out, i = [], 0
    while i < len(stream):
        rtype, length = struct.unpack_from("<BH", stream, i)
        out.append((rtype, stream[i + 3 : i + 3 + length]))
        i += 3 + length
    return out


def ncp_apply_restore(recs: list[bytes], nwk_margin: int, aps_margin: int) -> dict:
    """Model the NCP-side counter rules of BKP_RESTORE_COMMIT (section 9.4)."""
    parsed = parse_records(b"".join(recs))
    assert parsed[-1][0] == R_END
    count, crc = struct.unpack("<II", parsed[-1][1])
    assert count == len(parsed) - 1 and crc == crc32c(b"".join(recs[:-1]))
    state = {"devices": []}
    aps_max = 0
    for rtype, value in parsed[:-1]:
        if rtype == R_NODE:
            state["ieee"] = value[:8]
            state["nwk"] = struct.unpack_from("<H", value, 8)[0]
        elif rtype == R_NETWORK:
            state["network"] = value
        elif rtype == R_NETWORK_KEY:
            state["nwk_key"] = value[:16]
            state["nwk_key_seq"] = value[16]
            state["nwk_fc_in"] = struct.unpack_from("<I", value, 17)[0]
        elif rtype == R_TC_LINK_KEY:
            state["tclk"] = value[:16]
            state["tc_address"] = value[16:24]
            tc_counter = struct.unpack_from("<I", value, 24)[0]
            state["tclk_flags"] = value[28]
            aps_max = max(aps_max, tc_counter)
        elif rtype == R_EZSP_HASHED_TCLK:
            state["hashed_tclk"] = value
        elif rtype == R_RS_PARAMS:
            _stride, _epoch, nwk_floor, aps_floor, _flags = struct.unpack("<IIIIB", value)
            state["nwk_floor"] = nwk_floor
            aps_max = max(aps_max, aps_floor)
        elif rtype == R_DEVICE:
            state["devices"].append(value)
            flags = value[10]
            if flags & DEV_HAS_LINK_KEY:
                offset = 11 + (2 if flags & DEV_HAS_CHILD_INFO else 0)
                aps_max = max(aps_max, struct.unpack_from("<I", value, offset + 19)[0])
    state["nwk_fc_start"] = max(state["nwk_fc_in"], state.get("nwk_floor", 0)) + nwk_margin
    state["aps_fc_start"] = aps_max + aps_margin
    state["nwk_fc_limit"] = state["nwk_fc_start"] + FC_STRIDE
    state["aps_fc_limit"] = state["aps_fc_start"] + FC_STRIDE
    return state


def ncp_export_records(state: dict, table_epoch: int) -> list[bytes]:
    recs = [record(R_NODE, state["ieee"] + struct.pack("<HBB", state["nwk"], 0, 1)),
            record(R_NETWORK, state["network"]),
            record(R_NETWORK_KEY, state["nwk_key"] + struct.pack(
                "<BIB", state["nwk_key_seq"], state["nwk_fc_limit"], 0x01)),
            record(R_TC_LINK_KEY, state["tclk"] + state["tc_address"] + struct.pack(
                "<IB", state["aps_fc_limit"], state["tclk_flags"]))]
    if "hashed_tclk" in state:
        recs.append(record(R_EZSP_HASHED_TCLK, state["hashed_tclk"]))
    recs.append(record(R_RS_PARAMS, struct.pack(
        "<IIIIB", FC_STRIDE, table_epoch, state["nwk_fc_limit"], state["aps_fc_limit"], 0x01)))
    for value in state["devices"]:
        value = bytearray(value)
        flags = value[10]
        if flags & DEV_HAS_LINK_KEY:
            offset = 11 + (2 if flags & DEV_HAS_CHILD_INFO else 0)
            struct.pack_into("<I", value, offset + 19, state["aps_fc_limit"])
        recs.append(record(R_DEVICE, bytes(value)))
    recs.append(end_record(recs))
    return recs


def records_to_ocb(recs: list[bytes], *, source: str, creation_time: str,
                   passthrough_stack_specific: dict) -> dict:
    """Host-side conversion of exported records into an Open Coordinator Backup."""
    parsed = parse_records(b"".join(recs))
    assert parsed[-1][0] == R_END
    devices, seqs, stack_specific = [], {}, dict(passthrough_stack_specific)
    out: dict = {}
    for rtype, value in parsed[:-1]:
        if rtype == R_NODE:
            out["coordinator_ieee"] = ieee_text(value[:8])
            node_nwk = struct.unpack_from("<H", value, 8)[0]
        elif rtype == R_NETWORK:
            pan, = struct.unpack_from("<H", value, 0)
            epan = value[2:10]
            channel, mask, update_id, manager, sec_level, tx_power = struct.unpack_from(
                "<BIBHBb", value, 10)
            out.update(pan_id=f"{pan:04x}", extended_pan_id=ieee_text(epan), channel=channel,
                       channel_mask=mask_to_channels(mask), nwk_update_id=update_id,
                       security_level=sec_level)
        elif rtype == R_NETWORK_KEY:
            seq, counter = struct.unpack_from("<BI", value, 16)
            out["network_key"] = {"key": value[:16].hex(), "sequence_number": seq,
                                  "frame_counter": counter}
        elif rtype == R_TC_LINK_KEY:
            tclk = {"key": value[:16].hex(), "frame_counter": struct.unpack_from("<I", value, 24)[0]}
            tc_address = ieee_text(value[16:24])
        elif rtype == R_EZSP_HASHED_TCLK:
            stack_specific.setdefault("ezsp", {})["hashed_tclk"] = value.hex()
        elif rtype == R_RS_PARAMS:
            stride, epoch, nwk_limit, aps_limit, flags = struct.unpack("<IIIIB", value)
            stack_specific["zigbee_rs"] = {
                "version": 1, "frame_counter_stride": stride, "table_epoch": epoch,
                "nwk_frame_counter_limit": nwk_limit, "aps_frame_counter_limit": aps_limit,
                "host_backed_tables": bool(flags & 0x01), "gp_proxy_table": [],
                "gp_sink_table": []}
        elif rtype == R_DEVICE:
            ieee = ieee_text(value[:8])
            nwk, flags = struct.unpack_from("<HB", value, 8)
            dev = {"ieee_address": ieee,
                   "nwk_address": f"{nwk:04x}" if flags & DEV_NWK_VALID else None,
                   "is_child": bool(flags & DEV_IS_CHILD)}
            if flags & DEV_HAS_LINK_KEY:
                offset = 11 + (2 if flags & DEV_HAS_CHILD_INFO else 0)
                key = value[offset : offset + 16]
                key_seq, _attrs, _origin, tx, rx = struct.unpack_from("<BBBII", value, offset + 16)
                dev["link_key"] = {"key": key.hex(), "tx_counter": tx, "rx_counter": rx}
                seqs[ieee] = key_seq
            devices.append(dev)
    out = {
        "metadata": {
            "version": 1,
            "format": "zigpy/open-coordinator-backup",
            "source": source,
            "internal": {
                "creation_time": creation_time,
                "node": {"ieee": out["coordinator_ieee"], "nwk": f"{node_nwk:04x}",
                         "type": "coordinator"},
                "network": {"tc_link_key": tclk, "tc_address": tc_address, "nwk_manager": "0000"},
                "link_key_seqs": seqs,
                **({"tx_power": tx_power} if tx_power != 0x7F else {}),
            },
        },
        "stack_specific": stack_specific,
        **{k: out[k] for k in ("coordinator_ieee", "pan_id", "extended_pan_id", "nwk_update_id",
                               "security_level", "channel", "channel_mask", "network_key")},
        "devices": sorted(devices, key=lambda d: d["ieee_address"]),
    }
    return out


# --------------------------------------------------------------------------
# Vector sets
# --------------------------------------------------------------------------


def hx(data: bytes) -> str:
    return data.hex()


def link_vector(name, note, ftype, **kw):
    decoded, crc, wire = link_frame(ftype, **kw)
    fields = {k: v for k, v in kw.items() if k != "payload"}
    return {"name": name, "note": note, "type": ftype, "fields": fields,
            "payload_hex": hx(kw.get("payload", b"")), "decoded_hex": hx(decoded),
            "crc32c": f"{crc:08x}", "wire_hex": hx(wire)}


def build_crc_cobs():
    crc = {"algorithm": "CRC-32C (Castagnoli) reflected poly 0x82F63B78, init 0xFFFFFFFF, "
                        "xorout 0xFFFFFFFF, stored little-endian",
           "vectors": [{"input_ascii": "123456789", "crc32c": "e3069283"},
                       {"input_hex": "", "crc32c": f"{crc32c(b''):08x}"},
                       {"input_hex": "00" * 32, "crc32c": f"{crc32c(bytes(32)):08x}"},
                       {"input_hex": "ff" * 32, "crc32c": f"{crc32c(bytes([0xFF]) * 32):08x}"}]}
    cases = [("empty", b""), ("one_zero", b"\x00"), ("two_zeros", b"\x00\x00"),
             ("mixed", bytes.fromhex("11220033")), ("run_254", bytes([1]) * 254),
             ("run_255", bytes([1]) * 255), ("run_254_then_zero", bytes([2]) * 254 + b"\x00"),
             ("max_frame_no_zero", bytes((i % 255) + 1 for i in range(652)))]
    cobs = {"note": "encoded_hex excludes the 0x00 delimiter; wire = encoded + 00",
            "vectors": []}
    for name, raw in cases:
        enc = cobs_encode(raw)
        assert cobs_decode(enc) == raw and 0 not in enc
        assert len(enc) == max(1, len(raw) + -(-len(raw) // 254))  # n + ceil(n/254), 5.1
        cobs["vectors"].append({"name": name, "raw_hex": hx(raw), "encoded_hex": hx(enc),
                                "raw_len": len(raw), "encoded_len": len(enc)})
    # Non-minimal encodings that every decoder must still accept (5.1).
    nonminimal = bytes([0xFF]) + bytes([1]) * 254 + b"\x01"
    assert cobs_decode(nonminimal) == bytes([1]) * 254
    cobs["decode_only"] = [{"name": "run_254_trailing_empty_block", "raw_hex": hx(bytes([1]) * 254),
                            "encoded_hex": hx(nonminimal)}]
    return crc, cobs


def build_link():
    host_credits = [(EP_SYSTEM, 4), (EP_ZIGBEE, 8), (EP_TABLES, 4), (EP_BULK, 4), (EP_LOG, 2)]
    ncp_credits = [(EP_SYSTEM, 4), (EP_ZIGBEE, 8), (EP_TABLES, 4), (EP_BULK, 2), (EP_LOG, 0)]
    req = reset_payload(HOST_COOKIE, 0, 0, 32, 4, 640, 1000, 500, 3000, host_credits)
    ack = reset_payload(NCP_COOKIE, HOST_COOKIE, 1, 16, 0, 640, 1000, 500, 3000, ncp_credits)
    negotiate = message(K_REQUEST, SYS_NEGOTIATE, 0x0001, sys_negotiate_req())
    two = message(K_REQUEST, SYS_ECHO, 0x0002, b"ping") + \
        message(K_REQUEST, SYS_GET_STATUS, 0x0003, b"")
    vectors = [
        link_vector("reset_req_host", "host opens the port; epoch 0; reason 4 = host open",
                    T_RESET_REQ, payload=req),
        link_vector("reset_ack_ncp", "NCP accepts; window 16 toward the NCP; LOG credit 0",
                    T_RESET_ACK, payload=ack),
        link_vector("data_negotiate", "first DATA on SYSTEM: seq 0, ep_seq 0", T_DATA,
                    epoch=EPOCH, endpoint=EP_SYSTEM, seq=0, ack=0, ep_seq=0, payload=negotiate),
        link_vector("data_batch_two_messages", "SYS_ECHO and SYS_GET_STATUS batched in one frame",
                    T_DATA, epoch=EPOCH, endpoint=EP_SYSTEM, seq=1, ack=0, ep_seq=1, payload=two),
        link_vector("data_retransmit_poll", "retransmission of seq 1 after RTO, RETX|POLL",
                    T_DATA, epoch=EPOCH, endpoint=EP_SYSTEM, seq=1, ack=0, ep_seq=1,
                    flags=F_RETX | F_POLL, payload=two),
        link_vector("ack_plain", "cumulative ack 5, nothing beyond", T_ACK, epoch=EPOCH, ack=5,
                    payload=ack_payload(0)),
        link_vector("nak_gap", "seq 2 missing; 3 and 4 received (sack bits 0,1)", T_NAK,
                    epoch=EPOCH, ack=2, payload=ack_payload(0x00000003)),
        link_vector("nak_crc_error", "one CRC-failed frame since seq 0 was received", T_NAK,
                    epoch=EPOCH, ack=1, payload=ack_payload(0, nbad=1)),
        link_vector("ack_with_credits", "ack 9 and credit edges ZIGBEE=17, BULK=6", T_ACK,
                    epoch=EPOCH, ack=9, payload=ack_payload(0, credits=[(EP_ZIGBEE, 17), (EP_BULK, 6)])),
        link_vector("keepalive_poll", "idle keepalive: ACK with POLL", T_ACK, epoch=EPOCH, ack=9,
                    flags=F_POLL, payload=ack_payload(0)),
        link_vector("reject_no_credit", "DATA seq 7 on BULK ep_seq 6 rejected: NO_CREDIT",
                    T_REJECT, epoch=EPOCH, ack=8, payload=struct.pack("<BBBB", 7, EP_BULK, 6, 1)),
    ]
    max_payload = bytes((i * 7 + 1) % 256 for i in range(MAX_PAYLOAD))
    vectors.append(link_vector("data_max_payload", "640-byte payload; worst-case size check",
                               T_DATA, epoch=EPOCH, endpoint=EP_BULK, seq=200, ack=17,
                               ep_seq=99, payload=max_payload))
    worst = max(len(v["wire_hex"]) // 2 for v in vectors)
    return {"session": {"host_cookie": f"{HOST_COOKIE:08x}", "ncp_cookie": f"{NCP_COOKIE:08x}",
                        "epoch": EPOCH,
                        "epoch_rule": "1 + crc32c(host_cookie_le32 || ncp_cookie_le32) % 255"},
            "max_wire_len_in_set": worst, "vectors": vectors}


def build_messages():
    out = []

    def add(name, endpoint, kind, msg_id, tid, payload, note, fields=None):
        out.append({"name": name, "endpoint": endpoint, "kind": kind, "msg_id": f"0x{msg_id:04x}",
                    "tid": tid, "note": note, "fields": fields or {}, "payload_hex": hx(payload),
                    "message_hex": hx(message(kind, msg_id, tid, payload))})

    add("sys_negotiate_req", EP_SYSTEM, K_REQUEST, SYS_NEGOTIATE, 1, sys_negotiate_req(),
        "host supports major 1 only, no host capability bits",
        {"major_min": 1, "major_max": 1, "host_caps": 0, "name": "zigpy-zrs/0.1"})
    rsp, caps, capacities = sys_negotiate_rsp()
    add("sys_negotiate_rsp", EP_SYSTEM, K_RESPONSE, SYS_NEGOTIATE, 1, rsp, "v1.0 selected",
        {"status": 0, "major": 1, "minor": 0, "ncp_caps": f"0x{caps:016x}",
         "capacities": {str(k): v for k, v in capacities}, "fw": "0.1.0", "chip": 1})
    add("evt_ncp_ready", EP_SYSTEM, K_EVENT, EVT_NCP_READY, 1, ncp_ready_payload(),
        "power-on reset, network formed and up",
        {"ncp_cookie": f"{NCP_COOKIE:08x}", "reset_reason": 1, "network_state": 2,
         "table_epoch": 7, "nwk_fc_limit": 2011024, "aps_fc_limit": 14024,
         "dirty_writebacks": 0, "dropped_indications": 0})
    read_attr = bytes.fromhex("00" "2a" "00" "0400" "0500")  # ZCL Read Attributes 0x0004,0x0005
    add("aps_data_request_unicast", EP_ZIGBEE, K_REQUEST, APS_DATA_REQUEST, 0x0102,
        aps_data_request(0x1A2B, 1, 1, 0x0104, 0x0000, read_attr),
        "Basic cluster read attributes, APS ACK, NWK address mode",
        {"dst_mode": 2, "dst": "0x1a2b", "dst_ep": 1, "src_ep": 1, "profile": "0x0104",
         "cluster": "0x0000", "tx_options": 1, "radius": 0, "priority": 1})
    add("aps_data_request_source_routed", EP_ZIGBEE, K_REQUEST, APS_DATA_REQUEST, 0x0103,
        aps_data_request(0x8F3E, 1, 1, 0x0104, 0x0006, bytes.fromhex("012b02"),
                         relays=[0x1A2B, 0x3C4D]),
        "On/Off Toggle via relays 0x1a2b then 0x3c4d (path order from the coordinator)",
        {"relays": ["0x1a2b", "0x3c4d"]})
    add("aps_data_request_rsp_accepted", EP_ZIGBEE, K_RESPONSE, APS_DATA_REQUEST, 0x0102,
        b"\x00", "accepted into the APS queue; confirm follows")
    add("aps_data_request_rsp_busy", EP_ZIGBEE, K_RESPONSE, APS_DATA_REQUEST, 0x0103,
        b"\x02", "queue full: BUSY, nothing was sent")
    add("evt_aps_data_confirm", EP_ZIGBEE, K_EVENT, EVT_APS_DATA_CONFIRM, 0x0011,
        aps_data_confirm(0x0102, 0x00, 0x00, 0x37, 0x1A2B, 0x01, 23),
        "APS ACK received after 23 ms", {"req_tid": "0x0102", "zb_status": 0, "aps_counter": 0x37})
    report = bytes.fromhex("18" "05" "0a" "0000" "29" "3409")  # temperature 23.88 C
    add("evt_aps_data_indication", EP_ZIGBEE, K_EVENT, EVT_APS_DATA_INDICATION, 0x0012,
        aps_data_indication(0x8F3E, "00158d0004c5d6e7", 1, 1, 0x0104, 0x0402, report, 0xB4, -61,
                            0x1A2B, 0x5C, 123456, key_source=0),
        "attribute report, NWK secured, source IEEE known",
        {"src_nwk": "0x8f3e", "src_ieee": "00158d0004c5d6e7", "lqi": 180, "rssi": -61,
         "last_hop": "0x1a2b", "flags": 6, "key_source": 0})
    add("nwk_permit_join", EP_ZIGBEE, K_REQUEST, NWK_PERMIT_JOIN, 0x0104,
        struct.pack("<BHB", 254, 0xFFFC, 0x01), "254 s on all routers with TC significance")
    add("evt_device_joined", EP_ZIGBEE, K_EVENT, EVT_DEVICE_JOINED, 0x0013,
        struct.pack("<H", 0x4F21) + ieee_wire("842e14fffe0a0b0c") + struct.pack("<HBBB", 0x1A2B, 0, 0x80, 0),
        "association through router 0x1a2b, admitted")
    add("evt_route_record", EP_ZIGBEE, K_EVENT, EVT_ROUTE_RECORD, 0x0014,
        struct.pack("<H", 0x8F3E) + ieee_wire("00158d0004c5d6e7") + struct.pack("<BHH", 2, 0x3C4D, 0x1A2B),
        "relays in Route Record order (first entry nearest the originator)")
    add("evt_child_poll", EP_ZIGBEE, K_EVENT, EVT_CHILD_POLL, 0x001B,
        struct.pack("<H", 0x4F21) + ieee_wire("842e14fffe0a0b0c") + struct.pack("<BH", 0x01, 3),
        "child 0x4f21 polled; its ACK carried Frame Pending = 1; 3 earlier polls not reported",
        {"nwk": "0x4f21", "ieee": "842e14fffe0a0b0c", "ack_frame_pending": 1, "unreported": 3})
    add("evt_child_poll_unknown", EP_ZIGBEE, K_EVENT, EVT_CHILD_POLL, 0x001C,
        struct.pack("<H", 0x1A2C) + bytes(8) + struct.pack("<BH", 0x02, 0),
        "first report of child 0x1a2c: IEEE unknown (all zero), the radio does not report "
        "its ACK's Frame Pending (2)",
        {"nwk": "0x1a2c", "ieee": None, "ack_frame_pending": 2, "unreported": 0})
    code = SILABS_INSTALL_CODE
    add("sec_add_install_code", EP_ZIGBEE, K_REQUEST, SEC_ADD_INSTALL_CODE, 0x0105,
        ieee_wire("842e14fffe0a0b0c") + struct.pack("<B", len(code)) + code,
        "16-byte install code + CRC (little-endian 0x90D4)")
    derived = aes_mmo(code)
    if derived is not None:
        assert derived == SILABS_INSTALL_KEY, derived.hex()
    add("sec_add_install_code_rsp", EP_ZIGBEE, K_RESPONSE, SEC_ADD_INSTALL_CODE, 0x0105,
        b"\x00" + SILABS_INSTALL_KEY, "derived key returned for the host link-key store",
        {"derived_key": SILABS_INSTALL_KEY.hex()})
    add("evt_fc_checkpoint", EP_ZIGBEE, K_EVENT, EVT_FC_CHECKPOINT, 0x0015,
        struct.pack("<BBII", 1, 1, 2012048, 7), "NWK domain, key seq 1, new durable limit")
    add("tbl_lookup_req", EP_TABLES, K_REQUEST, TBL_LOOKUP, 0x0001,
        struct.pack("<BB", R_DEVICE, 2) + struct.pack("<H", 0x4F21),
        "NCP->host: DEVICE table (0x05) by NWK 0x4f21")
    dev = device_record("842e14fffe0a0b0c", 0x4F21, False,
                        {"key": "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf", "tx_counter": 14024, "rx_counter": 3})
    add("tbl_lookup_rsp", EP_TABLES, K_RESPONSE, TBL_LOOKUP, 0x0001,
        b"\x00" + struct.pack("<I", 41) + dev, "host returns rev 41 device record with link key")
    add("tbl_writeback_evict", EP_TABLES, K_REQUEST, TBL_WRITEBACK, 0x0002,
        struct.pack("<BBI", R_DEVICE, 3, 57) + dev, "NCP evicts a dirty entry; host must fsync first")
    groups = [record(R_APS_GROUP, struct.pack("<HB", 0x0B84, 1) + bytes([242])),
              record(R_APS_GROUP, struct.pack("<HB", 0x1234, 2) + bytes([1, 3]))]
    add("tbl_read_aps_group", EP_TABLES, K_REQUEST, TBL_READ, 0x0003,
        struct.pack("<BHB", R_APS_GROUP, 0, 8),
        "host reads the APS group table (0x0D) from cursor 0, at most 8 entries")
    add("tbl_read_aps_group_rsp", EP_TABLES, K_RESPONSE, TBL_READ, 0x0003,
        b"\x00" + struct.pack("<HB", 0xFFFF, len(groups))
        + b"".join(struct.pack("<IB", 0, 0) + g for g in groups),
        "APS group table (0x0D): GP sink group 0x0B84 with endpoint 242, 0x1234 with 1 and 3")
    add("sec_lock_key_export", EP_ZIGBEE, K_REQUEST, SEC_LOCK_KEY_EXPORT, 0x0106,
        struct.pack("<I", 0x4B434F4C), "confirm word 'LOCK'; key export refused until reboot")
    add("sec_lock_key_export_rsp", EP_ZIGBEE, K_RESPONSE, SEC_LOCK_KEY_EXPORT, 0x0106, b"\x00",
        "locked")
    add("sys_set_uart", EP_SYSTEM, K_REQUEST, SYS_SET_UART, 0x0005,
        struct.pack("<IBH", 921600, 0, 50), "switch to 921600, no RTS/CTS, after 50 ms")
    text = "zrhp: session up, window 16".encode()
    add("evt_log_record", EP_LOG, K_EVENT, LOG_RECORD, 0x0001,
        struct.pack("<IBBH", 5321, 3, 2, 17) + text, "UTF-8 text record (level 3 info)",
        {"time_ms": 5321, "level": 3, "module": 2, "seq": 17, "text": text.decode()})
    add("gp_sink_config_zigpy", EP_ZIGBEE, K_REQUEST, GP_SINK_CONFIG, 0x0107,
        struct.pack("<BBHBHBB", 1, 2, 0x0B84, 0x05, 180, 0x00, 0),
        "NCP sink authoritative; pre-commissioned groupcast to 0x0B84 and "
        "security level 0 like zigpy (decision N1)",
        {"mode": 1, "communication_mode": 2, "group": "0x0b84", "exit_mode": 5,
         "window_s": 180, "security_level": 0, "groupcast_radius": 0})
    add("gp_sink_commissioning_enter", EP_ZIGBEE, K_REQUEST, GP_SINK_COMMISSIONING, 0x0108,
        struct.pack("<BBH", 1, 0x01, 254), "enter, involve proxies, 254 s (zigpy permit)")
    gpd_key = bytes.fromhex("c0c1c2c3c4c5c6c7c8c9cacbcccdcecf")
    sink_rec = gp_sink_record(0x293C9879, 0x02, 0x0B84, gpd_key, 4448)
    add("evt_gp_commissioned", EP_ZIGBEE, K_EVENT, EVT_GP_COMMISSIONED, 0x0016,
        struct.pack("<BIBB", 0, 0x293C9879, 0x02, 0x02) + sink_rec,
        "SrcID 0x293c9879 commissioned; R_GP_SINK record (key present: export allowed)",
        {"record_hex": hx(sink_rec)})
    add("evt_gp_frame_toggle", EP_ZIGBEE, K_EVENT, EVT_GP_FRAME, 0x0017,
        struct.pack("<BBBIBBIBBBHBbBB", 1, 1, 0, 0x293C9879, 0x02, 0x22, 4449, 3, 4, 0x02,
                    0xFFFF, 0xFF, -48, 220, 0),
        "direct GPDF accepted by the NCP sink: Toggle, decrypted, passed through")
    # Remote GP provisioning (7.11): pair SrcID 0x7a6b9879 on the combo light
    # 0x7c87 and retarget its Toggle.
    pairing_config = (struct.pack("<HBBIBBBHBBB", 0x7C87, 0x0A, 0, 0x7A6B9879, 0x02, 1, 0x0D,
                                  0, 0, 3, 4) + gpd_key + struct.pack("<I", 10)
                      + b"\x00" + b"\xfe" + b"\x00" + struct.pack("<BHH", 0, 0, 0)
                      + b"\x00" + b"\x00" + b"\x00")
    add("gp_pairing_configuration", EP_ZIGBEE, K_REQUEST, GP_PAIRING_CONFIGURATION, 0x0109,
        pairing_config,
        "Replace + send GP Pairing, derived groupcast, level 3 individual key, counter 10, "
        "paired endpoints 0xFE (sink derives), no application information",
        {"dst": "0x7c87", "actions": 0x0A, "communication_mode": 1, "flags": 0x0D,
         "paired": 0xFE})
    add("gp_remote_rsp", EP_ZIGBEE, K_RESPONSE, GP_PAIRING_CONFIGURATION, 0x0109, b"\x00\x05",
        "accepted; ZCL TSN 5 (EVT_APS_DATA_CONFIRM and EVT_GP_REMOTE_STATUS follow)",
        {"zcl_tsn": 5})
    toggle_off = gp_translation(0x22, 1, 0x0006, 0x00)
    step_derived = gp_translation(0x32, 1, 0x0008, 0x06, payload_len=0xFE)
    move_to_level = gp_translation(0x21, 2, 0x0008, 0x04, payload=bytes([0x80, 0x0A, 0x00]))
    add("gp_translation_update", EP_ZIGBEE, K_REQUEST, GP_TRANSLATION_UPDATE, 0x010A,
        struct.pack("<HBBIB", 0x7C87, 0, 0, 0x7A6B9879, 3) + toggle_off + step_derived
        + move_to_level,
        "Add: Toggle -> ep 1 Off; Step Up -> ep 1 Step with On/Off, payload derived (0xFE); "
        "On -> ep 2 Move to Level with On/Off 0x80 in 1 s",
        {"dst": "0x7c87", "action": 0, "translations": 3})
    add("gp_send_pairing", EP_ZIGBEE, K_REQUEST, GP_SEND_PAIRING, 0x010B,
        struct.pack("<HBBIBBB", 0xFFFD, 0x01 | 0x08 | 0x10 | 0x40, 0, 0x7A6B9879, 3, 3, 4)
        + ieee_wire("0102030405060708") + struct.pack("<HHBI", 0x7C87, 0, 0x02, 10) + gpd_key
        + struct.pack("<HB", 0, 0),
        "GP Pairing broadcast: AddSink, lightweight unicast to 0x7c87, key and counter",
        {"dst": "0xfffd", "flags": 0x59, "communication_mode": 3})
    add("gp_translation_request", EP_ZIGBEE, K_REQUEST, GP_TRANSLATION_REQUEST, 0x010C,
        struct.pack("<HB", 0x7C87, 0), "Translation Table from index 0 (unicast only)")
    add("gp_remote_sink_table_request", EP_ZIGBEE, K_REQUEST, GP_REMOTE_SINK_TABLE_REQUEST,
        0x010D, struct.pack("<HBBI", 0x7C87, 0xFF, 0, 0x7A6B9879),
        "Sink Table entry of SrcID 0x7a6b9879 (index 0xFF: by GPD ID)")
    add("evt_gp_remote_status", EP_ZIGBEE, K_EVENT, EVT_GP_REMOTE_STATUS, 0x0018,
        struct.pack("<HBBB", 0x7C87, 5, 0x09, 0x00),
        "Default Response SUCCESS to the GP Pairing Configuration with TSN 5")
    tt_records = [gp_translation_record(0x7A6B9879, toggle_off),
                  gp_translation_record(0x7A6B9879, step_derived)]
    add("evt_gp_remote_translation_table", EP_ZIGBEE, K_EVENT, EVT_GP_REMOTE_TRANSLATION_TABLE,
        0x0019,
        struct.pack("<HBBBBBB", 0x7C87, 6, 0x00, 2, 0, 0, len(tt_records)) + b"".join(tt_records),
        "GP Translation Table Response: 2 of 2 entries from index 0 as R_GP_TRANSLATION records",
        {"records_hex": [hx(r) for r in tt_records]})
    # Report 0 of 2 of a multi-sensor GPD: Temperature Measurement server,
    # MeasuredValue (int16) at offset 0, timeout 60 s.
    temperature_report = bytes.fromhex("00013c00080802040000291100")
    add("evt_gp_app_description", EP_ZIGBEE, K_EVENT, EVT_GP_APP_DESCRIPTION, 0x001A,
        struct.pack("<BIBBB", 0, 0x0BADCAFE, 2, 1, len(temperature_report)) + temperature_report,
        "Application Description of SrcID 0x0badcafe: report 0 of 2 (temperature, offset 0)",
        {"report_hex": hx(temperature_report)})
    return out


def build_arq():
    """Normative receiver/sender traces (section 5.9). Times in microseconds."""
    return {
        "assumptions": {"rx_window": 8, "ack_delay_us": 1000, "rto_initial_us": 100000,
                        "processing_time_us": 0, "rto_adaptive": False,
                        "ack_every_frames": 4,
                        "note": "ack/sack/nbad as in link frames; RTO stays at rto_initial_us "
                                "until a timeout doubles it (no RTT samples in these traces)"},
        "scenarios": [
            {"name": "in_order_delayed_ack", "role": "receiver",
             "inputs": [{"t": 0, "data": 0}, {"t": 100, "data": 1}, {"t": 200, "data": 2}],
             "expect": [{"t": 1000, "send": "ACK", "ack": 3, "sack": 0}],
             "delivered": [0, 1, 2]},
            {"name": "gap_then_fill", "role": "receiver",
             "inputs": [{"t": 0, "data": 0}, {"t": 100, "data": 1}, {"t": 300, "data": 3},
                        {"t": 400, "data": 4}, {"t": 600, "data": 2, "retx": True}],
             "expect": [{"t": 300, "send": "NAK", "ack": 2, "sack": 1},
                        {"t": 400, "send": "NAK", "ack": 2, "sack": 3},
                        {"t": 600, "send": "ACK", "ack": 5, "sack": 0}],
             "delivered": [0, 1, 2, 3, 4]},
            {"name": "gap_sender_reaction", "role": "sender",
             "inputs": [{"t": 0, "send_new": [0, 1, 2, 3, 4]},
                        {"t": 300, "recv": "NAK", "ack": 2, "sack": 1},
                        {"t": 400, "recv": "NAK", "ack": 2, "sack": 3},
                        {"t": 600, "recv": "ACK", "ack": 5, "sack": 0}],
             "expect": [{"t": 300, "retransmit": 2, "flags": ["RETX"]}],
             "final": {"base": 5, "unacked": []}},
            {"name": "crc_error_nbad", "role": "receiver",
             "inputs": [{"t": 0, "data": 0}, {"t": 100, "crc_error": True}, {"t": 200, "data": 2},
                        {"t": 400, "data": 1, "retx": True}],
             "expect": [{"t": 100, "send": "NAK", "ack": 1, "sack": 0, "nbad": 1},
                        {"t": 200, "send": "NAK", "ack": 1, "sack": 1, "nbad": 0},
                        {"t": 400, "send": "ACK", "ack": 3, "sack": 0}],
             "delivered": [0, 1, 2]},
            {"name": "crc_error_sender_reaction", "role": "sender",
             "inputs": [{"t": 0, "send_new": [0, 1, 2]},
                        {"t": 100, "recv": "NAK", "ack": 1, "sack": 0, "nbad": 1},
                        {"t": 200, "recv": "NAK", "ack": 1, "sack": 1, "nbad": 0}],
             "expect": [{"t": 100, "retransmit": 1, "flags": ["RETX"]}],
             "final": {"base": 1, "unacked": [1]}},
            {"name": "tail_loss_rto", "role": "sender",
             "inputs": [{"t": 0, "send_new": [0]}, {"t": 100, "send_new": [1]},
                        {"t": 1000, "recv": "ACK", "ack": 1, "sack": 0}],
             "expect": [{"t": 100100, "retransmit": 1, "flags": ["RETX", "POLL"]}],
             "final": {"base": 1, "unacked": [1], "rto_us": 200000}},
            {"name": "duplicate_after_lost_ack", "role": "receiver",
             "inputs": [{"t": 0, "data": 0}, {"t": 100500, "data": 0, "retx": True, "poll": True}],
             "expect": [{"t": 1000, "send": "ACK", "ack": 1, "sack": 0},
                        {"t": 100500, "send": "ACK", "ack": 1, "sack": 0}],
             "delivered": [0], "counters": {"rx_duplicate": 1}},
            {"name": "credit_unblock", "role": "receiver",
             "inputs": [{"t": 0, "data": 0, "endpoint": 4, "ep_seq": 0},
                        {"t": 100, "data": 1, "endpoint": 4, "ep_seq": 1},
                        {"t": 5000, "consumer_frees": {"endpoint": 4, "frames": 1}}],
             "initial_credit_edge": {"4": 2},
             "expect": [{"t": 1000, "send": "ACK", "ack": 2, "sack": 0},
                        {"t": 5000, "send": "ACK", "ack": 2, "sack": 0, "credits": {"4": 3}}]},
        ],
    }


def build_backup():
    ocb_in = {
        "metadata": {
            "version": 1, "format": "zigpy/open-coordinator-backup", "source": "bellows@0.0.0-synthetic",
            "internal": {
                "creation_time": "2026-11-01T10:00:00+00:00",
                "node": {"ieee": "f4b3b1fffe1a2b3c", "nwk": "0000", "type": "coordinator"},
                "network": {"tc_link_key": {"key": TCLK_DEFAULT.hex(), "frame_counter": 3000},
                            "tc_address": "f4b3b1fffe1a2b3c", "nwk_manager": "0000"},
                "link_key_seqs": {"0017880102a1b2c3": 0, "842e14fffe0a0b0c": 0},
                "tx_power": 8, "ezspVersion": 13,
            },
        },
        "stack_specific": {"ezsp": {"hashed_tclk": "0f1e2d3c4b5a69788796a5b4c3d2e1f0"}},
        "coordinator_ieee": "f4b3b1fffe1a2b3c",
        "pan_id": "4c2a",
        "extended_pan_id": "a1b2c3d4e5f60718",
        "nwk_update_id": 2,
        "security_level": 5,
        "channel": 20,
        "channel_mask": [15, 20, 25],
        "network_key": {"key": "8b5d1f3c6a7e9d2b4f0a1c3e5d7f9b2a", "sequence_number": 1,
                        "frame_counter": 2000000},
        "devices": [
            {"ieee_address": "0017880102a1b2c3", "nwk_address": "1a2b", "is_child": False,
             "link_key": {"key": "11223344556677881122334455667788", "tx_counter": 0, "rx_counter": 57}},
            {"ieee_address": "00158d0004c5d6e7", "nwk_address": "8f3e", "is_child": True},
            {"ieee_address": "842e14fffe0a0b0c", "nwk_address": None, "is_child": False,
             "link_key": {"key": "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf", "tx_counter": 12, "rx_counter": 3}},
            {"ieee_address": "000d6f000e1f2a3b", "nwk_address": "abc"},
        ],
    }
    nwk_margin, aps_margin, table_epoch = 10000, 10000, 7
    recs = ocb_to_records(ocb_in)
    state = ncp_apply_restore(recs, nwk_margin, aps_margin)
    exported = ncp_export_records(state, table_epoch)
    ocb_out = records_to_ocb(exported, source="zigbee-rs-reference@spec-v1",
                             creation_time="2026-11-15T12:00:00+00:00", passthrough_stack_specific={})
    # Round trip: the exported backup must convert back to the same records except counters.
    reimported = ocb_to_records(ocb_out)
    again = ncp_apply_restore(reimported, 0, 0)
    assert again["nwk_fc_start"] == state["nwk_fc_limit"]
    assert again["aps_fc_start"] == state["aps_fc_limit"]
    assert [r for r in reimported if r[0] == R_DEVICE] == [r for r in exported if r[0] == R_DEVICE]
    begin = struct.pack("<BIII", 0x01, nwk_margin, aps_margin, len(recs))
    put = struct.pack("<H", 1) + b"".join(recs)
    commit_rsp = struct.pack("<BIIHHH", 0, state["nwk_fc_start"], state["aps_fc_start"], 2, 0, 2)
    names = {R_NODE: "NODE", R_NETWORK: "NETWORK", R_NETWORK_KEY: "NETWORK_KEY",
             R_TC_LINK_KEY: "TC_LINK_KEY", R_DEVICE: "DEVICE", R_EZSP_HASHED_TCLK: "EZSP_HASHED_TCLK",
             R_RS_PARAMS: "RS_PARAMS", R_END: "END"}
    return {
        "note": "Synthetic EZSP-origin backup restored into a zigbee-rs NCP and exported again. "
                "Keys and addresses are made up; do not use them on a real network.",
        "input_ocb": ocb_in,
        "restore": {
            "nwk_margin": nwk_margin, "aps_margin": aps_margin, "allow_ieee_change": True,
            "records": [{"type": names[r[0]], "hex": hx(r)} for r in recs],
            "messages": {
                "begin": hx(message(K_REQUEST, BKP_RESTORE_BEGIN, 0x0201, begin)),
                "begin_rsp": hx(message(K_RESPONSE, BKP_RESTORE_BEGIN, 0x0201,
                                        struct.pack("<BHH", 0, 1, 16))),
                "put": hx(message(K_REQUEST, BKP_RESTORE_PUT, 0x0202, put)),
                "put_rsp": hx(message(K_RESPONSE, BKP_RESTORE_PUT, 0x0202,
                                      struct.pack("<BHB", 0, len(recs), 0))),
                "commit": hx(message(K_REQUEST, BKP_RESTORE_COMMIT, 0x0203, struct.pack("<H", 1))),
                "commit_rsp": hx(message(K_RESPONSE, BKP_RESTORE_COMMIT, 0x0203, commit_rsp)),
            },
            "ncp_state": {"nwk_fc_start": state["nwk_fc_start"], "nwk_fc_limit": state["nwk_fc_limit"],
                          "aps_fc_start": state["aps_fc_start"], "aps_fc_limit": state["aps_fc_limit"],
                          "children": 2, "link_keys_on_ncp": 2, "link_keys_host_backed": 0},
        },
        "export": {
            "table_epoch": table_epoch,
            "records": [{"type": names[r[0]], "hex": hx(r)} for r in exported],
        },
        "output_ocb": ocb_out,
    }


def build_ezsp_hashed():
    seed = bytes.fromhex("0f1e2d3c4b5a69788796a5b4c3d2e1f0")
    out = {"status": "on-air order per the GSDK 4.5.0 source (zb_sec_man_derive_key, "
                     "TC_HASHED_LINK_KEY, over an EmberEUI64); hardware key match is gate H5",
           "algorithm": "HMAC with AES-MMO (Zigbee keyed hash), key = hashed_tclk, "
                        "message = device EUI64 in on-air (little-endian) order; the "
                        "big-endian column is kept as the negative case",
           "vectors": []}
    for ieee in ("0017880102a1b2c3", "842e14fffe0a0b0c"):
        le = hmac_mmo(seed, ieee_wire(ieee))
        be = hmac_mmo(seed, bytes.fromhex(ieee))
        out["vectors"].append({"hashed_tclk": seed.hex(), "ieee": ieee,
                               "candidate_eui64_on_air_order": le.hex() if le else None,
                               "candidate_eui64_big_endian": be.hex() if be else None})
    return out


def build_z2m_import():
    ocb = {
        "metadata": {"format": "zigpy/open-coordinator-backup", "version": 1,
                     "source": "zigbee-herdsman@0.0.0-synthetic",
                     "internal": {"date": "2026-11-02T08:00:00.000Z", "ezspVersion": 13}},
        "stack_specific": {"ezsp": {"hashed_tclk": "0f1e2d3c4b5a69788796a5b4c3d2e1f0"}},
        "coordinator_ieee": "f4b3b1fffe1a2b3c",
        "pan_id": "a62",
        "extended_pan_id": "a1b2c3d4e5f60718",
        "nwk_update_id": 0,
        "security_level": 5,
        "channel": 25,
        "channel_mask": [25],
        "network_key": {"key": "8b5d1f3c6a7e9d2b4f0a1c3e5d7f9b2a", "sequence_number": 0,
                        "frame_counter": 77000},
        "devices": [
            {"nwk_address": "abc", "ieee_address": "000d6f000e1f2a3b", "is_child": True},
            {"nwk_address": "1a2b", "ieee_address": "0017880102a1b2c3", "is_child": False,
             "link_key": {"key": "11223344556677881122334455667788", "rx_counter": 7,
                          "tx_counter": 5}},
        ],
    }
    normalized = json.loads(json.dumps(ocb))
    normalized["pan_id"] = normalized["pan_id"].zfill(4)
    recs = ocb_to_records(normalized)
    has_tc_counter = "tc_link_key" in ocb["metadata"]["internal"].get("network", {})
    aps_margin = 10000 if has_tc_counter else APS_UNKNOWN_TCLK_MARGIN
    state = ncp_apply_restore(recs, 10000, aps_margin)
    names = {R_NODE: "NODE", R_NETWORK: "NETWORK", R_NETWORK_KEY: "NETWORK_KEY",
             R_TC_LINK_KEY: "TC_LINK_KEY", R_DEVICE: "DEVICE", R_EZSP_HASHED_TCLK: "EZSP_HASHED_TCLK",
             R_END: "END"}
    return {
        "note": "Z2M-style backup: unpadded pan_id/nwk, internal.date, no TCLK counter. "
                "The host pads hex fields and, because the TCLK counter is unknown, uses "
                "aps_margin = 2^20.",
        "input_ocb": ocb,
        "nwk_margin": 10000, "aps_margin": aps_margin,
        "records": [{"type": names[r[0]], "hex": hx(r)} for r in recs],
        "ncp_state": {"nwk_fc_start": state["nwk_fc_start"], "aps_fc_start": state["aps_fc_start"]},
    }


def outputs() -> dict[str, object]:
    crc, cobs = build_crc_cobs()
    return {"crc32c.json": crc, "cobs.json": cobs, "link_frames.json": build_link(),
            "messages.json": build_messages(), "arq_scenarios.json": build_arq(),
            "ocb_roundtrip.json": build_backup(), "ocb_z2m_import.json": build_z2m_import(),
            "ezsp_hashed_tclk.json": build_ezsp_hashed()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    failed = False
    for name, data in outputs().items():
        text = json.dumps(data, indent=2, sort_keys=False) + "\n"
        path = HERE / name
        if args.check:
            if not path.exists() or path.read_text() != text:
                print(f"stale: {name}", file=sys.stderr)
                failed = True
        else:
            path.write_text(text)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
