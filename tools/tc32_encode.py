#!/usr/bin/env python3
"""TC32 instruction encoder - based on modern-tc32 LLVM code emitter."""

import struct, sys

def tc32_mov_rr(dst, src):
    """MOV Rd, Rs (low-low or high regs)"""
    if dst < 8 and src < 8:
        return 0xEC00 | (src << 3) | dst
    lo = 0
    if dst & 8: lo |= 0x80
    if src & 8: lo |= 0x40
    lo |= (src & 7) << 3
    lo |= dst & 7
    return 0x0600 | lo

def tc32_mov_i8(dst, imm):
    """MOVS Rd, #imm8"""
    assert dst < 8 and 0 <= imm <= 255
    return ((0xA0 + dst) << 8) | imm

def tc32_add_i8(dst, imm):
    """ADDS Rd, Rd, #imm8 (same src/dst)"""
    assert dst < 8 and 0 <= imm <= 255
    return ((0xB0 + dst) << 8) | imm

def tc32_sub_i8(dst, imm):
    """SUBS Rd, Rd, #imm8 (same src/dst)"""
    assert dst < 8 and 0 <= imm <= 255
    return ((0xB8 + dst) << 8) | imm

def tc32_add_rrr(dst, src, rhs):
    """ADDS Rd, Rs, Rn"""
    assert dst < 8 and src < 8 and rhs < 8
    enc = dst | (src << 3) | (rhs << 6)
    return ((0xE8 + (enc >> 8)) << 8) | (enc & 0xFF)

def tc32_sub_rrr(dst, src, rhs):
    """SUBS Rd, Rs, Rn"""
    assert dst < 8 and src < 8 and rhs < 8
    enc = dst | (src << 3) | (rhs << 6)
    return ((0xEA + (enc >> 8)) << 8) | (enc & 0xFF)

def tc32_add_ri3(dst, src, imm):
    """ADDS Rd, Rs, #imm3"""
    assert dst < 8 and src < 8 and 0 <= imm <= 7
    lo = (src << 3) | dst | ((imm & 3) << 6)
    hi = 0xEC + (imm >> 2)
    return (hi << 8) | lo

def tc32_sub_ri3(dst, src, imm):
    """SUBS Rd, Rs, #imm3"""
    assert dst < 8 and src < 8 and 0 <= imm <= 7
    enc = dst | (src << 3) | (imm << 6)
    return ((0xEE + (enc >> 8)) << 8) | (enc & 0xFF)

def tc32_lsl_i(dst, src, imm):
    """LSLS Rd, Rs, #imm5"""
    assert dst < 8 and src < 8 and 0 <= imm <= 31
    return 0xF000 | ((imm >> 2) << 8) | ((imm & 3) << 6) | (src << 3) | dst

def tc32_lsr_i(dst, src, imm):
    """LSRS Rd, Rs, #imm5 (imm=32 encoded as 0)"""
    assert dst < 8 and src < 8 and 0 <= imm <= 32
    enc_imm = 0 if imm == 32 else imm
    return 0xF800 | ((enc_imm >> 2) << 8) | ((enc_imm & 3) << 6) | (src << 3) | dst

def tc32_asr_i(dst, src, imm):
    """ASRS Rd, Rs, #imm5"""
    assert dst < 8 and src < 8 and 0 <= imm <= 32
    enc_imm = 0 if imm == 32 else imm
    return 0xE000 | ((enc_imm >> 2) << 8) | ((enc_imm & 3) << 6) | (src << 3) | dst

def tc32_and(dst, rhs):
    """ANDS Rd, Rn"""
    assert dst < 8 and rhs < 8
    return 0x0000 | (rhs << 3) | dst

def tc32_orr(dst, rhs):
    """ORRS Rd, Rn"""
    assert dst < 8 and rhs < 8
    return 0x0300 | (rhs << 3) | dst

def tc32_eor(dst, rhs):
    """EORS Rd, Rn"""
    assert dst < 8 and rhs < 8
    return 0x0040 | (rhs << 3) | dst

def tc32_bic(dst, rhs):
    """BICS Rd, Rn"""
    assert dst < 8 and rhs < 8
    return 0x0380 | (rhs << 3) | dst

def tc32_mvn(dst, src):
    """MVNS Rd, Rs"""
    assert dst < 8 and src < 8
    return 0x03C0 | (src << 3) | dst

def tc32_mul(dst, src):
    """MULS Rd, Rs"""
    assert dst < 8 and src < 8
    return 0x0340 | (src << 3) | dst

def tc32_cmp_i8(src, imm):
    """CMP Rn, #imm8"""
    assert src < 8 and 0 <= imm <= 255
    return ((0xA8 + src) << 8) | imm

def tc32_cmp_rr(lhs, rhs):
    """CMP Rn, Rm (low)"""
    assert lhs < 8 and rhs < 8
    return 0x0280 | (rhs << 3) | lhs

def tc32_str_i(rt, rn, imm):
    """STR Rt, [Rn, #imm] (word, imm in bytes, must be 4-aligned)"""
    assert rt < 8 and rn < 8 and imm >= 0 and imm <= 124 and (imm & 3) == 0
    return 0x5000 | ((imm >> 2) << 6) | (rn << 3) | rt

def tc32_ldr_i(rt, rn, imm):
    """LDR Rt, [Rn, #imm] (word, imm in bytes, must be 4-aligned)"""
    assert rt < 8 and rn < 8 and imm >= 0 and imm <= 124 and (imm & 3) == 0
    return 0x5800 | ((imm >> 2) << 6) | (rn << 3) | rt

def tc32_strb_i(rt, rn, imm):
    """STRB Rt, [Rn, #imm]"""
    assert rt < 8 and rn < 8 and 0 <= imm <= 31
    return 0x4000 | (imm << 6) | (rn << 3) | rt

def tc32_ldrb_i(rt, rn, imm):
    """LDRB Rt, [Rn, #imm]"""
    assert rt < 8 and rn < 8 and 0 <= imm <= 31
    return 0x4800 | (imm << 6) | (rn << 3) | rt

def tc32_strh_i(rt, rn, imm):
    """STRH Rt, [Rn, #imm] (halfword, imm in bytes, must be 2-aligned)"""
    assert rt < 8 and rn < 8 and imm >= 0 and (imm & 1) == 0 and imm <= 62
    return 0x2000 | ((imm >> 1) << 6) | (rn << 3) | rt

def tc32_ldrh_i(rt, rn, imm):
    """LDRH Rt, [Rn, #imm]"""
    assert rt < 8 and rn < 8 and imm >= 0 and (imm & 1) == 0 and imm <= 62
    return 0x2800 | ((imm >> 1) << 6) | (rn << 3) | rt

def tc32_str_r(rt, rn, rm):
    """STR Rt, [Rn, Rm]"""
    assert rt < 8 and rn < 8 and rm < 8
    return 0x1000 | (rm << 6) | (rn << 3) | rt

def tc32_ldr_r(rt, rn, rm):
    """LDR Rt, [Rn, Rm]"""
    assert rt < 8 and rn < 8 and rm < 8
    return 0x1800 | (rm << 6) | (rn << 3) | rt

def tc32_strb_r(rt, rn, rm):
    """STRB Rt, [Rn, Rm]"""
    assert rt < 8 and rn < 8 and rm < 8
    return 0x1400 | (rm << 6) | (rn << 3) | rt

def tc32_ldrb_r(rt, rn, rm):
    """LDRB Rt, [Rn, Rm]"""
    assert rt < 8 and rn < 8 and rm < 8
    return 0x1C00 | (rm << 6) | (rn << 3) | rt

def tc32_ldr_sp(rt, imm):
    """LDR Rt, [SP, #imm] (imm in bytes, 4-aligned, 0..1020)"""
    assert rt < 8 and imm >= 0 and imm <= 1020 and (imm & 3) == 0
    return ((0x38 + rt) << 8) | (imm >> 2)

def tc32_str_sp(rt, imm):
    """STR Rt, [SP, #imm]"""
    assert rt < 8 and imm >= 0 and imm <= 1020 and (imm & 3) == 0
    return ((0x30 + rt) << 8) | (imm >> 2)

def tc32_ldr_pc(rt, imm):
    """LDR Rt, [PC, #imm] (pc-relative literal load, imm in bytes, 4-aligned)"""
    assert rt < 8 and imm >= 0 and imm <= 1020 and (imm & 3) == 0
    return ((0x08 + rt) << 8) | (imm >> 2)

def tc32_adr(rt, imm):
    """ADR Rt, #imm (add PC)"""
    assert rt < 8 and imm >= 0 and imm <= 1020 and (imm & 3) == 0
    return 0xA000 | (rt << 8) | (imm >> 2)

def tc32_push(regs, lr=False):
    """PUSH {regs} / PUSH {regs, LR}"""
    mask = 0
    for r in regs:
        assert 0 <= r <= 7
        mask |= 1 << r
    base = 0x6500 if lr else 0x6400
    return base | mask

def tc32_pop(regs, pc=False):
    """POP {regs} / POP {regs, PC}"""
    mask = 0
    for r in regs:
        assert 0 <= r <= 7
        mask |= 1 << r
    base = 0x6D00 if pc else 0x6C00
    return base | mask

def tc32_add_sp_i(imm_words):
    """ADD SP, SP, #imm*4"""
    assert 0 <= imm_words <= 127
    return 0x6000 | imm_words

def tc32_sub_sp_i(imm_words):
    """SUB SP, SP, #imm*4"""
    assert 0 <= imm_words <= 127
    return 0x6000 | 0x0080 | imm_words

def tc32_add_r_sp(dst, imm_words):
    """ADD Rd, SP, #imm*4"""
    assert dst < 8 and 0 <= imm_words <= 255
    return ((0x78 + dst) << 8) | imm_words

def tc32_bx(reg):
    """BX Reg (branch exchange / indirect jump)"""
    return 0x0700 | (reg << 3)

def tc32_add_hirr(dst, src):
    """ADD Rd, Rs (high reg add, like ADD SP, Rn)"""
    lo = 0
    if dst & 8: lo |= 0x80
    if src & 8: lo |= 0x40
    lo |= (src & 7) << 3
    lo |= dst & 7
    return 0x0400 | lo

def tc32_cmp_hir(lhs, rhs):
    """CMP Rn, Rm (high reg)"""
    lo = 0
    if lhs & 8: lo |= 0x80
    lo |= (rhs & 0xF) << 3
    lo |= lhs & 7
    return 0x0500 | lo

def tc32_tchangei(reg):
    """TCHANGEI Rn (tc32 mode change - MCSR)"""
    return 0x6BC0 | reg

def tc32_mrss(reg):
    """MRSS Rn (read special status)"""
    return 0x6BD8 | reg

def tc32_mssr(reg):
    """MSSR Rn (write special status)"""
    return 0x6BD0 | reg

def tc32_reti():
    """RETI (return from interrupt)"""
    return 0x6900

def tc32_tj(pc, target):
    """TJ target (unconditional branch, 2 bytes)"""
    offset = target - pc - 4
    assert offset % 2 == 0
    enc = offset >> 1
    assert -1024 <= enc <= 1023, f"tj out of range: enc={enc}"
    return 0x8000 | (enc & 0x7FF)

def tc32_tjcc(pc, target, cc):
    """Conditional branch (2 bytes)"""
    cc_map = {'eq': 0xC000, 'ne': 0xC100, 'hs': 0xC200, 'lo': 0xC300,
              'mi': 0xC400, 'pl': 0xC500, 'vs': 0xC600, 'vc': 0xC700,
              'hi': 0xC800, 'ls': 0xC900, 'ge': 0xCA00, 'lt': 0xCB00,
              'gt': 0xCC00, 'le': 0xCD00}
    base = cc_map[cc]
    offset = target - pc - 4
    assert offset % 2 == 0
    enc = offset >> 1
    assert -128 <= enc <= 127, f"conditional branch out of range: enc={enc}"
    return base | (enc & 0xFF)

def tc32_bl(pc, target):
    """BL target (branch-and-link, 4 bytes) → returns (lo16, hi16) for LE emission"""
    offset = target - pc - 4
    assert offset % 2 == 0
    enc = offset >> 1
    assert -(1 << 21) <= enc < (1 << 21), f"bl out of range"
    enc_imm = enc & 0x3FFFFF
    lo16 = 0x9000 | ((enc_imm >> 11) & 0x7FF)
    hi16 = 0x9800 | (enc_imm & 0x7FF)
    # The base is 0x98009000 with imm fields, but let's compute properly:
    # Bits32 = 0x98009000 | ((enc_imm >> 11) & 0x7FF) | ((enc_imm & 0x7FF) << 16)
    # In LE bytes: lo_word first, hi_word second
    return lo16, hi16

def tc32_nop():
    """NOP (tc32 encoding: MOV r0,r0 via low-low = 0xEC00)"""
    # Low-low MOV r0,r0 = 0xEC00
    return 0xEC00

# Verify against original firmware
print("=== Verification against original firmware ===")
# Offset 0x00: tj to 0x50
hw = tc32_tj(0, 0x50)
print(f"tj @0x00 -> 0x50: 0x{hw:04X} (expect 0x8026)")

# Offset 0x10: tj to 0x1B0
hw10 = tc32_tj(0x10, 0x1B0)
print(f"tj @0x10 -> 0x1B0: 0x{hw10:04X} (expect 0x80CE)")

# Check what tc32 NOP should be - original has 0x06C0 at 0x28
# 0x06C0 = 0x0600 | 0xC0 — that's MOV r0, r0 (high-reg encoding form!)
# In encodeTC32MOVrr: Dst<8, Src<8 → return 0xEC00 (low-low form)
# BUT with Dst=0, Src>=8 it would use 0x0600 form
# 0x06C0 = 0x0600 | 0xC0 = 0x0600 | bit7(dst>=8)=1, bit6(src>=8)=1, src_lo=0<<3, dst_lo=0
# That means dst_hi=1→R8, src_hi=1→R8: MOV R8, R8 — that IS a NOP!
print(f"\nOriginal NOP at 0x28: 0x06C0 = MOV R8, R8 (high-reg NOP)")
hw_nop = tc32_mov_rr(8, 8)
print(f"tc32_mov_rr(8, 8) = 0x{hw_nop:04X}")

# 0x46C0 at 0x92 would be standard Thumb MOV R8, R8 — but what is it in tc32?
# In tc32, that encoding doesn't map to MOV. Let's check what 0x46C0 IS in tc32...
# Actually, the firmware at 0x92 with c046 may be data, not code.
