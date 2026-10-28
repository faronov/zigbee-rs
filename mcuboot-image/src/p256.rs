//! Compact verify-only ECDSA over NIST P-256 (secp256r1).
//!
//! Sized for a 16 KiB bootloader: one generic Montgomery multiplier serves
//! both the field prime `p` and the group order `n`, points use Jacobian
//! coordinates with the `a = -3` doubling formula, and `u1·G + u2·Q` is a
//! single Shamir double-scalar ladder. Inversion is Fermat exponentiation.
//!
//! Every input — image digest, signature, public key — is public, so the
//! arithmetic is deliberately *not* constant time. This module must never be
//! used with secret scalars (signing, key agreement).

/// Eight little-endian 32-bit limbs.
type Limbs = [u32; 8];

struct Modulus {
    m: Limbs,
    /// `-m⁻¹ mod 2³²`.
    m_inv: u32,
    /// `R² mod m` with `R = 2²⁵⁶`.
    r2: Limbs,
}

const P: Modulus = Modulus {
    m: [
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0001,
        0xFFFF_FFFF,
    ],
    m_inv: 0x0000_0001,
    r2: [
        0x0000_0003,
        0x0000_0000,
        0xFFFF_FFFF,
        0xFFFF_FFFB,
        0xFFFF_FFFE,
        0xFFFF_FFFF,
        0xFFFF_FFFD,
        0x0000_0004,
    ],
};

const N: Modulus = Modulus {
    m: [
        0xFC63_2551,
        0xF3B9_CAC2,
        0xA717_9E84,
        0xBCE6_FAAD,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0x0000_0000,
        0xFFFF_FFFF,
    ],
    m_inv: 0xEE00_BC4F,
    r2: [
        0xBE79_EEA2,
        0x8324_4C95,
        0x49BD_6FA6,
        0x4699_799C,
        0x2B6B_EC59,
        0x2845_B239,
        0xF3D9_5620,
        0x66E1_2D94,
    ],
};

const B: Limbs = [
    0x27D2_604B,
    0x3BCE_3C3E,
    0xCC53_B0F6,
    0x651D_06B0,
    0x7698_86BC,
    0xB3EB_BD55,
    0xAA3A_93E7,
    0x5AC6_35D8,
];
const GX: Limbs = [
    0xD898_C296,
    0xF4A1_3945,
    0x2DEB_33A0,
    0x7703_7D81,
    0x63A4_40F2,
    0xF8BC_E6E5,
    0xE12C_4247,
    0x6B17_D1F2,
];
const GY: Limbs = [
    0x37BF_51F5,
    0xCBB6_4068,
    0x6B31_5ECE,
    0x2BCE_3357,
    0x7C0F_9E16,
    0x8EE7_EB4A,
    0xFE1A_7F9B,
    0x4FE3_42E2,
];
const ZERO: Limbs = [0; 8];
const ONE: Limbs = [1, 0, 0, 0, 0, 0, 0, 0];

fn from_be(bytes: &[u8; 32]) -> Limbs {
    let mut limbs = ZERO;
    for (index, limb) in limbs.iter_mut().enumerate() {
        let at = 28 - 4 * index;
        *limb = u32::from_be_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]]);
    }
    limbs
}

fn is_zero(a: &Limbs) -> bool {
    a.iter().all(|limb| *limb == 0)
}

/// `a < b`.
fn less(a: &Limbs, b: &Limbs) -> bool {
    for index in (0..8).rev() {
        if a[index] != b[index] {
            return a[index] < b[index];
        }
    }
    false
}

/// `a -= b`, returning the borrow.
fn sub_in_place(a: &mut Limbs, b: &Limbs) -> bool {
    let mut borrow = 0u64;
    for index in 0..8 {
        let difference = (a[index] as u64)
            .wrapping_sub(b[index] as u64)
            .wrapping_sub(borrow);
        a[index] = difference as u32;
        borrow = (difference >> 63) & 1;
    }
    borrow != 0
}

/// `a += b`, returning the carry.
fn add_in_place(a: &mut Limbs, b: &Limbs) -> bool {
    let mut carry = 0u64;
    for index in 0..8 {
        let sum = a[index] as u64 + b[index] as u64 + carry;
        a[index] = sum as u32;
        carry = sum >> 32;
    }
    carry != 0
}

impl Modulus {
    #[inline(never)]
    fn add(&self, a: &Limbs, b: &Limbs) -> Limbs {
        let mut sum = *a;
        if add_in_place(&mut sum, b) || !less(&sum, &self.m) {
            sub_in_place(&mut sum, &self.m);
        }
        sum
    }

    #[inline(never)]
    fn sub(&self, a: &Limbs, b: &Limbs) -> Limbs {
        let mut difference = *a;
        if sub_in_place(&mut difference, b) {
            add_in_place(&mut difference, &self.m);
        }
        difference
    }

    /// Montgomery product `a·b·R⁻¹ mod m` (CIOS) for `a, b < m`.
    #[inline(never)]
    fn mul(&self, a: &Limbs, b: &Limbs) -> Limbs {
        let mut t = [0u32; 10];
        for &word in b.iter() {
            let mut carry = 0u64;
            for index in 0..8 {
                let x = t[index] as u64 + a[index] as u64 * word as u64 + carry;
                t[index] = x as u32;
                carry = x >> 32;
            }
            let x = t[8] as u64 + carry;
            t[8] = x as u32;
            t[9] = (x >> 32) as u32;

            let q = t[0].wrapping_mul(self.m_inv) as u64;
            let x = t[0] as u64 + q * self.m[0] as u64;
            let mut carry = x >> 32;
            for index in 1..8 {
                let x = t[index] as u64 + q * self.m[index] as u64 + carry;
                t[index - 1] = x as u32;
                carry = x >> 32;
            }
            let x = t[8] as u64 + carry;
            t[7] = x as u32;
            t[8] = t[9] + (x >> 32) as u32;
        }
        let mut result = ZERO;
        result.copy_from_slice(&t[..8]);
        if t[8] != 0 || !less(&result, &self.m) {
            sub_in_place(&mut result, &self.m);
        }
        result
    }

    fn enter_mont(&self, a: &Limbs) -> Limbs {
        self.mul(a, &self.r2)
    }

    fn leave_mont(&self, a: &Limbs) -> Limbs {
        self.mul(a, &ONE)
    }

    /// Montgomery-domain inverse `a⁻¹` by Fermat: `a^(m-2)`.
    #[inline(never)]
    fn invert(&self, a: &Limbs) -> Limbs {
        let mut exponent = self.m;
        // Both moduli are odd with a low limb far above 2: no borrow.
        exponent[0] -= 2;
        let mut result = self.enter_mont(&ONE);
        for bit in (0..256).rev() {
            result = self.mul(&result, &result);
            if (exponent[bit / 32] >> (bit % 32)) & 1 != 0 {
                result = self.mul(&result, a);
            }
        }
        result
    }
}

/// Jacobian point in the Montgomery domain; `z == 0` is infinity.
#[derive(Clone, Copy)]
struct Point {
    x: Limbs,
    y: Limbs,
    z: Limbs,
}

impl Point {
    const INFINITY: Self = Self {
        x: ZERO,
        y: ZERO,
        z: ZERO,
    };

    #[inline(never)]
    fn double(&self) -> Self {
        if is_zero(&self.z) {
            return *self;
        }
        let f = &P;
        let delta = f.mul(&self.z, &self.z);
        let gamma = f.mul(&self.y, &self.y);
        let beta = f.mul(&self.x, &gamma);
        let t = f.mul(&f.sub(&self.x, &delta), &f.add(&self.x, &delta));
        let alpha = f.add(&f.add(&t, &t), &t);
        let beta2 = f.add(&beta, &beta);
        let beta4 = f.add(&beta2, &beta2);
        let beta8 = f.add(&beta4, &beta4);
        let x = f.sub(&f.mul(&alpha, &alpha), &beta8);
        let yz = f.add(&self.y, &self.z);
        let z = f.sub(&f.sub(&f.mul(&yz, &yz), &gamma), &delta);
        let gamma_sq = f.mul(&gamma, &gamma);
        let g2 = f.add(&gamma_sq, &gamma_sq);
        let g4 = f.add(&g2, &g2);
        let g8 = f.add(&g4, &g4);
        let y = f.sub(&f.mul(&alpha, &f.sub(&beta4, &x)), &g8);
        Self { x, y, z }
    }

    #[inline(never)]
    fn add(&self, other: &Self) -> Self {
        if is_zero(&self.z) {
            return *other;
        }
        if is_zero(&other.z) {
            return *self;
        }
        let f = &P;
        let z1z1 = f.mul(&self.z, &self.z);
        let z2z2 = f.mul(&other.z, &other.z);
        let u1 = f.mul(&self.x, &z2z2);
        let u2 = f.mul(&other.x, &z1z1);
        let s1 = f.mul(&f.mul(&self.y, &other.z), &z2z2);
        let s2 = f.mul(&f.mul(&other.y, &self.z), &z1z1);
        let h = f.sub(&u2, &u1);
        let r = f.sub(&s2, &s1);
        if is_zero(&h) {
            return if is_zero(&r) {
                self.double()
            } else {
                Self::INFINITY
            };
        }
        let hh = f.mul(&h, &h);
        let hhh = f.mul(&h, &hh);
        let v = f.mul(&u1, &hh);
        let x = f.sub(&f.sub(&f.mul(&r, &r), &hhh), &f.add(&v, &v));
        let y = f.sub(&f.mul(&r, &f.sub(&v, &x)), &f.mul(&s1, &hhh));
        let z = f.mul(&f.mul(&self.z, &other.z), &h);
        Self { x, y, z }
    }
}

/// A validated P-256 public key (affine point on the curve, not infinity).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifyingKey {
    x: Limbs,
    y: Limbs,
}

impl VerifyingKey {
    /// Build a key from big-endian affine `x || y` (64 bytes, the SEC1
    /// uncompressed encoding without its `0x04` prefix). Returns `None`
    /// unless the point is on the curve.
    pub fn from_xy(xy: &[u8; 64]) -> Option<Self> {
        let mut x_bytes = [0u8; 32];
        let mut y_bytes = [0u8; 32];
        x_bytes.copy_from_slice(&xy[..32]);
        y_bytes.copy_from_slice(&xy[32..]);
        let x = from_be(&x_bytes);
        let y = from_be(&y_bytes);
        if !less(&x, &P.m) || !less(&y, &P.m) {
            return None;
        }
        // y² = x³ − 3x + b
        let f = &P;
        let xm = f.enter_mont(&x);
        let ym = f.enter_mont(&y);
        let lhs = f.mul(&ym, &ym);
        let x3 = f.mul(&f.mul(&xm, &xm), &xm);
        let three_x = f.add(&f.add(&xm, &xm), &xm);
        let rhs = f.add(&f.sub(&x3, &three_x), &f.enter_mont(&B));
        (lhs == rhs).then_some(Self { x, y })
    }

    /// Verify an ECDSA signature `(r, s)` (big-endian scalars) over a
    /// 32-byte message digest.
    #[inline(never)]
    pub fn verify_prehash(&self, digest: &[u8; 32], r: &[u8; 32], s: &[u8; 32]) -> bool {
        let n = &N;
        let r = from_be(r);
        let s = from_be(s);
        if is_zero(&r) || is_zero(&s) || !less(&r, &n.m) || !less(&s, &n.m) {
            return false;
        }
        let mut z = from_be(digest);
        if !less(&z, &n.m) {
            sub_in_place(&mut z, &n.m);
        }
        // w·R = (s·R)^(n-2); then mul(x, w·R) = x·w for plain x.
        let w = n.invert(&n.enter_mont(&s));
        let u1 = n.mul(&z, &w);
        let u2 = n.mul(&r, &w);

        let f = &P;
        let one = f.enter_mont(&ONE);
        let g = Point {
            x: f.enter_mont(&GX),
            y: f.enter_mont(&GY),
            z: one,
        };
        let q = Point {
            x: f.enter_mont(&self.x),
            y: f.enter_mont(&self.y),
            z: one,
        };
        let gq = g.add(&q);
        let mut acc = Point::INFINITY;
        for bit in (0..256).rev() {
            acc = acc.double();
            let b1 = (u1[bit / 32] >> (bit % 32)) & 1 != 0;
            let b2 = (u2[bit / 32] >> (bit % 32)) & 1 != 0;
            acc = match (b1, b2) {
                (true, true) => acc.add(&gq),
                (true, false) => acc.add(&g),
                (false, true) => acc.add(&q),
                (false, false) => acc,
            };
        }
        if is_zero(&acc.z) {
            return false;
        }
        let z_inv = f.invert(&acc.z);
        let mut x = f.leave_mont(&f.mul(&acc.x, &f.mul(&z_inv, &z_inv)));
        if !less(&x, &n.m) {
            sub_in_place(&mut x, &n.m);
        }
        x == r
    }
}

/// Parse a DER `ECDSA-Sig-Value ::= SEQUENCE { r INTEGER, s INTEGER }` into
/// fixed 32-byte big-endian scalars. Only the definite short-form lengths a
/// P-256 signature can have are accepted.
pub fn parse_der_signature(der: &[u8]) -> Option<([u8; 32], [u8; 32])> {
    let (&tag, rest) = der.split_first()?;
    let (&length, body) = rest.split_first()?;
    if tag != 0x30 || length as usize != body.len() || length >= 0x80 {
        return None;
    }
    let (r, rest) = parse_der_integer(body)?;
    let (s, rest) = parse_der_integer(rest)?;
    rest.is_empty().then_some((r, s))
}

fn parse_der_integer(input: &[u8]) -> Option<([u8; 32], &[u8])> {
    let (&tag, rest) = input.split_first()?;
    let (&length, rest) = rest.split_first()?;
    let length = length as usize;
    if tag != 0x02 || length == 0 || length > 33 || rest.len() < length {
        return None;
    }
    let (mut value, rest) = rest.split_at(length);
    // Negative integers are invalid scalars.
    if value[0] & 0x80 != 0 {
        return None;
    }
    if value[0] == 0 && length > 1 {
        // A leading zero is only allowed to clear the sign bit.
        if value[1] & 0x80 == 0 {
            return None;
        }
        value = &value[1..];
    }
    if value.len() > 32 {
        return None;
    }
    let mut scalar = [0u8; 32];
    scalar[32 - value.len()..].copy_from_slice(value);
    Some((scalar, rest))
}

#[cfg(test)]
mod tests {
    use super::*;
    use p256::ecdsa::signature::hazmat::PrehashSigner;
    use p256::ecdsa::{Signature, SigningKey};
    use sha2::{Digest, Sha256};

    fn signing_key(seed: u32) -> SigningKey {
        let scalar = Sha256::digest(seed.to_le_bytes());
        SigningKey::from_bytes(&scalar).unwrap()
    }

    fn xy(key: &SigningKey) -> [u8; 64] {
        let point = key.verifying_key().to_encoded_point(false);
        let mut xy = [0u8; 64];
        xy.copy_from_slice(&point.as_bytes()[1..]);
        xy
    }

    fn split(signature: &Signature) -> ([u8; 32], [u8; 32]) {
        let bytes = signature.to_bytes();
        let mut r = [0u8; 32];
        let mut s = [0u8; 32];
        r.copy_from_slice(&bytes[..32]);
        s.copy_from_slice(&bytes[32..]);
        (r, s)
    }

    #[test]
    fn montgomery_constants_are_consistent() {
        for modulus in [&P, &N] {
            assert_eq!(modulus.m[0].wrapping_mul(modulus.m_inv), u32::MAX);
            let one = modulus.enter_mont(&ONE);
            assert_eq!(modulus.leave_mont(&one), ONE);
            let seven = [7, 0, 0, 0, 0, 0, 0, 0];
            let inverse = modulus.invert(&modulus.enter_mont(&seven));
            let product = modulus.mul(&inverse, &seven);
            assert_eq!(product, ONE);
        }
    }

    #[test]
    fn generator_is_on_the_curve() {
        let mut xy = [0u8; 64];
        for (index, limb) in GX.iter().rev().enumerate() {
            xy[4 * index..4 * index + 4].copy_from_slice(&limb.to_be_bytes());
        }
        for (index, limb) in GY.iter().rev().enumerate() {
            xy[32 + 4 * index..36 + 4 * index].copy_from_slice(&limb.to_be_bytes());
        }
        assert!(VerifyingKey::from_xy(&xy).is_some());
        xy[63] ^= 1;
        assert!(VerifyingKey::from_xy(&xy).is_none());
    }

    #[test]
    fn agrees_with_rustcrypto_on_valid_and_mutated_signatures() {
        for seed in 0..48u32 {
            let key = signing_key(seed);
            let verifying = VerifyingKey::from_xy(&xy(&key)).unwrap();
            let digest: [u8; 32] = Sha256::digest(seed.to_be_bytes()).into();
            let signature: Signature = key.sign_prehash(&digest).unwrap();
            let (r, s) = split(&signature);
            assert!(verifying.verify_prehash(&digest, &r, &s), "seed {seed}");

            // The low-s and high-s forms are both valid ECDSA signatures.
            let high_s = Signature::from_scalars(signature.r(), -*signature.s()).unwrap();
            let (hr, hs) = split(&high_s);
            assert!(verifying.verify_prehash(&digest, &hr, &hs));

            let mut bad_digest = digest;
            bad_digest[(seed % 32) as usize] ^= 1 << (seed % 8);
            assert!(!verifying.verify_prehash(&bad_digest, &r, &s));
            let mut bad_r = r;
            bad_r[31 - (seed % 32) as usize] ^= 0x10;
            assert!(!verifying.verify_prehash(&digest, &bad_r, &s));
            let mut bad_s = s;
            bad_s[(seed % 32) as usize] ^= 0x01;
            assert!(!verifying.verify_prehash(&digest, &r, &bad_s));

            let other = VerifyingKey::from_xy(&xy(&signing_key(seed + 1000))).unwrap();
            assert!(!other.verify_prehash(&digest, &r, &s));
        }
    }

    #[test]
    fn digests_above_the_group_order_are_reduced() {
        let key = signing_key(7);
        let verifying = VerifyingKey::from_xy(&xy(&key)).unwrap();
        let digest = [0xFFu8; 32];
        let signature: Signature = key.sign_prehash(&digest).unwrap();
        let (r, s) = split(&signature);
        assert!(verifying.verify_prehash(&digest, &r, &s));
    }

    #[test]
    fn out_of_range_scalars_are_rejected() {
        let key = signing_key(3);
        let verifying = VerifyingKey::from_xy(&xy(&key)).unwrap();
        let digest = [0x42u8; 32];
        let signature: Signature = key.sign_prehash(&digest).unwrap();
        let (r, s) = split(&signature);
        let zero = [0u8; 32];
        assert!(!verifying.verify_prehash(&digest, &zero, &s));
        assert!(!verifying.verify_prehash(&digest, &r, &zero));
        let mut n_bytes = [0u8; 32];
        for (index, limb) in N.m.iter().rev().enumerate() {
            n_bytes[4 * index..4 * index + 4].copy_from_slice(&limb.to_be_bytes());
        }
        assert!(!verifying.verify_prehash(&digest, &n_bytes, &s));
        assert!(!verifying.verify_prehash(&digest, &r, &n_bytes));
        // r + n would alias r if it were reduced instead of rejected.
        assert!(!verifying.verify_prehash(&digest, &[0xFF; 32], &s));
    }

    #[test]
    fn der_signatures_parse_and_reject_malformed_encodings() {
        let key = signing_key(11);
        let digest = [0x24u8; 32];
        let signature: Signature = key.sign_prehash(&digest).unwrap();
        let der = signature.to_der();
        let (r, s) = parse_der_signature(der.as_bytes()).unwrap();
        assert_eq!((r, s), split(&signature));

        // Short integers keep their value.
        assert_eq!(
            parse_der_signature(&[0x30, 0x06, 0x02, 0x01, 0x05, 0x02, 0x01, 0x07]),
            Some((
                {
                    let mut v = [0u8; 32];
                    v[31] = 5;
                    v
                },
                {
                    let mut v = [0u8; 32];
                    v[31] = 7;
                    v
                }
            ))
        );
        // Negative, non-minimal, truncated, trailing data, wrong tags.
        assert!(parse_der_signature(&[0x30, 0x06, 0x02, 0x01, 0x85, 0x02, 0x01, 0x07]).is_none());
        assert!(
            parse_der_signature(&[0x30, 0x07, 0x02, 0x02, 0x00, 0x05, 0x02, 0x01, 0x07]).is_none()
        );
        assert!(parse_der_signature(&[0x30, 0x06, 0x02, 0x01, 0x05, 0x02, 0x02, 0x07]).is_none());
        assert!(
            parse_der_signature(&[0x30, 0x07, 0x02, 0x01, 0x05, 0x02, 0x01, 0x07, 0x00]).is_none()
        );
        assert!(parse_der_signature(&[0x31, 0x06, 0x02, 0x01, 0x05, 0x02, 0x01, 0x07]).is_none());
        let mut too_long = [0u8; 2 + 2 + 34 + 3];
        too_long[..4].copy_from_slice(&[0x30, 39, 0x02, 34]);
        too_long[4] = 0x00;
        too_long[5] = 0x80;
        too_long[38..].copy_from_slice(&[0x02, 0x01, 0x01]);
        assert!(parse_der_signature(&too_long).is_none());
    }
}
