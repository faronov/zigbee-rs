//! COBS framing (Cheshire & Baker), spec 5.1.
//!
//! * [`encode`] / [`Encoder`] produce the **minimal** encoding: a final
//!   block of exactly 254 non-zero bytes is closed by its `0xFF` code with no
//!   trailing empty `0x01` block, so `n` raw bytes encode to at most
//!   `n + ⌈n/254⌉` bytes (1 byte for an empty input).
//! * [`decode`] / [`decode_in_place`] accept both the minimal form and the
//!   non-minimal form with a trailing `0x01` code (spec 5.1).
//! * [`Accumulator`] is the bounded, byte-at-a-time receive side: it collects
//!   encoded bytes up to the `0x00` delimiter, resynchronises on every
//!   `0x00`, rejects oversize input and decodes in place.
//!
//! Neither direction allocates; the encoder writes into a caller buffer.

/// COBS errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CobsError {
    /// The output buffer is too small.
    BufferTooSmall,
    /// An encoded byte was `0x00` (only legal as the delimiter).
    ZeroByte,
    /// A code byte points past the end of the encoded data.
    Overrun,
}

/// Upper bound of the minimal COBS encoding of `n` raw bytes (no delimiter).
pub const fn max_encoded_len(n: usize) -> usize {
    if n == 0 { 1 } else { n + n.div_ceil(254) }
}

/// Streaming minimal COBS encoder writing into a caller buffer.
///
/// ```
/// use zigbee_zrhp::cobs::Encoder;
/// let mut out = [0u8; 8];
/// let mut enc = Encoder::new(&mut out).unwrap();
/// enc.push_slice(&[0x11, 0x22, 0x00, 0x33]).unwrap();
/// let n = enc.finish();
/// assert_eq!(&out[..n], &[0x03, 0x11, 0x22, 0x02, 0x33]);
/// ```
#[derive(Debug)]
pub struct Encoder<'a> {
    out: &'a mut [u8],
    pos: usize,
    code_idx: usize,
    code: u8,
}

impl<'a> Encoder<'a> {
    /// Start encoding into `out`.
    pub fn new(out: &'a mut [u8]) -> Result<Self, CobsError> {
        if out.is_empty() {
            return Err(CobsError::BufferTooSmall);
        }
        Ok(Self {
            out,
            pos: 1,
            code_idx: 0,
            code: 1,
        })
    }

    /// Encode one raw byte.
    #[inline]
    pub fn push(&mut self, byte: u8) -> Result<(), CobsError> {
        if self.code == 0xFF {
            // The previous block is full: close it and open a new one only
            // now that more data follows (minimal encoding).
            if self.pos >= self.out.len() {
                return Err(CobsError::BufferTooSmall);
            }
            self.out[self.code_idx] = 0xFF;
            self.code_idx = self.pos;
            self.pos += 1;
            self.code = 1;
        }
        if self.pos >= self.out.len() {
            return Err(CobsError::BufferTooSmall);
        }
        if byte == 0 {
            self.out[self.code_idx] = self.code;
            self.code_idx = self.pos;
            self.code = 1;
        } else {
            self.out[self.pos] = byte;
            self.code += 1;
        }
        self.pos += 1;
        Ok(())
    }

    /// Encode raw bytes.
    pub fn push_slice(&mut self, data: &[u8]) -> Result<(), CobsError> {
        for &b in data {
            self.push(b)?;
        }
        Ok(())
    }

    /// Close the last block and return the encoded length (no delimiter).
    pub fn finish(self) -> usize {
        self.out[self.code_idx] = self.code;
        self.pos
    }
}

/// Encode `src` into `dst` (no delimiter). Returns the encoded length.
pub fn encode(src: &[u8], dst: &mut [u8]) -> Result<usize, CobsError> {
    let mut enc = Encoder::new(dst)?;
    enc.push_slice(src)?;
    Ok(enc.finish())
}

/// Decode `src` (no delimiter) into `dst`. Returns the decoded length.
pub fn decode(src: &[u8], dst: &mut [u8]) -> Result<usize, CobsError> {
    let mut r = 0;
    let mut w = 0;
    while r < src.len() {
        let code = src[r] as usize;
        if code == 0 {
            return Err(CobsError::ZeroByte);
        }
        if r + code > src.len() {
            return Err(CobsError::Overrun);
        }
        r += 1;
        for _ in 1..code {
            let b = src[r];
            if b == 0 {
                return Err(CobsError::ZeroByte);
            }
            *dst.get_mut(w).ok_or(CobsError::BufferTooSmall)? = b;
            w += 1;
            r += 1;
        }
        if code != 0xFF && r < src.len() {
            *dst.get_mut(w).ok_or(CobsError::BufferTooSmall)? = 0;
            w += 1;
        }
    }
    Ok(w)
}

/// Decode in place. Returns the decoded length; the decoded bytes occupy
/// `buf[..len]`.
pub fn decode_in_place(buf: &mut [u8]) -> Result<usize, CobsError> {
    let n = buf.len();
    let mut r = 0;
    let mut w = 0;
    while r < n {
        let code = buf[r] as usize;
        if code == 0 {
            return Err(CobsError::ZeroByte);
        }
        if r + code > n {
            return Err(CobsError::Overrun);
        }
        r += 1;
        for _ in 1..code {
            let b = buf[r];
            if b == 0 {
                return Err(CobsError::ZeroByte);
            }
            // w < r always holds: every block consumed one code byte that
            // produced no output.
            buf[w] = b;
            w += 1;
            r += 1;
        }
        if code != 0xFF && r < n {
            buf[w] = 0;
            w += 1;
        }
    }
    Ok(w)
}

/// Result of feeding one byte to an [`Accumulator`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Feed {
    /// Nothing complete yet (also returned for empty frames and while
    /// discarding after an error).
    Pending,
    /// A frame was decoded; its bytes are [`Accumulator::frame`]`(len)`.
    Frame(usize),
    /// A frame was discarded. Counted once per discarded frame.
    Bad(BadFrame),
}

/// Why the accumulator discarded a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BadFrame {
    /// More than `N` encoded bytes without a delimiter.
    TooLong,
    /// COBS structure error.
    Cobs,
    /// A transport (UART framing/parity/overrun) error aborted the frame.
    LineError,
}

/// Bounded streaming receive buffer: collects encoded bytes up to the
/// `0x00` delimiter and decodes them in place.
///
/// `N` is the maximum encoded frame size without the delimiter. ZRHP v1 uses
/// [`crate::frame::MAX_ENCODED_FRAME`] (656), which also covers every legal
/// frame (at most 655 encoded bytes).
///
/// Work per input byte is O(1) except for the in-place decode at the
/// delimiter, which is O(frame length).
#[derive(Debug)]
pub struct Accumulator<const N: usize> {
    buf: [u8; N],
    len: usize,
    discarding: bool,
}

impl<const N: usize> Accumulator<N> {
    /// An empty accumulator.
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            discarding: false,
        }
    }

    /// Drop any partial frame and resynchronise at the next `0x00` without
    /// counting an error (used after opening a port).
    pub fn reset(&mut self) {
        self.len = 0;
        self.discarding = false;
    }

    /// Abort the current frame because of a transport line error; bytes are
    /// discarded until the next `0x00`. Returns `Feed::Bad(LineError)`.
    pub fn line_error(&mut self) -> Feed {
        self.len = 0;
        self.discarding = true;
        Feed::Bad(BadFrame::LineError)
    }

    /// Feed one received byte.
    #[inline]
    pub fn push(&mut self, byte: u8) -> Feed {
        if byte == 0 {
            let len = core::mem::take(&mut self.len);
            if core::mem::take(&mut self.discarding) || len == 0 {
                return Feed::Pending;
            }
            return match decode_in_place(&mut self.buf[..len]) {
                Ok(n) => Feed::Frame(n),
                Err(_) => Feed::Bad(BadFrame::Cobs),
            };
        }
        if self.discarding {
            return Feed::Pending;
        }
        if self.len >= N {
            self.len = 0;
            self.discarding = true;
            return Feed::Bad(BadFrame::TooLong);
        }
        self.buf[self.len] = byte;
        self.len += 1;
        Feed::Pending
    }

    /// The decoded frame after [`Feed::Frame`]`(len)`. Valid until the next
    /// [`push`](Self::push).
    pub fn frame(&self, len: usize) -> &[u8] {
        &self.buf[..len]
    }
}

impl<const N: usize> Default for Accumulator<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minimal_254_run() {
        let raw = [1u8; 254];
        let mut out = [0u8; 300];
        let n = encode(&raw, &mut out).unwrap();
        assert_eq!(n, 255);
        assert_eq!(n, max_encoded_len(254));
        let mut dec = [0u8; 300];
        assert_eq!(decode(&out[..n], &mut dec).unwrap(), 254);
        // Non-minimal trailing 0x01 is accepted.
        out[n] = 0x01;
        assert_eq!(decode(&out[..n + 1], &mut dec).unwrap(), 254);
        assert_eq!(&dec[..254], &raw[..]);
    }

    #[test]
    fn accumulator_resync_and_bounds() {
        let mut acc = Accumulator::<8>::new();
        for b in [9u8; 8] {
            assert_eq!(acc.push(b), Feed::Pending);
        }
        assert_eq!(acc.push(9), Feed::Bad(BadFrame::TooLong));
        assert_eq!(acc.push(9), Feed::Pending);
        assert_eq!(acc.push(0), Feed::Pending);
        // Empty frame.
        assert_eq!(acc.push(0), Feed::Pending);
        for b in [0x03, 0x11, 0x22, 0x02, 0x33] {
            assert_eq!(acc.push(b), Feed::Pending);
        }
        assert_eq!(acc.push(0), Feed::Frame(4));
        assert_eq!(acc.frame(4), &[0x11, 0x22, 0x00, 0x33]);
        // Code pointing past the end.
        acc.push(0x05);
        acc.push(0x01);
        assert_eq!(acc.push(0), Feed::Bad(BadFrame::Cobs));
    }
}
