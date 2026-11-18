//! Little-endian wire primitives shared by the message and record codecs.
//!
//! * [`Reader`] / [`Writer`] — bounds-checked cursors over caller buffers.
//! * [`Wire`] — implemented by every field type and every message/record
//!   body; decoding borrows from the input (`'a`), encoding writes into a
//!   [`Writer`].
//! * [`List`] — a counted list that is either a zero-copy view of received
//!   bytes or a borrowed slice of items to send.
//! * [`Bytes8`], [`Bytes16`], [`Rest`] — length-prefixed and to-the-end byte
//!   strings.

use core::fmt;
use core::marker::PhantomData;

/// Codec errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    /// The input ended before a required field.
    Truncated,
    /// The output buffer is full.
    BufferFull,
    /// A field value is not allowed (unknown discriminant, inconsistent
    /// length, list too long for its count byte).
    Invalid,
}

impl fmt::Display for CodecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Truncated => "truncated input",
            Self::BufferFull => "output buffer full",
            Self::Invalid => "invalid field value",
        })
    }
}

#[cfg(feature = "std")]
impl std::error::Error for CodecError {}

/// Bounds-checked little-endian reader.
#[derive(Debug, Clone)]
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// Read from `buf`.
    pub const fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes not yet consumed.
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// `true` when all bytes are consumed.
    pub fn is_empty(&self) -> bool {
        self.remaining() == 0
    }

    /// Current offset.
    pub const fn position(&self) -> usize {
        self.pos
    }

    /// Take `n` bytes.
    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8], CodecError> {
        let end = self.pos.checked_add(n).ok_or(CodecError::Truncated)?;
        let out = self.buf.get(self.pos..end).ok_or(CodecError::Truncated)?;
        self.pos = end;
        Ok(out)
    }

    /// Take everything that is left.
    pub fn rest(&mut self) -> &'a [u8] {
        let out = &self.buf[self.pos..];
        self.pos = self.buf.len();
        out
    }

    /// Take a fixed array.
    pub fn array<const N: usize>(&mut self) -> Result<[u8; N], CodecError> {
        let b = self.bytes(N)?;
        let mut out = [0u8; N];
        out.copy_from_slice(b);
        Ok(out)
    }

    /// `u8`.
    pub fn u8(&mut self) -> Result<u8, CodecError> {
        Ok(self.bytes(1)?[0])
    }

    /// `i8`.
    pub fn i8(&mut self) -> Result<i8, CodecError> {
        Ok(self.u8()? as i8)
    }

    /// `u16` LE.
    pub fn u16(&mut self) -> Result<u16, CodecError> {
        Ok(u16::from_le_bytes(self.array()?))
    }

    /// `u32` LE.
    pub fn u32(&mut self) -> Result<u32, CodecError> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    /// `u64` LE.
    pub fn u64(&mut self) -> Result<u64, CodecError> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    /// Decode a [`Wire`] value.
    pub fn read<T: Wire<'a>>(&mut self) -> Result<T, CodecError> {
        T::read(self)
    }
}

/// Bounds-checked little-endian writer.
#[derive(Debug)]
pub struct Writer<'b> {
    buf: &'b mut [u8],
    pos: usize,
}

impl<'b> Writer<'b> {
    /// Write into `buf`.
    pub fn new(buf: &'b mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes written so far.
    pub const fn len(&self) -> usize {
        self.pos
    }

    /// `true` when nothing was written.
    pub const fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// Free space.
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// The written bytes.
    pub fn written(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Consume the writer, returning the written prefix of the buffer.
    pub fn into_written(self) -> &'b mut [u8] {
        &mut self.buf[..self.pos]
    }

    /// Append bytes.
    pub fn bytes(&mut self, data: &[u8]) -> Result<(), CodecError> {
        let end = self.pos + data.len();
        self.buf
            .get_mut(self.pos..end)
            .ok_or(CodecError::BufferFull)?
            .copy_from_slice(data);
        self.pos = end;
        Ok(())
    }

    /// `u8`.
    pub fn u8(&mut self, v: u8) -> Result<(), CodecError> {
        self.bytes(&[v])
    }

    /// `i8`.
    pub fn i8(&mut self, v: i8) -> Result<(), CodecError> {
        self.u8(v as u8)
    }

    /// `u16` LE.
    pub fn u16(&mut self, v: u16) -> Result<(), CodecError> {
        self.bytes(&v.to_le_bytes())
    }

    /// `u32` LE.
    pub fn u32(&mut self, v: u32) -> Result<(), CodecError> {
        self.bytes(&v.to_le_bytes())
    }

    /// `u64` LE.
    pub fn u64(&mut self, v: u64) -> Result<(), CodecError> {
        self.bytes(&v.to_le_bytes())
    }

    /// Drop everything written after `len`.
    pub fn truncate(&mut self, len: usize) {
        self.pos = self.pos.min(len);
    }

    /// Reserve `n` bytes to be patched later; returns their offset.
    pub fn reserve(&mut self, n: usize) -> Result<usize, CodecError> {
        let at = self.pos;
        let end = at + n;
        if end > self.buf.len() {
            return Err(CodecError::BufferFull);
        }
        self.buf[at..end].fill(0);
        self.pos = end;
        Ok(at)
    }

    /// Overwrite previously written bytes at `at`.
    pub fn patch(&mut self, at: usize, data: &[u8]) {
        self.buf[at..at + data.len()].copy_from_slice(data);
    }

    /// Encode a [`Wire`] value.
    pub fn write<'a, T: Wire<'a>>(&mut self, v: &T) -> Result<(), CodecError> {
        v.write(self)
    }
}

/// A value with a ZRHP wire encoding.
///
/// Decoding borrows variable-length data from the input buffer (`'a`), so
/// received messages are parsed without copying. Decoders ignore nothing
/// inside a value; message-level trailing bytes are ignored by the message
/// layer (spec 6.6).
pub trait Wire<'a>: Sized {
    /// Decode from `r`.
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError>;
    /// Encode into `w`.
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError>;
}

macro_rules! prim {
    ($t:ty, $r:ident) => {
        impl<'a> Wire<'a> for $t {
            fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
                r.$r()
            }
            fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
                w.$r(*self)
            }
        }
    };
}
prim!(u8, u8);
prim!(i8, i8);
prim!(u16, u16);
prim!(u32, u32);
prim!(u64, u64);

impl<'a, const N: usize> Wire<'a> for [u8; N] {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        r.array()
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.bytes(self)
    }
}

impl<'a> Wire<'a> for () {
    fn read(_: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(())
    }
    fn write(&self, _: &mut Writer<'_>) -> Result<(), CodecError> {
        Ok(())
    }
}

/// `u8 len, bytes[len]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Bytes8<'a>(pub &'a [u8]);

impl<'a> Bytes8<'a> {
    /// The bytes as UTF-8, if valid.
    pub fn as_str(&self) -> Option<&'a str> {
        core::str::from_utf8(self.0).ok()
    }
}

impl<'a> Wire<'a> for Bytes8<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let n = r.u8()? as usize;
        Ok(Self(r.bytes(n)?))
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let n = u8::try_from(self.0.len()).map_err(|_| CodecError::Invalid)?;
        w.u8(n)?;
        w.bytes(self.0)
    }
}

/// `u16 len, bytes[len]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Bytes16<'a>(pub &'a [u8]);

impl<'a> Wire<'a> for Bytes16<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let n = r.u16()? as usize;
        Ok(Self(r.bytes(n)?))
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let n = u16::try_from(self.0.len()).map_err(|_| CodecError::Invalid)?;
        w.u16(n)?;
        w.bytes(self.0)
    }
}

/// All remaining bytes of the payload (`data[]` at the end of a layout).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rest<'a>(pub &'a [u8]);

impl<'a> Rest<'a> {
    /// The bytes as UTF-8, if valid.
    pub fn as_str(&self) -> Option<&'a str> {
        core::str::from_utf8(self.0).ok()
    }
}

impl<'a> Wire<'a> for Rest<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(Self(r.rest()))
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.bytes(self.0)
    }
}

/// Width of a list's count prefix.
pub trait CountPrefix {
    /// Read the count.
    fn read_count(r: &mut Reader<'_>) -> Result<usize, CodecError>;
    /// Write the count.
    fn write_count(n: usize, w: &mut Writer<'_>) -> Result<(), CodecError>;
}

/// `u8 n` prefix (the default).
#[derive(Debug, Clone, Copy)]
pub struct U8Count;

/// No prefix: the list runs to the end of the payload.
#[derive(Debug, Clone, Copy)]
pub struct ToEnd;

impl CountPrefix for U8Count {
    fn read_count(r: &mut Reader<'_>) -> Result<usize, CodecError> {
        Ok(r.u8()? as usize)
    }
    fn write_count(n: usize, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(u8::try_from(n).map_err(|_| CodecError::Invalid)?)
    }
}

/// A list of `T`: either a zero-copy view of received bytes or a slice of
/// items to encode.
///
/// `P` selects the count prefix: [`U8Count`] (`u8 n, n×T`) or [`ToEnd`]
/// (items until the end of the payload).
pub enum List<'a, T, P = U8Count> {
    /// Received bytes (`count` items, already validated by [`Wire::read`]).
    Raw {
        /// Number of items.
        count: usize,
        /// Their encoded bytes.
        bytes: &'a [u8],
        /// Prefix marker.
        _p: PhantomData<P>,
    },
    /// Items to send.
    Items(&'a [T]),
}

impl<'a, T, P> List<'a, T, P> {
    /// A list of items to send.
    pub const fn new(items: &'a [T]) -> Self {
        Self::Items(items)
    }

    /// Number of items.
    pub fn len(&self) -> usize {
        match self {
            Self::Raw { count, .. } => *count,
            Self::Items(items) => items.len(),
        }
    }

    /// `true` when empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<'a, T: Wire<'a> + Clone, P> List<'a, T, P> {
    /// Iterate the items (decoding raw bytes on the fly).
    pub fn iter(&self) -> ListIter<'a, '_, T> {
        match self {
            Self::Raw { count, bytes, .. } => ListIter::Raw {
                r: Reader::new(bytes),
                left: *count,
            },
            Self::Items(items) => ListIter::Items(items.iter()),
        }
    }

    /// The item at `i`.
    pub fn get(&self, i: usize) -> Option<T> {
        self.iter().nth(i)
    }
}

/// Iterator over a [`List`].
pub enum ListIter<'a, 's, T> {
    /// Decoding raw bytes.
    Raw {
        /// Cursor.
        r: Reader<'a>,
        /// Items left.
        left: usize,
    },
    /// Borrowed items.
    Items(core::slice::Iter<'s, T>),
}

impl<'a, T: Wire<'a> + Clone> Iterator for ListIter<'a, '_, T> {
    type Item = T;
    fn next(&mut self) -> Option<T> {
        match self {
            Self::Raw { r, left } => {
                if *left == 0 {
                    return None;
                }
                *left -= 1;
                // Validated when the list was read.
                T::read(r).ok()
            }
            Self::Items(it) => it.next().cloned(),
        }
    }
}

impl<T, P> Clone for List<'_, T, P> {
    fn clone(&self) -> Self {
        match self {
            Self::Raw { count, bytes, .. } => Self::Raw {
                count: *count,
                bytes,
                _p: PhantomData,
            },
            Self::Items(items) => Self::Items(items),
        }
    }
}

impl<'a, T: Wire<'a> + Clone + fmt::Debug, P> fmt::Debug for List<'a, T, P> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.iter()).finish()
    }
}

impl<'a, T: Wire<'a> + Clone + PartialEq, P> PartialEq for List<'a, T, P> {
    fn eq(&self, other: &Self) -> bool {
        self.len() == other.len() && self.iter().zip(other.iter()).all(|(a, b)| a == b)
    }
}

impl<'a, T: Wire<'a>> Wire<'a> for List<'a, T, U8Count> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let count = U8Count::read_count(r)?;
        let start = r.clone();
        let begin = r.position();
        for _ in 0..count {
            T::read(r)?;
        }
        let n = r.position() - begin;
        let mut s = start;
        Ok(Self::Raw {
            count,
            bytes: s.bytes(n)?,
            _p: PhantomData,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        U8Count::write_count(self.len(), w)?;
        write_items(self, w)
    }
}

impl<'a, T: Wire<'a>> Wire<'a> for List<'a, T, ToEnd> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let bytes = r.rest();
        let mut inner = Reader::new(bytes);
        let mut count = 0;
        while !inner.is_empty() {
            T::read(&mut inner)?;
            count += 1;
        }
        Ok(Self::Raw {
            count,
            bytes,
            _p: PhantomData,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        write_items(self, w)
    }
}

fn write_items<'a, T: Wire<'a>, P>(
    list: &List<'a, T, P>,
    w: &mut Writer<'_>,
) -> Result<(), CodecError> {
    match list {
        List::Raw { bytes, .. } => w.bytes(bytes),
        List::Items(items) => {
            for item in *items {
                item.write(w)?;
            }
            Ok(())
        }
    }
}

/// Encode `v` into `buf`, returning the encoded length.
pub fn encode_into<'a, T: Wire<'a>>(v: &T, buf: &mut [u8]) -> Result<usize, CodecError> {
    let mut w = Writer::new(buf);
    v.write(&mut w)?;
    Ok(w.len())
}

/// Define a plain struct whose fields are encoded in declaration order.
macro_rules! wire_struct {
    (
        $(#[$m:meta])*
        pub struct $name:ident<'a> { $( $(#[$fm:meta])* pub $f:ident : $t:ty ),* $(,)? }
    ) => {
        $(#[$m])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name<'a> { $( $(#[$fm])* pub $f: $t ),* }

        impl<'a> $crate::codec::Wire<'a> for $name<'a> {
            fn read(r: &mut $crate::codec::Reader<'a>) -> Result<Self, $crate::codec::CodecError> {
                Ok(Self { $( $f: r.read()? ),* })
            }
            fn write(&self, w: &mut $crate::codec::Writer<'_>) -> Result<(), $crate::codec::CodecError> {
                $( w.write(&self.$f)?; )*
                Ok(())
            }
        }
    };
    (
        $(#[$m:meta])*
        pub struct $name:ident { $( $(#[$fm:meta])* pub $f:ident : $t:ty ),* $(,)? }
    ) => {
        $(#[$m])*
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name { $( $(#[$fm])* pub $f: $t ),* }

        impl<'a> $crate::codec::Wire<'a> for $name {
            #[allow(unused_variables)]
            fn read(r: &mut $crate::codec::Reader<'a>) -> Result<Self, $crate::codec::CodecError> {
                Ok(Self { $( $f: r.read()? ),* })
            }
            #[allow(unused_variables)]
            fn write(&self, w: &mut $crate::codec::Writer<'_>) -> Result<(), $crate::codec::CodecError> {
                $( w.write(&self.$f)?; )*
                Ok(())
            }
        }
    };
}
pub(crate) use wire_struct;
