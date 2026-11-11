//! Green Power Device (GPD) side: building GPDFs for transmission and
//! receiving GPP → GPD frames (bidirectional operation).
//!
//! The caller owns the radio: it transmits the bytes produced here (a MAC
//! frame without FCS) on the configured channel(s) and, for `rxAfterTx`
//! frames, listens for the reply window. Frame counters must be persisted
//! *before* use; [`CounterReservation`] implements a block-reservation
//! scheme so that flash is written once per `block` frames.

use zigbee_crypto::{AesKey, ForwardAesProvider};

use crate::commissioning::{CommissioningPayload, CommissioningReply};
use crate::frame::{
    FrameType, GPD_MAC_HEADER_LEN, GpdfParams, MAX_GPD_PAYLOAD, parse, parse_mac, write_header,
    write_mac_header,
};
use crate::security::{self, Direction, SealContext, WrappedKey};
use crate::types::{GpdId, KeyType, SecurityLevel, cmd};
use crate::{Error, Result};

/// Maximum MAC frame length (without FCS) produced by [`Gpd`].
pub const MAX_GPD_FRAME: usize = 125;

/// Static GPD security configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpdSecurity {
    /// Security level (0b00, 0b10 or 0b11).
    pub level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// GPD key.
    pub key: AesKey,
}

/// Static GPD identity and options.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpdConfig {
    /// GPD SrcID (must not be a reserved value).
    pub src_id: u32,
    /// Security configuration; `None` for an unprotected GPD.
    pub security: Option<GpdSecurity>,
}

/// Crash-safe security frame counter reservation.
///
/// The persisted value is an exclusive upper bound of counters that may
/// have been used. On boot, counters resume at that bound, so a power loss
/// can skip at most `block` values but never reuses one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterReservation {
    next: u32,
    limit: u32,
    block: u32,
}

impl CounterReservation {
    /// Resume from the persisted bound (0 for a fresh device).
    pub const fn resume(persisted_limit: u32, block: u32) -> Self {
        Self {
            next: persisted_limit,
            limit: persisted_limit,
            block: if block == 0 { 1 } else { block },
        }
    }

    /// Next counter value that would be used.
    pub fn peek(&self) -> u32 {
        self.next
    }

    /// Whether the next [`take`](Self::take) needs a new reservation
    /// persisted first, and the bound to persist.
    pub fn pending_reservation(&self) -> Result<Option<u32>> {
        if self.next < self.limit {
            return Ok(None);
        }
        self.next
            .checked_add(self.block)
            .map(Some)
            .ok_or(Error::BufferFull)
    }

    /// Record that `limit` was durably persisted.
    pub fn reserved(&mut self, limit: u32) {
        if limit > self.limit {
            self.limit = limit;
        }
    }

    /// Take the next counter. Fails if the reservation has not been
    /// persisted (see [`pending_reservation`](Self::pending_reservation)).
    pub fn take(&mut self) -> Result<u32> {
        if self.next >= self.limit {
            return Err(Error::Unsupported);
        }
        let v = self.next;
        self.next += 1;
        Ok(v)
    }
}

/// GPD frame builder.
pub struct Gpd {
    config: GpdConfig,
    mac_seq: u8,
}

/// Options for one outgoing GPDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TxOptions {
    /// Request a reply window after this frame (bidirectional).
    pub rx_after_tx: bool,
    /// Set the Auto-Commissioning bit.
    pub auto_commissioning: bool,
}

impl Gpd {
    /// Create a GPD builder. `initial_mac_seq` should be random or
    /// persisted for unprotected GPDs with incremental sequence numbers.
    pub fn new(config: GpdConfig, initial_mac_seq: u8) -> Result<Self> {
        if GpdId::SrcId(config.src_id).is_reserved() {
            return Err(Error::Unsupported);
        }
        if let Some(s) = config.security
            && !s.level.is_protected()
        {
            return Err(Error::Unsupported);
        }
        Ok(Self {
            config,
            mac_seq: initial_mac_seq,
        })
    }

    /// GPD configuration.
    pub fn config(&self) -> &GpdConfig {
        &self.config
    }

    fn next_seq(&mut self, frame_counter: Option<u32>) -> u8 {
        match frame_counter {
            // A.1.4.1: protected GPDFs carry the counter LSB as MAC seq.
            Some(fc) => fc as u8,
            None => {
                self.mac_seq = self.mac_seq.wrapping_add(1);
                self.mac_seq
            }
        }
    }

    fn build(
        &mut self,
        aes: &mut impl ForwardAesProvider,
        security: Option<(GpdSecurity, u32)>,
        opts: TxOptions,
        command_id: u8,
        payload: &[u8],
        out: &mut [u8],
    ) -> Result<usize> {
        if payload.len() + 1 > MAX_GPD_PAYLOAD {
            return Err(Error::BufferFull);
        }
        let seq = self.next_seq(security.map(|(_, fc)| fc));
        let mac = write_mac_header(out, seq)?;
        let params = GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: opts.auto_commissioning,
            src_id: Some(self.config.src_id),
            security_level: security.map_or(SecurityLevel::None, |(s, _)| s.level),
            individual_key: security.is_some_and(|(s, _)| s.key_type.is_individual()),
            rx_after_tx: opts.rx_after_tx,
            to_gpd: false,
            frame_counter: security.map_or(0, |(_, fc)| fc),
        };
        let hl = write_header(out.get_mut(mac..).ok_or(Error::BufferFull)?, &params)?;
        let body = mac + hl;
        let mut clear = [0u8; MAX_GPD_PAYLOAD];
        clear[0] = command_id;
        clear[1..=payload.len()].copy_from_slice(payload);
        let clear = &clear[..=payload.len()];
        let n = match security {
            None => {
                let dst = out
                    .get_mut(body..body + clear.len())
                    .ok_or(Error::BufferFull)?;
                dst.copy_from_slice(clear);
                clear.len()
            }
            Some((s, fc)) => {
                let (head, tail) = out.split_at_mut(body);
                security::seal(
                    aes,
                    &s.key,
                    &SealContext {
                        gpd: GpdId::SrcId(self.config.src_id),
                        frame_counter: fc,
                        level: s.level,
                        direction: Direction::FromGpd,
                    },
                    &head[mac..],
                    clear,
                    tail,
                )?
            }
        };
        Ok(body + n)
    }

    /// Build a data GPDF carrying `command_id` + `payload`, protected with
    /// the configured security using `counter`.
    pub fn data_frame(
        &mut self,
        aes: &mut impl ForwardAesProvider,
        counter: &mut CounterReservation,
        opts: TxOptions,
        command_id: u8,
        payload: &[u8],
        out: &mut [u8],
    ) -> Result<usize> {
        let sec = match self.config.security {
            Some(s) => Some((s, counter.take()?)),
            None => None,
        };
        self.build(aes, sec, opts, command_id, payload, out)
    }

    /// Build an unprotected GPD Commissioning (0xE0) GPDF.
    ///
    /// If the GPD is protected, the extended options advertise the level,
    /// key type, the TC-LK protected key and the outgoing counter.
    /// `payload.security` is overwritten from the configuration.
    ///
    /// The advertised counter is *taken* from `counter`: the sink stores it
    /// as the last counter received and rejects any later GPDF whose counter
    /// is not greater (zigpy `GreenPowerDevice.is_duplicate`, A.3.6.1.2), so
    /// the first data frame must use the next value. Like
    /// [`data_frame`](Self::data_frame) this fails until the reservation is
    /// persisted.
    pub fn commissioning_frame(
        &mut self,
        aes: &mut impl ForwardAesProvider,
        counter: &mut CounterReservation,
        tc_link_key: &AesKey,
        mut payload: CommissioningPayload,
        rx_after_tx: bool,
        out: &mut [u8],
    ) -> Result<usize> {
        payload.security = match self.config.security {
            None => None,
            Some(s) => {
                let w = security::wrap_key(
                    aes,
                    tc_link_key,
                    &GpdId::SrcId(self.config.src_id),
                    Direction::FromGpd,
                    0,
                    &s.key,
                )?;
                Some(crate::commissioning::CommissioningSecurity {
                    level: s.level,
                    key_type: s.key_type,
                    key: Some((w.key, Some(u32::from_le_bytes(w.mic)))),
                    outgoing_counter: Some(counter.take()?),
                })
            }
        };
        let mut buf = [0u8; MAX_GPD_PAYLOAD];
        let n = payload.encode(&mut buf)?;
        self.build(
            aes,
            None,
            TxOptions {
                rx_after_tx,
                auto_commissioning: false,
            },
            cmd::COMMISSIONING,
            &buf[..n],
            out,
        )
    }

    /// Build a protected command without payload (e.g. Success 0xE2 or
    /// Decommissioning 0xE1).
    pub fn simple_frame(
        &mut self,
        aes: &mut impl ForwardAesProvider,
        counter: &mut CounterReservation,
        command_id: u8,
        out: &mut [u8],
    ) -> Result<usize> {
        self.data_frame(aes, counter, TxOptions::default(), command_id, &[], out)
    }
}

/// A received GPP → GPD command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Downlink {
    /// GPD CommandID.
    pub command_id: u8,
    /// Clear payload.
    pub payload: heapless::Vec<u8, MAX_GPD_PAYLOAD>,
    /// Security frame counter of the downlink frame.
    pub frame_counter: u32,
}

/// Parse a received MAC frame (without FCS) addressed to this GPD.
///
/// Frames that are not GPDFs, not directed to a GPD, or carry another
/// SrcID return `Err(NotGpdf)`. Maintenance frames (Channel
/// Configuration) are accepted without SrcID.
pub fn parse_downlink(
    aes: &mut impl ForwardAesProvider,
    config: &GpdConfig,
    frame: &[u8],
) -> Result<Downlink> {
    let mac = parse_mac(frame)?;
    let msdu = &frame[mac.payload_offset..];
    let f = parse(msdu, mac.seq, mac.src_ieee)?;
    if !f.header.to_gpd && f.header.frame_type == FrameType::Data {
        return Err(Error::NotGpdf);
    }
    match f.header.gpd_id {
        Some(GpdId::SrcId(id)) if id == config.src_id => {}
        None if f.header.frame_type == FrameType::Maintenance => {}
        _ => return Err(Error::NotGpdf),
    }
    let clear = if f.header.security_level.is_protected() {
        let s = config.security.ok_or(Error::AuthFailed)?;
        security::open(aes, &s.key, &f, Direction::ToGpd)?
    } else {
        heapless::Vec::from_slice(f.payload).map_err(|_| Error::BufferFull)?
    };
    let (&command_id, rest) = clear.split_first().ok_or(Error::Truncated)?;
    Ok(Downlink {
        command_id,
        payload: heapless::Vec::from_slice(rest).map_err(|_| Error::BufferFull)?,
        frame_counter: f.header.frame_counter,
    })
}

/// Recover the key carried by a Commissioning Reply, if TC-LK protected.
pub fn reply_key(
    aes: &mut impl ForwardAesProvider,
    tc_link_key: &AesKey,
    src_id: u32,
    reply: &CommissioningReply,
) -> Result<Option<AesKey>> {
    match reply.key {
        None => Ok(None),
        Some((k, None)) => Ok(Some(k)),
        Some((k, Some((mic, fc)))) => security::unwrap_key(
            aes,
            tc_link_key,
            &GpdId::SrcId(src_id),
            Direction::ToGpd,
            fc,
            &WrappedKey {
                key: k,
                mic: mic.to_le_bytes(),
            },
        )
        .map(Some),
    }
}

/// Size of the fixed GPD MAC header.
pub const MAC_HEADER_LEN: usize = GPD_MAC_HEADER_LEN;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::ApplicationInfo;
    use crate::frame::parse;
    use crate::security::DEFAULT_TC_LINK_KEY;
    use zigbee_crypto::SoftwareAesProvider;

    const KEY_C0: AesKey = [
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
        0xCF,
    ];

    #[test]
    fn counter_reservation_never_reuses() {
        let mut c = CounterReservation::resume(0, 4);
        assert_eq!(c.take(), Err(Error::Unsupported));
        let lim = c.pending_reservation().unwrap().unwrap();
        assert_eq!(lim, 4);
        c.reserved(lim);
        for i in 0..4 {
            assert_eq!(c.pending_reservation().unwrap(), None);
            assert_eq!(c.take().unwrap(), i);
        }
        assert_eq!(c.pending_reservation().unwrap(), Some(8));
        // Power loss: resume from the persisted bound.
        let c2 = CounterReservation::resume(4, 4);
        assert_eq!(c2.peek(), 4);
        let near_wrap = CounterReservation::resume(u32::MAX - 1, 4);
        assert_eq!(near_wrap.pending_reservation(), Err(Error::BufferFull));
    }

    #[test]
    fn builds_spec_level3_shared_frame() {
        let mut aes = SoftwareAesProvider;
        let mut g = Gpd::new(
            GpdConfig {
                src_id: 0x8765_4321,
                security: Some(GpdSecurity {
                    level: SecurityLevel::Encrypted,
                    key_type: KeyType::GpdGroupKey,
                    key: KEY_C0,
                }),
            },
            0,
        )
        .unwrap();
        let mut c = CounterReservation::resume(2, 16);
        c.reserved(c.pending_reservation().unwrap().unwrap());
        let mut out = [0u8; MAX_GPD_FRAME];
        let n = g
            .data_frame(
                &mut aes,
                &mut c,
                TxOptions::default(),
                cmd::OFF,
                &[],
                &mut out,
            )
            .unwrap();
        // MAC: data, PAN ID compression off, dst short, seq = FC LSB.
        assert_eq!(&out[..7], &[0x01, 0x08, 0x02, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(
            &out[7..17],
            &[0x8C, 0x18, 0x21, 0x43, 0x65, 0x87, 0x02, 0, 0, 0]
        );
        let f = parse(&out[7..n], 0x02, None).unwrap();
        let clear = security::open(&mut aes, &KEY_C0, &f, Direction::FromGpd).unwrap();
        assert_eq!(&clear[..], &[cmd::OFF]);
        assert_eq!(c.peek(), 3);
    }

    #[test]
    fn commissioning_frame_carries_wrapped_key_and_counter() {
        let mut aes = SoftwareAesProvider;
        let mut g = Gpd::new(
            GpdConfig {
                src_id: 0x1234_5678,
                security: Some(GpdSecurity {
                    level: SecurityLevel::Encrypted,
                    key_type: KeyType::IndividualKey,
                    key: KEY_C0,
                }),
            },
            9,
        )
        .unwrap();
        let mut c = CounterReservation::resume(100, 16);
        let mut out = [0u8; MAX_GPD_FRAME];
        let p = CommissioningPayload {
            device_id: 0xFE,
            mac_seq_num_capability: true,
            rx_on_capability: false,
            pan_id_request: false,
            security_key_request: false,
            fixed_location: true,
            security: None,
            app_info: Some(ApplicationInfo {
                manufacturer_id: Some(0x6D6B),
                model_id: Some(0xF300),
                ..ApplicationInfo::default()
            }),
        };
        // Not reserved yet: the advertised counter must be durable first.
        assert_eq!(
            g.commissioning_frame(
                &mut aes,
                &mut c,
                &DEFAULT_TC_LINK_KEY,
                p.clone(),
                false,
                &mut out
            ),
            Err(Error::Unsupported)
        );
        c.reserved(c.pending_reservation().unwrap().unwrap());
        let n = g
            .commissioning_frame(&mut aes, &mut c, &DEFAULT_TC_LINK_KEY, p, false, &mut out)
            .unwrap();
        // The first data frame must carry a counter above the advertised one.
        assert_eq!(c.peek(), 101);
        assert_eq!(&out[..3], &[0x01, 0x08, 10]);
        let f = parse(&out[7..n], 10, None).unwrap();
        assert_eq!(f.header.security_level, SecurityLevel::None);
        assert_eq!(f.payload[0], cmd::COMMISSIONING);
        let parsed = CommissioningPayload::decode(&f.payload[1..]).unwrap();
        let sec = parsed.security.unwrap();
        assert_eq!(sec.outgoing_counter, Some(100));
        let (k, mic) = sec.key.unwrap();
        // Matches the spec TC-LK vector for SrcID 0x12345678 / key C0..CF.
        assert_eq!(&k[..4], &[0x7D, 0x17, 0x7B, 0xD2]);
        assert_eq!(mic, Some(u32::from_le_bytes([0x61, 0xF1, 0x63, 0xA9])));
    }
}
