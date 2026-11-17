//! A Green Power TempMaster reply armed in the radio (`timed-tx`).
//!
//! A bidirectional GPD listens for its reply gpTxOffset (20 ms) after its
//! GPDF. When the reply waits for the task that processes received frames,
//! any work in that task (relaying broadcasts, flash writes, logging) can
//! delay it past the GPD's window: on the ESP32-H2 combo the trigger
//! reached the stack 68–173 ms after it ended during a commissioning burst
//! (2026-11-17). A PHY that implements [`crate::RadioPhy::arm_gp_reply`]
//! instead starts the pre-built reply from its receive interrupt: it
//! recognises the trigger with [`gpdf_triggers_reply`] and transmits the
//! frame at the instant itself.
//!
//! Only unprotected triggers are recognised (Commissioning and other
//! security-level-0 GPDFs). Their reply is unprotected too, exactly what
//! the proxy builds for such a trigger in the task, so no security check is
//! skipped; a protected trigger still takes the task path, where the proxy
//! verifies its MIC and freshness.

/// Outcome of an armed reply ([`crate::RadioPhy::take_gp_reply_outcome`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpReplyOutcome {
    /// SrcID of the GPD the reply was armed for.
    pub src_id: u32,
    /// `Ok`: the reply started; `Err`: it did not (busy radio, late).
    pub result: Result<(), crate::PhyError>,
    /// µs after the end of the triggering GPDF as the PHY time-stamps it:
    /// the estimated start on air (`Ok`) or give-up time (`ChannelBusy`),
    /// or, for `Missed`, when the interrupt handled the trigger.
    pub after_us: u32,
}

/// Whether `frame` — a received MAC frame from its frame control field on;
/// trailing FCS or RSSI/LQI bytes are allowed — is an unprotected GP Data
/// GPDF sent by the GPD with ApplicationID 0 SrcID `src_id` and
/// `rxAfterTx` set (GP 1.1 A.1.4.1: NWK frame control with protocol
/// version 3 and the Extended NWK Frame Control field).
///
/// Cheap and allocation-free: PHYs call it in their receive interrupt.
pub fn gpdf_triggers_reply(frame: &[u8], src_id: u32) -> bool {
    let Some(&[fc0, fc1]) = frame.first_chunk::<2>() else {
        return false;
    };
    let fc = u16::from_le_bytes([fc0, fc1]);
    // MAC data frame without MAC security.
    if fc & 0x0007 != 0x0001 || fc & 0x0008 != 0 {
        return false;
    }
    let pan_compression = fc & 0x0040 != 0;
    let addr_len = |mode: u16| match mode {
        2 => Some(2usize),
        3 => Some(8usize),
        0 => Some(0usize),
        _ => None,
    };
    let dst_mode = (fc >> 10) & 3;
    let src_mode = (fc >> 14) & 3;
    let (Some(dst_len), Some(src_len)) = (addr_len(dst_mode), addr_len(src_mode)) else {
        return false;
    };
    let mut at = 3; // frame control, sequence number
    if dst_mode != 0 {
        at += 2 + dst_len;
    }
    if src_mode != 0 {
        if !(pan_compression && dst_mode != 0) {
            at += 2;
        }
        at += src_len;
    }
    let Some(nwk) = frame.get(at..at + 6) else {
        return false;
    };
    let nwk_fc = nwk[0];
    let ext = nwk[1];
    // Data frame, protocol version 3, Extended NWK FC present.
    nwk_fc & 0x03 == 0x00
        && (nwk_fc >> 2) & 0x0F == 3
        && nwk_fc & 0x80 != 0
        // ApplicationID 0, no security, rxAfterTx, from the GPD.
        && ext & 0x07 == 0
        && (ext >> 3) & 0x03 == 0
        && ext & 0x40 != 0
        && ext & 0x80 == 0
        && u32::from_le_bytes([nwk[2], nwk[3], nwk[4], nwk[5]]) == src_id
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MAC: data, short broadcast destination, no source (as GPDs send).
    const MAC: [u8; 7] = [0x01, 0x08, 0x2A, 0xFF, 0xFF, 0xFF, 0xFF];
    const SRC: u32 = 0x293C_9879;

    fn gpdf(nwk_fc: u8, ext: u8, src: u32) -> heapless::Vec<u8, 32> {
        let mut f = heapless::Vec::new();
        f.extend_from_slice(&MAC).unwrap();
        f.push(nwk_fc).unwrap();
        f.push(ext).unwrap();
        f.extend_from_slice(&src.to_le_bytes()).unwrap();
        // Commissioning command, then RSSI/LQI in place of the FCS.
        f.extend_from_slice(&[0xE0, 0x33, 0x81, 0xC8, 0x99])
            .unwrap();
        f
    }

    #[test]
    fn unprotected_rx_after_tx_gpdf_from_the_src_id_triggers() {
        // NWK FC 0x8C: data, version 3, extended FC; ext 0x40: SrcID,
        // no security, rxAfterTx, from the GPD.
        assert!(gpdf_triggers_reply(&gpdf(0x8C, 0x40, SRC), SRC));
        // Auto-commissioning bit set: still a data GPDF.
        assert!(gpdf_triggers_reply(&gpdf(0xCC, 0x40, SRC), SRC));
    }

    #[test]
    fn everything_else_does_not_trigger() {
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0x40, SRC), SRC ^ 1));
        // No rxAfterTx.
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0x00, SRC), SRC));
        // Protected (security level 3).
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0x58, SRC), SRC));
        // Towards the GPD (our own reply).
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0xC0, SRC), SRC));
        // ApplicationID 2 (IEEE).
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0x42, SRC), SRC));
        // Maintenance frame type.
        assert!(!gpdf_triggers_reply(&gpdf(0x8D, 0x40, SRC), SRC));
        // Zigbee NWK (protocol version 2).
        assert!(!gpdf_triggers_reply(&gpdf(0x88, 0x40, SRC), SRC));
        // No extended NWK FC.
        assert!(!gpdf_triggers_reply(&gpdf(0x0C, 0x40, SRC), SRC));
        // Truncated, empty, a MAC ACK and a MAC command.
        assert!(!gpdf_triggers_reply(&gpdf(0x8C, 0x40, SRC)[..10], SRC));
        assert!(!gpdf_triggers_reply(&[], SRC));
        assert!(!gpdf_triggers_reply(&[0x02, 0x00, 0x2A], SRC));
        let mut command = gpdf(0x8C, 0x40, SRC);
        command[0] = 0x03;
        assert!(!gpdf_triggers_reply(&command, SRC));
        // MAC security.
        let mut secured = gpdf(0x8C, 0x40, SRC);
        secured[0] |= 0x08;
        assert!(!gpdf_triggers_reply(&secured, SRC));
    }

    #[test]
    fn mac_addressing_variants_are_skipped_correctly() {
        // Extended source, PAN ID compression, short destination.
        let mut f: heapless::Vec<u8, 40> = heapless::Vec::new();
        f.extend_from_slice(&[0x41, 0xC8, 0x01, 0xFF, 0xFF, 0xFF, 0xFF])
            .unwrap();
        f.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        f.extend_from_slice(&[0x8C, 0x40]).unwrap();
        f.extend_from_slice(&SRC.to_le_bytes()).unwrap();
        f.push(0xE0).unwrap();
        assert!(gpdf_triggers_reply(&f, SRC));
        // Without PAN ID compression a source PAN ID precedes it.
        let mut g: heapless::Vec<u8, 40> = heapless::Vec::new();
        g.extend_from_slice(&[0x01, 0xC8, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x34, 0x12])
            .unwrap();
        g.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        g.extend_from_slice(&[0x8C, 0x40]).unwrap();
        g.extend_from_slice(&SRC.to_le_bytes()).unwrap();
        assert!(gpdf_triggers_reply(&g, SRC));
    }
}
