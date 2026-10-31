//! Minimal hardware boundary for an IEEE 802.15.4 radio.

use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

/// Maximum MAC frame length excluding the two-byte FCS.
pub const MAX_PHY_FRAME_LEN: usize = 125;

/// PHY-level operation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyError {
    /// A single clear-channel assessment found the channel busy.
    ChannelBusy,
    /// The requested channel is invalid or unsupported.
    InvalidChannel,
    /// The frame exceeds the IEEE 802.15.4 PSDU limit.
    FrameTooLong,
    /// The received frame failed its FCS check.
    CrcFailed,
    /// The radio peripheral, DMA engine, or calibration failed.
    Hardware,
    /// The operation is not implemented by this PHY.
    Unsupported,
}

/// Address filter programmed into PHYs with hardware filtering or auto-ACK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyAddressFilter {
    pub pan_id: PanId,
    pub short_address: ShortAddress,
    pub extended_address: IeeeAddress,
}

/// Optional hardware features exposed to the software MAC.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PhyCapabilities {
    /// Matching ACK-request frames are acknowledged by hardware.
    pub hardware_auto_ack: bool,
    /// The PHY can discard frames that do not match the configured addresses.
    pub hardware_address_filter: bool,
    /// Minimum supported transmit power in dBm.
    pub tx_power_min: i8,
    /// Maximum supported transmit power in dBm.
    pub tx_power_max: i8,
}

/// Matching Imm-Ack observed for an acknowledged transmission.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AckResult {
    /// Frame Pending bit of the received ACK.
    pub frame_pending: bool,
}

/// MAC PIB transmit parameters handed to a [`crate::FullTx`] PHY.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxParams {
    pub max_csma_backoffs: u8,
    pub min_be: u8,
    pub max_be: u8,
    /// Retransmissions after the first attempt when an ACK is requested.
    pub max_frame_retries: u8,
}

/// FCS-validated MAC bytes received from the PHY.
///
/// `data[..len]` contains the MAC header and payload without the two-byte FCS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PhyRxFrame {
    data: [u8; MAX_PHY_FRAME_LEN],
    len: u8,
    /// IEEE 802.15.4 Link Quality Indicator, normalized to `0..=255` (larger
    /// is better).
    ///
    /// Backends whose hardware reports a different scale must convert before
    /// constructing this frame; see [`crate::lqi`].
    pub lqi: u8,
}

impl PhyRxFrame {
    pub fn from_slice(data: &[u8], lqi: u8) -> Result<Self, PhyError> {
        if data.len() > MAX_PHY_FRAME_LEN {
            return Err(PhyError::FrameTooLong);
        }

        let mut frame = Self {
            data: [0; MAX_PHY_FRAME_LEN],
            len: data.len() as u8,
            lqi,
        };
        frame.data[..data.len()].copy_from_slice(data);
        Ok(frame)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }

    pub fn len(&self) -> usize {
        usize::from(self.len)
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// Single-radio IEEE 802.15.4 hardware interface.
///
/// This trait deliberately excludes PIB state, CSMA-CA backoff policy, scan
/// protocol, association, polling, and frame retries. Those belong in the
/// shared software MAC. Platform clock, delay, and entropy remain a separate
/// [`crate::PlatformServices`] bound on that software MAC.
///
/// # Optional hardware offload and policy hooks
///
/// The transmit path is selected by the required [`RadioPhy::Tx`] marker
/// type. Every other item after the required methods has a default that
/// preserves the pure-software behaviour. They are associated types,
/// constants or provided methods, never runtime flags or `dyn` dispatch, so a
/// PHY compiles to exactly the path it selects. See
/// `docs/book/src/advanced/soft-mac-porting.md` for the porting contract.
pub trait RadioPhy {
    /// Transmit-path offload level: [`crate::SoftwareTx`] (software CSMA-CA,
    /// ACK wait and retries), [`crate::AckWaitTx`] or [`crate::FullTx`].
    ///
    /// The software MAC selects its transmit path by this type, so only the
    /// selected path is compiled and its future is exactly as large as that
    /// path needs.
    type Tx: crate::TxOffload;

    /// How long a poll waits for the parent's indirect frame after an ACK
    /// with Frame Pending set, in microseconds.
    const POLL_RESPONSE_WAIT_US: u32 = 30_000;

    /// After a short-source Data Request returns no frame, repeat the poll
    /// once with the extended source address.
    ///
    /// Some parents (EmberZNet) queue a Rejoin Response or Transport-Key by
    /// the child's EUI-64, which only an extended-source Data Request
    /// retrieves. The second poll costs airtime on every poll, so it is
    /// opt-in per backend.
    const POLL_EXTENDED_FALLBACK: bool = false;

    /// Return the optional hardware features implemented by this PHY.
    fn capabilities(&self) -> PhyCapabilities;

    /// Perform exactly one CCA and transmit one MAC frame if the channel is
    /// clear.
    ///
    /// `frame` excludes the FCS; the PHY must append it. A busy CCA is
    /// reported as [`PhyError::ChannelBusy`] so the software MAC can apply its
    /// own backoff and retry policy.
    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError>;

    /// Send an immediate ACK without CCA.
    ///
    /// The PHY owns the turnaround-critical path and must begin transmission
    /// within the IEEE 802.15.4 ACK timing window. A backend with hardware
    /// auto-ACK may return [`PhyError::Unsupported`]; the software MAC must not
    /// call this method when `hardware_auto_ack` is true.
    async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError>;

    /// Receive one valid MAC frame within `timeout_us`.
    ///
    /// Returns `Ok(None)` on timeout. Returned bytes exclude the FCS; the PHY
    /// must validate it before returning. Dropping this future before
    /// completion must leave the radio usable by subsequent PHY operations.
    async fn receive(&mut self, timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError>;

    /// Select an IEEE 802.15.4 channel.
    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError>;

    /// Set transmit power in dBm.
    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError>;

    /// Perform energy detection for `duration_us`.
    ///
    /// The result follows the IEEE 802.15.4 ED scale: 0 is the receiver floor,
    /// 255 is the maximum detectable energy, and larger values always mean
    /// more channel energy.
    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError>;

    /// Configure or disable hardware address filtering and auto-ACK matching.
    ///
    /// Software-only implementations may ignore `filter` when both related
    /// capability flags are false.
    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError>;

    /// [`crate::AckWaitTx`]: one CCA + TX, then wait for the matching
    /// Imm-Ack in the ISR / radio core.
    ///
    /// Returns `Ok(Some(ack))` for the matching ACK, `Ok(None)` when the ACK
    /// window closed without one, and [`PhyError::ChannelBusy`] for a busy
    /// CCA. Non-ACK frames received inside the window must stay available to
    /// later [`Self::receive`] calls (and be acknowledged by the PHY when
    /// `hardware_auto_ack` is set).
    async fn try_transmit_acknowledged(
        &mut self,
        _frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        Err(PhyError::Unsupported)
    }

    /// [`crate::FullTx`]: complete unslotted CSMA-CA and, when
    /// `ack_requested`, ACK wait with up to `params.max_frame_retries`
    /// retransmissions.
    ///
    /// Returns `Ok(Some(ack))` for an acknowledged frame, `Ok(None)` for a
    /// sent frame without an ACK request **or** when every retransmission
    /// went unacknowledged (the software MAC distinguishes the two by
    /// `ack_requested`), and [`PhyError::ChannelBusy`] for a channel-access
    /// failure after `params.max_csma_backoffs`.
    async fn transmit_offloaded(
        &mut self,
        _frame: &[u8],
        _ack_requested: bool,
        _params: TxParams,
    ) -> Result<Option<AckResult>, PhyError> {
        Err(PhyError::Unsupported)
    }

    /// Arm or clear the Frame Pending bit that a **hardware / ISR**
    /// auto-ACK sends in reply to a Data Request from `source`.
    ///
    /// Called by a parent-role [`crate::SoftMacCore`] only when
    /// `capabilities().hardware_auto_ack` is set; software-ACK PHYs receive
    /// the bit as the `frame_pending` argument of [`Self::send_ack`] instead.
    /// A PHY whose table is full returns [`PhyError::Hardware`].
    fn set_ack_frame_pending(
        &mut self,
        _source: &MacAddress,
        _pending: bool,
    ) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    /// Forget every armed Frame Pending source (MLME-RESET).
    fn clear_ack_frame_pending(&mut self) {}

    /// Put the transceiver into its lowest-power idle state between
    /// sleepy-device poll windows. The next PHY operation, or
    /// [`Self::wake`], must make it usable again.
    fn sleep(&mut self) -> Result<(), PhyError> {
        Ok(())
    }

    /// Restore clocks, calibration and radio state after [`Self::sleep`] or a
    /// retained low-power mode. The software MAC re-applies channel, TX
    /// power and address filter afterwards.
    fn wake(&mut self) -> Result<(), PhyError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn received_frame_owns_fcs_free_bytes() {
        let frame = PhyRxFrame::from_slice(&[0x61, 0x88, 0x2a], 173).unwrap();
        assert_eq!(frame.as_slice(), [0x61, 0x88, 0x2a]);
        assert_eq!(frame.len(), 3);
        assert_eq!(frame.lqi, 173);
    }

    #[test]
    fn received_frame_rejects_psdu_larger_than_125_bytes() {
        assert_eq!(
            PhyRxFrame::from_slice(&[0; MAX_PHY_FRAME_LEN + 1], 0),
            Err(PhyError::FrameTooLong)
        );
    }
}
