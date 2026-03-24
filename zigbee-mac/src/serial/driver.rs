//! `MacDriver` implementation over the custom serial protocol.
//!
//! [`SerialMac`] sends serial command frames to a thin MCU bridge dongle
//! and awaits responses, translating between the `MacDriver` trait and
//! the serial wire format defined in [`super`].

use core::cell::UnsafeCell;

use crate::pib::{PibAttribute, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use zigbee_types::TxPower;

use super::{
    SerialCodec, SerialError, SerialFrame, SerialPort,
    PayloadBuilder, PayloadParser,
    MAX_FRAME_SIZE, MAX_PAYLOAD_SIZE,
    CMD_RESET_REQ, CMD_RESET_CONF,
    CMD_SCAN_REQ, CMD_SCAN_CONF,
    CMD_ASSOCIATE_REQ, CMD_ASSOCIATE_CONF,
    CMD_ASSOCIATE_RSP,
    CMD_DISASSOCIATE_REQ,
    CMD_START_REQ,
    CMD_GET_REQ, CMD_GET_CONF,
    CMD_SET_REQ, CMD_SET_CONF,
    CMD_POLL_REQ, CMD_POLL_CONF,
    CMD_DATA_REQ, CMD_DATA_CONF, CMD_DATA_IND,
    CMD_STATUS,
};

// ── Helper: SerialError → MacError ──────────────────────────────

fn to_mac_error(e: SerialError) -> MacError {
    match e {
        SerialError::Timeout => MacError::Other,
        SerialError::CrcError => MacError::RadioError,
        SerialError::IoError => MacError::RadioError,
        SerialError::UnexpectedCommand => MacError::Other,
        _ => MacError::Other,
    }
}

fn status_to_mac_error(status: u8) -> Result<(), MacError> {
    match status {
        0x00 => Ok(()),
        0x01 => Err(MacError::NoBeacon),
        0x02 => Err(MacError::InvalidParameter),
        0x03 => Err(MacError::RadioError),
        0x04 => Err(MacError::ChannelAccessFailure),
        0x05 => Err(MacError::NoAck),
        0x06 => Err(MacError::FrameTooLong),
        0x07 => Err(MacError::Unsupported),
        0x08 => Err(MacError::SecurityError),
        0x09 => Err(MacError::TransactionOverflow),
        0x0A => Err(MacError::TransactionExpired),
        0x0B => Err(MacError::ScanInProgress),
        0x0C => Err(MacError::AssociationDenied),
        0x0D => Err(MacError::PanAtCapacity),
        _ => Err(MacError::Other),
    }
}

// ── Interior-mutable I/O state (for &self mlme_get) ─────────────

/// Wraps the port, codec, and sequence counter so `mlme_get(&self)` can
/// perform I/O. Safety: `MacDriver` is documented as single-threaded only.
struct IoState<S: SerialPort> {
    port: UnsafeCell<S>,
    codec: UnsafeCell<SerialCodec>,
    seq: UnsafeCell<u8>,
}

impl<S: SerialPort> IoState<S> {
    fn new(port: S) -> Self {
        Self {
            port: UnsafeCell::new(port),
            codec: UnsafeCell::new(SerialCodec::new()),
            seq: UnsafeCell::new(0),
        }
    }

    /// SAFETY: caller must guarantee exclusive access (single-threaded executor).
    #[inline]
    fn port_mut(&self) -> &mut S {
        unsafe { &mut *self.port.get() }
    }
    #[inline]
    fn codec_mut(&self) -> &mut SerialCodec {
        unsafe { &mut *self.codec.get() }
    }
    #[inline]
    fn seq_mut(&self) -> &mut u8 {
        unsafe { &mut *self.seq.get() }
    }
}

// ── SerialMac ───────────────────────────────────────────────────

/// MAC driver that communicates with a thin 802.15.4 bridge over a serial port.
///
/// The bridge firmware runs on an MCU with a native 802.15.4 radio (e.g.
/// nRF52840 USB dongle) and implements the simple serial protocol defined in
/// [`super`]. This driver sends request frames and waits for the
/// corresponding confirm/indication.
pub struct SerialMac<S: SerialPort> {
    io: IoState<S>,
}

impl<S: SerialPort> SerialMac<S> {
    /// Create a new serial MAC driver over the given port.
    pub fn new(port: S) -> Self {
        Self {
            io: IoState::new(port),
        }
    }

    /// Get the next sequence number.
    fn next_seq(&self) -> u8 {
        let seq = self.io.seq_mut();
        let s = *seq;
        *seq = s.wrapping_add(1);
        s
    }

    /// Send a command frame and wait for a response with the given command ID.
    async fn transact(
        &self,
        cmd: u8,
        payload: &[u8],
        expected_rsp: u8,
    ) -> Result<SerialFrame, MacError> {
        let seq = self.next_seq();
        let frame = SerialFrame::new(cmd, seq, payload).map_err(to_mac_error)?;
        let mut tx_buf = [0u8; MAX_FRAME_SIZE];
        let tx_len = frame.serialize(&mut tx_buf).map_err(to_mac_error)?;

        self.io
            .port_mut()
            .write(&tx_buf[..tx_len])
            .await
            .map_err(to_mac_error)?;

        // Read response
        self.io.codec_mut().reset();
        let mut rx_buf = [0u8; MAX_FRAME_SIZE];
        loop {
            let n = self
                .io
                .port_mut()
                .read(&mut rx_buf)
                .await
                .map_err(to_mac_error)?;
            if n == 0 {
                return Err(MacError::Other);
            }
            if let Some(rsp_frame) =
                self.io.codec_mut().feed(&rx_buf[..n]).map_err(to_mac_error)?
            {
                if rsp_frame.cmd == CMD_STATUS {
                    let status =
                        PayloadParser::status(&rsp_frame.payload).map_err(to_mac_error)?;
                    status_to_mac_error(status)?;
                }
                if rsp_frame.cmd == expected_rsp {
                    return Ok(rsp_frame);
                }
            }
        }
    }

    /// Wait for an unsolicited indication (e.g. DATA_IND).
    async fn wait_indication(&self, expected_cmd: u8) -> Result<SerialFrame, MacError> {
        self.io.codec_mut().reset();
        let mut rx_buf = [0u8; MAX_FRAME_SIZE];
        loop {
            let n = self
                .io
                .port_mut()
                .read(&mut rx_buf)
                .await
                .map_err(to_mac_error)?;
            if n == 0 {
                return Err(MacError::Other);
            }
            if let Some(frame) = self.io.codec_mut().feed(&rx_buf[..n]).map_err(to_mac_error)? {
                if frame.cmd == expected_cmd {
                    return Ok(frame);
                }
            }
        }
    }
}

// ── MacDriver implementation ────────────────────────────────────

impl<S: SerialPort> MacDriver for SerialMac<S> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::scan_req(&req, &mut payload_buf);
        let rsp = self.transact(CMD_SCAN_REQ, &payload_buf[..plen], CMD_SCAN_CONF).await?;
        PayloadParser::scan_conf(&rsp.payload).map_err(to_mac_error)
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::associate_req(&req, &mut payload_buf);
        let rsp = self
            .transact(CMD_ASSOCIATE_REQ, &payload_buf[..plen], CMD_ASSOCIATE_CONF)
            .await?;
        PayloadParser::associate_conf(&rsp.payload).map_err(to_mac_error)
    }

    async fn mlme_associate_response(
        &mut self,
        rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::associate_rsp(&rsp, &mut payload_buf);
        let resp = self
            .transact(CMD_ASSOCIATE_RSP, &payload_buf[..plen], CMD_STATUS)
            .await?;
        let status = PayloadParser::status(&resp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)
    }

    async fn mlme_disassociate(
        &mut self,
        req: MlmeDisassociateRequest,
    ) -> Result<(), MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::disassociate_req(&req, &mut payload_buf);
        let rsp = self
            .transact(CMD_DISASSOCIATE_REQ, &payload_buf[..plen], CMD_STATUS)
            .await?;
        let status = PayloadParser::status(&rsp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::reset_req(set_default_pib, &mut payload_buf);
        let rsp = self
            .transact(CMD_RESET_REQ, &payload_buf[..plen], CMD_RESET_CONF)
            .await?;
        let status = PayloadParser::reset_conf(&rsp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::start_req(&req, &mut payload_buf);
        let rsp = self
            .transact(CMD_START_REQ, &payload_buf[..plen], CMD_STATUS)
            .await?;
        let status = PayloadParser::status(&rsp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::get_req(attr, &mut payload_buf);
        let rsp = self
            .transact(CMD_GET_REQ, &payload_buf[..plen], CMD_GET_CONF)
            .await?;
        let (status, value) = PayloadParser::get_conf(&rsp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)?;
        Ok(value)
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::set_req(attr, &value, &mut payload_buf);
        let rsp = self
            .transact(CMD_SET_REQ, &payload_buf[..plen], CMD_SET_CONF)
            .await?;
        let status = PayloadParser::set_conf(&rsp.payload).map_err(to_mac_error)?;
        status_to_mac_error(status)
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        let rsp = self.transact(CMD_POLL_REQ, &[], CMD_POLL_CONF).await?;
        PayloadParser::poll_conf(&rsp.payload).map_err(to_mac_error)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let mut payload_buf = [0u8; MAX_PAYLOAD_SIZE];
        let plen = PayloadBuilder::data_req(&req, &mut payload_buf);
        let rsp = self
            .transact(CMD_DATA_REQ, &payload_buf[..plen], CMD_DATA_CONF)
            .await?;
        PayloadParser::data_conf(&rsp.payload).map_err(to_mac_error)
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        let frame = self.wait_indication(CMD_DATA_IND).await?;
        PayloadParser::data_ind(&frame.payload).map_err(to_mac_error)
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(8),
        }
    }
}
