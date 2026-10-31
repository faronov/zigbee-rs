//! Portable MAC state layered over a hardware radio PHY.
//!
//! [`SoftMacCore`] implements every MLME/MCPS procedure (scan, association,
//! polling, CSMA-CA, retries, ACK wait, pending RX, and — for the
//! [`Parent`] role — association responses, beacon/orphan responses and
//! indirect frame-pending) once, on top of a thin [`RadioPhy`] +
//! [`PlatformServices`] backend.
//!
//! Optional behaviour is selected at compile time and costs nothing when a
//! backend does not use it:
//!
//! * the transmit path through the [`RadioPhy::Tx`] marker type
//!   ([`SoftwareTx`], [`AckWaitTx`], [`FullTx`]): only the selected path is
//!   compiled, and its future has exactly the size that path needs;
//! * poll policy through [`RadioPhy::POLL_RESPONSE_WAIT_US`] and
//!   [`RadioPhy::POLL_EXTENDED_FALLBACK`];
//! * the parent role through the `R` type parameter ([`EndDevice`] is a
//!   zero-sized type, [`Parent`] owns const-generic queues);
//! * the retained-RX depth through the `RX` const parameter.
//!
//! Every receive window (ACK wait, poll response, association response,
//! post-association capture, data indication, beacon collection) is the one
//! [`SoftMacCore::listen`] loop. A received frame is classified, retained and
//! acknowledged synchronously, so no future ever holds a received frame
//! across an await.
//!
//! See `docs/book/src/advanced/soft-mac-porting.md`.

use zigbee_types::{MacAddress, PanId, ShortAddress};

use crate::frames::{
    build_association_request, build_beacon_request, build_data_frame, build_data_request,
    build_data_request_short, build_disassociation_notification, parse_association_response,
    parse_beacon, parse_mac_addresses,
};
pub use crate::phy::AckResult;
use crate::phy::TxParams;
use crate::pib::scan_duration_us;
use crate::{
    AssociationStatus, EdValue, MacCapabilities, MacDriver, MacError, MacFrame, MacPib,
    McpsDataConfirm, McpsDataIndication, McpsDataRequest, MlmeAssociateConfirm,
    MlmeAssociateRequest, MlmeAssociateResponse, MlmeDisassociateRequest, MlmeScanConfirm,
    MlmeScanRequest, MlmeStartRequest, PanDescriptor, PhyAddressFilter, PhyError, PhyRxFrame,
    PibAttribute, PibError, PibValue, PlatformServices, RadioPhy, ScanType,
};
use zigbee_types::TxPower;

mod parent;
pub use parent::Parent;

const UNIT_BACKOFF_PERIOD_US: u32 = 320;
const ACK_WAIT_US: u32 = 1_200;
const ASSOCIATION_DIRECT_WAIT_US: u32 = 500_000;
const POLL_RESPONSE_WAIT_US: u32 = 30_000;
const POST_ASSOCIATION_RX_US: u32 = 250_000;
const ASSOCIATION_POLL_DELAY_US: u32 = 100_000;
const DATA_INDICATION_WAIT_US: u32 = 1_000_000;
const MAX_ASSOCIATION_POLLS: u8 = 32;
/// Upper bound on the frames handled by one receive window or receive loop,
/// so a PHY whose clock stalls cannot keep the MAC spinning forever.
const MAX_RX_FRAMES: u16 = 256;
/// Default retained-RX depth (`RX` const parameter of [`SoftMacCore`]).
pub const DEFAULT_PENDING_RX: usize = 4;

type PanDescriptors = heapless::Vec<PanDescriptor, { crate::primitives::MAX_PAN_DESCRIPTORS }>;

mod tx_sealed {
    pub trait Sealed {}
}

/// Transmit-path offload level of a [`RadioPhy`], selected by the
/// [`RadioPhy::Tx`] associated type.
///
/// Sealed: the three levels are [`SoftwareTx`], [`AckWaitTx`] and
/// [`FullTx`]. Because the path is selected by *type*, only the selected
/// transmit future exists after monomorphisation: a backend pays neither
/// flash for the other paths nor future (stack) space for their states.
pub trait TxOffload: tx_sealed::Sealed {
    /// The PHY waits for the Imm-Ack itself
    /// ([`RadioPhy::try_transmit_acknowledged`]).
    #[doc(hidden)]
    const ACK_WAIT: bool = false;

    /// Transmit `frame` with the PIB CSMA-CA parameters and, when
    /// `ack_requested`, ACK wait and `macMaxFrameRetries` retransmissions.
    /// An unacknowledged transmission returns `AckResult::default()`.
    ///
    /// Implementations return an `async move` block rather than being an
    /// `async fn`: an `async fn` keeps every argument twice in its future.
    #[doc(hidden)]
    fn transmit<'a, P: RadioPhy + PlatformServices, R: MacRole, const RX: usize>(
        core: &'a mut SoftMacCore<P, R, RX>,
        frame: &'a [u8],
        ack_requested: bool,
    ) -> impl Future<Output = Result<AckResult, MacError>> + 'a;
}

/// Software CSMA-CA, software ACK wait and software retries. The PHY
/// implements only [`RadioPhy::try_transmit`] (one CCA + TX) and
/// [`RadioPhy::send_ack`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SoftwareTx;

/// The PHY performs one CCA + TX and waits for the matching Imm-Ack in its
/// ISR or radio core ([`RadioPhy::try_transmit_acknowledged`]); the software
/// MAC keeps the CSMA-CA backoff and `macMaxFrameRetries` policy.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AckWaitTx;

/// The PHY performs the complete unslotted CSMA-CA, ACK wait and
/// retransmission sequence ([`RadioPhy::transmit_offloaded`]) using the
/// [`TxParams`] supplied from the MAC PIB.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FullTx;

impl tx_sealed::Sealed for SoftwareTx {}
impl tx_sealed::Sealed for AckWaitTx {}
impl tx_sealed::Sealed for FullTx {}

impl TxOffload for SoftwareTx {
    async fn transmit<P: RadioPhy + PlatformServices, R: MacRole, const RX: usize>(
        core: &mut SoftMacCore<P, R, RX>,
        frame: &[u8],
        ack_requested: bool,
    ) -> Result<AckResult, MacError> {
        let sequence = match (ack_requested, frame.get(2)) {
            (false, _) => 0,
            (true, Some(sequence)) => *sequence,
            (true, None) => return Err(MacError::InvalidParameter),
        };
        let mut retries = core.pib.max_frame_retries();
        loop {
            core.csma(frame, false).await?;
            if !ack_requested {
                return Ok(AckResult::default());
            }
            if let Some(Heard::Ack(ack)) = core.listen(ACK_WAIT_US, Want::Ack(sequence)).await? {
                return Ok(ack);
            }
            if retries == 0 {
                return Err(MacError::NoAck);
            }
            retries -= 1;
        }
    }
}

impl TxOffload for AckWaitTx {
    const ACK_WAIT: bool = true;

    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn transmit<'a, P: RadioPhy + PlatformServices, R: MacRole, const RX: usize>(
        core: &'a mut SoftMacCore<P, R, RX>,
        frame: &'a [u8],
        ack_requested: bool,
    ) -> impl Future<Output = Result<AckResult, MacError>> + 'a {
        async move {
            if ack_requested && frame.len() < 3 {
                return Err(MacError::InvalidParameter);
            }
            let mut retries = core.pib.max_frame_retries();
            loop {
                if let Some(ack) = core.csma(frame, ack_requested).await? {
                    return Ok(ack);
                }
                if retries == 0 {
                    return Err(MacError::NoAck);
                }
                retries -= 1;
            }
        }
    }
}

impl TxOffload for FullTx {
    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn transmit<'a, P: RadioPhy + PlatformServices, R: MacRole, const RX: usize>(
        core: &'a mut SoftMacCore<P, R, RX>,
        frame: &'a [u8],
        ack_requested: bool,
    ) -> impl Future<Output = Result<AckResult, MacError>> + 'a {
        async move {
            let params = core.tx_params();
            match core
                .phy
                .transmit_offloaded(frame, ack_requested, params)
                .await
            {
                Ok(Some(ack)) => Ok(ack),
                Ok(None) if ack_requested => Err(MacError::NoAck),
                Ok(None) => Ok(AckResult::default()),
                Err(error) => Err(phy_error(error)),
            }
        }
    }
}

/// What a [`SoftMacCore::listen`] window is waiting for.
enum Want<'a> {
    /// Nothing: retain and acknowledge traffic until the window closes.
    Nothing,
    /// Return as soon as a frame was retained (data indication / command).
    Retained,
    /// The Imm-Ack with this sequence number.
    Ack(u8),
    /// Our parent's indirect data frame; it is queued at the front.
    PollResponse,
    /// An Association Response addressed to us.
    AssocResponse,
    /// Beacons on this channel, merged into the descriptor list. Nothing is
    /// retained or acknowledged during a scan.
    Beacons(u8, &'a mut PanDescriptors),
}

/// Outcome of a [`SoftMacCore::listen`] window that ended early.
enum Heard {
    Ack(AckResult),
    /// A frame was retained (pending RX or, on a parent, the command queue).
    Queued,
    Association(ShortAddress, u8),
}

fn phy_error(error: PhyError) -> MacError {
    match error {
        PhyError::ChannelBusy => MacError::ChannelAccessFailure,
        PhyError::InvalidChannel => MacError::InvalidParameter,
        PhyError::FrameTooLong => MacError::FrameTooLong,
        PhyError::CrcFailed | PhyError::Hardware => MacError::RadioError,
        PhyError::Unsupported => MacError::Unsupported,
    }
}

fn pib_error(error: PibError) -> MacError {
    match error {
        PibError::InvalidValue | PibError::ReadOnly => MacError::InvalidParameter,
    }
}

fn address_filter(pib: &MacPib) -> Option<PhyAddressFilter> {
    (!pib.promiscuous()).then_some(PhyAddressFilter {
        pan_id: pib.pan_id(),
        short_address: pib.short_address(),
        extended_address: pib.extended_address(),
    })
}

fn upsert_pan_descriptor(descriptors: &mut PanDescriptors, descriptor: PanDescriptor) {
    if let Some(existing) = descriptors.iter_mut().find(|existing| {
        existing.channel == descriptor.channel
            && existing.coord_address == descriptor.coord_address
            && existing.zigbee_beacon.extended_pan_id == descriptor.zigbee_beacon.extended_pan_id
    }) {
        if descriptor.lqi > existing.lqi {
            *existing = descriptor;
        }
    } else {
        let _ = descriptors.push(descriptor);
    }
}

mod role_sealed {
    pub trait Sealed {}
}

/// Compile-time MAC role of a [`SoftMacCore`].
///
/// Sealed: the two roles are [`EndDevice`] (zero-sized, the default) and
/// [`Parent`]. Every shared receive path tests `R::PARENT`, a constant, so an
/// end-device image contains no parent command parsing, queues or
/// frame-pending logic.
pub trait MacRole: role_sealed::Sealed {
    /// True for a router/coordinator parent.
    const PARENT: bool;

    #[doc(hidden)]
    fn new_role() -> Self;

    /// Frame Pending bit for a software ACK to a Data Request from `source`.
    #[doc(hidden)]
    fn frame_pending_for(&self, _source: &MacAddress) -> bool {
        false
    }

    /// Queue a received parent-side MAC command.
    #[doc(hidden)]
    fn queue_command(&mut self, _data: &[u8], _lqi: u8) {}

    /// True (once) after a Data Request was queued: receive loops end early
    /// so the runtime can service the child poll promptly.
    #[doc(hidden)]
    fn take_service_request(&mut self) -> bool {
        false
    }

    /// Drop every queued command, association transaction and pending flag.
    #[doc(hidden)]
    fn clear_role(&mut self) {}
}

/// End-device / sleepy-end-device role: no parent state at all.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EndDevice;

impl role_sealed::Sealed for EndDevice {}

impl MacRole for EndDevice {
    const PARENT: bool = false;

    fn new_role() -> Self {
        Self
    }
}

/// Fixed-capacity FIFO of retained received frames.
///
/// Like `heapless::Deque`, but with byte-sized indices (the frames are
/// byte-aligned, so this saves the padding of two `usize` indices and a flag
/// in every copy of the MAC), and a full queue evicts its oldest frame.
///
/// The PHY receives straight into the free slot behind the last frame
/// ([`Self::rx_slot`]); [`Self::commit_back`] / [`Self::commit_front`] then
/// retain it without copying. A frame that is not retained simply leaves the
/// slot free again.
struct RxQueue<const N: usize> {
    frames: [PhyRxFrame; N],
    head: u8,
    len: u8,
}

impl<const N: usize> RxQueue<N> {
    fn new() -> Self {
        const { assert!(N > 0 && N <= u8::MAX as usize, "RX must be 1..=255") };
        Self {
            frames: [const { PhyRxFrame::EMPTY }; N],
            head: 0,
            len: 0,
        }
    }

    fn len(&self) -> usize {
        usize::from(self.len)
    }

    #[cfg(test)]
    fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    fn wrap(index: usize) -> u8 {
        (if index >= N { index - N } else { index }) as u8
    }

    /// The `index`-th oldest frame; `index` must be below [`Self::len`].
    fn get(&self, index: usize) -> &PhyRxFrame {
        &self.frames[usize::from(Self::wrap(usize::from(self.head) + index))]
    }

    fn front(&self) -> Option<&PhyRxFrame> {
        (self.len != 0).then(|| &self.frames[usize::from(self.head)])
    }

    fn drop_front(&mut self) {
        if self.len != 0 {
            self.head = Self::wrap(usize::from(self.head) + 1);
            self.len -= 1;
        }
    }

    fn pop_front(&mut self) -> Option<PhyRxFrame> {
        let frame = *self.front()?;
        self.drop_front();
        Some(frame)
    }

    fn tail(&self) -> usize {
        usize::from(Self::wrap(usize::from(self.head) + self.len()))
    }

    /// The free slot the next received frame is written to. A full queue
    /// evicts its oldest frame to make room.
    fn rx_slot(&mut self) -> &mut PhyRxFrame {
        if self.len() == N {
            self.drop_front();
        }
        let tail = self.tail();
        &mut self.frames[tail]
    }

    /// The frame last written to [`Self::rx_slot`].
    fn received(&self) -> &PhyRxFrame {
        &self.frames[self.tail()]
    }

    /// Retain the received frame as the newest one.
    fn commit_back(&mut self) {
        self.len += 1;
    }

    /// Retain the received frame as the oldest one (consumed next).
    fn commit_front(&mut self) {
        let tail = self.tail();
        self.head = Self::wrap(usize::from(self.head) + N - 1);
        // The free slots run from the tail to just before the head, so the
        // received frame moves to the new head without touching any other
        // retained frame.
        self.frames[usize::from(self.head)] = self.frames[tail];
        self.len += 1;
    }

    /// Remove the `index`-th oldest frame, keeping the others in order.
    fn remove(&mut self, index: usize) {
        // Shift the older frames one slot towards the tail, over the removed
        // one, then release the head slot.
        let mut slot = usize::from(Self::wrap(usize::from(self.head) + index));
        for _ in 0..index {
            let previous = usize::from(Self::wrap(slot + N - 1));
            self.frames[slot] = self.frames[previous];
            slot = previous;
        }
        self.drop_front();
    }

    #[cfg(test)]
    fn push_back(&mut self, frame: PhyRxFrame) {
        *self.rx_slot() = frame;
        self.commit_back();
    }

    #[cfg(test)]
    fn push_front(&mut self, frame: PhyRxFrame) {
        *self.rx_slot() = frame;
        self.commit_front();
    }
}

/// Shared software-MAC state for one radio.
///
/// * `P` — the radio backend ([`RadioPhy`] + [`PlatformServices`]).
/// * `R` — [`EndDevice`] (default) or [`Parent`] for a router.
/// * `RX` — how many received frames may be retained while the MAC waits for
///   an ACK, poll response or association response.
pub struct SoftMacCore<P: RadioPhy, R: MacRole = EndDevice, const RX: usize = DEFAULT_PENDING_RX> {
    phy: P,
    pib: MacPib,
    pending_rx: RxQueue<RX>,
    pending_error: Option<MacError>,
    random_state: u32,
    role: R,
}

impl<P: RadioPhy> SoftMacCore<P> {
    /// Construct the default end-device MAC (`RX` = [`DEFAULT_PENDING_RX`]).
    pub fn new(phy: P, pib: MacPib) -> Result<Self, MacError> {
        Self::new_custom(phy, pib)
    }
}

impl<P: RadioPhy, R: MacRole, const RX: usize> SoftMacCore<P, R, RX> {
    /// Construct a MAC with an explicit role and retained-RX depth, e.g.
    /// `SoftMacCore::<Phy, Parent, 8>::new_custom(phy, pib)`.
    pub fn new_custom(mut phy: P, pib: MacPib) -> Result<Self, MacError> {
        Self::apply_full_config(&mut phy, &pib)?;
        let random_state = Self::random_seed(&pib);
        Ok(Self {
            phy,
            pib,
            pending_rx: RxQueue::new(),
            pending_error: None,
            random_state,
            role: R::new_role(),
        })
    }

    pub fn phy(&self) -> &P {
        &self.phy
    }

    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    pub fn into_phy(self) -> P {
        self.phy
    }

    pub fn pib(&self) -> &MacPib {
        &self.pib
    }

    pub fn get_pib(&self, attr: PibAttribute) -> PibValue {
        self.pib.get(attr)
    }

    /// MLME-SET. A value whose radio side effect fails is not committed.
    pub fn set_pib(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        let previous = self.pib.get(attr);
        self.pib.set(attr, value).map_err(pib_error)?;
        if let Err(error) = self.apply_changed_config(attr) {
            // `previous` was read from a valid PIB, so restoring it succeeds.
            let _ = self.pib.set(attr, previous);
            return Err(error);
        }
        Ok(())
    }

    pub fn reset_pib(&mut self, dsn: u8, bsn: u8) -> Result<(), MacError> {
        let next = MacPib::new(self.pib.extended_address(), dsn, bsn);
        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        self.pending_rx.clear();
        self.pending_error = None;
        self.random_state = Self::random_seed(&self.pib);
        self.clear_role_state();
        Ok(())
    }

    pub fn take_pending_rx(&mut self) -> Option<PhyRxFrame> {
        self.pending_rx.pop_front()
    }

    pub fn accepts_destination(&self, destination: &MacAddress) -> bool {
        if self.pib.promiscuous() {
            return true;
        }

        match destination {
            MacAddress::Short(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && (*address == self.pib.short_address() || address.0 == 0xFFFF)
            }
            MacAddress::Extended(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && *address == self.pib.extended_address()
            }
        }
    }

    pub fn is_exact_destination(&self, destination: &MacAddress) -> bool {
        match destination {
            MacAddress::Short(pan_id, address) => {
                address.0 != 0xFFFF
                    && *pan_id == self.pib.pan_id()
                    && *address == self.pib.short_address()
            }
            MacAddress::Extended(pan_id, address) => {
                *pan_id == self.pib.pan_id() && *address == self.pib.extended_address()
            }
        }
    }

    /// Apply the radio side effect of the (already updated) PIB attribute.
    fn apply_changed_config(&mut self, attr: PibAttribute) -> Result<(), MacError> {
        use PibAttribute::*;
        match attr {
            PhyCurrentChannel => self.phy.set_channel(self.pib.current_channel()),
            PhyTransmitPower => self.phy.set_tx_power(self.pib.transmit_power()),
            MacShortAddress | MacPanId | MacExtendedAddress | MacPromiscuousMode => {
                self.phy.set_address_filter(address_filter(&self.pib))
            }
            MacRxOnWhenIdle => {
                self.phy.set_rx_on_when_idle(self.pib.rx_on_when_idle());
                Ok(())
            }
            _ => Ok(()),
        }
        .map_err(phy_error)
    }

    fn apply_full_config(phy: &mut P, pib: &MacPib) -> Result<(), MacError> {
        phy.set_channel(pib.current_channel()).map_err(phy_error)?;
        phy.set_tx_power(pib.transmit_power()).map_err(phy_error)?;
        phy.set_address_filter(address_filter(pib))
            .map_err(phy_error)?;
        phy.set_rx_on_when_idle(pib.rx_on_when_idle());
        Ok(())
    }

    /// Tune the radio, then commit `macCurrentChannel`.
    fn set_channel(&mut self, channel: u8) -> Result<(), MacError> {
        if !(11..=26).contains(&channel) {
            return Err(MacError::InvalidParameter);
        }
        self.phy.set_channel(channel).map_err(phy_error)?;
        self.pib.current_channel = channel;
        Ok(())
    }

    /// Program the address filter, then commit `macPANId` and
    /// `macShortAddress`.
    fn set_addresses(
        &mut self,
        pan_id: PanId,
        short_address: ShortAddress,
    ) -> Result<(), MacError> {
        let filter = (!self.pib.promiscuous()).then_some(PhyAddressFilter {
            pan_id,
            short_address,
            extended_address: self.pib.extended_address(),
        });
        self.phy.set_address_filter(filter).map_err(phy_error)?;
        self.pib.pan_id = pan_id;
        self.pib.short_address = short_address;
        Ok(())
    }

    fn clear_role_state(&mut self) {
        if R::PARENT {
            self.role.clear_role();
            if self.phy.capabilities().hardware_auto_ack {
                self.phy.clear_ack_frame_pending();
            }
        }
    }

    fn tx_params(&self) -> TxParams {
        TxParams {
            max_csma_backoffs: self.pib.max_csma_backoffs(),
            min_be: self.pib.min_be(),
            max_be: self.pib.max_be(),
            max_frame_retries: self.pib.max_frame_retries(),
        }
    }

    fn random_seed(pib: &MacPib) -> u32 {
        let address = pib.extended_address();
        let seed = u32::from_le_bytes([address[0], address[1], address[2], address[3]])
            ^ u32::from_le_bytes([address[4], address[5], address[6], address[7]])
            ^ u32::from(pib.dsn())
            ^ (u32::from(pib.bsn()) << 8)
            ^ 0x9E37_79B9;
        seed.max(1)
    }
}

/// A [`SoftMacCore`] is a [`zigbee_crypto::ForwardAesProvider`] by delegating to its phy,
/// so a platform whose `RadioPhy` overrides `forward_cipher` with a
/// hardware AES backend (e.g. the BL702 phy under `hardware-aes-bl702`)
/// serves CCM*/AES-MMO from hardware, while every phy that keeps the
/// default software provider yields the exact software behaviour as before.
impl<P: RadioPhy + PlatformServices, R: MacRole, const RX: usize> zigbee_crypto::ForwardAesProvider
    for SoftMacCore<P, R, RX>
{
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        self.phy.forward_cipher(key)
    }
}
impl<P: RadioPhy + PlatformServices, R: MacRole, const RX: usize> PlatformServices
    for SoftMacCore<P, R, RX>
{
    fn monotonic_micros(&self) -> u32 {
        self.phy.monotonic_micros()
    }

    fn delay_micros(&mut self, duration_us: u32) -> impl Future<Output = ()> {
        self.phy.delay_micros(duration_us)
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.phy.fill_random(output)
    }
}

impl<P: RadioPhy + PlatformServices, R: MacRole, const RX: usize> SoftMacCore<P, R, RX> {
    /// MLME-ASSOCIATE: send the Association Request, then poll for the
    /// response (one transmit, one listen and one delay await in a loop).
    #[allow(clippy::manual_async_fn)] // avoids a second copy of by-value args
    pub fn associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> impl Future<Output = Result<MlmeAssociateConfirm, MacError>> + '_ {
        // Not an `async fn`, see `send_data`.
        async move {
            self.prepare_association(&req)?;

            let mut polls = 0u8;
            loop {
                let ack = {
                    let sequence = self.pib.next_dsn();
                    let own = self.pib.extended_address();
                    let frame = if polls == 0 {
                        build_association_request(
                            sequence,
                            &req.coord_address,
                            &own,
                            &req.capability_info,
                        )
                    } else {
                        let mut frame = heapless::Vec::new();
                        let _ = frame.extend_from_slice(&build_data_request(
                            sequence,
                            &req.coord_address,
                            &own,
                        ));
                        frame
                    };
                    self.transmit(&frame, true).await?
                };

                // The direct response window follows the request; after a poll,
                // a Frame Pending ACK announces the queued response, so wait the
                // backend's poll-response window for it. Otherwise listen only
                // briefly before the next poll.
                let wait_us = if polls == 0 {
                    ASSOCIATION_DIRECT_WAIT_US
                } else if ack.frame_pending {
                    P::POLL_RESPONSE_WAIT_US
                } else {
                    POLL_RESPONSE_WAIT_US
                };
                let response = match self.take_pending_association_response() {
                    Some(response) => Some(response),
                    None => match self.listen(wait_us, Want::AssocResponse).await? {
                        Some(Heard::Association(address, status)) => Some((address, status)),
                        _ => None,
                    },
                };
                if let Some(response) = response {
                    let confirm = self.finish_association(response)?;
                    if confirm.status == AssociationStatus::Success
                        && let Err(error) = self.listen(POST_ASSOCIATION_RX_US, Want::Nothing).await
                    {
                        self.pending_error = Some(error);
                    }
                    return Ok(confirm);
                }

                if polls != 0 {
                    self.phy.delay_micros(ASSOCIATION_POLL_DELAY_US).await;
                }
                if polls == MAX_ASSOCIATION_POLLS {
                    return Err(MacError::NoData);
                }
                polls += 1;
            }
        }
    }

    /// MLME-DISASSOCIATE (direct notification to the parent).
    pub fn disassociate(
        &mut self,
        req: MlmeDisassociateRequest,
    ) -> impl Future<Output = Result<(), MacError>> + '_ {
        let notification = self.disassociation_notification(&req);
        self.send_disassociation(notification)
    }

    fn disassociation_notification(
        &mut self,
        req: &MlmeDisassociateRequest,
    ) -> Result<heapless::Vec<u8, 32>, MacError> {
        if req.tx_indirect {
            return Err(MacError::Unsupported);
        }
        if self.pib.short_address().0 >= 0xFFF8
            || req.device_address.pan_id() != self.pib.pan_id()
            || match req.device_address {
                MacAddress::Short(_, address) => address.0 >= 0xFFF8,
                MacAddress::Extended(_, address) => address == [0; 8] || address == [0xFF; 8],
            }
        {
            return Err(MacError::InvalidParameter);
        }

        Ok(build_disassociation_notification(
            self.pib.next_dsn(),
            &req.device_address,
            self.pib.short_address(),
            &self.pib.extended_address(),
            req.reason,
        ))
    }

    // Not an `async fn`, see `send_data`.
    #[allow(clippy::manual_async_fn)] // avoids a second copy of by-value args
    fn send_disassociation(
        &mut self,
        notification: Result<heapless::Vec<u8, 32>, MacError>,
    ) -> impl Future<Output = Result<(), MacError>> + '_ {
        async move {
            let frame = match &notification {
                Ok(frame) => frame,
                Err(error) => return Err(*error),
            };
            let transmit_result = self.transmit(frame, true).await;
            self.clear_association()?;
            transmit_result.map(|_| ())
        }
    }

    pub fn reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            let random = self.next_random_u32();
            return self.reset_pib(random as u8, (random >> 8) as u8);
        }

        Self::apply_full_config(&mut self.phy, &self.pib)?;
        self.pending_rx.clear();
        self.pending_error = None;
        self.clear_role_state();
        Ok(())
    }

    /// Put the radio into its lowest-power idle state ([`RadioPhy::sleep`]).
    pub fn sleep(&mut self) -> Result<(), MacError> {
        self.phy.sleep().map_err(phy_error)
    }

    /// Wake the radio ([`RadioPhy::wake`]) and re-apply channel, TX power
    /// and address filter from the PIB.
    pub fn wake(&mut self) -> Result<(), MacError> {
        self.phy.wake().map_err(phy_error)?;
        Self::apply_full_config(&mut self.phy, &self.pib)
    }

    /// MCPS-DATA. The frame is built synchronously, so the returned future
    /// holds only the frame and the transmit state, not the request.
    pub fn transmit_data(
        &mut self,
        req: McpsDataRequest<'_>,
    ) -> impl Future<Output = Result<McpsDataConfirm, MacError>> + '_ {
        let frame = self.data_frame(&req);
        self.send_data(frame, req.msdu_handle)
    }

    /// Build the MPDU of a direct MCPS-DATA request and decide whether it
    /// requests an ACK.
    fn data_frame(
        &mut self,
        req: &McpsDataRequest<'_>,
    ) -> Result<(heapless::Vec<u8, 125>, bool), MacError> {
        if req.tx_options.indirect {
            return Err(MacError::Unsupported);
        }
        if req.tx_options.security_enabled {
            return Err(MacError::SecurityError);
        }
        if req.src_addr_mode == crate::AddressMode::Short && self.pib.short_address().0 >= 0xFFF8 {
            return Err(MacError::InvalidParameter);
        }

        let ack_requested = req.tx_options.ack_tx
            && !matches!(
                req.dst_address,
                MacAddress::Short(_, address) if address.0 == 0xFFFF
            );
        let frame = build_data_frame(
            self.pib.next_dsn(),
            req.src_addr_mode,
            self.pib.short_address(),
            &self.pib.extended_address(),
            &req.dst_address,
            req.payload,
            ack_requested,
            req.tx_options.frame_pending,
        )
        .map_err(|_| MacError::FrameTooLong)?;
        Ok((frame, ack_requested))
    }

    // Not an `async fn`: its body would move `frame` from the argument into
    // a second local copy. The `async move` block only borrows the capture.
    #[allow(clippy::manual_async_fn)] // avoids a second copy of by-value args
    fn send_data(
        &mut self,
        frame: Result<(heapless::Vec<u8, 125>, bool), MacError>,
        msdu_handle: u8,
    ) -> impl Future<Output = Result<McpsDataConfirm, MacError>> + '_ {
        async move {
            let (frame, ack_requested) = match &frame {
                Ok((frame, ack_requested)) => (frame, *ack_requested),
                Err(error) => return Err(*error),
            };
            self.transmit(frame, ack_requested).await?;
            Ok(McpsDataConfirm {
                msdu_handle,
                timestamp: None,
            })
        }
    }

    /// Send one MAC Data Request to the parent and return the indirect frame
    /// it delivers, if any.
    ///
    /// PHYs with [`RadioPhy::POLL_EXTENDED_FALLBACK`] follow a short-source
    /// poll that returned nothing with a second, extended-source poll.
    pub fn poll(&mut self) -> impl Future<Output = Result<Option<MacFrame>, MacError>> + '_ {
        self.poll_within(u32::MAX)
    }

    /// [`Self::poll`] with the `mlme_poll_timeout` contract: a frame whose
    /// poll completed `timeout_us` or later after the call is discarded.
    ///
    /// A short-source poll that returned nothing is followed (with
    /// [`RadioPhy::POLL_EXTENDED_FALLBACK`]) by a second, extended-source
    /// poll that retrieves a frame the parent queued by our EUI-64 (Rejoin
    /// Response, Transport-Key). A transmit failure of that optional second
    /// poll is reported as "no data" so it never masks the first result.
    pub async fn poll_within(&mut self, timeout_us: u32) -> Result<Option<MacFrame>, MacError> {
        let started_at = self.phy.monotonic_micros();
        let frame = 'poll: {
            if let Some(frame) = self.take_pending_data() {
                break 'poll Some(frame);
            }
            if let Some(error) = self.pending_error.take() {
                return Err(error);
            }

            let (coordinator, own_short, has_short_source, has_extended_source) =
                self.poll_addresses()?;
            let mut short_source = has_short_source;
            let mut first_pass = true;
            loop {
                let sent = {
                    let sequence = self.pib.next_dsn();
                    let request = if short_source {
                        build_data_request_short(sequence, &coordinator, own_short)
                    } else {
                        build_data_request(sequence, &coordinator, &self.pib.extended_address())
                    };
                    self.transmit(&request, true).await
                };
                let ack = match sent {
                    Ok(ack) => ack,
                    Err(error) if first_pass => return Err(error),
                    Err(_) => break 'poll None,
                };
                if let Some(frame) = self.take_pending_data() {
                    break 'poll Some(frame);
                }
                // IEEE 802.15.4-2015 §6.7.3: after Frame Pending the parent
                // delivers the buffered frame from itself to this device;
                // `listen` queues it at the front of pending RX.
                if ack.frame_pending
                    && self
                        .listen(P::POLL_RESPONSE_WAIT_US, Want::PollResponse)
                        .await?
                        .is_some()
                {
                    break 'poll self.take_pending_data();
                }
                if !(P::POLL_EXTENDED_FALLBACK && short_source && has_extended_source) {
                    break 'poll None;
                }
                short_source = false;
                first_pass = false;
            }
        };
        let elapsed = self.phy.monotonic_micros().wrapping_sub(started_at);
        Ok(frame.filter(|_| elapsed < timeout_us))
    }

    /// Resolve the parent address and our preferred source address for a
    /// Data Request. Returns `(coordinator, own_short, has_short_source,
    /// has_extended_source)`.
    fn poll_addresses(&self) -> Result<(MacAddress, ShortAddress, bool, bool), MacError> {
        let own_short = self.pib.short_address();
        let coordinator_short = self.pib.coord_short_address();
        let coordinator_extended = self.pib.coord_extended_address();
        let has_short_source = own_short.0 < 0xFFF8;
        let has_extended_source = self.pib.extended_address() != [0; 8];
        let coordinator = if coordinator_short.0 < 0xFFF8 {
            MacAddress::Short(self.pib.pan_id(), coordinator_short)
        } else if coordinator_extended != [0; 8] {
            MacAddress::Extended(self.pib.pan_id(), coordinator_extended)
        } else {
            return Err(MacError::InvalidParameter);
        };
        if !has_short_source && !has_extended_source {
            return Err(MacError::InvalidParameter);
        }
        Ok((
            coordinator,
            own_short,
            has_short_source,
            has_extended_source,
        ))
    }

    fn is_from_coordinator(&self, source: &MacAddress) -> bool {
        let coordinator_short = self.pib.coord_short_address();
        let coordinator_extended = self.pib.coord_extended_address();
        match source {
            MacAddress::Short(pan_id, address) => {
                coordinator_short.0 < 0xFFF8
                    && *address == coordinator_short
                    && *pan_id == self.pib.pan_id()
            }
            MacAddress::Extended(_, address) => {
                coordinator_extended != [0; 8] && *address == coordinator_extended
            }
        }
    }

    /// MCPS-DATA.indication: the next retained data frame, listening up to
    /// `timeout_us` for one.
    pub async fn receive_data(&mut self, timeout_us: u32) -> Result<McpsDataIndication, MacError> {
        let started_at = self.phy.monotonic_micros();

        for _ in 0..MAX_RX_FRAMES {
            // Queued frames were acknowledged when they were queued.
            if let Some(indication) = self.take_pending_indication() {
                return Ok(indication);
            }
            if R::PARENT && self.role.take_service_request() {
                return Err(MacError::NoData);
            }
            if let Some(error) = self.pending_error.take() {
                return Err(error);
            }
            let elapsed = self.phy.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                break;
            };
            if self.listen(remaining, Want::Retained).await?.is_none() {
                break;
            }
        }

        Err(MacError::NoData)
    }

    /// MLME-SCAN (ED, active or passive). The address filter is disabled for
    /// the scan; the original channel and filter are restored afterwards,
    /// also when the scan fails.
    #[allow(clippy::manual_async_fn)] // avoids a second copy of by-value args
    pub fn scan(
        &mut self,
        req: MlmeScanRequest,
    ) -> impl Future<Output = Result<MlmeScanConfirm, MacError>> + '_ {
        // Not an `async fn`, see `send_data`.
        async move {
            if req.scan_duration > 14 {
                return Err(MacError::InvalidParameter);
            }
            if req.scan_type == ScanType::Orphan {
                return Err(MacError::Unsupported);
            }
            let duration_us = u32::try_from(scan_duration_us(req.scan_duration))
                .map_err(|_| MacError::InvalidParameter)?;

            let original_channel = self.pib.current_channel();
            self.phy.set_address_filter(None).map_err(phy_error)?;

            let mut confirm = MlmeScanConfirm {
                scan_type: req.scan_type,
                pan_descriptors: heapless::Vec::new(),
                energy_list: heapless::Vec::new(),
            };
            let scan_result: Result<(), MacError> = 'scan: {
                for channel in req.channel_mask.iter() {
                    let channel = channel.number();
                    if let Err(error) = self.set_channel(channel) {
                        break 'scan Err(error);
                    }

                    if req.scan_type == ScanType::Ed {
                        match self.phy.energy_detect(duration_us).await {
                            Ok(energy) => {
                                let _ = confirm.energy_list.push(EdValue { channel, energy });
                            }
                            Err(error) => break 'scan Err(phy_error(error)),
                        }
                        continue;
                    }
                    if req.scan_type == ScanType::Active {
                        let sent = {
                            let beacon_request = build_beacon_request(self.pib.next_dsn());
                            self.transmit(&beacon_request, false).await
                        };
                        match sent {
                            Ok(_) => {}
                            // A busy channel only means no beacon request went
                            // out on it; keep scanning the remaining channels.
                            Err(MacError::ChannelAccessFailure) => continue,
                            Err(error) => break 'scan Err(error),
                        }
                    }
                    if let Err(error) = self
                        .listen(
                            duration_us,
                            Want::Beacons(channel, &mut confirm.pan_descriptors),
                        )
                        .await
                    {
                        break 'scan Err(error);
                    }
                }
                Ok(())
            };

            self.set_channel(original_channel)?;
            self.phy
                .set_address_filter(address_filter(&self.pib))
                .map_err(phy_error)?;
            scan_result?;

            if req.scan_type != ScanType::Ed && confirm.pan_descriptors.is_empty() {
                return Err(MacError::NoBeacon);
            }
            Ok(confirm)
        }
    }

    /// Transmit `frame` through the backend's [`RadioPhy::Tx`] path.
    fn transmit<'s>(
        &'s mut self,
        frame: &'s [u8],
        ack_requested: bool,
    ) -> impl Future<Output = Result<AckResult, MacError>> + 's {
        P::Tx::transmit(self, frame, ack_requested)
    }

    /// Transmit `frame` with unslotted CSMA-CA and no ACK request.
    pub async fn transmit_csma(&mut self, frame: &[u8]) -> Result<(), MacError> {
        self.transmit(frame, false).await.map(|_| ())
    }

    /// Transmit `frame` (ACK request set) with CSMA-CA, ACK wait and
    /// `macMaxFrameRetries` retransmissions, using the backend's
    /// [`RadioPhy::Tx`] path.
    pub fn transmit_acknowledged<'s>(
        &'s mut self,
        frame: &'s [u8],
    ) -> impl Future<Output = Result<AckResult, MacError>> + 's {
        self.transmit(frame, true)
    }

    /// Unslotted CSMA-CA around one PHY transmit attempt. With `ack_wait`
    /// (only for an [`AckWaitTx`] PHY) the attempt includes the PHY's
    /// hardware ACK wait; otherwise a sent frame reports a default ACK.
    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn csma<'s>(
        &'s mut self,
        frame: &'s [u8],
        ack_wait: bool,
    ) -> impl Future<Output = Result<Option<AckResult>, MacError>> + 's {
        async move {
            let mut backoff_count = 0u8;
            let mut backoff_exponent = self.pib.min_be();

            loop {
                let slots = self.random_backoff_slots(backoff_exponent);
                if slots != 0 {
                    self.phy
                        .delay_micros(u32::from(slots) * UNIT_BACKOFF_PERIOD_US)
                        .await;
                }

                let result = if <P::Tx as TxOffload>::ACK_WAIT && ack_wait {
                    self.phy.try_transmit_acknowledged(frame).await
                } else {
                    self.phy
                        .try_transmit(frame)
                        .await
                        .map(|()| Some(AckResult::default()))
                };
                match result {
                    Ok(ack) => return Ok(ack),
                    Err(PhyError::ChannelBusy) if backoff_count < self.pib.max_csma_backoffs() => {
                        backoff_count = backoff_count.saturating_add(1);
                        backoff_exponent =
                            core::cmp::min(backoff_exponent.saturating_add(1), self.pib.max_be());
                    }
                    Err(error) => return Err(phy_error(error)),
                }
            }
        }
    }

    fn random_backoff_slots(&mut self, backoff_exponent: u8) -> u16 {
        let random = self.next_random_u32();
        let mask = (1u16 << backoff_exponent) - 1;
        random as u16 & mask
    }

    fn next_random_u32(&mut self) -> u32 {
        let mut value = self.random_state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.random_state = value.max(1);
        value
    }

    /// The single receive loop of the MAC.
    ///
    /// Listens up to `timeout_us` and returns early when a frame matching
    /// `want` was heard. Every frame is received straight into the
    /// retained-frame queue's free slot and classified by [`Self::accept`]
    /// (synchronously, so no frame is held across an await or copied through
    /// the stack); a software Imm-Ack, when one is due, is sent afterwards.
    async fn listen(
        &mut self,
        timeout_us: u32,
        mut want: Want<'_>,
    ) -> Result<Option<Heard>, MacError> {
        let started_at = self.phy.monotonic_micros();

        for _ in 0..MAX_RX_FRAMES {
            let elapsed = self.phy.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                break;
            };
            let (ack, heard) = match self.phy.receive(remaining, self.pending_rx.rx_slot()).await {
                Ok(true) => self.accept(&mut want),
                Ok(false) => break,
                Err(PhyError::CrcFailed) => continue,
                Err(error) => return Err(phy_error(error)),
            };
            // Software Imm-Ack fallback for PHYs without hardware / ISR
            // auto-ACK. A busy channel is not fatal: the ACK is simply not
            // sent and the originator retransmits, exactly as if it had been
            // lost on air.
            if let Some((sequence, frame_pending)) = ack {
                match self.phy.send_ack(sequence, frame_pending).await {
                    Ok(()) | Err(PhyError::ChannelBusy) => {}
                    Err(error) => return Err(phy_error(error)),
                }
            }
            if heard.is_some() {
                return Ok(heard);
            }
        }

        Ok(None)
    }

    /// Classify one received frame for [`Self::listen`].
    ///
    /// * During a scan only beacons are used.
    /// * An Imm-Ack ends the window only when it matches `Want::Ack`.
    /// * Data frames and MAC commands addressed exactly to us with the ACK
    ///   request bit get a software ACK (unless the PHY auto-ACKs).
    /// * Retained frames: data frames to an accepted destination (pending
    ///   RX), MAC commands on a parent (command queue), and Association
    ///   Responses addressed to us (pending RX, unless it is the response
    ///   being waited for). Everything else is dropped.
    ///
    /// Returns the software ACK to send (`sequence`, `frame_pending`) and
    /// the early-exit outcome. The frame is the one just written to
    /// [`RxQueue::rx_slot`]; retaining it commits that slot.
    fn accept(&mut self, want: &mut Want<'_>) -> (Option<(u8, bool)>, Option<Heard>) {
        let frame = self.pending_rx.received();
        let data = frame.as_slice();
        if data.len() < 3 {
            return (None, None);
        }
        let kind = data[0] & 0x07;
        if let Want::Beacons(channel, descriptors) = want {
            if kind == 0x00
                && let Some(descriptor) = parse_beacon(*channel, data, frame.lqi)
            {
                upsert_pan_descriptor(descriptors, descriptor);
            }
            return (None, None);
        }
        if kind == 0x02 {
            let heard = match want {
                Want::Ack(sequence) if data[2] == *sequence => Some(Heard::Ack(AckResult {
                    frame_pending: data[0] & (1 << 4) != 0,
                })),
                _ => None,
            };
            return (None, heard);
        }
        let command = kind == 0x03;
        if kind != 0x01 && !command {
            return (None, None);
        }

        let (source, destination, payload_offset, _) = parse_mac_addresses(data);
        if payload_offset < 3 || payload_offset > data.len() {
            return (None, None);
        }
        let exact = self.is_exact_destination(&destination);
        let ack = (data[0] & (1 << 5) != 0 && exact && !self.phy.capabilities().hardware_auto_ack)
            .then(|| {
                (
                    data[2],
                    R::PARENT && command && self.role.frame_pending_for(&source),
                )
            });

        let retained = matches!(want, Want::Retained).then_some(Heard::Queued);
        let association = if command && exact {
            parse_association_response(data)
        } else {
            None
        };
        let heard = if let Some((address, status)) = association {
            if matches!(want, Want::AssocResponse) {
                Some(Heard::Association(address, status))
            } else {
                // Kept for `take_pending_association_response`.
                self.pending_rx.commit_back();
                None
            }
        } else if !self.accepts_destination(&destination) {
            None
        } else if command {
            if R::PARENT {
                let lqi = frame.lqi;
                self.role.queue_command(data, lqi);
                retained
            } else {
                None
            }
        } else if matches!(want, Want::PollResponse) && exact && self.is_from_coordinator(&source) {
            // The poll response is consumed next, ahead of other traffic.
            self.pending_rx.commit_front();
            Some(Heard::Queued)
        } else {
            // Kept for a later `receive_data` / `poll`.
            self.pending_rx.commit_back();
            retained
        };
        (ack, heard)
    }

    /// Convert an already-acknowledged frame into a data indication.
    ///
    /// Non-data frames and frames for other destinations yield `None`.
    fn indication_from(&self, frame: &PhyRxFrame) -> Option<McpsDataIndication> {
        let data = frame.as_slice();
        if data.len() < 3 || data[0] & 0x07 != 0x01 {
            return None;
        }

        let (source, destination, payload_offset, security_use) = parse_mac_addresses(data);
        if payload_offset < 3
            || payload_offset > data.len()
            || !self.accepts_destination(&destination)
        {
            return None;
        }

        Some(McpsDataIndication {
            src_address: source,
            dst_address: destination,
            lqi: frame.lqi,
            payload: MacFrame::from_slice(&data[payload_offset..])?,
            security_use,
        })
    }

    /// The next queued data indication; non-data frames are discarded.
    ///
    /// Converts the frame in place before dropping it from the queue, so no
    /// copy of the received frame is made in the caller's (poll) frame.
    fn take_pending_indication(&mut self) -> Option<McpsDataIndication> {
        while let Some(frame) = self.pending_rx.front() {
            let indication = self.indication_from(frame);
            self.pending_rx.drop_front();
            if indication.is_some() {
                return indication;
            }
        }
        None
    }

    fn take_pending_data(&mut self) -> Option<MacFrame> {
        self.take_pending_indication()
            .map(|indication| indication.payload)
    }

    fn prepare_association(&mut self, req: &MlmeAssociateRequest) -> Result<(), MacError> {
        if self.pib.extended_address() == [0; 8]
            || self.pib.extended_address() == [0xFF; 8]
            || req.coord_address.pan_id().0 == 0xFFFF
            || match req.coord_address {
                MacAddress::Short(_, address) => address.0 >= 0xFFF8,
                MacAddress::Extended(_, address) => address == [0; 8] || address == [0xFF; 8],
            }
        {
            return Err(MacError::InvalidParameter);
        }

        self.set_channel(req.channel)?;
        self.set_addresses(req.coord_address.pan_id(), ShortAddress(0xFFFF))?;
        let (coord_short, coord_extended) = match req.coord_address {
            MacAddress::Short(_, address) => (address, [0; 8]),
            MacAddress::Extended(_, address) => (ShortAddress(0xFFFF), address),
        };
        self.pib.coord_short_address = coord_short;
        self.pib.coord_extended_address = coord_extended;
        self.pib.associated_pan_coord = false;
        self.pending_rx.clear();
        self.pending_error = None;
        Ok(())
    }

    fn clear_association(&mut self) -> Result<(), MacError> {
        self.set_addresses(PanId(0xFFFF), ShortAddress(0xFFFF))?;
        self.pib.coord_short_address = ShortAddress(0xFFFF);
        self.pib.coord_extended_address = [0; 8];
        self.pib.associated_pan_coord = false;
        self.pending_rx.clear();
        self.pending_error = None;
        Ok(())
    }

    /// Remove the oldest retained Association Response addressed to us; the
    /// other retained frames keep their order.
    fn take_pending_association_response(&mut self) -> Option<(ShortAddress, u8)> {
        let (index, response) = (0..self.pending_rx.len()).find_map(|index| {
            self.association_response_for_us(self.pending_rx.get(index))
                .map(|response| (index, response))
        })?;
        self.pending_rx.remove(index);
        Some(response)
    }

    fn association_response_for_us(&self, frame: &PhyRxFrame) -> Option<(ShortAddress, u8)> {
        let data = frame.as_slice();
        let (_, destination, payload_offset, _) = parse_mac_addresses(data);
        (payload_offset >= 3
            && payload_offset <= data.len()
            && self.is_exact_destination(&destination))
        .then(|| parse_association_response(data))
        .flatten()
    }

    /// Commit a successful Association Response to the PIB.
    fn finish_association(
        &mut self,
        (short_address, status): (ShortAddress, u8),
    ) -> Result<MlmeAssociateConfirm, MacError> {
        let status = match status {
            0x00 => AssociationStatus::Success,
            0x01 => AssociationStatus::PanAtCapacity,
            _ => AssociationStatus::PanAccessDenied,
        };
        if status == AssociationStatus::Success {
            if short_address.0 >= 0xFFF8 {
                return Err(MacError::AssociationDenied);
            }
            self.set_addresses(self.pib.pan_id(), short_address)?;
            self.pib.associated_pan_coord = true;
        }
        Ok(MlmeAssociateConfirm {
            short_address,
            status,
        })
    }
}

/// `MacDriver` methods shared verbatim by the end-device and parent roles.
///
/// The asynchronous methods return the inner future directly instead of
/// wrapping it in another `async fn`, so the request is stored once and no
/// extra future layer is added to every NWK caller.
macro_rules! soft_mac_driver_common {
    () => {
        fn mlme_scan(
            &mut self,
            req: MlmeScanRequest,
        ) -> impl Future<Output = Result<MlmeScanConfirm, MacError>> {
            self.scan(req)
        }

        fn mlme_associate(
            &mut self,
            req: MlmeAssociateRequest,
        ) -> impl Future<Output = Result<MlmeAssociateConfirm, MacError>> {
            self.associate(req)
        }

        fn mlme_disassociate(
            &mut self,
            req: MlmeDisassociateRequest,
        ) -> impl Future<Output = Result<(), MacError>> {
            self.disassociate(req)
        }

        fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
            self.reset(set_default_pib)
        }

        async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
            Ok(self.get_pib(attr))
        }

        fn mlme_set(
            &mut self,
            attr: PibAttribute,
            value: PibValue,
        ) -> impl Future<Output = Result<(), MacError>> {
            core::future::ready(self.set_pib(attr, value))
        }

        fn mlme_poll(&mut self) -> impl Future<Output = Result<Option<MacFrame>, MacError>> {
            self.poll_within(u32::MAX)
        }

        fn mlme_poll_timeout(
            &mut self,
            timeout_us: u32,
        ) -> impl Future<Output = Result<Option<MacFrame>, MacError>> {
            self.poll_within(timeout_us)
        }

        fn mcps_data(
            &mut self,
            req: McpsDataRequest<'_>,
        ) -> impl Future<Output = Result<McpsDataConfirm, MacError>> {
            self.transmit_data(req)
        }

        fn mcps_data_indication(
            &mut self,
        ) -> impl Future<Output = Result<McpsDataIndication, MacError>> {
            self.receive_data(DATA_INDICATION_WAIT_US)
        }

        fn mcps_data_indication_timeout(
            &mut self,
            timeout_us: u32,
        ) -> impl Future<Output = Result<McpsDataIndication, MacError>> {
            self.receive_data(timeout_us)
        }

        fn capabilities(&self) -> MacCapabilities {
            let phy = self.phy.capabilities();
            MacCapabilities {
                coordinator: false,
                router: <Self as RoleOf>::Role::PARENT,
                hardware_security: false,
                max_payload: 102,
                tx_power_min: TxPower(phy.tx_power_min),
                tx_power_max: TxPower(phy.tx_power_max),
            }
        }
    };
}
use soft_mac_driver_common;

/// Names the role type of a `SoftMacCore` inside the shared macro body.
trait RoleOf {
    type Role: MacRole;
}

impl<P: RadioPhy, R: MacRole, const RX: usize> RoleOf for SoftMacCore<P, R, RX> {
    type Role = R;
}

impl<P: RadioPhy + PlatformServices, const RX: usize> MacDriver for SoftMacCore<P, EndDevice, RX> {
    soft_mac_driver_common!();

    fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> impl Future<Output = Result<(), MacError>> {
        core::future::ready(Err(MacError::Unsupported))
    }

    fn mlme_start(&mut self, req: MlmeStartRequest) -> impl Future<Output = Result<(), MacError>> {
        core::future::ready(crate::start_requires_parent_capability(&req))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;

    use super::*;
    use crate::{
        AddressMode, MacCommandEvent, MlmeBeaconResponse, MlmeOrphanResponse, PhyCapabilities,
        TxOptions, pib::PibPayload,
    };
    use zigbee_types::{ChannelMask, PanId, ShortAddress};

    const IEEE: [u8; 8] = [0x02, 1, 2, 3, 4, 5, 6, 7];

    /// `T`: transmit offload marker. `FALLBACK` = `POLL_EXTENDED_FALLBACK`.
    struct TestPhy<T: TxOffload = SoftwareTx, const FALLBACK: bool = false> {
        offload: core::marker::PhantomData<T>,
        channel: u8,
        tx_power: i8,
        filter: Option<PhyAddressFilter>,
        fail_channel: Option<u8>,
        tx_attempts: u8,
        tx_results: heapless::Deque<Result<(), PhyError>, 8>,
        rx_frames: heapless::Deque<Result<Option<PhyRxFrame>, PhyError>, 8>,
        delayed_us: u32,
        last_tx: heapless::Vec<u8, { crate::MAX_PHY_FRAME_LEN }>,
        energy: Result<u8, PhyError>,
        capabilities: PhyCapabilities,
        sent_acks: heapless::Vec<(u8, bool), 8>,
        ack_result: Result<(), PhyError>,
        now_us: u32,
        offload_results: heapless::Deque<Result<Option<AckResult>, PhyError>, 8>,
        offload_calls: heapless::Vec<(bool, TxParams), 8>,
        hw_pending: heapless::Vec<(MacAddress, bool), 8>,
        hw_pending_cleared: bool,
        rx_on_when_idle: Option<bool>,
    }

    impl<T: TxOffload, const FALLBACK: bool> TestPhy<T, FALLBACK> {
        fn new() -> Self {
            Self {
                offload: core::marker::PhantomData,
                channel: 0,
                tx_power: i8::MIN,
                filter: None,
                fail_channel: None,
                tx_attempts: 0,
                tx_results: heapless::Deque::new(),
                rx_frames: heapless::Deque::new(),
                delayed_us: 0,
                last_tx: heapless::Vec::new(),
                energy: Ok(0),
                capabilities: PhyCapabilities {
                    tx_power_min: -8,
                    tx_power_max: 4,
                    ..Default::default()
                },
                sent_acks: heapless::Vec::new(),
                ack_result: Ok(()),
                now_us: 123,
                offload_results: heapless::Deque::new(),
                offload_calls: heapless::Vec::new(),
                hw_pending: heapless::Vec::new(),
                hw_pending_cleared: false,
                rx_on_when_idle: None,
            }
        }
    }

    impl<T: TxOffload + 'static, const FALLBACK: bool> RadioPhy for TestPhy<T, FALLBACK> {
        type Tx = T;
        const POLL_EXTENDED_FALLBACK: bool = FALLBACK;

        fn capabilities(&self) -> PhyCapabilities {
            self.capabilities
        }

        async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
            self.tx_attempts = self.tx_attempts.saturating_add(1);
            self.last_tx.clear();
            self.last_tx
                .extend_from_slice(frame)
                .map_err(|_| PhyError::FrameTooLong)?;
            self.tx_results.pop_front().unwrap_or(Ok(()))
        }

        async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
            self.ack_result?;
            self.sent_acks
                .push((sequence, frame_pending))
                .map_err(|_| PhyError::Hardware)
        }

        async fn receive(
            &mut self,
            _timeout_us: u32,
            frame: &mut PhyRxFrame,
        ) -> Result<bool, PhyError> {
            match self.rx_frames.pop_front().unwrap_or(Ok(None)) {
                Ok(Some(received)) => {
                    *frame = received;
                    Ok(true)
                }
                Ok(None) => Ok(false),
                Err(error) => Err(error),
            }
        }

        fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
            if self.fail_channel == Some(channel) {
                return Err(PhyError::Hardware);
            }
            self.channel = channel;
            Ok(())
        }

        fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
            self.tx_power = dbm;
            Ok(())
        }

        async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
            self.energy
        }

        fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
            self.filter = filter;
            Ok(())
        }

        async fn try_transmit_acknowledged(
            &mut self,
            frame: &[u8],
        ) -> Result<Option<AckResult>, PhyError> {
            assert!(T::ACK_WAIT, "AckWait hook used by a non-AckWait PHY");
            self.try_transmit(frame).await?;
            self.offload_results.pop_front().unwrap_or(Ok(None))
        }

        async fn transmit_offloaded(
            &mut self,
            frame: &[u8],
            ack_requested: bool,
            params: TxParams,
        ) -> Result<Option<AckResult>, PhyError> {
            assert_eq!(
                core::any::TypeId::of::<T>(),
                core::any::TypeId::of::<FullTx>(),
                "Full offload hook used by a non-Full PHY"
            );
            self.last_tx.clear();
            self.last_tx
                .extend_from_slice(frame)
                .map_err(|_| PhyError::FrameTooLong)?;
            self.offload_calls
                .push((ack_requested, params))
                .map_err(|_| PhyError::Hardware)?;
            self.offload_results.pop_front().unwrap_or(Ok(None))
        }

        fn set_ack_frame_pending(
            &mut self,
            source: &MacAddress,
            pending: bool,
        ) -> Result<(), PhyError> {
            self.hw_pending
                .push((*source, pending))
                .map_err(|_| PhyError::Hardware)
        }

        fn clear_ack_frame_pending(&mut self) {
            self.hw_pending_cleared = true;
        }

        fn set_rx_on_when_idle(&mut self, enabled: bool) {
            self.rx_on_when_idle = Some(enabled);
        }
    }

    impl<T: TxOffload, const FALLBACK: bool> zigbee_crypto::ForwardAesProvider
        for TestPhy<T, FALLBACK>
    {
    }
    impl<T: TxOffload + 'static, const FALLBACK: bool> PlatformServices for TestPhy<T, FALLBACK> {
        fn monotonic_micros(&self) -> u32 {
            self.now_us
        }

        async fn delay_micros(&mut self, duration_us: u32) {
            self.delayed_us = self.delayed_us.saturating_add(duration_us);
        }

        fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
            output.fill(3);
            Ok(())
        }
    }

    struct NoopWake;

    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);

        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    fn core() -> SoftMacCore<TestPhy> {
        core_of()
    }

    fn core_of<T: TxOffload + 'static, const FALLBACK: bool>() -> SoftMacCore<TestPhy<T, FALLBACK>>
    {
        SoftMacCore::new(TestPhy::new(), MacPib::new(IEEE, 1, 2)).unwrap()
    }

    #[test]
    fn rx_on_when_idle_follows_the_pib_and_full_reconfiguration() {
        let mut core = core();
        assert_eq!(core.phy().rx_on_when_idle, Some(false));

        core.set_pib(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))
            .unwrap();
        assert_eq!(core.phy().rx_on_when_idle, Some(true));

        core.phy_mut().rx_on_when_idle = None;
        core.wake().unwrap();
        assert_eq!(core.phy().rx_on_when_idle, Some(true));

        core.set_pib(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(false))
            .unwrap();
        assert_eq!(core.phy().rx_on_when_idle, Some(false));
    }

    fn association_response(
        sequence: u8,
        assigned_address: ShortAddress,
        status: u8,
    ) -> heapless::Vec<u8, 24> {
        let mut frame = heapless::Vec::new();
        frame.extend_from_slice(&0x8C63u16.to_le_bytes()).unwrap();
        frame.push(sequence).unwrap();
        frame.extend_from_slice(&0x1234u16.to_le_bytes()).unwrap();
        frame.extend_from_slice(&IEEE).unwrap();
        frame.extend_from_slice(&0x0000u16.to_le_bytes()).unwrap();
        frame.push(0x02).unwrap();
        frame
            .extend_from_slice(&assigned_address.0.to_le_bytes())
            .unwrap();
        frame.push(status).unwrap();
        frame
    }

    fn associate_request() -> MlmeAssociateRequest {
        MlmeAssociateRequest {
            channel: 15,
            coord_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            capability_info: crate::CapabilityInfo {
                allocate_address: true,
                ..Default::default()
            },
        }
    }

    fn associated_core() -> SoftMacCore<TestPhy> {
        associated_core_of()
    }

    fn associated_core_of<T: TxOffload + 'static, const FALLBACK: bool>()
    -> SoftMacCore<TestPhy<T, FALLBACK>> {
        let mut core = core_of();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.set_pib(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true))
            .unwrap();
        core
    }

    #[test]
    fn constructor_applies_pib_to_phy() {
        let core = core();
        assert_eq!(core.phy().channel, 11);
        assert_eq!(core.phy().tx_power, 0);
        assert_eq!(
            core.phy().filter,
            Some(PhyAddressFilter {
                pan_id: PanId(0xFFFF),
                short_address: ShortAddress(0xFFFF),
                extended_address: IEEE,
            })
        );
    }

    #[test]
    fn failed_phy_update_does_not_commit_pib_state() {
        let mut core = core();
        core.phy_mut().fail_channel = Some(26);

        assert_eq!(
            core.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(26)),
            Err(MacError::RadioError)
        );
        assert_eq!(
            core.get_pib(PibAttribute::PhyCurrentChannel),
            PibValue::U8(11)
        );
    }

    #[test]
    fn destination_filter_accepts_exact_and_broadcast_addresses() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();

        let exact = MacAddress::Short(PanId(0x1234), ShortAddress(0x5678));
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));
        let other = MacAddress::Short(PanId(0x1234), ShortAddress(0x9ABC));

        assert!(core.accepts_destination(&exact));
        assert!(core.is_exact_destination(&exact));
        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
        assert!(!core.accepts_destination(&other));
    }

    #[test]
    fn unassociated_address_is_not_an_exact_broadcast_match() {
        let core = core();
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));

        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
    }

    #[test]
    fn promiscuous_mode_disables_hardware_and_software_filtering() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPromiscuousMode, PibValue::Bool(true))
            .unwrap();

        assert_eq!(core.phy().filter, None);
        assert!(core.accepts_destination(&MacAddress::Short(PanId(0x1111), ShortAddress(0x2222))));
    }

    #[test]
    fn platform_services_delegate_to_phy() {
        let mut core = core();
        let mut random = [0; 4];
        core.fill_random(&mut random).unwrap();

        assert_eq!(core.monotonic_micros(), 123);
        assert_eq!(random, [3; 4]);
    }

    #[test]
    fn csma_retries_busy_channel_with_pib_backoff_policy() {
        let mut core = core();
        core.phy_mut()
            .tx_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        core.phy_mut()
            .tx_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        core.phy_mut().tx_results.push_back(Ok(())).unwrap();

        block_on(core.transmit_csma(&[1, 2, 3])).unwrap();

        assert_eq!(core.phy().tx_attempts, 3);
        assert!(core.phy().delayed_us >= UNIT_BACKOFF_PERIOD_US);
    }

    #[test]
    fn acknowledged_transmit_retries_until_matching_ack() {
        let mut core = core();
        core.phy_mut().rx_frames.push_back(Ok(None)).unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x12, 0x00, 0x2A], 255).unwrap(),
            )))
            .unwrap();

        let result = block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert_eq!(
            result,
            AckResult {
                frame_pending: true
            }
        );
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn ack_wait_preserves_non_ack_frames() {
        let mut core = core();
        let data = PhyRxFrame::from_slice(&[0x41, 0x88, 0x10, 0xAA], 100).unwrap();
        core.phy_mut().rx_frames.push_back(Ok(Some(data))).unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x2A], 255).unwrap(),
            )))
            .unwrap();

        block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert_eq!(core.take_pending_rx(), Some(data));
    }

    #[test]
    fn acknowledged_transmit_stops_after_configured_retries() {
        let mut core = core();
        core.set_pib(PibAttribute::MacMaxFrameRetries, PibValue::U8(2))
            .unwrap();

        assert_eq!(
            block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])),
            Err(MacError::NoAck)
        );
        assert_eq!(core.phy().tx_attempts, 3);
    }

    #[test]
    fn active_scan_collects_beacon_and_restores_radio_config() {
        let mut core = core();
        let beacon = [
            0x00, 0x80, 0x55, 0xE9, 0xDF, 0x2D, 0x7D, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x22, 0x84, 1,
            2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0x09,
        ];
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&beacon, 200).unwrap())))
            .unwrap();
        core.phy_mut().rx_frames.push_back(Ok(None)).unwrap();

        let confirm = block_on(core.scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask(1 << 15),
            scan_duration: 3,
        }))
        .unwrap();

        assert_eq!(confirm.pan_descriptors.len(), 1);
        assert_eq!(confirm.pan_descriptors[0].channel, 15);
        assert_eq!(core.phy().channel, 11);
        assert!(core.phy().filter.is_some());
        assert_eq!(&core.phy().last_tx[..3], &[0x03, 0x08, 1]);
    }

    #[test]
    fn ed_scan_uses_phy_measurement_and_restores_channel() {
        let mut core = core();
        core.phy_mut().energy = Ok(177);

        let confirm = block_on(core.scan(MlmeScanRequest {
            scan_type: ScanType::Ed,
            channel_mask: ChannelMask(1 << 20),
            scan_duration: 2,
        }))
        .unwrap();

        assert_eq!(confirm.energy_list.len(), 1);
        assert_eq!(confirm.energy_list[0].channel, 20);
        assert_eq!(confirm.energy_list[0].energy, 177);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn scan_rejects_invalid_duration_without_changing_filter() {
        let mut core = core();
        let filter = core.phy().filter;

        assert!(matches!(
            block_on(core.scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask: ChannelMask(1 << 15),
                scan_duration: 15,
            })),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().filter, filter);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn transmit_data_uses_shared_frame_builder_and_ack_engine() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let confirm = block_on(core.transmit_data(McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            payload: &[0xAA, 0xBB],
            msdu_handle: 7,
            tx_options: TxOptions {
                ack_tx: true,
                ..TxOptions::default()
            },
        }))
        .unwrap();

        assert_eq!(confirm.msdu_handle, 7);
        assert_eq!(
            core.phy().last_tx.as_slice(),
            [
                0x61, 0x88, 0x01, 0x34, 0x12, 0x00, 0x00, 0x78, 0x56, 0xAA, 0xBB
            ]
        );
    }

    #[test]
    fn receive_data_sends_software_ack_for_exact_unicast() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x2A,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xCC],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 180).unwrap())))
            .unwrap();

        let indication = block_on(core.receive_data(5_000)).unwrap();

        assert_eq!(indication.payload.as_slice(), [0xCC]);
        assert_eq!(indication.lqi, 180);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x2A, false)]);
    }

    #[test]
    fn transmit_data_clears_ack_request_for_broadcast() {
        let mut core = core();

        block_on(core.transmit_data(McpsDataRequest {
            src_addr_mode: AddressMode::Extended,
            dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            payload: &[0xAA],
            msdu_handle: 8,
            tx_options: TxOptions {
                ack_tx: true,
                ..TxOptions::default()
            },
        }))
        .unwrap();

        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().last_tx[0] & (1 << 5), 0);
    }

    #[test]
    fn receive_data_never_acks_broadcast() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        let incoming = build_data_frame(
            0x2B,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            &[0xDD],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 170).unwrap())))
            .unwrap();

        let indication = block_on(core.receive_data(5_000)).unwrap();

        assert_eq!(indication.payload.as_slice(), [0xDD]);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn hardware_auto_ack_suppresses_software_ack() {
        let mut core = core();
        core.phy_mut().capabilities.hardware_auto_ack = true;
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x2C,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xEE],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 160).unwrap())))
            .unwrap();

        block_on(core.receive_data(5_000)).unwrap();

        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn association_accepts_a_direct_response_and_commits_pib_state() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacCoordShortAddress),
            PibValue::ShortAddress(ShortAddress(0x0000))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(true)
        );
        assert_eq!(core.phy().channel, 15);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x44, false)]);
    }

    #[test]
    fn association_uses_a_response_retained_during_the_ack_window() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x44, false)]);
    }

    #[test]
    fn retained_association_response_is_taken_without_reordering() {
        let mut core = core();
        core.prepare_association(&associate_request()).unwrap();
        let data = |tag: u8| PhyRxFrame::from_slice(&[0x41, 0x88, tag], 200).unwrap();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.pending_rx.push_back(data(1));
        core.pending_rx
            .push_back(PhyRxFrame::from_slice(&response, 210).unwrap());
        core.pending_rx.push_back(data(2));
        core.pending_rx.push_back(data(3));

        assert_eq!(
            core.take_pending_association_response(),
            Some((ShortAddress(0x5678), 0))
        );
        assert_eq!(core.take_pending_association_response(), None);
        let mut order = heapless::Vec::<u8, 4>::new();
        while let Some(frame) = core.pending_rx.pop_front() {
            order.push(frame.as_slice()[2]).unwrap();
        }
        assert_eq!(order.as_slice(), [1, 2, 3]);
    }

    #[test]
    fn association_polls_for_an_indirect_response() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        for frame in [
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap()),
            None,
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x02], 255).unwrap()),
            Some(PhyRxFrame::from_slice(&response, 210).unwrap()),
        ] {
            core.phy_mut().rx_frames.push_back(Ok(frame)).unwrap();
        }

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn association_captures_and_acks_post_response_data() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        let transport_key = build_data_frame(
            0x55,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xA5, 0x5A],
            true,
            false,
        )
        .unwrap();
        for frame in [
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap()),
            Some(PhyRxFrame::from_slice(&response, 210).unwrap()),
            Some(PhyRxFrame::from_slice(&transport_key, 200).unwrap()),
            None,
        ] {
            core.phy_mut().rx_frames.push_back(Ok(frame)).unwrap();
        }

        block_on(core.associate(associate_request())).unwrap();
        let tx_attempts = core.phy().tx_attempts;
        let captured = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(captured.as_slice(), [0xA5, 0x5A]);
        assert_eq!(core.phy().tx_attempts, tx_attempts);
        assert_eq!(
            core.phy().sent_acks.as_slice(),
            [(0x44, false), (0x55, false)]
        );
    }

    #[test]
    fn post_association_capture_error_is_reported_by_the_next_poll() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Err(PhyError::Hardware))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();
        let tx_attempts = core.phy().tx_attempts;

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert!(matches!(block_on(core.poll()), Err(MacError::RadioError)));
        assert_eq!(core.phy().tx_attempts, tx_attempts);
    }

    #[test]
    fn association_denial_does_not_assign_a_short_address() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0xFFFE), 1);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::PanAtCapacity);
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
    }

    #[test]
    fn association_rejects_a_broadcast_coordinator() {
        let mut core = core();
        let mut request = associate_request();
        request.coord_address = MacAddress::Short(PanId(0x1234), ShortAddress::BROADCAST);

        assert!(matches!(
            block_on(core.associate(request)),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn disassociation_notifies_the_parent_before_clearing_state() {
        let mut core = associated_core();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        block_on(core.disassociate(MlmeDisassociateRequest {
            device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            reason: crate::DisassociateReason::DeviceLeave,
            tx_indirect: false,
        }))
        .unwrap();

        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().last_tx.last(), Some(&0x02));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn indirect_disassociation_is_rejected_without_changing_state() {
        let mut core = associated_core();

        assert!(matches!(
            block_on(core.disassociate(MlmeDisassociateRequest {
                device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                reason: crate::DisassociateReason::DeviceLeave,
                tx_indirect: true,
            })),
            Err(MacError::Unsupported)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
    }

    #[test]
    fn failed_disassociation_notification_still_clears_local_state() {
        let mut core = associated_core();

        assert!(matches!(
            block_on(core.disassociate(MlmeDisassociateRequest {
                device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                reason: crate::DisassociateReason::DeviceLeave,
                tx_indirect: false,
            })),
            Err(MacError::NoAck)
        ));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
    }

    #[test]
    fn default_reset_preserves_eui_and_restores_portable_pib_defaults() {
        let mut core = associated_core();
        core.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(20))
            .unwrap();

        core.reset(true).unwrap();

        assert_eq!(
            core.get_pib(PibAttribute::MacExtendedAddress),
            PibValue::ExtendedAddress(IEEE)
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::PhyCurrentChannel),
            PibValue::U8(11)
        );
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn non_default_reset_preserves_pib_and_clears_transient_receive_state() {
        let mut core = associated_core();
        core.pending_rx
            .push_back(PhyRxFrame::from_slice(&[0x01, 0x00, 0x33], 100).unwrap());
        core.pending_error = Some(MacError::RadioError);

        core.reset(false).unwrap();

        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
        assert!(core.pending_rx.is_empty());
        assert_eq!(core.pending_error, None);
    }

    #[test]
    fn soft_mac_core_implements_the_end_device_mac_driver_surface() {
        fn assert_mac_driver<T: MacDriver>() {}
        assert_mac_driver::<SoftMacCore<TestPhy>>();

        let mut core = core();
        core.set_pib(PibAttribute::PhyTransmitPower, PibValue::I8(-4))
            .unwrap();
        let capabilities = MacDriver::capabilities(&core);

        assert!(!capabilities.coordinator);
        assert!(!capabilities.router);
        assert!(!capabilities.hardware_security);
        assert_eq!(capabilities.max_payload, 102);
        assert_eq!(capabilities.tx_power_min, TxPower(-8));
        assert_eq!(capabilities.tx_power_max, TxPower(4));
        assert_eq!(
            block_on(MacDriver::mlme_get(&core, PibAttribute::MacExtendedAddress)).unwrap(),
            PibValue::ExtendedAddress(IEEE)
        );
    }

    #[test]
    fn poll_returns_none_when_parent_ack_has_no_pending_data() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacExtendedAddress,
            PibValue::ExtendedAddress([0; 8]),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 1);
    }

    #[test]
    fn poll_returns_indirect_data_when_frame_pending_is_set() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x33,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xA1, 0xB2],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x12, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 190).unwrap())))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xA1, 0xB2]);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x33, false)]);
    }

    #[test]
    fn poll_response_ignores_traffic_not_sent_by_parent_to_us() {
        let mut core = associated_core();
        let neighbour_broadcast = build_data_frame(
            0x40,
            AddressMode::Short,
            ShortAddress(0x1111),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            &[0xBB],
            false,
            false,
        )
        .unwrap();
        let neighbour_unicast = build_data_frame(
            0x41,
            AddressMode::Short,
            ShortAddress(0x2222),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xCC],
            true,
            false,
        )
        .unwrap();
        let parent_broadcast = build_data_frame(
            0x42,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            &[0xDD],
            false,
            false,
        )
        .unwrap();
        let parent_unicast = build_data_frame(
            0x43,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xEE],
            true,
            false,
        )
        .unwrap();
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x12, 0x00, 0x01], 255).unwrap(),
        )))
        .unwrap();
        for frame in [
            &neighbour_broadcast,
            &neighbour_unicast,
            &parent_broadcast,
            &parent_unicast,
        ] {
            rx.push_back(Ok(Some(PhyRxFrame::from_slice(frame, 190).unwrap())))
                .unwrap();
        }

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xEE]);
        // The neighbour unicast is ACKed on receipt and the parent response
        // when returned; broadcasts are never ACKed.
        assert_eq!(
            core.phy().sent_acks.as_slice(),
            [(0x41, false), (0x43, false)]
        );
        // Everything else stays available to normal reception, in order.
        for expected in [[0xBB], [0xCC], [0xDD]] {
            let indication = block_on(core.receive_data(1_000)).unwrap();
            assert_eq!(indication.payload.as_slice(), expected);
        }
        assert_eq!(core.phy().sent_acks.len(), 2);
    }

    #[test]
    fn poll_consumes_data_queued_during_the_ack_window() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x33,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xC3],
            false,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 190).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xC3]);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn ack_window_immediately_acknowledges_queued_unicast_data() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x34,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xD4],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 190).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x44], 255).unwrap(),
            )))
            .unwrap();

        let Some(Heard::Ack(ack)) = block_on(core.listen(ACK_WAIT_US, Want::Ack(0x44))).unwrap()
        else {
            panic!("ACK not matched");
        };

        assert!(!ack.frame_pending);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x34, false)]);
        assert_eq!(core.take_pending_data().unwrap().as_slice(), [0xD4]);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x34, false)]);
    }

    #[test]
    fn poll_uses_short_source_once_when_assigned() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(
            core.phy().last_tx.as_slice(),
            [0x63, 0x88, 0x01, 0x34, 0x12, 0x00, 0x00, 0x78, 0x56, 0x04]
        );
    }

    #[test]
    fn poll_uses_extended_source_before_short_assignment() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(
            &core.phy().last_tx[..7],
            &[0x63, 0xC8, 0x01, 0x34, 0x12, 0x00, 0x00]
        );
        assert_eq!(&core.phy().last_tx[7..15], &IEEE);
        assert_eq!(core.phy().last_tx[15], 0x04);
    }

    #[test]
    fn poll_rejects_missing_coordinator_addresses() {
        let mut core = core();

        assert!(matches!(
            block_on(core.poll()),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
    }

    #[test]
    fn short_source_data_requires_an_assigned_address() {
        let mut core = core();

        assert!(matches!(
            block_on(core.transmit_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                payload: &[0xAA],
                msdu_handle: 9,
                tx_options: TxOptions::default(),
            })),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
    }

    // ── Hardware offload path selection ─────────────────────────

    #[test]
    fn full_offload_hands_csma_ack_and_retries_to_the_phy() {
        let mut core = core_of::<FullTx, false>();
        core.phy_mut()
            .offload_results
            .push_back(Ok(Some(AckResult {
                frame_pending: true,
            })))
            .unwrap();

        let ack = block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert!(ack.frame_pending);
        // The software CSMA/ACK engine is not used at all.
        assert_eq!(core.phy().tx_attempts, 0);
        assert_eq!(core.phy().delayed_us, 0);
        assert_eq!(
            core.phy().offload_calls.as_slice(),
            [(true, core.tx_params())]
        );
    }

    #[test]
    fn full_offload_maps_missing_ack_and_busy_channel() {
        let mut core = core_of::<FullTx, false>();
        core.phy_mut().offload_results.push_back(Ok(None)).unwrap();
        core.phy_mut()
            .offload_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        core.phy_mut().offload_results.push_back(Ok(None)).unwrap();

        assert_eq!(
            block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])),
            Err(MacError::NoAck)
        );
        assert_eq!(
            block_on(core.transmit_csma(&[0x41, 0x88, 0x2B])),
            Err(MacError::ChannelAccessFailure)
        );
        // Without an ACK request `Ok(None)` is a successful transmission.
        assert_eq!(block_on(core.transmit_csma(&[0x41, 0x88, 0x2C])), Ok(()));
        assert_eq!(core.phy().offload_calls.len(), 3);
        assert!(!core.phy().offload_calls[2].0);
        assert_eq!(core.phy().tx_attempts, 0);
    }

    #[test]
    fn ack_wait_offload_keeps_software_backoff_and_retries() {
        let mut core = core_of::<AckWaitTx, false>();
        let phy = core.phy_mut();
        phy.tx_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        phy.tx_results.push_back(Ok(())).unwrap();
        phy.tx_results.push_back(Ok(())).unwrap();
        // First transmission: ACK window closes empty; retry is ACKed.
        phy.offload_results.push_back(Ok(None)).unwrap();
        phy.offload_results
            .push_back(Ok(Some(AckResult::default())))
            .unwrap();
        // A frame the software ACK wait would have consumed stays unread.
        phy.rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x2A], 255).unwrap(),
            )))
            .unwrap();

        let ack = block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert!(!ack.frame_pending);
        assert_eq!(core.phy().tx_attempts, 3);
        assert!(core.phy().delayed_us >= UNIT_BACKOFF_PERIOD_US);
        assert_eq!(core.phy().rx_frames.len(), 1);
        assert!(core.phy().offload_calls.is_empty());
    }

    #[test]
    fn ack_wait_offload_stops_after_configured_retries() {
        let mut core = core_of::<AckWaitTx, false>();
        core.set_pib(PibAttribute::MacMaxFrameRetries, PibValue::U8(1))
            .unwrap();

        assert_eq!(
            block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])),
            Err(MacError::NoAck)
        );
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn busy_software_ack_is_not_fatal() {
        let mut core = associated_core();
        core.phy_mut().ack_result = Err(PhyError::ChannelBusy);
        let incoming = build_data_frame(
            0x2A,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xCC],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 180).unwrap())))
            .unwrap();

        let indication = block_on(core.receive_data(5_000)).unwrap();

        assert_eq!(indication.payload.as_slice(), [0xCC]);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn hardware_ack_error_other_than_busy_is_reported() {
        let mut core = associated_core();
        core.phy_mut().ack_result = Err(PhyError::Hardware);
        let incoming = build_data_frame(
            0x2A,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xCC],
            true,
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 180).unwrap())))
            .unwrap();

        assert_eq!(
            block_on(core.receive_data(5_000)).map(|_| ()),
            Err(MacError::RadioError)
        );
    }

    // ── Poll policy ─────────────────────────────────────────────

    #[test]
    fn extended_fallback_repeats_an_empty_short_poll_with_the_eui64() {
        let mut core = associated_core_of::<SoftwareTx, true>();
        let first = core.pib().dsn();
        let second = first.wrapping_add(1);
        let incoming = build_data_frame(
            0x50,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0x5A],
            false,
            false,
        )
        .unwrap();
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x02, 0x00, first], 255).unwrap(),
        )))
        .unwrap();
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x12, 0x00, second], 255).unwrap(),
        )))
        .unwrap();
        rx.push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 150).unwrap())))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0x5A]);
        assert_eq!(core.phy().tx_attempts, 2);
        // The second Data Request carries the extended source address.
        let last = core.phy().last_tx.as_slice();
        assert_eq!(last[1] & 0xC0, 0xC0);
        assert_eq!(last.last(), Some(&0x04));
    }

    #[test]
    fn extended_fallback_failure_does_not_mask_an_empty_first_poll() {
        let mut core = associated_core_of::<SoftwareTx, true>();
        core.set_pib(PibAttribute::MacMaxFrameRetries, PibValue::U8(0))
            .unwrap();
        let first = core.pib().dsn();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, first], 255).unwrap(),
            )))
            .unwrap();

        // The extended-source poll gets no ACK: reported as "no data".
        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn default_poll_never_repeats_with_the_extended_source() {
        let mut core = associated_core();
        let first = core.pib().dsn();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, first], 255).unwrap(),
            )))
            .unwrap();

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 1);
    }

    #[test]
    fn active_scan_skips_a_busy_channel_and_continues() {
        let mut core = core();
        for _ in 0..=core.pib().max_csma_backoffs() {
            core.phy_mut()
                .tx_results
                .push_back(Err(PhyError::ChannelBusy))
                .unwrap();
        }
        let beacon = [
            0x00, 0x80, 0x55, 0xE9, 0xDF, 0x2D, 0x7D, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x22, 0x84, 1,
            2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0x09,
        ];
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&beacon, 200).unwrap())))
            .unwrap();
        core.phy_mut().rx_frames.push_back(Ok(None)).unwrap();

        let confirm = block_on(core.scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask((1 << 15) | (1 << 16)),
            scan_duration: 3,
        }))
        .unwrap();

        assert_eq!(confirm.pan_descriptors.len(), 1);
        assert_eq!(confirm.pan_descriptors[0].channel, 16);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn end_device_start_is_validated_then_unsupported() {
        let mut core = core();
        let mut req = router_start(15);
        req.channel = 27;
        assert_eq!(
            block_on(MacDriver::mlme_start(&mut core, req)),
            Err(MacError::InvalidParameter)
        );
        assert_eq!(
            block_on(MacDriver::mlme_start(&mut core, router_start(15))),
            Err(MacError::Unsupported)
        );
        assert_eq!(core.phy().channel, 11);
    }

    // ── Parent role ─────────────────────────────────────────────

    const CHILD: [u8; 8] = [0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8];
    const PAN: PanId = PanId(0x1234);
    const ROUTER: ShortAddress = ShortAddress(0x0001);
    const CHILD_SHORT: ShortAddress = ShortAddress(0x4321);

    type ParentCore = SoftMacCore<TestPhy, Parent<4, 2, 2>, 4>;

    fn router_start(channel: u8) -> MlmeStartRequest {
        MlmeStartRequest {
            pan_id: PAN,
            channel,
            beacon_order: 15,
            superframe_order: 15,
            pan_coordinator: false,
            battery_life_ext: false,
        }
    }

    fn parent_core() -> ParentCore {
        let mut core = ParentCore::new_custom(TestPhy::new(), MacPib::new(IEEE, 1, 2)).unwrap();
        core.start(router_start(15)).unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ROUTER),
        )
        .unwrap();
        core
    }

    fn child_response() -> MlmeAssociateResponse {
        MlmeAssociateResponse {
            device_address: CHILD,
            short_address: CHILD_SHORT,
            status: AssociationStatus::Success,
        }
    }

    fn rx(core: &mut ParentCore, frame: &[u8]) {
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(frame, 200).unwrap())))
            .unwrap();
    }

    fn extended_data_request(sequence: u8) -> heapless::Vec<u8, 24> {
        build_data_request(sequence, &MacAddress::Short(PAN, ROUTER), &CHILD)
    }

    fn short_data_request(sequence: u8) -> heapless::Vec<u8, 24> {
        build_data_request_short(sequence, &MacAddress::Short(PAN, ROUTER), CHILD_SHORT)
    }

    #[test]
    fn parent_start_programs_the_radio_and_rejects_coordinator_starts() {
        let mut core = ParentCore::new_custom(TestPhy::new(), MacPib::new(IEEE, 1, 2)).unwrap();
        let mut coordinator = router_start(20);
        coordinator.pan_coordinator = true;
        assert_eq!(core.start(coordinator), Err(MacError::Unsupported));
        assert_eq!(core.phy().channel, 11);

        block_on(MacDriver::mlme_start(&mut core, router_start(20))).unwrap();

        assert_eq!(core.phy().channel, 20);
        assert_eq!(core.pib().pan_id(), PAN);
        assert_eq!(
            core.get_pib(PibAttribute::MacRxOnWhenIdle),
            PibValue::Bool(true)
        );
        assert_eq!(core.phy().rx_on_when_idle, Some(true));
        assert!(MacDriver::capabilities(&core).router);
    }

    #[test]
    fn parent_delivers_association_response_on_extended_data_request() {
        let mut core = parent_core();
        core.associate_response(child_response()).unwrap();
        let response_sequence = core.pib().dsn();
        rx(&mut core, &extended_data_request(0x10));
        rx(&mut core, &[0x02, 0x00, response_sequence]);

        let event = block_on(core.command_event(10_000)).unwrap();

        assert_eq!(
            event,
            MacCommandEvent::AssociationResponseDelivery(crate::MlmeAssociateResponseDelivery {
                device_address: CHILD,
                short_address: CHILD_SHORT,
                status: AssociationStatus::Success,
                result: Ok(()),
            })
        );
        // The software ACK to the Data Request announced the response.
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x10, true)]);
        assert_eq!(
            parse_association_response(&core.phy().last_tx),
            Some((CHILD_SHORT, 0))
        );

        // The transaction is gone: the next poll is ACKed without FP.
        rx(&mut core, &extended_data_request(0x11));
        let event = block_on(core.command_event(10_000)).unwrap();
        assert!(matches!(event, MacCommandEvent::DataRequest(_)));
        assert_eq!(core.phy().sent_acks.last(), Some(&(0x11, false)));
    }

    #[test]
    fn parent_retries_an_unacknowledged_association_response_on_the_next_poll() {
        let mut core = parent_core();
        core.set_pib(PibAttribute::MacMaxFrameRetries, PibValue::U8(0))
            .unwrap();
        core.associate_response(child_response()).unwrap();
        rx(&mut core, &extended_data_request(0x10));

        // No ACK for the response: the poll itself is reported, the
        // transaction stays queued with Frame Pending armed.
        let event = block_on(core.command_event(10_000)).unwrap();
        assert!(matches!(event, MacCommandEvent::DataRequest(_)));

        rx(&mut core, &extended_data_request(0x11));
        let response_sequence = core.pib().dsn();
        rx(&mut core, &[0x02, 0x00, response_sequence]);
        let event = block_on(core.command_event(10_000)).unwrap();
        assert!(matches!(
            event,
            MacCommandEvent::AssociationResponseDelivery(crate::MlmeAssociateResponseDelivery {
                result: Ok(()),
                ..
            })
        ));
        assert_eq!(
            core.phy().sent_acks.as_slice(),
            [(0x10, true), (0x11, true)]
        );
    }

    #[test]
    fn parent_expires_association_transactions_without_listening() {
        let mut core = parent_core();
        core.associate_response(child_response()).unwrap();
        let persistence = u32::from(core.pib().transaction_persistence_time());
        core.phy_mut().now_us = 123 + persistence * 960 * 16;

        let event = block_on(core.command_event(0)).unwrap();

        assert!(matches!(
            event,
            MacCommandEvent::AssociationResponseDelivery(crate::MlmeAssociateResponseDelivery {
                result: Err(MacError::TransactionExpired),
                ..
            })
        ));
        assert_eq!(block_on(core.command_event(0)), Err(MacError::NoData));
        assert_eq!(core.phy().tx_attempts, 0);
    }

    #[test]
    fn parent_software_frame_pending_follows_indirect_data_queue() {
        let mut core = parent_core();
        let child = MacAddress::Short(PAN, CHILD_SHORT);
        core.set_frame_pending(child, true).unwrap();
        rx(&mut core, &short_data_request(0x20));

        let event = block_on(core.command_event(10_000)).unwrap();

        let MacCommandEvent::DataRequest(indication) = event else {
            panic!("expected a Data Request event");
        };
        assert_eq!(indication.source_address, child);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x20, true)]);

        core.set_frame_pending(child, false).unwrap();
        rx(&mut core, &short_data_request(0x21));
        block_on(core.command_event(10_000)).unwrap();
        assert_eq!(core.phy().sent_acks.last(), Some(&(0x21, false)));
    }

    #[test]
    fn parent_frame_pending_table_rejects_invalid_and_overflowing_children() {
        let mut core = parent_core();
        assert_eq!(
            core.set_frame_pending(MacAddress::Short(PanId(0x9999), CHILD_SHORT), true),
            Err(MacError::InvalidParameter)
        );
        assert_eq!(
            core.set_frame_pending(MacAddress::Short(PAN, ShortAddress(0xFFFF)), true),
            Err(MacError::InvalidParameter)
        );
        core.set_frame_pending(MacAddress::Short(PAN, ShortAddress(1)), true)
            .unwrap();
        core.set_frame_pending(MacAddress::Short(PAN, ShortAddress(2)), true)
            .unwrap();
        assert_eq!(
            core.set_frame_pending(MacAddress::Short(PAN, ShortAddress(3)), true),
            Err(MacError::TransactionOverflow)
        );
    }

    #[test]
    fn parent_with_hardware_auto_ack_arms_the_phy_pending_table() {
        let mut core = parent_core();
        core.phy_mut().capabilities.hardware_auto_ack = true;
        core.associate_response(child_response()).unwrap();
        core.set_frame_pending(MacAddress::Short(PAN, CHILD_SHORT), true)
            .unwrap();
        rx(&mut core, &extended_data_request(0x10));
        let response_sequence = core.pib().dsn();
        rx(&mut core, &[0x02, 0x00, response_sequence]);

        block_on(core.command_event(10_000)).unwrap();

        // The PHY acknowledged the poll; the software never sends an ACK.
        assert!(core.phy().sent_acks.is_empty());
        assert_eq!(
            core.phy().hw_pending.as_slice(),
            [
                (MacAddress::Extended(PAN, CHILD), true),
                (MacAddress::Short(PAN, CHILD_SHORT), true),
                (MacAddress::Extended(PAN, CHILD), false),
            ]
        );
        core.reset(false).unwrap();
        assert!(core.phy().hw_pending_cleared);
    }

    #[test]
    fn parent_reports_beacon_requests_and_orphans_without_acking() {
        let mut core = parent_core();
        rx(&mut core, &build_beacon_request(0x30));
        rx(
            &mut core,
            &crate::frames::build_orphan_notification(0x31, &CHILD),
        );

        assert!(matches!(
            block_on(core.command_event(10_000)).unwrap(),
            MacCommandEvent::BeaconRequest(_)
        ));
        let MacCommandEvent::OrphanNotification(orphan) =
            block_on(core.command_event(10_000)).unwrap()
        else {
            panic!("expected an orphan notification");
        };
        assert_eq!(orphan.orphan_address, CHILD);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn parent_reports_association_requests_and_keeps_data_for_mcps() {
        let mut core = parent_core();
        let data = build_data_frame(
            0x40,
            AddressMode::Short,
            CHILD_SHORT,
            &[0; 8],
            &MacAddress::Short(PAN, ROUTER),
            &[0xDA],
            true,
            false,
        )
        .unwrap();
        rx(&mut core, &data);
        rx(
            &mut core,
            &build_association_request(
                0x41,
                &MacAddress::Short(PAN, ROUTER),
                &CHILD,
                &crate::CapabilityInfo {
                    allocate_address: true,
                    ..Default::default()
                },
            ),
        );

        let MacCommandEvent::AssociationRequest(request) =
            block_on(core.command_event(10_000)).unwrap()
        else {
            panic!("expected an association request");
        };
        assert_eq!(request.device_address, CHILD);
        assert_eq!(
            core.phy().sent_acks.as_slice(),
            [(0x40, false), (0x41, false)]
        );
        // The data frame heard while listening is delivered once, unACKed
        // a second time.
        let indication = block_on(core.receive_data(1_000)).unwrap();
        assert_eq!(indication.payload.as_slice(), [0xDA]);
        assert_eq!(core.phy().sent_acks.len(), 2);
    }

    #[test]
    fn parent_beacon_and_orphan_responses_use_shared_builders() {
        let mut core = parent_core();
        let zigbee_payload = [
            0x00, 0x22, 0x84, 1, 2, 3, 4, 5, 6, 7, 8, 0xFF, 0xFF, 0xFF, 0x00,
        ];
        let mut beacon = MlmeBeaconResponse::new(PibPayload::from_slice(&zigbee_payload).unwrap());
        beacon.association_permit = true;
        block_on(core.beacon_response(beacon)).unwrap();
        let sent = core.phy().last_tx.clone();
        let descriptor = parse_beacon(15, &sent, 0).unwrap();
        assert_eq!(descriptor.coord_address, MacAddress::Short(PAN, ROUTER));
        assert!(descriptor.superframe_spec.association_permit);

        // A non-member orphan is ignored without transmitting.
        let attempts = core.phy().tx_attempts;
        block_on(core.orphan_response(MlmeOrphanResponse {
            orphan_address: CHILD,
            short_address: CHILD_SHORT,
            associated_member: false,
        }))
        .unwrap();
        assert_eq!(core.phy().tx_attempts, attempts);

        let sequence = core.pib().dsn();
        rx(&mut core, &[0x02, 0x00, sequence]);
        block_on(core.orphan_response(MlmeOrphanResponse {
            orphan_address: CHILD,
            short_address: CHILD_SHORT,
            associated_member: true,
        }))
        .unwrap();
        let (_, coordinator, orphan) =
            crate::frames::parse_coordinator_realignment_orphan_response(&core.phy().last_tx)
                .unwrap();
        assert_eq!(coordinator, IEEE);
        assert_eq!(orphan, CHILD);
    }

    #[test]
    fn parent_indirect_data_requires_an_indirect_unicast_in_our_pan() {
        let mut core = parent_core();
        let request = |dst, indirect| McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: dst,
            payload: &[0x01],
            msdu_handle: 1,
            tx_options: TxOptions {
                ack_tx: true,
                indirect,
                ..TxOptions::default()
            },
        };
        let child = MacAddress::Short(PAN, CHILD_SHORT);
        assert_eq!(
            block_on(core.indirect_data(request(child, false))).map(|_| ()),
            Err(MacError::InvalidParameter)
        );
        assert_eq!(
            block_on(
                core.indirect_data(request(MacAddress::Short(PAN, ShortAddress(0xFFFF)), true))
            )
            .map(|_| ()),
            Err(MacError::InvalidParameter)
        );

        let sequence = core.pib().dsn();
        rx(&mut core, &[0x02, 0x00, sequence]);
        block_on(core.indirect_data(request(child, true))).unwrap();
        // Sent directly with an ACK request to the polling child.
        assert_eq!(core.phy().last_tx[0] & (1 << 5), 1 << 5);
    }

    fn unicast_data(sequence: u8, source: u16, destination: u16, payload: u8) -> PhyRxFrame {
        let frame = build_data_frame(
            sequence,
            AddressMode::Short,
            ShortAddress(source),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(destination)),
            &[payload],
            true,
            false,
        )
        .unwrap();
        PhyRxFrame::from_slice(&frame, 190).unwrap()
    }

    #[test]
    fn ack_window_drops_frames_addressed_to_other_devices() {
        let mut core = associated_core();
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(unicast_data(0x50, 0x2222, 0x9999, 0x01))))
            .unwrap();
        rx.push_back(Ok(Some(unicast_data(0x51, 0x2222, 0x5678, 0x02))))
            .unwrap();
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x02, 0x00, 0x2A], 255).unwrap(),
        )))
        .unwrap();

        block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        // Only the frame for us is ACKed and retained.
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x51, false)]);
        assert_eq!(core.take_pending_data().unwrap().as_slice(), [0x02]);
        assert!(core.take_pending_data().is_none());
    }

    #[test]
    fn end_device_acknowledges_but_drops_unicast_mac_commands() {
        let mut core = associated_core();
        // Command, ACK request, PAN ID compression, short source/destination.
        let command = [0x63, 0x88, 0x60, 0x34, 0x12, 0x78, 0x56, 0x00, 0x00, 0x08];
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(PhyRxFrame::from_slice(&command, 200).unwrap())))
            .unwrap();
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x02, 0x00, 0x2A], 255).unwrap(),
        )))
        .unwrap();

        block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert_eq!(core.phy().sent_acks.as_slice(), [(0x60, false)]);
        assert!(core.take_pending_rx().is_none());
    }

    #[test]
    fn poll_response_evicts_the_oldest_frame_when_pending_rx_is_full() {
        let mut core = associated_core();
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x12, 0x00, 0x01], 255).unwrap(),
        )))
        .unwrap();
        for index in 0..5 {
            rx.push_back(Ok(Some(unicast_data(0x60 + index, 0x2222, 0x5678, index))))
                .unwrap();
        }
        rx.push_back(Ok(Some(unicast_data(0x70, 0x0000, 0x5678, 0xEE))))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xEE]);
        // Frames 0 and 1 were evicted to make room (queue depth 4).
        for expected in 2..5 {
            let indication = block_on(core.receive_data(1_000)).unwrap();
            assert_eq!(indication.payload.as_slice(), [expected]);
        }
        assert!(core.take_pending_rx().is_none());
    }

    #[test]
    fn poll_within_discards_a_response_completed_at_the_deadline() {
        let mut core = associated_core();
        let rx = &mut core.phy_mut().rx_frames;
        rx.push_back(Ok(Some(
            PhyRxFrame::from_slice(&[0x12, 0x00, 0x01], 255).unwrap(),
        )))
        .unwrap();
        rx.push_back(Ok(Some(unicast_data(0x70, 0x0000, 0x5678, 0xEE))))
            .unwrap();

        assert!(block_on(core.poll_within(0)).unwrap().is_none());
        // The response was still acknowledged and consumed.
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x70, false)]);
        assert!(core.take_pending_rx().is_none());
    }

    #[test]
    fn rx_queue_wraps_and_evicts_the_oldest_frame() {
        let frame = |tag: u8| PhyRxFrame::from_slice(&[tag], 0).unwrap();
        let tags = |queue: &mut RxQueue<3>| {
            let mut out = heapless::Vec::<u8, 3>::new();
            while let Some(frame) = queue.pop_front() {
                out.push(frame.as_slice()[0]).unwrap();
            }
            out
        };

        let mut queue = RxQueue::<3>::new();
        assert!(queue.is_empty() && queue.front().is_none());
        for tag in 1..=4 {
            queue.push_back(frame(tag));
        }
        // Full: pushing 4 evicted 1; the ring has wrapped once.
        assert_eq!(queue.len(), 3);
        assert_eq!(tags(&mut queue).as_slice(), [2, 3, 4]);

        queue.push_back(frame(5));
        queue.push_back(frame(6));
        queue.push_front(frame(7));
        assert_eq!(tags(&mut queue).as_slice(), [7, 5, 6]);

        queue.push_back(frame(8));
        queue.push_back(frame(9));
        queue.push_back(frame(10));
        // Full: the front insertion evicts the oldest frame (8).
        queue.push_front(frame(11));
        assert_eq!(tags(&mut queue).as_slice(), [11, 9, 10]);

        queue.push_back(frame(12));
        queue.drop_front();
        queue.clear();
        assert!(queue.is_empty() && queue.pop_front().is_none());
        queue.drop_front();
        assert_eq!(queue.len(), 0);
    }

    #[test]
    fn rx_queue_receives_in_place_and_removes_in_order() {
        let frame = |tag: u8| PhyRxFrame::from_slice(&[tag], 0).unwrap();
        let tags = |queue: &mut RxQueue<3>| {
            let mut out = heapless::Vec::<u8, 3>::new();
            while let Some(frame) = queue.pop_front() {
                out.push(frame.as_slice()[0]).unwrap();
            }
            out
        };

        let mut queue = RxQueue::<3>::new();
        // A received frame that is not committed leaves the queue unchanged.
        queue.push_back(frame(1));
        *queue.rx_slot() = frame(2);
        assert_eq!(queue.received().as_slice(), [2]);
        assert_eq!(queue.len(), 1);
        *queue.rx_slot() = frame(3);
        queue.commit_back();
        assert_eq!(tags(&mut queue).as_slice(), [1, 3]);

        // Receiving into a full queue evicts the oldest frame, even if the
        // new frame is then not retained.
        for tag in 4..=6 {
            queue.push_back(frame(tag));
        }
        *queue.rx_slot() = frame(7);
        assert_eq!(queue.len(), 2);
        // A front commit lands ahead of the retained frames.
        queue.commit_front();
        assert_eq!(tags(&mut queue).as_slice(), [7, 5, 6]);

        // Removal keeps the other frames in order, across the wrap.
        for tag in 8..=10 {
            queue.push_back(frame(tag));
        }
        queue.remove(1);
        assert_eq!(queue.len(), 2);
        queue.push_back(frame(11));
        queue.remove(2);
        queue.remove(0);
        assert_eq!(tags(&mut queue).as_slice(), [10]);
    }
}
