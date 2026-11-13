//! Coprocessor MAC transport over the IPCC mailbox.
//!
//! [`WbTransport`] implements [`WbMacTransport`] with
//! `embassy-stm32-wpan`'s [`ControllerAdapter`]. It owns the memory the
//! coprocessor reads by pointer after a request was accepted (PIB values
//! and MSDUs) and copies every indication out of the shared event buffer
//! before acknowledging it, so no coprocessor buffer outlives one
//! [`WbMacTransport::event`] call.
//!
//! # Fault latch
//!
//! `embassy-stm32-wpan` leaves an event it cannot parse unacknowledged, and
//! the mailbox then stalls. A write that does not complete, or an event that
//! fails to parse, latches [`WbTransport::faulted`]: every later request
//! fails with `FAILURE` and no further events are delivered. The
//! application must reset the chip to recover (CPU2 restarts with it).

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::interrupt::typelevel::{Binding, IPCC_C1_RX, IPCC_C1_TX};
use embassy_stm32::ipcc::{
    Config as IpccConfig, ReceiveInterruptHandler, TransmitInterruptHandler,
};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{IPCC, RNG};
use embassy_stm32::rng::Rng;
use embassy_stm32_wpan::TlMbox;
use embassy_stm32_wpan::net::commands::{
    AssociateRequest, AssociateResponse, DataRequest, DisassociateRequest, OrphanResponse,
    PollRequest, PurgeRequest, ResetRequest, ScanRequest, SetRequest, StartRequest,
};
use embassy_stm32_wpan::net::iface::{
    Controller, ControllerToHostPacket, ControllerToHostPacketBox, mcps, mlme,
};
use embassy_stm32_wpan::net::typedefs::{
    AddressMode, Capabilities, DisassociationReason, KeyIdMode, MacAddress as WpanAddress,
    MacChannel, MacStatus, PanId as WpanPanId, PibId, ScanType, SecurityLevel,
};
use embassy_stm32_wpan::sub::mac::ControllerAdapter;
use embassy_stm32_wpan::sub::mm::MemoryManager;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::Vec;
use zigbee_mac::primitives::MacFrame;
use zigbee_mac::stm32wb::sap::{BeaconNotify, DataIndication, Event, Request, SapAddressMode};
use zigbee_mac::stm32wb::{MAX_INDIRECT_IN_FLIGHT, WbMacTransport, status};
use zigbee_mac::{MacError, PlatformServices};
use zigbee_types::{MacAddress, PanId, ShortAddress};

use crate::hsem;

/// MSDU buffers: every indirect frame the backend may have in flight plus
/// one direct frame.
const MSDU_SLOTS: usize = MAX_INDIRECT_IN_FLIGHT + 1;
/// aMaxPHYPacketSize.
const MSDU_CAPACITY: usize = 127;
/// Largest PIB value the backend sets (the beacon payload is 15 octets).
const PIB_VALUE_CAPACITY: usize = 64;
/// Bound on a mailbox command round trip; the coprocessor answers
/// synchronously within microseconds.
const WRITE_TIMEOUT: Duration = Duration::from_millis(500);
/// Bound on coprocessor start-up (`SHCI_C2_MAC_802_15_4_Init`).
const MAC_INIT_TIMEOUT: Duration = Duration::from_millis(1000);
/// RNG semaphore spins; CPU2 holds it for a few word reads at most.
const RNG_SEMAPHORE_ATTEMPTS: u32 = 1_000_000;

/// ST transport-layer reference table: the fixed CPU1/CPU2 mailbox ABI at
/// the start of SRAM2a (AN5289), placed there by `tl_mbox.x`.
const TL_REF_TABLE: usize = 0x2003_0000;
/// `RefTable::mac_802_15_4_table`.
const REF_TABLE_MAC_OFFSET: usize = 6 * 4;
/// `Mac802_15_4Table::p_notack_buffer`.
const MAC_TABLE_NOTACK_OFFSET: usize = 4;
/// `EvtPacket` = 8-byte list header, `kind`, `evt_code`, `payload_len`,
/// payload.
const EVT_PAYLOAD_LEN_OFFSET: usize = 10;
const EVT_PAYLOAD_OFFSET: usize = 11;

/// Copy the raw MAC notification currently held in the shared
/// notification buffer (2-byte opcode then the C structure) into `out`.
/// Returns the notification's payload length. Diagnostics only: it reads
/// the buffer CPU2 last wrote and never acknowledges it.
pub fn pending_notification(out: &mut [u8]) -> usize {
    // SAFETY: the reference table and the MAC table it points to are
    // statics laid out by embassy-stm32-wpan for CPU2 and initialised by
    // `start()`; only volatile byte/word reads are made.
    unsafe {
        let mac_table =
            core::ptr::read_volatile((TL_REF_TABLE + REF_TABLE_MAC_OFFSET) as *const usize);
        if mac_table == 0 {
            return 0;
        }
        let buffer =
            core::ptr::read_volatile((mac_table + MAC_TABLE_NOTACK_OFFSET) as *const usize);
        if buffer == 0 {
            return 0;
        }
        let len = usize::from(core::ptr::read_volatile(
            (buffer + EVT_PAYLOAD_LEN_OFFSET) as *const u8,
        ));
        for (index, byte) in out.iter_mut().take(len).enumerate() {
            *byte = core::ptr::read_volatile((buffer + EVT_PAYLOAD_OFFSET + index) as *const u8);
        }
        len
    }
}

/// Memory the coprocessor reads by pointer. Lives in a static so moving the
/// transport never invalidates a pointer CPU2 still holds.
struct SharedBuffers {
    pib_value: [u8; PIB_VALUE_CAPACITY],
    msdu: [[u8; MSDU_CAPACITY]; MSDU_SLOTS],
}

struct SharedCell(UnsafeCell<SharedBuffers>);

// SAFETY: handed out at most once, guarded by `BUFFERS_TAKEN`.
unsafe impl Sync for SharedCell {}

static BUFFERS: SharedCell = SharedCell(UnsafeCell::new(SharedBuffers {
    pib_value: [0; PIB_VALUE_CAPACITY],
    msdu: [[0; MSDU_CAPACITY]; MSDU_SLOTS],
}));
static BUFFERS_TAKEN: AtomicBool = AtomicBool::new(false);

/// Why the coprocessor MAC could not be started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartError {
    /// [`start`] was already called.
    AlreadyStarted,
    /// CPU2 did not report a running wireless stack (FUS only, no stack, or
    /// a stack that is not the 802.15.4 MAC).
    WirelessStackNotRunning,
    /// `SHCI_C2_MAC_802_15_4_Init` failed or timed out.
    MacInit,
    /// CPU1 could not take the RNG semaphore.
    RngBusy,
}

/// Boot CPU2 and start its 802.15.4 MAC.
///
/// Returns the transport and the mailbox memory manager. The caller must
/// keep `MemoryManager::run_queue` polled for as long as the transport is
/// used (typically from a dedicated Embassy task).
///
/// The RNG is claimed before CPU2 boots, so its reset cannot disturb a
/// running coprocessor.
pub async fn start(
    ipcc: Peri<'static, IPCC>,
    irqs: impl Binding<IPCC_C1_RX, ReceiveInterruptHandler>
    + Binding<IPCC_C1_TX, TransmitInterruptHandler>,
    rng: Peri<'static, RNG>,
) -> Result<(WbTransport, MemoryManager<'static>), StartError> {
    if BUFFERS_TAKEN.swap(true, Ordering::AcqRel) {
        return Err(StartError::AlreadyStarted);
    }
    hsem::enable();
    let rng = {
        let _owner =
            hsem::Guard::acquire(hsem::RNG, RNG_SEMAPHORE_ATTEMPTS).ok_or(StartError::RngBusy)?;
        Rng::new_blocking(rng)
    };

    let mbox = TlMbox::wait_ready(ipcc, irqs, IpccConfig::default())
        .await
        .map_err(|()| StartError::WirelessStackNotRunning)?;
    let (mac, memory_manager) = with_timeout(MAC_INIT_TIMEOUT, mbox.init_mac())
        .await
        .map_err(|_| StartError::MacInit)?
        .map_err(|()| StartError::MacInit)?;

    // SAFETY: `BUFFERS_TAKEN` guarantees this is the only reference.
    let buffers = unsafe { &mut *BUFFERS.0.get() };
    Ok((
        WbTransport {
            controller: ControllerAdapter::new(mac),
            rng,
            buffers,
            msdu_handles: [None; MSDU_SLOTS],
            pan_id: PanId(0xFFFF),
            faulted: false,
        },
        memory_manager,
    ))
}

/// [`WbMacTransport`] over the STM32WB IPCC mailbox.
pub struct WbTransport {
    controller: ControllerAdapter<'static>,
    rng: Rng<'static, Blocking>,
    buffers: &'static mut SharedBuffers,
    /// MSDU handle owning each slot until its DATA/PURGE confirm.
    msdu_handles: [Option<u8>; MSDU_SLOTS],
    /// `macPANId` as last programmed, for poll indications (which carry no
    /// PAN identifier).
    pan_id: PanId,
    faulted: bool,
}

impl WbTransport {
    /// The mailbox stalled; only a chip reset recovers it.
    pub fn faulted(&self) -> bool {
        self.faulted
    }

    fn claim_msdu_slot(&mut self, handle: u8) -> Option<usize> {
        // A handle that is reused before its confirm arrived belonged to a
        // request the backend already gave up on: recycle its slot.
        if let Some(slot) = self
            .msdu_handles
            .iter()
            .position(|owner| *owner == Some(handle))
        {
            return Some(slot);
        }
        let slot = self.msdu_handles.iter().position(Option::is_none)?;
        self.msdu_handles[slot] = Some(handle);
        Some(slot)
    }

    fn release_msdu_slot(&mut self, handle: u8) {
        for owner in &mut self.msdu_handles {
            if *owner == Some(handle) {
                *owner = None;
            }
        }
    }

    async fn write(
        &mut self,
        packet: &impl embassy_stm32_wpan::net::iface::HostToControllerPacket,
    ) -> Result<(), u8> {
        match select(self.controller.write(packet), Timer::after(WRITE_TIMEOUT)).await {
            Either::First(Ok(())) => Ok(()),
            // The coprocessor rejected the request synchronously; the
            // adapter does not expose its status code.
            Either::First(Err(_)) => Err(status::FAILURE),
            Either::Second(()) => {
                self.faulted = true;
                Err(status::FAILURE)
            }
        }
    }
}

fn wpan_pan(pan: PanId) -> WpanPanId {
    WpanPanId(pan.0.to_le_bytes())
}

fn wpan_address(address: &MacAddress) -> (AddressMode, WpanPanId, WpanAddress) {
    match *address {
        MacAddress::Short(pan, short) => (
            AddressMode::Short,
            wpan_pan(pan),
            WpanAddress {
                short: short.0.to_le_bytes(),
            },
        ),
        MacAddress::Extended(pan, ieee) => (
            AddressMode::Extended,
            wpan_pan(pan),
            WpanAddress { extended: ieee },
        ),
    }
}

fn zigbee_pan(pan: WpanPanId) -> PanId {
    PanId(u16::from_le_bytes(pan.0))
}

fn zigbee_address(mode: AddressMode, pan: WpanPanId, address: WpanAddress) -> Option<MacAddress> {
    let pan = zigbee_pan(pan);
    match mode {
        // SAFETY: the address mode selects the union member; both are
        // plain byte arrays, so either read is defined.
        AddressMode::Short => Some(MacAddress::Short(
            pan,
            ShortAddress(u16::from_le_bytes(unsafe { address.short })),
        )),
        AddressMode::Extended => Some(MacAddress::Extended(pan, unsafe { address.extended })),
        _ => None,
    }
}

const NO_KEY_SOURCE: [u8; 8] = [0; 8];

impl zigbee_mac::ForwardAesProvider for WbTransport {}

impl PlatformServices for WbTransport {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        let _owner =
            hsem::Guard::acquire(hsem::RNG, RNG_SEMAPHORE_ATTEMPTS).ok_or(MacError::RadioError)?;
        self.rng.blocking_fill_bytes(output);
        Ok(())
    }
}

impl WbMacTransport for WbTransport {
    async fn request(&mut self, request: &Request<'_>) -> Result<(), u8> {
        if self.faulted {
            return Err(status::FAILURE);
        }
        match *request {
            Request::Reset { set_default_pib } => {
                // A MAC reset discards every pending transaction.
                self.msdu_handles = [None; MSDU_SLOTS];
                self.write(&ResetRequest {
                    set_default_pib,
                    a_stuffing: [0; 3],
                })
                .await
            }
            Request::Set { attribute, value } => {
                if value.len() > PIB_VALUE_CAPACITY {
                    return Err(status::INVALID_PARAMETER);
                }
                if attribute == zigbee_mac::stm32wb::sap::pib_id::PAN_ID && value.len() == 2 {
                    self.pan_id = PanId(u16::from_le_bytes([value[0], value[1]]));
                }
                self.buffers.pib_value[..value.len()].copy_from_slice(value);
                let pointer = self.buffers.pib_value.as_ptr();
                self.write(&SetRequest {
                    pib_attribute_ptr: pointer,
                    pib_attribute: PibId(attribute),
                })
                .await
            }
            Request::Scan {
                scan_type,
                channels,
                duration,
            } => {
                self.write(&ScanRequest {
                    scan_type: ScanType(scan_type as u8),
                    scan_duration: duration,
                    channel_page: 0,
                    security_level: SecurityLevel::Unsecure,
                    scan_channels: channels.to_le_bytes(),
                    key_source: NO_KEY_SOURCE,
                    key_id_mode: KeyIdMode::Implicite,
                    key_index: 0,
                    a_stuffing: [0; 2],
                })
                .await
            }
            Request::Associate {
                channel,
                ref coordinator,
                capability,
            } => {
                let (coord_addr_mode, coord_pan_id, coord_address) = wpan_address(coordinator);
                self.write(&AssociateRequest {
                    channel_number: MacChannel(channel),
                    channel_page: 0,
                    coord_addr_mode,
                    capability_information: Capabilities::from_bits_retain(capability),
                    coord_pan_id,
                    security_level: SecurityLevel::Unsecure,
                    key_id_mode: KeyIdMode::Implicite,
                    key_source: NO_KEY_SOURCE,
                    coord_address,
                    key_index: 0,
                })
                .await
            }
            Request::AssociateResponse {
                device,
                short_address,
                status: association_status,
            } => {
                self.write(&AssociateResponse {
                    device_address: device,
                    assoc_short_address: short_address.0.to_le_bytes(),
                    status: MacStatus(association_status),
                    security_level: SecurityLevel::Unsecure,
                    key_source: NO_KEY_SOURCE,
                    key_id_mode: KeyIdMode::Implicite,
                    key_index: 0,
                    a_stuffing: [0; 2],
                })
                .await
            }
            Request::OrphanResponse {
                orphan,
                short_address,
                associated_member,
            } => {
                self.write(&OrphanResponse {
                    orphan_address: orphan,
                    short_address: short_address.0.to_le_bytes(),
                    associated_member,
                    security_level: SecurityLevel::Unsecure,
                    key_source: NO_KEY_SOURCE,
                    key_id_mode: KeyIdMode::Implicite,
                    key_index: 0,
                    a_stuffing: [0; 2],
                })
                .await
            }
            Request::Disassociate {
                ref device,
                reason,
                tx_indirect,
            } => {
                let (device_addr_mode, device_pan_id, device_address) = wpan_address(device);
                self.write(&DisassociateRequest {
                    device_addr_mode,
                    device_pan_id,
                    disassociation_reason: DisassociationReason(reason),
                    device_address,
                    tx_indirect,
                    security_level: SecurityLevel::Unsecure,
                    key_id_mode: KeyIdMode::Implicite,
                    key_index: 0,
                    key_source: NO_KEY_SOURCE,
                })
                .await
            }
            Request::Start {
                pan_id,
                channel,
                beacon_order,
                superframe_order,
                pan_coordinator,
                battery_life_ext,
            } => {
                self.pan_id = pan_id;
                self.write(&StartRequest {
                    pan_id: wpan_pan(pan_id),
                    channel_number: MacChannel(channel),
                    beacon_order,
                    superframe_order,
                    pan_coordinator,
                    battery_life_extension: battery_life_ext,
                    ..Default::default()
                })
                .await
            }
            Request::Poll { ref coordinator } => {
                let (coord_addr_mode, coord_pan_id, coord_address) = wpan_address(coordinator);
                self.write(&PollRequest {
                    coord_addr_mode,
                    security_level: SecurityLevel::Unsecure,
                    key_id_mode: KeyIdMode::Implicite,
                    key_index: 0,
                    coord_address,
                    key_source: NO_KEY_SOURCE,
                    coord_pan_id,
                    a_stuffing: [0; 2],
                })
                .await
            }
            Request::Data {
                src_addr_mode,
                ref destination,
                handle,
                ack,
                indirect,
                msdu,
            } => {
                if msdu.len() > MSDU_CAPACITY {
                    return Err(status::FRAME_TOO_LONG);
                }
                let slot = self
                    .claim_msdu_slot(handle)
                    .ok_or(status::TRANSACTION_OVERFLOW)?;
                self.buffers.msdu[slot][..msdu.len()].copy_from_slice(msdu);
                let (dst_addr_mode, dst_pan_id, dst_address) = wpan_address(destination);
                let request = DataRequest {
                    msdu_ptr: self.buffers.msdu[slot].as_ptr(),
                    src_addr_mode: AddressMode(match src_addr_mode {
                        SapAddressMode::None => 0x00,
                        SapAddressMode::Short => 0x02,
                        SapAddressMode::Extended => 0x03,
                    }),
                    dst_addr_mode,
                    dst_pan_id,
                    dst_address,
                    msdu_length: msdu.len() as u8,
                    msdu_handle: handle,
                    ack_tx: u8::from(ack),
                    gts_tx: false,
                    indirect_tx: u8::from(indirect),
                    security_level: SecurityLevel::Unsecure,
                    ..Default::default()
                };
                let result = self.write(&request).await;
                if result.is_err() {
                    self.release_msdu_slot(handle);
                }
                result
            }
            Request::Purge { handle } => {
                self.write(&PurgeRequest {
                    msdu_handle: handle,
                    a_stuffing: [0; 3],
                })
                .await
            }
        }
    }

    async fn event(&mut self, timeout_us: u32) -> Option<Event> {
        if self.faulted {
            Timer::after_micros(u64::from(timeout_us)).await;
            return None;
        }
        let packet = match select(
            self.controller.read(),
            Timer::after_micros(u64::from(timeout_us)),
        )
        .await
        {
            Either::First(Ok(packet)) => packet,
            Either::First(Err(_)) => {
                // The adapter left the event unacknowledged; the mailbox is
                // stalled from here on.
                self.faulted = true;
                return None;
            }
            Either::Second(()) => return None,
        };
        // Copy out, then drop `packet` to acknowledge the shared buffer.
        let event = convert(packet.packet(), self.pan_id);
        drop(packet);
        match &event {
            Event::DataConfirm { handle, .. } => self.release_msdu_slot(*handle),
            Event::PurgeConfirm { handle, status } if *status == status::SUCCESS => {
                self.release_msdu_slot(*handle)
            }
            _ => {}
        }
        Some(event)
    }
}

fn convert(packet: ControllerToHostPacket<'_>, own_pan: PanId) -> Event {
    use mlme::{ConfirmPacket as Mc, IndicationPacket as Mi};
    match packet {
        ControllerToHostPacket::Mlme(mlme::Packet::Confirm(confirm)) => match confirm {
            Mc::Associate(c) => Event::AssociateConfirm {
                status: c.status.0,
                short_address: ShortAddress(u16::from_le_bytes(c.assoc_short_address)),
            },
            Mc::Disassociate(c) => Event::DisassociateConfirm { status: c.status.0 },
            Mc::Reset(c) => Event::ResetConfirm { status: c.status.0 },
            Mc::Set(c) => Event::SetConfirm {
                status: c.status.0,
                attribute: c.pin_attribute.0,
            },
            Mc::Scan(c) => {
                let count = usize::from(c.result_list_size)
                    .min(c.energy_detect_list.len())
                    .min(16);
                let mut energy = Vec::new();
                for value in &c.energy_detect_list[..count] {
                    let _ = energy.push(*value);
                }
                Event::ScanConfirm {
                    status: c.status.0,
                    scan_type: c.scan_type.0,
                    unscanned_channels: u32::from_le_bytes(c.unscanned_channels),
                    energy,
                }
            }
            Mc::Start(c) => Event::StartConfirm { status: c.status.0 },
            Mc::Poll(c) => Event::PollConfirm { status: c.status.0 },
            Mc::Get(_) => Event::Other { id: 2 },
            Mc::Gts(_) => Event::Other { id: 3 },
            Mc::RxEnable(_) => Event::Other { id: 5 },
            Mc::Dps(_) => Event::Other { id: 10 },
            Mc::Sounding(_) => Event::Other { id: 11 },
            Mc::Calibrate(_) => Event::Other { id: 12 },
        },
        ControllerToHostPacket::Mlme(mlme::Packet::Indication(indication)) => match indication {
            Mi::Associate(i) => Event::AssociateIndication {
                device: i.device_address,
                capability: i.capability_information.bits(),
                security_use: i.security_level != SecurityLevel::Unsecure,
            },
            Mi::Disassociate(i) => Event::DisassociateIndication {
                device: i.device_address,
                reason: i.disassociation_reason.0,
            },
            Mi::BeaconNotify(i) => {
                let descriptor = &i.pan_descriptor;
                let coordinator = zigbee_address(
                    descriptor.coord_addr_mode,
                    descriptor.coord_pan_id,
                    descriptor.coord_addr,
                )
                .unwrap_or(MacAddress::Short(
                    zigbee_pan(descriptor.coord_pan_id),
                    ShortAddress(0xFFFF),
                ));
                let length = usize::from(i.sdu_length);
                let mut payload = Vec::new();
                if length > 0 && !i.sdu_ptr.is_null() {
                    // SAFETY: the coprocessor's SDU pointer and length
                    // describe its event buffer, valid until acknowledged.
                    let sdu = unsafe { core::slice::from_raw_parts(i.sdu_ptr, length) };
                    let _ = payload.extend_from_slice(&sdu[..length.min(payload.capacity())]);
                }
                Event::BeaconNotify(BeaconNotify {
                    channel: descriptor.logical_channel.0,
                    coordinator,
                    superframe_spec: u16::from_le_bytes(descriptor.superframe_spec),
                    lqi: descriptor.link_quality,
                    security_use: descriptor.security_level != 0,
                    payload,
                })
            }
            Mi::CommStatus(i) => Event::CommStatus {
                pan_id: zigbee_pan(i.pan_id),
                source: zigbee_address(i.src_addr_mode, i.pan_id, i.src_address),
                destination: zigbee_address(i.dst_addr_mode, i.pan_id, i.dst_address),
                status: i.status.0,
            },
            Mi::Orphan(i) => Event::OrphanIndication {
                orphan: i.orphan_address,
            },
            Mi::SyncLoss(i) => Event::SyncLoss {
                reason: i.loss_reason,
            },
            Mi::Poll(i) => {
                match zigbee_address(i.addr_mode, wpan_pan(own_pan), i.request_address) {
                    Some(requester) => Event::PollIndication { requester },
                    None => Event::Other { id: 24 },
                }
            }
            Mi::Gts(_) => Event::Other { id: 19 },
            Mi::Dps(_) => Event::Other { id: 22 },
        },
        ControllerToHostPacket::Mcps(mcps::Packet::Confirm(confirm)) => match confirm {
            mcps::ConfirmPacket::Data(c) => Event::DataConfirm {
                handle: c.msdu_handle,
                status: c.status.0,
                timestamp: u32::from_le_bytes(c.time_stamp),
            },
            mcps::ConfirmPacket::Purge(c) => Event::PurgeConfirm {
                handle: c.msdu_handle,
                status: c.status.0,
            },
        },
        ControllerToHostPacket::Mcps(mcps::Packet::Indication(mcps::IndicationPacket::Data(i))) => {
            let source = zigbee_address(i.src_addr_mode, i.src_pan_id, i.src_address);
            let destination = zigbee_address(i.dst_addr_mode, i.dst_pan_id, i.dst_address);
            let length = usize::from(i.msdu_length);
            let msdu = if i.msdu_ptr.is_null() {
                None
            } else {
                // SAFETY: as for the beacon SDU above.
                MacFrame::from_slice(unsafe { core::slice::from_raw_parts(i.msdu_ptr, length) })
            };
            match (source, destination, msdu) {
                (Some(source), Some(destination), Some(msdu)) => {
                    Event::DataIndication(DataIndication {
                        source,
                        destination,
                        lqi: i.mpdu_link_quality,
                        rssi: i.rssi as i8,
                        // Zigbee never enables MAC security, and the adapter
                        // keeps the field private.
                        security_use: false,
                        msdu,
                    })
                }
                _ => Event::Other { id: 23 },
            }
        }
    }
}
