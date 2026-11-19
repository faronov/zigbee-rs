//! FIRMWARE group (`0x08xx`, endpoint BULK), spec 7.9: NCP firmware update
//! into the inactive A/B slot, trial boot, host confirmation.
//!
//! The NCP only routes the commands; the product's [`NcpPlatform`] owns the
//! slot, the image checks, the signature policy and `otadata`. Rollback is
//! the H2's own (decision N6, amended 2026-11-18): an image activated with
//! `FW_ACTIVATE` boots once as a trial; without `FW_CONFIRM` (window
//! expired, crash, any reset) the bootloader returns to the previous slot.

use zigbee_zrhp::codec::List;
use zigbee_zrhp::msg::firmware::{
    FwActivate, FwBegin, FwBeginRsp, FwFinishRsp, FwGetSlotsRsp, FwSlot, FwWrite, FwWriteRsp,
};
use zigbee_zrhp::msg::{RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::types::{Endpoint, Status};

use crate::ncp::QUEUE_BYTES;
use crate::platform::{self, NcpPlatform, PlatformError};
use crate::port::{CoordinatorPort, NetworkState};

/// Whether `id` belongs to the FIRMWARE group.
pub(crate) const fn is_fw_request(id: u16) -> bool {
    matches!(
        id,
        ids::FW_BEGIN
            | ids::FW_WRITE
            | ids::FW_FINISH
            | ids::FW_ACTIVATE
            | ids::FW_CONFIRM
            | ids::FW_ABORT
            | ids::FW_GET_SLOTS
    )
}

pub(crate) fn status_of(error: PlatformError) -> Status {
    match error {
        PlatformError::Unsupported => Status::UnsupportedCommand,
        PlatformError::InvalidState => Status::InvalidState,
        PlatformError::InvalidParameter => Status::InvalidParameter,
        PlatformError::TooLarge => Status::TooLarge,
        PlatformError::Conflict => Status::Conflict,
        PlatformError::Storage => Status::StorageError,
        PlatformError::AccessDenied => Status::AccessDenied,
        PlatformError::VerifyFailed => Status::SecurityError,
        PlatformError::Hardware => Status::Failure,
    }
}

/// Handle one FIRMWARE request. `FW_ACTIVATE` schedules the reboot into
/// the trial image through `pending_reset` (mode 0 after `delay_ms`).
#[allow(clippy::too_many_arguments)]
// Own frame: its buffers stay out of the root future's poll frame.
#[inline(never)]
pub(crate) fn dispatch_fw<P: NcpPlatform, C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    message: &RawMessage<'_>,
    platform: &mut P,
    port: &C,
    pending_reset: &mut Option<(u8, u64)>,
    now_us: u64,
) {
    let id = message.header.msg_id;
    let tid = message.header.tid;
    let status = |queues: &mut EndpointQueues<QUEUE_BYTES>, status: Status| {
        let _ = queues
            .queue(endpoint)
            .push_with(|w| w.response_status(id, tid, status));
    };
    if !platform.fw_supported() {
        status(queues, Status::UnsupportedCommand);
        return;
    }
    macro_rules! decode {
        ($ty:ty) => {
            match message.decode::<$ty>() {
                Ok(value) => value,
                Err(_) => {
                    status(queues, Status::InvalidParameter);
                    return;
                }
            }
        };
    }
    match id {
        ids::FW_BEGIN => {
            let request = decode!(FwBegin);
            let image = platform::FwImage {
                image_size: request.image_size,
                format: request.format,
                sha256: request.sha256,
                version: request.version,
                flags: request.flags,
            };
            match platform.fw_begin(&image) {
                Ok(begin) => {
                    let response = FwBeginRsp {
                        max_chunk: begin.max_chunk,
                        slot: begin.slot,
                    };
                    let _ = queues.response(tid, Status::Success, Some(&response));
                }
                Err(error) => status(queues, status_of(error)),
            }
        }
        ids::FW_WRITE => {
            let request = decode!(FwWrite);
            match platform.fw_write(request.offset, request.data.0) {
                Ok(next_offset) => {
                    let _ =
                        queues.response(tid, Status::Success, Some(&FwWriteRsp { next_offset }));
                }
                Err(error) => status(queues, status_of(error)),
            }
        }
        ids::FW_FINISH => match platform.fw_finish() {
            Ok(verify_result) => {
                let _ = queues.response(tid, Status::Success, Some(&FwFinishRsp { verify_result }));
            }
            Err(error) => status(queues, status_of(error)),
        },
        ids::FW_ACTIVATE => {
            let request = decode!(FwActivate);
            match platform.fw_activate() {
                Ok(()) => {
                    status(queues, Status::Success);
                    *pending_reset = Some((0, now_us + u64::from(request.delay_ms) * 1000));
                }
                Err(error) => status(queues, status_of(error)),
            }
        }
        ids::FW_CONFIRM => {
            // The trial image proves itself by resuming the network first
            // (spec 7.9); an NCP without a network may confirm right away.
            if port.state() == NetworkState::FormedDown {
                status(queues, Status::InvalidState);
                return;
            }
            match platform.fw_confirm() {
                Ok(()) => status(queues, Status::Success),
                Err(error) => status(queues, status_of(error)),
            }
        }
        ids::FW_ABORT => match platform.fw_abort() {
            Ok(()) => status(queues, Status::Success),
            Err(error) => status(queues, status_of(error)),
        },
        ids::FW_GET_SLOTS => {
            let mut slots = [platform::FwSlot {
                slot: 0,
                state: 0,
                version: 0,
                sha256: [0; 32],
            }; 2];
            match platform.fw_slots(&mut slots) {
                Ok(count) => {
                    let out: [FwSlot; 2] = core::array::from_fn(|index| FwSlot {
                        slot: slots[index].slot,
                        state: slots[index].state,
                        version: slots[index].version,
                        sha256: slots[index].sha256,
                    });
                    let response = FwGetSlotsRsp {
                        slots: List::new(&out[..count.min(2)]),
                    };
                    let _ = queues.response(tid, Status::Success, Some(&response));
                }
                Err(error) => status(queues, status_of(error)),
            }
        }
        _ => status(queues, Status::UnsupportedCommand),
    }
}
