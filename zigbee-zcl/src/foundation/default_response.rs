//! Default Response command (0x0B).

use crate::ZclStatus;

/// Default Response payload (command 0x0B).
///
/// Sent in reply to any command that does not have a cluster-specific response,
/// unless the sender set "disable default response" in the frame control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DefaultResponse {
    /// The command ID that this is a response to.
    pub command_id: u8,
    /// Status result for the command.
    pub status: ZclStatus,
}

impl DefaultResponse {
    /// Parse from ZCL payload (2 bytes: command_id + status).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 2 {
            return None;
        }
        Some(Self {
            command_id: data[0],
            status: ZclStatus::from_u8(data[1]),
        })
    }

    /// Serialize to ZCL payload bytes. Returns bytes written (always 2).
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 2 {
            return 0;
        }
        buf[0] = self.command_id;
        buf[1] = self.status as u8;
        2
    }
}
