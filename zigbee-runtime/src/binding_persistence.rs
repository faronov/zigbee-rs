//! Bounded completion of a ZDO binding mutation owned by an external APS store.

use zigbee_mac::MacDriver;
use zigbee_zdo::ZdoError;
use zigbee_zdo::handler::PreparedBindingResponse;

use crate::ZigbeeDevice;
use crate::role::DeviceRole;
use crate::security_store::{PersistentReplayCounter, SecurityStateStore, SecurityStoreError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BindingPersistenceError {
    SnapshotRequired,
    Security(SecurityStoreError),
    Acknowledgement(zigbee_aps::ApsStatus),
    Response(ZdoError),
}

impl From<SecurityStoreError> for BindingPersistenceError {
    fn from(error: SecurityStoreError) -> Self {
        Self::Security(error)
    }
}

#[derive(Default)]
pub(crate) struct BindingPersistence {
    pub(crate) enabled: bool,
    pub(crate) response: Option<PreparedBindingResponse>,
}

pub(crate) fn is_binding_request(payload: &[u8]) -> bool {
    use zigbee_aps::frames::{ApsFrameType, ApsHeader};

    ApsHeader::parse(payload).is_some_and(|(header, _)| {
        header.frame_control.frame_type == ApsFrameType::Data as u8
            && header.dst_endpoint == Some(0)
            && matches!(
                header.cluster_id,
                Some(zigbee_zdo::BIND_REQ | zigbee_zdo::UNBIND_REQ)
            )
    })
}

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// Select only when the composition owns a durable APS-table store.
    /// Store-backed receive then retains one prepared Bind/Unbind response
    /// until that store and the security journal have committed. Compositions
    /// without this opt-in retain their existing volatile behavior.
    pub fn set_binding_persistence_enabled(&mut self, enabled: bool) {
        self.binding_persistence.enabled = enabled;
    }

    pub fn binding_persistence_pending(&self) -> bool {
        self.binding_persistence.response.is_some()
    }

    /// Complete after successfully saving the APS-table snapshot. No new
    /// receive may overwrite this single transaction before completion.
    ///
    /// Replay floors precede the ACK, which precedes the ZDO response. The
    /// prepared response and any unsent ACK remain owned across errors or
    /// cancellation; retry never reapplies the binding mutation.
    pub async fn complete_binding_persistence<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<(), BindingPersistenceError> {
        let Some(response) = self.binding_persistence.response else {
            return Ok(());
        };
        if self.aps_tables_dirty() {
            return Err(BindingPersistenceError::SnapshotRequired);
        }
        self.refresh_security_state(store)?;
        if let Some((replay, _)) = self.pending_nwk_lifecycle_replay() {
            store.commit_replay_counter(PersistentReplayCounter::Nwk(replay))?;
        }
        if let Some(replay) = self.bdb.zdo().aps().pending_data_replay() {
            store.commit_replay_counter(PersistentReplayCounter::Aps(replay))?;
        }
        self.bdb.zdo_mut().aps_mut().complete_data_persistence();
        self.bdb
            .zdo_mut()
            .aps_mut()
            .complete_network_key_persistence();
        self.bdb
            .zdo_mut()
            .nwk_mut()
            .complete_lifecycle_persistence()
            .await;
        self.bdb
            .zdo_mut()
            .aps_mut()
            .send_pending_aps_ack()
            .await
            .map_err(BindingPersistenceError::Acknowledgement)?;
        self.bdb
            .zdo_mut()
            .send_prepared_binding_response(&response)
            .await
            .map_err(BindingPersistenceError::Response)?;
        self.binding_persistence.response = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn binding_transaction_metadata_stays_bounded() {
        let response = size_of::<PreparedBindingResponse>();
        let runtime = size_of::<BindingPersistence>();
        let aps = size_of::<Option<zigbee_aps::security::ApsReplayCounter>>();
        assert_eq!(response, 6);
        assert!(runtime <= 12);
        assert!(aps <= 32);
        std::println!("binding metadata: response={response}, runtime={runtime}, APS replay={aps}");
    }
}
