//! Network Formation commissioning (BDB v3.0.1 spec §8.3).
//!
//! Network Formation is only available on coordinator-capable devices.
//! It creates a new Zigbee PAN using `NLME-NETWORK-FORMATION`.
//!
//! ## Procedure
//! 1. Verify device is coordinator-capable
//! 2. Generate the network key
//! 3. Form network on primary channels (`NLME-NETWORK-FORMATION`)
//! 4. If primary fails, retry on secondary channels
//! 5. Durably commit the network identity, key, and counter reservation
//! 6. Set up Trust Center policy and expose the joined state
//! 7. Open permit joining so other devices can join
//!
//! ## Security modes
//! - **Centralized**: coordinator acts as Trust Center, distributes NWK key
//! - **Distributed**: routers form their own trust domain (no TC)

use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::ShortAddress;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{
    BdbLayer, BdbStatus, CounterReservation, NetworkSecurityState, SecurityPersistenceError,
};

/// Durable commit point for a freshly formed network.
///
/// Implementations must persist the complete network state, reserve the
/// returned outgoing NWK frame-counter range, and mark the formed network
/// committed before returning `Ok`. BDB does not expose the network or open
/// permit joining until this operation succeeds.
pub trait FormationPersistence {
    fn commit_formed_network(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityPersistenceError>;
}

impl<F> FormationPersistence for F
where
    F: FnMut(&NetworkSecurityState) -> Result<CounterReservation, SecurityPersistenceError>,
{
    fn commit_formed_network(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityPersistenceError> {
        self(state)
    }
}

impl<M: MacDriver> BdbLayer<M> {
    /// Execute the Network Formation procedure (BDB spec §8.3).
    ///
    /// Formation without durable storage is deliberately rejected. Use
    /// [`Self::network_formation_with_persistence`] so the network key,
    /// identity, and outgoing counter reservation survive reset before any
    /// joining window is exposed.
    pub async fn network_formation(&mut self) -> Result<(), BdbStatus> {
        log::error!("[BDB:Formation] Durable persistence is required");
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::FormationFailure;
        Err(BdbStatus::PersistenceFailure)
    }

    /// Execute Network Formation with a durable formed-network commit.
    ///
    /// A Zigbee coordinator forms a centralized network and becomes its Trust
    /// Center. A coordinator-capable Zigbee router forms a distributed-
    /// security network with no Trust Center.
    pub async fn network_formation_with_persistence<P>(
        &mut self,
        persistence: &mut P,
    ) -> Result<(), BdbStatus>
    where
        P: FormationPersistence + ?Sized,
    {
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::InProgress;

        // Step 1: verify that this logical role may own a PAN.
        let device_type = self.zdo.nwk().device_type();
        if matches!(device_type, DeviceType::EndDevice)
            || !self.zdo.nwk().mac().capabilities().coordinator
        {
            log::warn!("[BDB:Formation] Device cannot form a network");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        }
        if device_type == DeviceType::Router
            && self
                .zdo
                .aps()
                .security()
                .distributed_security_link_key()
                .is_none()
        {
            log::warn!("[BDB:Formation] Distributed-security link key is not provisioned");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        }

        if self.attributes.node_is_on_a_network {
            log::info!("[BDB:Formation] Already on a network");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::OnANetwork;
            return Err(BdbStatus::NotPermitted);
        }

        // Acquire security material before NLME starts a PAN. If entropy is
        // unavailable, fail without leaving an active unsecured network.
        let mut nwk_key = [0u8; 16];
        if self
            .zdo
            .nwk_mut()
            .mac_mut()
            .fill_random(&mut nwk_key)
            .is_err()
        {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::FormationFailure;
            return Err(BdbStatus::FormationFailure);
        }

        log::info!("[BDB:Formation] Forming new network…");

        // Step 2: Attempt formation on primary channels
        let channel_sets = [
            self.attributes.primary_channel_set,
            self.attributes.secondary_channel_set,
        ];

        for (idx, &channel_mask) in channel_sets.iter().enumerate() {
            if channel_mask.0 == 0 {
                continue;
            }

            let set_name = if idx == 0 { "primary" } else { "secondary" };
            log::debug!(
                "[BDB:Formation] Trying {} channels: 0x{:08X}",
                set_name,
                channel_mask.0,
            );

            match self
                .zdo
                .nlme_network_formation(channel_mask, self.attributes.scan_duration)
                .await
            {
                Ok(()) => {
                    log::info!("[BDB:Formation] Network formed on {} channels", set_name);
                    return self.post_formation_setup(nwk_key, persistence).await;
                }
                Err(e) => {
                    log::debug!(
                        "[BDB:Formation] Formation failed on {} channels: {:?}",
                        set_name,
                        e,
                    );
                }
            }
        }

        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::FormationFailure;
        Err(BdbStatus::FormationFailure)
    }

    /// Post-formation setup: key, durable commit, TC policy, permit joining.
    async fn post_formation_setup<P>(
        &mut self,
        nwk_key: [u8; 16],
        persistence: &mut P,
    ) -> Result<(), BdbStatus>
    where
        P: FormationPersistence + ?Sized,
    {
        let ieee = self.zdo.nwk().nib().ieee_address;
        let centralized = self.zdo.nwk().device_type() == DeviceType::Coordinator;
        let (trust_center_address, designated_coordinator, node_join_link_key_type) = if centralized
        {
            (
                ieee,
                true,
                crate::attributes::NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey,
            )
        } else {
            (
                [0xFF; 8],
                false,
                crate::attributes::NodeJoinLinkKeyType::DistributedSecurityGlobalLinkKey,
            )
        };

        // Install the fresh key only long enough to construct and reserve the
        // exact state that will be used on air. A failed durable commit resets
        // the provisional PAN before any BDB on-network flag or permit-join
        // request becomes visible.
        let key_seq: u8 = 0;
        self.zdo
            .nwk_mut()
            .security_mut()
            .set_network_key(nwk_key, key_seq);
        {
            let nib = self.zdo.nwk_mut().nib_mut();
            nib.active_key_seq_number = key_seq;
            nib.security_enabled = true;
        }

        let nib = self.zdo.nwk().nib();
        let state = NetworkSecurityState {
            extended_pan_id: nib.extended_pan_id,
            pan_id: nib.pan_id.0,
            short_address: nib.network_address.0,
            ieee_address: nib.ieee_address,
            channel: nib.logical_channel,
            depth: nib.depth,
            parent_address: nib.parent_address.0,
            update_id: nib.update_id,
            update_id_valid: nib.update_id_valid,
            network_key: nwk_key,
            key_sequence: key_seq,
            outgoing_frame_counter: nib.outgoing_frame_counter,
            trust_center_address,
            node_join_link_key_type,
        };
        let reservation = match persistence.commit_formed_network(&state) {
            Ok(reservation)
                if reservation.is_valid()
                    && reservation.current >= state.outgoing_frame_counter =>
            {
                reservation
            }
            Ok(_) => {
                log::error!("[BDB:Formation] Persistence returned an invalid counter reservation");
                self.rollback_uncommitted_formation();
                return Err(BdbStatus::PersistenceFailure);
            }
            Err(error) => {
                log::error!(
                    "[BDB:Formation] Failed to commit formed network: {:?}",
                    error
                );
                self.rollback_uncommitted_formation();
                return Err(BdbStatus::PersistenceFailure);
            }
        };
        if !self
            .zdo
            .nwk_mut()
            .nib_mut()
            .set_frame_counter_reservation(reservation.current, reservation.limit)
        {
            log::error!("[BDB:Formation] Failed to install persisted counter reservation");
            self.rollback_uncommitted_formation();
            return Err(BdbStatus::PersistenceFailure);
        }

        if centralized {
            log::debug!("[BDB:Formation] Configuring centralized Trust Center");
        } else {
            log::debug!("[BDB:Formation] Configuring distributed security");
        }
        self.zdo.aps_mut().aib_mut().aps_trust_center_address = trust_center_address;
        self.zdo.aps_mut().aib_mut().aps_designated_coordinator = designated_coordinator;
        self.attributes.node_join_link_key_type = node_join_link_key_type;
        self.attributes.node_is_on_a_network = true;
        log::info!(
            "[BDB:Formation] Network state committed, NWK key installed (seq={})",
            key_seq
        );

        // Only a durably committed network may advertise a joining window.
        let duration = core::cmp::min(BDB_MIN_COMMISSIONING_TIME, 254) as u8;
        if let Err(e) = self.zdo.nlme_permit_joining(duration).await {
            log::warn!("[BDB:Formation] Failed to open permit joining: {:?}", e);
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::FormationFailure;
            return Err(BdbStatus::FormationFailure);
        }

        // Broadcast Mgmt_Permit_Joining_req so all routers open
        if let Err(e) = self
            .zdo
            .mgmt_permit_joining_req(
                ShortAddress::BROADCAST_ROUTERS_AND_COORDINATOR,
                duration,
                true,
            )
            .await
        {
            log::warn!(
                "[BDB:Formation] Failed to broadcast permit joining: {:?}",
                e
            );
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::FormationFailure;
            return Err(BdbStatus::FormationFailure);
        }

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;

        log::info!(
            "[BDB:Formation] Network ready, permit joining open for {}s",
            duration,
        );
        Ok(())
    }

    fn rollback_uncommitted_formation(&mut self) {
        let _ = self.zdo.nlme_reset(false);
        self.attributes.node_is_on_a_network = false;
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::FormationFailure;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::NwkLayer;
    use zigbee_nwk::frames::NwkHeader;
    use zigbee_zdo::ZdoLayer;

    const IEEE: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, Waker};

        let mut context = Context::from_waker(Waker::noop());
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    fn coordinator_bdb() -> BdbLayer<MockMac> {
        let mac = MockMac::new(IEEE);
        let nwk = NwkLayer::new(mac, DeviceType::Coordinator);
        let aps = ApsLayer::new(nwk);
        BdbLayer::new(ZdoLayer::new(aps))
    }

    #[test]
    fn formation_without_persistence_fails_before_starting_a_pan() {
        let mut bdb = coordinator_bdb();

        assert_eq!(
            block_on(bdb.network_formation()),
            Err(BdbStatus::PersistenceFailure)
        );
        assert!(!bdb.zdo().nwk().is_joined());
        assert!(!bdb.is_on_network());
        assert!(bdb.zdo().nwk().mac().tx_history().is_empty());
    }

    #[test]
    fn persistence_failure_rolls_back_before_permit_joining() {
        let mut bdb = coordinator_bdb();
        let mut persist_calls = 0;
        let mut persistence = |state: &NetworkSecurityState| {
            persist_calls += 1;
            assert_eq!(state.short_address, ShortAddress::COORDINATOR.0);
            assert_eq!(state.ieee_address, IEEE);
            assert_eq!(state.trust_center_address, IEEE);
            Err(SecurityPersistenceError::Storage)
        };

        assert_eq!(
            block_on(bdb.network_formation_with_persistence(&mut persistence)),
            Err(BdbStatus::PersistenceFailure)
        );
        assert_eq!(persist_calls, 1);
        assert!(!bdb.zdo().nwk().is_joined());
        assert!(!bdb.is_on_network());
        assert!(bdb.zdo().nwk().security().active_key().is_none());
        assert!(!bdb.zdo().nwk().nib().permit_joining);
        assert!(
            bdb.zdo().nwk().mac().tx_history().is_empty(),
            "permit joining must not be transmitted before durable commit"
        );
    }

    #[test]
    fn committed_formation_opens_permit_joining_after_counter_reservation() {
        let mut bdb = coordinator_bdb();
        let mut committed = None;
        let mut persistence = |state: &NetworkSecurityState| {
            committed = Some(*state);
            Ok(CounterReservation {
                current: 0x400,
                limit: 0x800,
            })
        };

        assert_eq!(
            block_on(bdb.network_formation_with_persistence(&mut persistence)),
            Ok(())
        );

        let committed = committed.expect("formed state must be committed");
        assert_eq!(committed.short_address, ShortAddress::COORDINATOR.0);
        assert_eq!(committed.ieee_address, IEEE);
        assert_eq!(committed.trust_center_address, IEEE);
        assert!(bdb.zdo().nwk().is_joined());
        assert!(bdb.is_on_network());
        assert!(bdb.zdo().nwk().nib().permit_joining);
        assert_eq!(bdb.zdo().nwk().nib().outgoing_frame_counter_limit, 0x800);
        assert!(bdb.zdo().nwk().nib().outgoing_frame_counter > 0x400);

        let history = bdb.zdo().nwk().mac().tx_history();
        assert_eq!(history.len(), 1);
        let (header, _) = NwkHeader::parse(history[0].payload.as_slice()).unwrap();
        assert_eq!(
            header.dst_addr,
            ShortAddress::BROADCAST_ROUTERS_AND_COORDINATOR
        );
    }
}
