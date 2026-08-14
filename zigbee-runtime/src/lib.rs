//! Zigbee Device Runtime — the top-level integration layer.
//!
//! This crate provides:
//! - `ZigbeeDevice` builder API for easy device creation
//! - Event loop that drives MAC→NWK→APS→ZCL processing
//! - NV storage abstraction for persistent state
//! - Power management hooks for sleepy end devices
//! - Pre-built device type templates (sensor, light, switch, etc.)
//!
//! # Example
//! ```rust,no_run,ignore
//! use zigbee_runtime::ZigbeeDevice;
//! use zigbee_mac::mock::MockMac;
//! use zigbee_zcl::{ClusterId, DeviceId};
//!
//! let mac = MockMac::new([1,2,3,4,5,6,7,8]);
//! let mut device = ZigbeeDevice::builder(mac)
//!     .device_type(DeviceType::EndDevice)
//!     .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |ep| {
//!         ep.cluster_server(ClusterId::BASIC)
//!           .cluster_server(ClusterId::TEMPERATURE)
//!     })
//!     .build();
//!
//! device.start().await;
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

#[cfg(test)]
extern crate std;

#[cfg(feature = "trace")]
macro_rules! rt_trace {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}
#[cfg(not(feature = "trace"))]
macro_rules! rt_trace {
    ($($arg:tt)*) => {};
}

pub mod builder;
pub mod child_store;
pub mod event_loop;
pub mod firmware_writer;
pub mod log_nv;
pub mod node;
pub mod nv_storage;
#[cfg(feature = "ota")]
pub mod ota;
#[cfg(feature = "ota")]
pub mod ota_transport;
pub mod power;
pub mod profile;
pub mod remote_reporting;
pub mod role;
pub mod security_journal;
pub mod security_store;
pub mod synthetic_sensor;
pub mod templates;
pub(crate) mod zcl_dispatch;

use zigbee_aps::ApsAddress;
use zigbee_bdb::BdbLayer;
/// Re-exported so composition roots that depend only on `zigbee-runtime` can
/// select the unique-TCLK entry policy exposed via [`BdbLayer`].
use zigbee_mac::pib::PibPayload;
use zigbee_mac::{
    AssociationStatus, MacCommandEvent, MacDriver, MacError, McpsDataIndication,
    MlmeAssociateResponse, MlmeBeaconResponse,
};
// `CapabilityInfo` is only decoded on the parent-side Rejoin Request path,
// which is compiled solely in `router` builds.
#[cfg(feature = "router")]
use zigbee_mac::CapabilityInfo;
use zigbee_types::*;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::foundation::reporting::ReportingEngine;
use zigbee_zcl::{ClusterId, DeviceId};

use crate::nv_storage::{NvItemId, NvStorage};
use crate::power::PowerManager;
use crate::security_store::{
    CommissioningSecurityPersistence, PersistentSecurityState, SecurityStateStore,
    SecurityStoreError,
};

/// Per-device scratch used while buffer-backed indications are alive.
///
/// `process_incoming()` has exclusive `&mut ZigbeeDevice` access, so these
/// cells cannot be accessed concurrently. Keeping them per instance avoids
/// the aliasing and false `Sync` guarantee of the previous global buffers.
struct RuntimeScratch {
    nwk: core::cell::UnsafeCell<[u8; 128]>,
    aps: core::cell::UnsafeCell<zigbee_aps::apsde::ApsFrameBuffer>,
    zcl: core::cell::UnsafeCell<[u8; 253]>,
}

impl RuntimeScratch {
    const fn new() -> Self {
        Self {
            nwk: core::cell::UnsafeCell::new([0; 128]),
            aps: core::cell::UnsafeCell::new(zigbee_aps::apsde::ApsFrameBuffer {
                data: [0; 128],
                len: 0,
            }),
            zcl: core::cell::UnsafeCell::new([0; 253]),
        }
    }
}

#[cfg(test)]
mod runtime_scratch_tests {
    use super::RuntimeScratch;

    #[test]
    fn scratch_storage_is_owned_by_each_runtime_instance() {
        let first = RuntimeScratch::new();
        let second = RuntimeScratch::new();

        unsafe {
            (*first.nwk.get())[0] = 0xA5;
            assert_eq!((*second.nwk.get())[0], 0);
            assert_ne!(first.nwk.get(), second.nwk.get());
        }
    }
}

/// Copy an NWK data indication's payload into the receive scratch and extract
/// its routing/security metadata.
///
/// Hoisted out of `process_incoming` as a plain **synchronous, non-generic**
/// helper — it never touches the `MacDriver`, so it is compiled once instead
/// of being monomorphised into every backend's async receive future, the same
/// codegen boundary the synchronous ZCL dispatcher relies on. The two
/// [`NwkIndication`](zigbee_nwk::nlde::NwkIndication) arms differ only in how
/// the payload is stored (borrowed from the MAC buffer vs owned after
/// decryption); folding them into a single copy+unpack removes the duplicated
/// arm from the receive future. Returns `None` when the frame carried no
/// NLDE-DATA payload (a dropped frame), matching the former inline `None` arm.
#[inline(never)]
fn unpack_nwk_indication(
    scratch_nwk: &mut [u8; 128],
    nwk_indication: Option<zigbee_nwk::nlde::NwkIndication<'_>>,
) -> Option<(ShortAddress, ShortAddress, bool, Option<IeeeAddress>, usize)> {
    let nwk = nwk_indication?;
    let (payload, dst, src, security_use, security_source): (
        &[u8],
        ShortAddress,
        ShortAddress,
        bool,
        Option<IeeeAddress>,
    ) = match &nwk {
        zigbee_nwk::nlde::NwkIndication::Borrowed(data) => (
            data.payload,
            data.dst_addr,
            data.src_addr,
            data.security_use,
            data.security_source,
        ),
        zigbee_nwk::nlde::NwkIndication::Owned(data) => (
            data.payload.as_slice(),
            data.dst_addr,
            data.src_addr,
            data.security_use,
            data.security_source,
        ),
    };
    let len = payload.len().min(scratch_nwk.len());
    scratch_nwk[..len].copy_from_slice(&payload[..len]);
    Some((dst, src, security_use, security_source, len))
}

/// Extract the APS routing metadata (destination endpoint, cluster and source
/// short address) from a parsed indication and emit the RX trace/log line.
///
/// Like [`unpack_nwk_indication`], this is a synchronous, non-generic step of
/// the receive path, so one `#[inline(never)]` copy is shared by every backend
/// instead of being inlined into each per-`MacDriver` receive future.
/// `profile_id` is only observed by the diagnostic lines, so it is consumed
/// here rather than returned.
#[inline(never)]
fn aps_route_metadata(
    aps_indication: &zigbee_aps::apsde::ApsdeDataIndication<'_>,
) -> (u8, u16, u16) {
    let dst_ep = aps_indication.dst_endpoint;
    let cluster_id = aps_indication.cluster_id;
    let profile_id = aps_indication.profile_id;
    let src_addr = match aps_indication.src_address {
        ApsAddress::Short(a) => a.0,
        _ => 0,
    };

    rt_trace!(
        "[RT] aps dst_ep={} prof=0x{:04X} cluster=0x{:04X} src=0x{:04X} payload={}",
        dst_ep,
        profile_id,
        cluster_id,
        src_addr,
        aps_indication.payload.len()
    );
    log::info!(
        "[RX] APS dst_ep={} prof=0x{:04X} cluster=0x{:04X} src=0x{:04X} len={}",
        dst_ep,
        profile_id,
        cluster_id,
        src_addr,
        aps_indication.payload.len()
    );
    (dst_ep, cluster_id, src_addr)
}

#[cfg(test)]
mod receive_stage_helper_tests {
    use super::*;
    use zigbee_nwk::nlde::{NldeDataIndication, NldeDataIndicationOwned, NwkIndication};

    const DST: ShortAddress = ShortAddress(0x1234);
    const SRC: ShortAddress = ShortAddress(0xABCD);
    const SRC_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];

    fn borrowed(payload: &[u8]) -> NwkIndication<'_> {
        NwkIndication::Borrowed(NldeDataIndication {
            dst_addr: DST,
            src_addr: SRC,
            payload,
            lqi: 200,
            security_use: true,
            security_source: Some(SRC_IEEE),
        })
    }

    fn owned(payload: &[u8]) -> NwkIndication<'static> {
        NwkIndication::Owned(NldeDataIndicationOwned {
            dst_addr: DST,
            src_addr: SRC,
            payload: heapless::Vec::from_slice(payload).unwrap(),
            lqi: 200,
            security_use: true,
            security_source: Some(SRC_IEEE),
        })
    }

    /// The `Borrowed` (unsecured, MAC-buffer) and `Owned` (decrypted) arms must
    /// stage identical scratch bytes and identical metadata — the whole point
    /// of folding the two former inline arms into one helper.
    #[test]
    fn borrowed_and_owned_unpack_identically() {
        let data = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02];
        let mut buf_b = [0u8; 128];
        let mut buf_o = [0u8; 128];

        let rb = unpack_nwk_indication(&mut buf_b, Some(borrowed(&data))).unwrap();
        let ro = unpack_nwk_indication(&mut buf_o, Some(owned(&data))).unwrap();

        assert_eq!(rb, ro);
        assert_eq!(rb, (DST, SRC, true, Some(SRC_IEEE), data.len()));
        assert_eq!(&buf_b[..data.len()], &data[..]);
        assert_eq!(buf_b, buf_o);
    }

    /// A payload larger than the 128-byte NWK scratch is clamped to the buffer
    /// capacity, exactly as the former inline `min(len, buf.len())` did.
    #[test]
    fn payload_is_truncated_to_scratch_capacity() {
        let data = [0x5A; 200];
        let mut buf = [0u8; 128];

        let r = unpack_nwk_indication(&mut buf, Some(borrowed(&data))).unwrap();

        assert_eq!(r.4, 128);
        assert_eq!(buf, [0x5A; 128]);
    }

    /// A frame that yielded no NLDE-DATA payload (the former inline `None` arm)
    /// stays a drop and leaves the scratch untouched.
    #[test]
    fn none_indication_yields_none() {
        let mut buf = [0u8; 128];
        assert!(unpack_nwk_indication(&mut buf, None).is_none());
        assert_eq!(buf, [0u8; 128]);
    }
}

#[cfg(test)]
mod builder_cluster_tests {
    use core::mem::MaybeUninit;

    use super::{ClusterRef, ZigbeeDevice};
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::DeviceType;
    use zigbee_types::ShortAddress;
    use zigbee_zcl::clusters::basic::{ATTR_MANUFACTURER_NAME, ATTR_POWER_SOURCE, PowerSource};
    use zigbee_zcl::clusters::identify::{CMD_IDENTIFY, CMD_TRIGGER_EFFECT};
    use zigbee_zcl::data_types::ZclValue;
    use zigbee_zcl::frame::ZclFrame;
    use zigbee_zcl::{ClusterDirection, ClusterId, CommandId, DeviceId, ZclStatus};

    #[test]
    fn builder_owned_clusters_are_dispatched_only_when_declared() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .manufacturer("TestCo")
            .power_source(PowerSource::Battery)
            .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |endpoint| {
                endpoint
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
            })
            .endpoint(2, 0x0104, DeviceId::THERMOSTAT, |endpoint| {
                endpoint.cluster_server(ClusterId::IDENTIFY)
            })
            .build();
        let mut clusters: [ClusterRef<'_>; 0] = [];

        assert_eq!(
            device.with_cluster(1, ClusterId::BASIC, &clusters, |cluster| {
                cluster.attributes().get(ATTR_MANUFACTURER_NAME).cloned()
            }),
            Some(Some(ZclValue::CharString(
                heapless::Vec::from_slice(b"TestCo").unwrap()
            )))
        );
        assert_eq!(
            device.with_cluster(1, ClusterId::BASIC, &clusters, |cluster| {
                cluster.attributes().get(ATTR_POWER_SOURCE).cloned()
            }),
            Some(Some(ZclValue::Enum8(PowerSource::Battery as u8)))
        );
        assert!(
            device
                .with_cluster(2, ClusterId::BASIC, &clusters, |_| ())
                .is_none()
        );

        let result = device.with_cluster_mut(1, ClusterId::IDENTIFY, &mut clusters, |cluster| {
            cluster.handle_command(CMD_IDENTIFY, &[5, 0])
        });
        assert!(matches!(result, Some(Ok(_))));
        assert!(device.is_identifying(1));
        assert!(!device.is_identifying(2));
        device.tick_identify_clusters(2);
        assert!(device.is_identifying(1));
        assert!(!device.is_identifying(2));

        let effect = device.with_cluster_mut(2, ClusterId::IDENTIFY, &mut clusters, |cluster| {
            cluster.handle_command(CMD_TRIGGER_EFFECT, &[0x01, 0x02])
        });
        assert!(matches!(effect, Some(Ok(_))));
        assert_eq!(device.take_identify_effect(1), None);
        assert_eq!(device.take_identify_effect(2), Some((0x01, 0x02)));

        device.reset_identify_clusters();
        assert!(!device.is_identifying(1));
        assert!(!device.is_identifying(2));

        let unsupported =
            device.with_cluster_mut(1, ClusterId::IDENTIFY, &mut clusters, |cluster| {
                cluster.handle_command(CommandId(0xFE), &[])
            });
        assert!(matches!(
            unsupported,
            Some(Err(zigbee_zcl::ZclStatus::UnsupClusterCommand))
        ));
    }

    #[test]
    fn build_into_keeps_identify_state_per_endpoint() {
        let mut storage = MaybeUninit::uninit();
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(1, 0x0104, DeviceId::TEMPERATURE_SENSOR, |endpoint| {
                endpoint.cluster_server(ClusterId::IDENTIFY)
            })
            .endpoint(2, 0x0104, DeviceId::THERMOSTAT, |endpoint| {
                endpoint.cluster_server(ClusterId::IDENTIFY)
            })
            .build_into(&mut storage);
        let mut clusters: [ClusterRef<'_>; 0] = [];

        let result = device.with_cluster_mut(2, ClusterId::IDENTIFY, &mut clusters, |cluster| {
            cluster.handle_command(CMD_IDENTIFY, &[5, 0])
        });
        assert!(matches!(result, Some(Ok(_))));
        assert!(!device.is_identifying(1));
        assert!(device.is_identifying(2));
    }

    #[test]
    fn router_node_descriptor_reports_ffd_mains_and_receiver_on() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::Router)
            .power_source(PowerSource::MainsSinglePhase)
            .build_router();

        assert_eq!(device.bdb().zdo().node_descriptor().mac_capabilities, 0x8E);
    }

    #[test]
    fn sleepy_battery_end_device_does_not_claim_ffd_mains_or_idle_rx() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::EndDevice)
            .power_mode(crate::power::PowerMode::Sleepy {
                poll_interval_ms: 1_000,
                wake_duration_ms: 100,
            })
            .power_source(PowerSource::Battery)
            .build();

        assert_eq!(device.bdb().zdo().node_descriptor().mac_capabilities, 0x80);
    }

    #[test]
    fn end_device_node_descriptor_advertises_r22_without_service_bits() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::EndDevice)
            .build();

        let node_descriptor = *device.bdb().zdo().node_descriptor();
        assert_eq!(node_descriptor.stack_revision(), 22);
        assert_eq!(node_descriptor.server_mask, 22 << 9);
    }

    #[test]
    fn router_node_descriptor_advertises_r22_without_service_bits() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::Router)
            .build_router();

        let node_descriptor = *device.bdb().zdo().node_descriptor();
        assert_eq!(node_descriptor.stack_revision(), 22);
        assert_eq!(node_descriptor.server_mask, 22 << 9);
    }

    #[test]
    fn coordinator_node_descriptor_advertises_r22_and_primary_trust_center() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::Coordinator)
            .build_router();

        let node_descriptor = *device.bdb().zdo().node_descriptor();
        assert_eq!(node_descriptor.stack_revision(), 22);
        assert_eq!(node_descriptor.server_mask, (22 << 9) | 0x0001);

        // Bits 7-8 are reserved and the Network Manager / cache services are
        // not implemented, so they must stay clear.
        assert_eq!(node_descriptor.server_mask & 0x0180, 0);
        assert_eq!(node_descriptor.server_mask & 0x007E, 0);
    }

    #[test]
    fn both_builder_paths_agree_on_the_node_descriptor() {
        // The role and its device type are now chosen together by the terminal
        // build method, so exercise each role's `build`/`build_into` pair.
        fn assert_agrees<R>(
            build: impl FnOnce(crate::builder::DeviceBuilder<MockMac>) -> ZigbeeDevice<MockMac, R>,
            build_into: impl FnOnce(
                crate::builder::DeviceBuilder<MockMac>,
                &mut MaybeUninit<ZigbeeDevice<MockMac, R>>,
            ) -> &mut ZigbeeDevice<MockMac, R>,
        ) where
            R: crate::role::DeviceRole,
        {
            let built = build(ZigbeeDevice::builder(MockMac::new([
                1, 2, 3, 4, 5, 6, 7, 8,
            ])));
            let mut storage = MaybeUninit::uninit();
            let built_into = build_into(
                ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8])),
                &mut storage,
            );
            assert_eq!(
                built.bdb().zdo().node_descriptor(),
                built_into.bdb().zdo().node_descriptor(),
                "descriptors must match across build paths"
            );
            assert_eq!(
                built.remote_reporting(),
                built_into.remote_reporting(),
                "remote reporting state must be initialized across build paths"
            );
        }

        assert_agrees(|b| b.build(), |b, d| b.build_into(d));
        assert_agrees(|b| b.build_relay(), |b, d| b.build_relay_into(d));
        assert_agrees(|b| b.build_router(), |b, d| b.build_router_into(d));
        assert_agrees(
            |b| b.build_coordinator(),
            |b, d| b.build_coordinator_into(d),
        );
    }

    #[test]
    fn node_descriptor_serializes_the_stack_revision_bytes() {
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::Coordinator)
            .build_router();

        let mut buf = [0u8; 13];
        let len = device
            .bdb()
            .zdo()
            .node_descriptor()
            .serialize(&mut buf)
            .unwrap();

        assert_eq!(len, 13);
        // Bytes 8-9: server mask, little endian — 0x2C01 = R22 + primary TC.
        assert_eq!([buf[8], buf[9]], [0x01, 0x2C]);
    }

    #[test]
    fn default_response_reverses_command_direction() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8])).build();

        crate::zcl_dispatch::queue_default_response(
            &mut device.pending_responses,
            ShortAddress(0x1234),
            1,
            1,
            ClusterId::BASIC.0,
            7,
            0x55,
            ZclStatus::UnsupGeneralCommand,
            ClusterDirection::ServerToClient,
        );

        let response = ZclFrame::parse(device.pending_responses[0].zcl_data.as_slice()).unwrap();
        assert_eq!(
            response.header.direction(),
            ClusterDirection::ClientToServer
        );
    }
}

#[cfg(test)]
mod resume_tests {
    use core::future::Future;
    use core::task::{Context, Poll, Waker};

    use super::ZigbeeDevice;
    use crate::child_store::ChildTableStore;
    use crate::role::Router;
    use crate::security_store::{
        PersistentSecurityState, RamSecurityStateStore, SecurityStateStore,
    };
    use zigbee_mac::mock::MockMac;
    use zigbee_mac::{MacDriver, PibAttribute, PibValue};
    use zigbee_nwk::DeviceType;
    use zigbee_types::ShortAddress;

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(Waker::noop());
        let mut future = std::pin::pin!(future);

        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    #[test]
    fn a_sensor_role_never_acts_as_a_parent() {
        // The parent operational surface (announce_parent, save/restore child
        // table, permit_joining, service_parent_commands) is now bounded on the
        // typed `ParentRole`, so a leaf `EndDevice` cannot even *name* it — the
        // calls that previously had to be proven inert at runtime no longer
        // compile on a sensor. The `compile_fail` doctest on `ZigbeeDevice`
        // covers that; here we confirm the sensor still builds and its joined
        // tick emits no parent traffic.
        let mut sensor = ZigbeeDevice::builder(MockMac::new([9; 8]))
            .device_type(DeviceType::EndDevice)
            .build();

        // A populated store is irrelevant to a sensor: it owns no child table
        // API to load it through, and its tick performs no parent maintenance.
        let mut store = crate::child_store::RamChildTableStore::new();
        let mut table = crate::child_store::PersistentChildTable::new();
        table
            .push(crate::child_store::PersistentChild {
                ieee_address: [1; 8],
                short_address: 0x1234,
                rx_on_when_idle: false,
                security_capable: true,
                is_router: false,
                end_device_timeout: 8,
            })
            .unwrap();
        store.store(&table).unwrap();
        assert!(store.load().unwrap().is_some());

        // The sensor owns no Parent Announce state at all — its role state is
        // the zero-sized `NonParentState`, so there is no `parent_annce_due`
        // field to inspect — and its tick transmits nothing.
        block_on(sensor.tick(1, &mut []));
        assert!(sensor.mac().tx_history().is_empty());
    }

    #[test]
    fn resume_restores_parent_address_into_mac_pib() {
        const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const PARENT_ADDRESS: ShortAddress = ShortAddress(0x3344);
        let mac = MockMac::new(IEEE_ADDRESS);
        let mut device = ZigbeeDevice::builder(mac)
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.depth = 1;
        state.parent_address = PARENT_ADDRESS.0;
        state.network_key = [2; 16];
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = [3; 8];
        state.trust_center_link_key = [4; 16];
        state.tclk_counter_limit = 0x400;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();

        assert_eq!(
            block_on(device.start_or_resume_with_security_store(&mut store)).unwrap(),
            state.short_address
        );
        assert_eq!(
            block_on(
                device
                    .mac_mut()
                    .mlme_get(PibAttribute::MacCoordShortAddress)
            )
            .unwrap(),
            PibValue::ShortAddress(PARENT_ADDRESS)
        );
        assert_eq!(
            block_on(
                device
                    .mac_mut()
                    .mlme_get(PibAttribute::MacAssociatedPanCoord)
            )
            .unwrap(),
            PibValue::Bool(true)
        );
        assert!(device.is_joined());
        assert!(device.bdb.is_on_network());
        device.bdb.zdo_mut().nwk_mut().set_joined(false);
        assert!(
            !device.is_joined(),
            "runtime join state must reflect operational NWK connectivity"
        );
        assert!(
            device.bdb.is_on_network(),
            "losing the parent must not erase commissioned credentials"
        );

        let mut pending_state = state;
        pending_state.rejoin_pending = true;
        let mut pending_store = RamSecurityStateStore::new();
        pending_store.store(&pending_state).unwrap();
        let mut pending_device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert!(
            pending_device
                .restore_security_state(&mut pending_store)
                .unwrap()
        );
        assert!(pending_device.secure_rejoin_pending());
        assert!(!pending_device.is_joined());
        block_on(pending_device.leave()).unwrap();
        assert!(!pending_device.secure_rejoin_pending());
        assert!(!pending_device.bdb.is_on_network());

        let mut reboot_store = RamSecurityStateStore::new();
        reboot_store.store(&pending_state).unwrap();
        let mut rebooted = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert!(matches!(
            block_on(rebooted.start_or_resume_with_security_store(&mut reboot_store)),
            Err(crate::event_loop::StartError::CommissioningFailed(_))
        ));
        assert!(
            !rebooted.is_joined(),
            "persisted rejoin-pending state must not silently resume"
        );
        assert!(reboot_store.load().unwrap().unwrap().rejoin_pending);
    }

    /// An ESP32-C6/H2 upgraded from the legacy `LogStructuredNv` persistence
    /// keeps its network: the migrated record is commissioned but has no
    /// persisted unique Trust Center link key. The recoverable post-reboot
    /// path therefore uses the default global key, drawn from the NWK counter
    /// space, without fabricating a Trust Center identity.
    #[test]
    fn legacy_default_tclk_state_resumes_on_the_previous_network() {
        const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const FLOOR: u32 = 0x2400;
        const RESERVATION: u32 = zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE;

        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.legacy_default_tclk = true;
        state.extended_pan_id = [0xAA; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.depth = 2;
        state.parent_address = 0x0001;
        state.update_id = 7;
        state.update_id_valid = true;
        state.network_key = [0xCC; 16];
        state.key_sequence = 3;
        state.global_counter_limit = FLOOR;
        state.tclk_counter_limit = FLOOR;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();

        assert_eq!(
            block_on(device.start_or_resume_with_security_store(&mut store)).unwrap(),
            state.short_address,
            "a migrated installation must resume, not re-commission"
        );
        assert!(device.is_joined());
        assert_eq!(device.pan_id(), 0x1234);
        assert_eq!(device.channel(), 15);
        assert_eq!(
            device
                .bdb
                .zdo()
                .nwk()
                .security()
                .active_key()
                .map(|key| (key.key, key.seq_number)),
            Some(([0xCC; 16], 3)),
            "the previous network key must survive the format switch"
        );

        // Fresh ranges are reserved from both persisted floors, and committed,
        // before any secured frame can be sent.
        let reserved = store.load().unwrap().unwrap();
        assert!(reserved.commissioned);
        assert!(reserved.legacy_default_tclk);
        assert!(!reserved.tclk_present);
        assert_eq!(reserved.global_counter_limit, FLOOR + RESERVATION);
        assert_eq!(reserved.tclk_counter_limit, FLOOR + RESERVATION);
        let nib = device.bdb.zdo().nwk().nib();
        assert_eq!(
            nib.nwk_update_id(),
            Some(7),
            "a record that marks its update state valid restores it as \
             authoritative"
        );
        assert!(
            (FLOOR..nib.outgoing_frame_counter_limit).contains(&nib.outgoing_frame_counter),
            "the live counter resumes inside the fresh reservation (the resume \
             Device_annce consumes from it) and never below the persisted floor"
        );
        assert_eq!(nib.outgoing_frame_counter_limit, FLOOR + RESERVATION);

        // No Trust Center identity or key is fabricated: APS link-key traffic
        // falls back to the default global key over the NWK counter space.
        assert_eq!(device.bdb.zdo().aps().security().key_count(), 0);
        assert_eq!(
            device.bdb.zdo().aps().aib().aps_trust_center_address,
            [0; 8]
        );

        // A missing unique TCLK is a supported state, not corruption, and the
        // NWK reservation still extends at the low-water mark.
        assert!(!device.refresh_security_state(&mut store).unwrap());
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .outgoing_frame_counter = FLOOR + RESERVATION - 1;
        assert!(device.refresh_security_state(&mut store).unwrap());
        let extended = store.load().unwrap().unwrap();
        assert_eq!(extended.global_counter_limit, FLOOR + 2 * RESERVATION);
        assert_eq!(
            extended.tclk_counter_limit,
            FLOOR + RESERVATION,
            "there is no unique-TCLK counter in flight to extend"
        );
        assert_eq!(
            device.bdb.zdo().nwk().nib().outgoing_frame_counter_limit,
            FLOOR + 2 * RESERVATION
        );

        // Rebooting on the committed record stays on the same network.
        let mut rebooted = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert_eq!(
            block_on(rebooted.start_or_resume_with_security_store(&mut store)).unwrap(),
            state.short_address
        );
        assert_eq!(rebooted.pan_id(), 0x1234);
    }

    /// A record migrated from a persistence format that never stored
    /// `NwkUpdateId` restores as *unknown*, and stays unknown across further
    /// persistence — the reboot must not be able to promote the placeholder
    /// into an authoritative `0`, which would make every beacon advertising
    /// `0x81..=0xFF` look stale and strand the device off its own network.
    #[test]
    fn an_unknown_persisted_update_id_restores_and_persists_as_unknown() {
        const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const FLOOR: u32 = 0x2400;
        const RESERVATION: u32 = zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE;

        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.legacy_default_tclk = true;
        state.extended_pan_id = [0xAA; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.depth = 2;
        state.parent_address = 0x0001;
        // The migrated record carries no update state at all.
        state.update_id = 0;
        state.update_id_valid = false;
        state.network_key = [0xCC; 16];
        state.key_sequence = 3;
        state.global_counter_limit = FLOOR;
        state.tclk_counter_limit = FLOOR;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();

        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert!(device.restore_security_state(&mut store).unwrap());

        let nib = device.bdb.zdo().nwk().nib();
        assert_eq!(
            nib.nwk_update_id(),
            None,
            "an unknown persisted update state must never restore as a known 0"
        );
        assert!(!nib.update_id_valid);
        assert_eq!(
            nib.pan_id.0, 0x1234,
            "the rest of the record still restored"
        );

        // Rejoin parent selection therefore rejects nothing as stale.
        assert_eq!(
            device
                .bdb
                .zdo()
                .nwk()
                .rejoin_parent_criteria()
                .nwk_update_id,
            None
        );

        // Further persistence keeps it unknown rather than writing a known 0.
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .outgoing_frame_counter = FLOOR + RESERVATION - 1;
        assert!(device.refresh_security_state(&mut store).unwrap());
        let persisted = store.load().unwrap().unwrap();
        assert_eq!(persisted.update_id, 0);
        assert!(!persisted.update_id_valid);

        // A reboot on that record is still unknown, not a known 0.
        let mut rebooted = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert!(rebooted.restore_security_state(&mut store).unwrap());
        assert_eq!(rebooted.bdb.zdo().nwk().nib().nwk_update_id(), None);

        // Once the network's update state is genuinely learned — a rejoin or
        // an accepted `Mgmt_NWK_Update` — it is adopted durably.
        rebooted
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .set_nwk_update_id(0x90);
        assert!(rebooted.refresh_security_state(&mut store).unwrap());
        let learned = store.load().unwrap().unwrap();
        assert_eq!(learned.update_id, 0x90);
        assert!(learned.update_id_valid);
    }

    #[test]
    fn staged_network_key_rotation_survives_reboot_and_switch() {
        const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const FLOOR: u32 = 0x2400;
        const NEXT_KEY: [u8; 16] = [0xDD; 16];

        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.legacy_default_tclk = true;
        state.extended_pan_id = [0xAA; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.network_key = [0xCC; 16];
        state.key_sequence = 3;
        state.global_counter_limit = FLOOR;
        state.tclk_counter_limit = FLOOR;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        assert!(device.restore_security_state(&mut store).unwrap());

        {
            let nwk = device.bdb.zdo_mut().nwk_mut();
            assert!(nwk.security_mut().stage_network_key(NEXT_KEY, 4));
        }
        assert!(device.refresh_security_state(&mut store).unwrap());
        let staged = store.load().unwrap().unwrap();
        assert_eq!(staged.network_key, [0xCC; 16]);
        assert_eq!(staged.key_sequence, 3);
        assert!(staged.staged_network_key_present);
        assert_eq!(staged.staged_network_key, NEXT_KEY);
        assert_eq!(staged.staged_key_sequence, 4);

        let mut rebooted = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        assert!(rebooted.restore_security_state(&mut store).unwrap());
        assert_eq!(
            rebooted
                .bdb
                .zdo()
                .nwk()
                .security()
                .staged_key()
                .unwrap()
                .key,
            NEXT_KEY
        );
        {
            let nwk = rebooted.bdb.zdo_mut().nwk_mut();
            assert!(nwk.security_mut().activate_network_key(4));
            nwk.nib_mut().active_key_seq_number = 4;
        }
        assert!(rebooted.refresh_security_state(&mut store).unwrap());
        let switched = store.load().unwrap().unwrap();
        assert_eq!(switched.network_key, NEXT_KEY);
        assert_eq!(switched.key_sequence, 4);
        assert!(!switched.staged_network_key_present);
    }

    /// If the Trust Center later transports a unique link key to a migrated
    /// node, the APS layer installs it with an outgoing counter of zero and no
    /// reservation. That must become durable before it is used, and the
    /// transitional marker must go away with it.
    #[test]
    fn a_runtime_trust_center_key_is_adopted_into_the_durable_store() {
        const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const TC_ADDRESS: [u8; 8] = [0x11; 8];
        const FLOOR: u32 = 0x2400;
        const RESERVATION: u32 = zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE;

        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.legacy_default_tclk = true;
        state.extended_pan_id = [0xAA; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.network_key = [0xCC; 16];
        state.global_counter_limit = FLOOR;
        state.tclk_counter_limit = FLOOR;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        assert!(device.restore_security_state(&mut store).unwrap());

        // What `Apsde::handle_transport_key` installs for a TC link key.
        device
            .bdb
            .zdo_mut()
            .aps_mut()
            .security_mut()
            .add_key(zigbee_aps::security::ApsLinkKeyEntry {
                partner_address: TC_ADDRESS,
                key: [0xDD; 16],
                key_type: zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: u32::MAX,
                incoming_frame_counter: 7,
                incoming_frame_counter_valid: true,
            })
            .unwrap();

        assert!(device.refresh_security_state(&mut store).unwrap());
        let adopted = store.load().unwrap().unwrap();
        assert!(adopted.commissioned);
        assert!(adopted.tclk_present);
        assert!(
            !adopted.legacy_default_tclk,
            "the transitional marker retires once a real key exists"
        );
        assert_eq!(adopted.trust_center_address, TC_ADDRESS);
        assert_eq!(adopted.trust_center_link_key, [0xDD; 16]);
        assert_eq!(adopted.tclk_incoming_counter, 7);
        assert!(adopted.tclk_incoming_counter_valid);
        // Continues above the migrated floor, never from the live zero.
        assert_eq!(
            adopted.tclk_counter_limit,
            FLOOR + RESERVATION + RESERVATION
        );

        let entry = device
            .bdb
            .zdo()
            .aps()
            .security()
            .find_key(
                &TC_ADDRESS,
                zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
            )
            .expect("adopted key stays installed");
        assert_eq!(entry.outgoing_frame_counter, FLOOR + RESERVATION);
        assert_eq!(
            entry.outgoing_frame_counter_limit,
            adopted.tclk_counter_limit
        );

        // Steady state afterwards: nothing left to persist, and the record now
        // validates as an ordinary commissioned network.
        assert!(!device.refresh_security_state(&mut store).unwrap());
        assert_eq!(adopted.validate(), Ok(()));

        device
            .bdb
            .zdo_mut()
            .aps_mut()
            .security_mut()
            .add_key(zigbee_aps::security::ApsLinkKeyEntry {
                partner_address: TC_ADDRESS,
                key: [0xEE; 16],
                key_type: zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: u32::MAX,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();
        assert!(device.refresh_security_state(&mut store).unwrap());
        let replaced = store.load().unwrap().unwrap();
        assert_eq!(replaced.trust_center_link_key, [0xEE; 16]);
        assert_eq!(
            replaced.tclk_counter_limit,
            adopted.tclk_counter_limit + RESERVATION
        );
        let live = device
            .bdb
            .zdo()
            .aps()
            .security()
            .find_key(
                &TC_ADDRESS,
                zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
            )
            .unwrap();
        assert_eq!(live.outgoing_frame_counter, adopted.tclk_counter_limit);
        assert_eq!(
            live.outgoing_frame_counter_limit,
            replaced.tclk_counter_limit
        );
    }

    #[test]
    fn factory_reset_clears_credentials_and_preserves_counter_bounds() {
        let mut device = ZigbeeDevice::builder(MockMac::new([0x99; 8]))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [0x11; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = [0x99; 8];
        state.network_key = [0xA5; 16];
        state.staged_network_key_present = true;
        state.staged_network_key = [0xC3; 16];
        state.tclk_present = true;
        state.trust_center_address = [0x22; 8];
        state.trust_center_link_key = [0x5A; 16];
        state.rejoin_pending = true;
        state.global_counter_limit = 0x2A400;
        state.tclk_counter_limit = 0x4804;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        device.factory_reset_security_state(&mut store).unwrap();

        let mut expected = PersistentSecurityState::empty();
        expected.global_counter_limit = state.global_counter_limit;
        expected.tclk_counter_limit = state.tclk_counter_limit;
        assert_eq!(store.load().unwrap(), Some(expected));
    }

    #[test]
    fn identity_change_clears_credentials_and_preserves_counter_bounds() {
        const CURRENT_IEEE: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x99];
        const OLD_IEEE: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];

        let mut device = ZigbeeDevice::builder(MockMac::new(CURRENT_IEEE))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.ieee_address = OLD_IEEE;
        state.network_key = [0xA5; 16];
        state.trust_center_link_key = [0x5A; 16];
        state.global_counter_limit = 0x2400;
        state.tclk_counter_limit = 0x1800;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();

        assert!(
            device
                .reset_security_state_if_identity_changed(&mut store, CURRENT_IEEE)
                .unwrap()
        );
        let reset = store.load().unwrap().unwrap();
        assert!(!reset.commissioned);
        assert_eq!(reset.ieee_address, [0; 8]);
        assert_eq!(reset.network_key, [0; 16]);
        assert_eq!(reset.trust_center_link_key, [0; 16]);
        assert_eq!(reset.global_counter_limit, 0x2400);
        assert_eq!(reset.tclk_counter_limit, 0x1800);
        assert!(
            !device
                .reset_security_state_if_identity_changed(&mut store, CURRENT_IEEE)
                .unwrap()
        );
    }

    #[test]
    fn matching_configured_identity_preserves_commissioned_state_before_start() {
        const CURRENT_IEEE: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x99];

        let mut device = ZigbeeDevice::builder(MockMac::new(CURRENT_IEEE))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.ieee_address = CURRENT_IEEE;
        state.network_key = [0xA5; 16];
        state.global_counter_limit = 0x2400;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();

        assert!(
            !device
                .reset_security_state_if_identity_changed(&mut store, CURRENT_IEEE)
                .unwrap()
        );
        assert_eq!(store.load().unwrap(), Some(state));
    }

    // ── Shared NWK receive path ──────────────────────────────

    const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
    const COORDINATOR_IEEE: [u8; 8] = [0x11; 8];
    const NETWORK_KEY: [u8; 16] = [2; 16];
    const KEY_SEQUENCE: u8 = 0;
    const OUR_SHORT: u16 = 0x5678;
    const COORDINATOR: ShortAddress = ShortAddress(0x0000);
    // Only the router relay tests, which need `zigbee-nwk/router`, name a
    // neighbour that is neither us nor our parent.
    #[cfg(feature = "router")]
    const NEIGHBOUR: ShortAddress = ShortAddress(0x2222);

    fn commissioned_state() -> PersistentSecurityState {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = OUR_SHORT;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.depth = 1;
        state.parent_address = COORDINATOR.0;
        state.network_key = NETWORK_KEY;
        state.key_sequence = KEY_SEQUENCE;
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = COORDINATOR_IEEE;
        state.trust_center_link_key = [4; 16];
        state.tclk_counter_limit = 0x400;
        state
    }

    fn resumed_device(device_type: DeviceType) -> ZigbeeDevice<MockMac> {
        assert_eq!(device_type, DeviceType::EndDevice);
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS)).build();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 1);
        device
    }

    fn resumed_router() -> ZigbeeDevice<MockMac, Router> {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();
        device
    }

    #[test]
    fn durable_router_resume_does_not_reannounce_or_rejoin() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();

        assert_eq!(
            block_on(device.start_or_resume_with_security_store(&mut store)).unwrap(),
            OUR_SHORT
        );
        assert!(device.mac().tx_history().is_empty());
        assert!(device.is_joined());
    }

    #[test]
    fn factory_reset_clears_live_security_and_preserves_counter_bounds() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        let preserved = store.load().unwrap().unwrap();

        assert!(device.bdb().zdo().nwk().security().active_key().is_some());
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 1);
        let _ = device.remote_reporting.record(1, 0x0402);
        assert_eq!(device.remote_reporting_cluster_count(1), 1);

        block_on(device.factory_reset_with_security_store(&mut store)).unwrap();

        assert!(!device.is_joined());
        assert!(!device.bdb().is_on_network());
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
        assert!(device.bdb().zdo().nwk().security().active_key().is_none());
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 0);
        let reset = store.load().unwrap().unwrap();
        assert!(!reset.commissioned);
        assert_eq!(reset.global_counter_limit, preserved.global_counter_limit);
        assert_eq!(reset.tclk_counter_limit, preserved.tclk_counter_limit);
    }

    /// Build the NWK frame a coordinator would put on air, secured with the
    /// restored network key exactly like `nlde_data_request` does.
    fn nwk_frame(
        frame_type: zigbee_nwk::frames::NwkFrameType,
        dst: ShortAddress,
        payload: &[u8],
        frame_counter: u32,
        secured: bool,
    ) -> zigbee_mac::MacFrame {
        use zigbee_nwk::frames::{NwkFrameControl, NwkHeader};
        use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: frame_type as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: secured,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: dst,
            src_addr: COORDINATOR,
            radius: 5,
            seq_number: frame_counter as u8,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };

        let mut buf = [0u8; 128];
        let hdr_len = header.serialize(&mut buf);
        let total = if secured {
            let sec_hdr = NwkSecurityHeader {
                security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
                frame_counter,
                source_address: COORDINATOR_IEEE,
                key_seq_number: KEY_SEQUENCE,
            };
            let sec_len = sec_hdr.serialize(&mut buf[hdr_len..]);
            let aad_len = hdr_len + sec_len;
            let ciphertext = NwkSecurity::new()
                .encrypt(&buf[..aad_len], payload, &NETWORK_KEY, &sec_hdr)
                .expect("test frame encrypts");
            buf[aad_len..aad_len + ciphertext.len()].copy_from_slice(&ciphertext);
            // Zigbee transmits zero in the OTA security-level bits.
            buf[hdr_len] &= !0x07;
            aad_len + ciphertext.len()
        } else {
            buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
            hdr_len + payload.len()
        };

        zigbee_mac::MacFrame::from_slice(&buf[..total]).expect("frame fits")
    }

    fn indication(payload: zigbee_mac::MacFrame) -> zigbee_mac::McpsDataIndication {
        zigbee_mac::McpsDataIndication {
            src_address: zigbee_types::MacAddress::Short(zigbee_types::PanId(0x1234), COORDINATOR),
            dst_address: zigbee_types::MacAddress::Short(
                zigbee_types::PanId(0x1234),
                ShortAddress(OUR_SHORT),
            ),
            lqi: 200,
            payload,
            security_use: false,
        }
    }

    /// The same indication, but transmitted by `previous_hop` rather than
    /// directly by the NWK source.
    #[cfg(feature = "router")]
    fn indication_from(
        payload: zigbee_mac::MacFrame,
        previous_hop: ShortAddress,
    ) -> zigbee_mac::McpsDataIndication {
        let mut indication = indication(payload);
        indication.src_address =
            zigbee_types::MacAddress::Short(zigbee_types::PanId(0x1234), previous_hop);
        indication
    }

    fn leave_command(rejoin: bool) -> [u8; 2] {
        [
            zigbee_nwk::frames::NwkCommandId::Leave as u8,
            zigbee_nwk::frames::LeaveCommand {
                remove_children: false,
                request: true,
                rejoin,
            }
            .serialize(),
        ]
    }

    /// A parent leaving of its own accord (`request = false`), which the NWK
    /// layer surfaces as [`NwkCommandOutcome::ParentLeft`] rather than a
    /// requested leave.
    fn parent_leave_command() -> [u8; 2] {
        [
            zigbee_nwk::frames::NwkCommandId::Leave as u8,
            zigbee_nwk::frames::LeaveCommand {
                remove_children: false,
                request: false,
                rejoin: false,
            }
            .serialize(),
        ]
    }

    fn mgmt_leave_aps_payload(remove_children: bool, rejoin: bool) -> ([u8; 32], usize) {
        let header = zigbee_aps::frames::ApsHeader {
            frame_control: zigbee_aps::frames::ApsFrameControl {
                frame_type: zigbee_aps::frames::ApsFrameType::Data as u8,
                delivery_mode: zigbee_aps::frames::ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: Some(0),
            group_address: None,
            cluster_id: Some(zigbee_zdo::MGMT_LEAVE_REQ),
            profile_id: Some(zigbee_zdo::ZDP_PROFILE_ID),
            src_endpoint: Some(0),
            aps_counter: 1,
            extended_header: None,
        };
        let mut payload = [0u8; 32];
        let header_len = header.serialize(&mut payload);
        payload[header_len] = 0x42;
        payload[header_len + 9] = (u8::from(remove_children) << 6) | (u8::from(rejoin) << 7);
        (payload, header_len + 10)
    }

    #[test]
    fn secured_mgmt_leave_with_remove_children_clears_persisted_credentials() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 1);
        let _ = device.remote_reporting.record(1, 0x0402);
        assert_eq!(device.remote_reporting_cluster_count(1), 1);
        let preserved = store.load().unwrap().unwrap();

        let (aps_payload, aps_len) = mgmt_leave_aps_payload(true, false);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &aps_payload[..aps_len],
            1,
            true,
        );
        let event = block_on(device.process_incoming_with_security_store(
            &indication(frame),
            &mut [],
            &mut store,
        ))
        .unwrap();

        assert!(matches!(event, Some(crate::event_loop::StackEvent::Left)));
        assert!(!device.is_joined());
        let reset = store.load().unwrap().unwrap();
        assert!(!reset.commissioned);
        assert_eq!(reset.ieee_address, [0; 8]);
        assert_eq!(reset.network_key, [0; 16]);
        assert_eq!(reset.trust_center_link_key, [0; 16]);
        assert_eq!(reset.global_counter_limit, preserved.global_counter_limit);
        assert_eq!(reset.tclk_counter_limit, preserved.tclk_counter_limit);
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 0);
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
    }

    #[test]
    fn secured_mgmt_leave_rejoin_flag_requests_secure_rejoin() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();

        let (aps_payload, aps_len) = mgmt_leave_aps_payload(true, true);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &aps_payload[..aps_len],
            1,
            true,
        );
        let event = block_on(device.process_incoming_with_security_store(
            &indication(frame),
            &mut [],
            &mut store,
        ))
        .unwrap();

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        assert!(!device.is_joined());
        assert!(device.secure_rejoin_pending());
        let persisted = store.load().unwrap().unwrap();
        assert!(persisted.commissioned);
        assert!(persisted.rejoin_pending);
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 1);
    }

    #[test]
    fn failed_leave_notification_still_clears_local_security_state() {
        let mut device = resumed_device(DeviceType::EndDevice);
        device.mac_mut().set_tx_failures(1);

        block_on(device.leave()).unwrap();

        assert!(!device.is_joined());
        assert!(device.bdb().zdo().nwk().security().active_key().is_none());
        assert_eq!(device.bdb().zdo().aps().security().key_count(), 0);
    }

    #[test]
    fn secured_parent_leave_still_drives_rejoin_through_the_shared_nwk_path() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &leave_command(true),
            1,
            true,
        );

        let event = block_on(device.process_incoming(&indication(frame), &mut []));

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        assert!(!device.is_joined());
        assert!(device.secure_rejoin_pending());
        assert_eq!(
            device.nwk_rx_security_stats().decrypt_successes,
            1,
            "the shared NWK path owns decryption and counter commits"
        );
    }

    #[test]
    fn secured_leave_without_rejoin_reports_a_leave_request() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &leave_command(false),
            1,
            true,
        );

        let event = block_on(device.process_incoming(&indication(frame), &mut []));

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::LeaveRequested)
        ));
        assert!(!device.is_joined());
        assert!(!device.secure_rejoin_pending());
    }

    #[test]
    fn nwk_leave_with_rejoin_clears_remote_reporting_immediately() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let _ = device.remote_reporting.record(1, 0x0402);
        assert_eq!(device.remote_reporting_cluster_count(1), 1);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &leave_command(true),
            1,
            true,
        );

        let event = block_on(device.process_incoming(&indication(frame), &mut []));

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        // Cleared immediately on the accepted inbound leave, before any later
        // rejoin/lifecycle action.
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
    }

    #[test]
    fn nwk_leave_without_rejoin_clears_remote_reporting_immediately() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let _ = device.remote_reporting.record(1, 0x0402);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &leave_command(false),
            1,
            true,
        );

        let event = block_on(device.process_incoming(&indication(frame), &mut []));

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::LeaveRequested)
        ));
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
    }

    #[test]
    fn parent_leave_clears_remote_reporting_immediately() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let _ = device.remote_reporting.record(1, 0x0402);
        assert_eq!(device.remote_reporting_cluster_count(1), 1);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &parent_leave_command(),
            1,
            true,
        );

        let event = block_on(device.process_incoming(&indication(frame), &mut []));

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
    }

    #[test]
    fn secured_mgmt_leave_rejoin_clears_remote_reporting_immediately() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();
        let _ = device.remote_reporting.record(1, 0x0402);
        assert_eq!(device.remote_reporting_cluster_count(1), 1);

        let (aps_payload, aps_len) = mgmt_leave_aps_payload(true, true);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &aps_payload[..aps_len],
            1,
            true,
        );
        let event = block_on(device.process_incoming_with_security_store(
            &indication(frame),
            &mut [],
            &mut store,
        ))
        .unwrap();

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        // The Mgmt_Leave rejoin transition drops the interview record without
        // waiting for the later `mark_left`.
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
    }

    #[test]
    fn accepted_mgmt_leave_rejoin_survives_response_tx_failure() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        device.mac_mut().clear_tx_history();
        let _ = device.remote_reporting.record(1, 0x0402);
        // Fail the Mgmt_Leave_rsp transmission. The accepted request must
        // still clear interview state and enter the requested rejoin path.
        device.mac_mut().set_tx_failures(1);

        let (aps_payload, aps_len) = mgmt_leave_aps_payload(true, true);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &aps_payload[..aps_len],
            1,
            true,
        );
        let event = block_on(device.process_incoming_with_security_store(
            &indication(frame),
            &mut [],
            &mut store,
        ))
        .unwrap();

        assert!(matches!(
            event,
            Some(crate::event_loop::StackEvent::RejoinRequested)
        ));
        assert!(!device.is_joined());
        assert!(device.secure_rejoin_pending());
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
        assert_eq!(device.bdb().zdo().diagnostics().response_failures, 1);
    }

    #[test]
    fn unsecured_leave_is_ignored_on_a_secured_network() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &leave_command(true),
            1,
            false,
        );

        assert!(block_on(device.process_incoming(&indication(frame), &mut [])).is_none());
        assert!(device.is_joined());
    }

    #[test]
    fn replayed_secured_frames_are_rejected_once_the_counter_is_committed() {
        let mut device = resumed_device(DeviceType::EndDevice);
        let payload = [0x00u8, 0x01, 0x02];
        let first = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &payload,
            7,
            true,
        );
        let replay = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &payload,
            7,
            true,
        );

        let _ = block_on(device.process_incoming(&indication(first), &mut []));
        assert_eq!(device.nwk_rx_security_stats().decrypt_successes, 1);

        let _ = block_on(device.process_incoming(&indication(replay), &mut []));
        assert_eq!(
            device.nwk_rx_security_stats().replay_rejections,
            1,
            "a committed frame counter must reject the replay"
        );
        assert_eq!(device.nwk_rx_security_stats().decrypt_successes, 1);
    }

    // Relaying needs the router routing/BTR/source-route tables, which are
    // compiled to zero capacity without `zigbee-nwk/router`. Run with
    // `cargo test -p zigbee-runtime --features router`.
    #[cfg(feature = "router")]
    #[test]
    fn router_relays_traffic_for_other_devices_and_delivers_its_own_locally() {
        let mut device = resumed_router();

        // Addressed to another device: authenticated first, then forwarded
        // toward the best next hop (here the parent, the last resort of
        // `resolve_next_hop`) under freshly applied NWK security.
        let relayed = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            NEIGHBOUR,
            &[0xAA, 0xBB],
            11,
            true,
        );
        assert!(block_on(device.process_incoming(&indication(relayed), &mut [])).is_none());
        {
            let history = device.mac_mut().tx_history();
            assert_eq!(history.len(), 1, "a unicast for another device is relayed");
            let bytes = history[0].payload.as_slice();
            let (header, consumed) =
                zigbee_nwk::frames::NwkHeader::parse(bytes).expect("relayed frame parses");
            assert_eq!(
                header.dst_addr, NEIGHBOUR,
                "relay preserves the destination"
            );
            assert_eq!(
                header.src_addr, COORDINATOR,
                "relay preserves the originator"
            );
            assert_eq!(header.radius, 4, "relay decrements the radius");
            assert!(header.frame_control.security);
            assert!(matches!(
                history[0].dst,
                zigbee_types::MacAddress::Short(_, COORDINATOR)
            ));

            // The mutated header is CCM* additional authenticated data, so the
            // relay must re-secure the frame under its own identity instead of
            // forwarding the originator's ciphertext and MIC.
            let (aux, _) = zigbee_nwk::security::NwkSecurityHeader::parse(&bytes[consumed..])
                .expect("the relay writes its own auxiliary header");
            assert_eq!(aux.source_address, IEEE_ADDRESS);
            assert_ne!(
                aux.frame_counter, 11,
                "the relay allocates its own durable frame counter"
            );
        }
        assert_eq!(
            device.nwk_rx_security_stats().decrypt_successes,
            1,
            "a relayed frame is authenticated before it is forwarded"
        );

        device.mac_mut().clear_tx_history();

        // Addressed to us: delivered upwards, never relayed.
        let local = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &[0xAA, 0xBB],
            12,
            true,
        );
        let _ = block_on(device.process_incoming(&indication(local), &mut []));
        assert!(
            device.mac_mut().tx_history().is_empty(),
            "a frame addressed to us must not be relayed"
        );
        assert_eq!(device.nwk_rx_security_stats().decrypt_successes, 2);
    }

    // Relaying needs the router routing/BTR/source-route tables, which are
    // compiled to zero capacity without `zigbee-nwk/router`. Run with
    // `cargo test -p zigbee-runtime --features router`.
    #[cfg(feature = "router")]
    #[test]
    fn router_relays_to_an_announced_neighbour_instead_of_queueing_it() {
        let mut device = resumed_router();
        // A Device_annce only carries the address pair, so the neighbour entry
        // keeps the default `rx_on_when_idle == false`. It is not a child of
        // ours and must not be treated as a sleeping one: nothing drains the
        // indirect queue, so buffering here would blackhole the traffic.
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .update_neighbor_address(NEIGHBOUR, [9u8; 8]);

        let relayed = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            NEIGHBOUR,
            &[0xAA, 0xBB],
            11,
            true,
        );
        assert!(block_on(device.process_incoming(&indication(relayed), &mut [])).is_none());

        assert!(
            !device
                .bdb
                .zdo()
                .nwk()
                .indirect_queue()
                .has_pending(NEIGHBOUR),
            "an announced neighbour must not be parked in an undrainable queue"
        );
        let history = device.mac_mut().tx_history();
        assert_eq!(history.len(), 1, "the frame is relayed directly");
        assert!(matches!(
            history[0].dst,
            zigbee_types::MacAddress::Short(_, NEIGHBOUR)
        ));
    }

    #[test]
    fn router_resume_restarts_router_operation() {
        let device = resumed_router();

        assert_eq!(
            block_on(
                device
                    .bdb
                    .zdo()
                    .nwk()
                    .mac()
                    .mlme_get(PibAttribute::MacRxOnWhenIdle)
            )
            .unwrap(),
            PibValue::Bool(true),
            "a resumed router must restart MAC router operation"
        );
    }

    #[test]
    fn a_router_that_cannot_start_is_reported_instead_of_half_joined() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();

        // NLME-START-ROUTER refuses a device that is not on a network.
        assert_eq!(
            block_on(device.restore_router_operation()),
            Err(crate::event_loop::StartError::InitFailed),
            "an unsupported router start must not be silently ignored"
        );
        assert!(!device.is_joined());
    }

    // Relaying needs the router routing/BTR/source-route tables, which are
    // compiled to zero capacity without `zigbee-nwk/router`. Run with
    // `cargo test -p zigbee-runtime --features router`.
    #[cfg(feature = "router")]
    #[test]
    fn a_relayed_many_to_one_request_routes_through_the_transmitting_neighbour() {
        use zigbee_nwk::frames::{NwkCommandId, RouteRequest};

        let mut device = resumed_router();

        // The concentrator (our coordinator) originated this many-to-one
        // request; the NWK header still names it several hops later. The
        // frame reached us from NEIGHBOUR, and that is the only device the
        // route to the concentrator may go through.
        let rreq = RouteRequest {
            command_options: zigbee_nwk::routing::ConcentratorType::LowRam.rreq_options(),
            route_request_id: 3,
            dst_addr: ShortAddress(0xFFFC),
            path_cost: 1,
            dst_ieee: None,
        };
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .update_neighbor_address(NEIGHBOUR, [0x22; 8]);
        let mut command = [0u8; 16];
        command[0] = NwkCommandId::RouteRequest as u8;
        let len = 1 + rreq.serialize(&mut command[1..]);
        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(0xFFFC),
            &command[..len],
            21,
            true,
        );

        assert!(
            block_on(device.process_incoming(&indication_from(frame, NEIGHBOUR), &mut []))
                .is_none()
        );

        let entry = device
            .bdb
            .zdo()
            .nwk()
            .routing_table()
            .get_entry(COORDINATOR)
            .expect("the many-to-one route is installed");
        assert_eq!(
            entry.next_hop, NEIGHBOUR,
            "the runtime carries the MAC previous hop into the NWK layer"
        );
        assert!(entry.many_to_one);

        // And the forwarded copy is still the concentrator's own broadcast.
        block_on(device.bdb.zdo_mut().nwk_mut().process_pending_routing());
        let history = device.mac_mut().tx_history();
        assert_eq!(
            history.len(),
            3,
            "the request is relayed once and retried twice"
        );
        let (header, consumed) =
            zigbee_nwk::frames::NwkHeader::parse(history[0].payload.as_slice())
                .expect("the forward parses");
        assert_eq!(header.src_addr, COORDINATOR);
        assert_eq!(header.radius, 4, "the received radius is decremented");
        assert!(header.frame_control.security);
        let (aux, aux_len) = zigbee_nwk::security::NwkSecurityHeader::parse(
            &history[0].payload.as_slice()[consumed..],
        )
        .expect("the forward carries a fresh auxiliary header");
        assert_eq!(aux_len, zigbee_nwk::security::NWK_AUX_HEADER_LEN);
        assert_eq!(
            aux.source_address, IEEE_ADDRESS,
            "NWK security is hop by hop: this relay signs the forward itself"
        );
        assert_ne!(
            aux.frame_counter, 21,
            "and spends its own durable counter, not the concentrator's"
        );
    }

    /// R22 §2.2.4.1.3 / §2.2.5.1.1.5: an APS *command* frame that requests an
    /// acknowledgement is acknowledged, and Tunnel is such a command. The
    /// Tunnel path returns early from the receive routine — it forwards the
    /// embedded key to the joining child instead of producing a data
    /// indication — so the acknowledgement the APS layer queued has to be
    /// flushed on that path too. Leaving it in the single pending slot would
    /// either drop it or spend it on the APS counter of an unrelated later
    /// frame, and the Trust Center would keep retransmitting the tunnelled
    /// key through us.
    #[test]
    #[cfg(feature = "router")]
    fn a_tunnelled_key_that_requests_an_ack_is_acknowledged_and_still_forwarded() {
        use zigbee_aps::frames::{
            ApsCommandId, ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader,
        };
        use zigbee_aps::security::{ApsSecurityHeader, KEY_ID_KEY_TRANSPORT};
        use zigbee_mac::CapabilityInfo;

        const CHILD_IEEE: [u8; 8] = [0x33; 8];
        const TUNNEL_APS_COUNTER: u8 = 0x37;

        let mut device = resumed_router();

        // The joining child the Trust Center is tunnelling a key to.
        let child = {
            let nwk = device.bdb.zdo_mut().aps_mut().nwk_mut();
            nwk.nib_mut().permit_joining = true;
            nwk.handle_child_association(
                CHILD_IEEE,
                CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: false,
                    rx_on_when_idle: false,
                    security_capable: true,
                    allocate_address: true,
                }
                .to_byte(),
            )
            .expect("the child associates")
        };

        // The tunnelled payload: an APS-secured, key-transport-keyed command
        // from the Trust Center. Only its header is inspected on this hop —
        // the child owns the decryption — so the ciphertext is opaque here.
        let embedded_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: true,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: 0x12,
            extended_header: None,
        };
        let embedded_security = ApsSecurityHeader {
            security_control: (KEY_ID_KEY_TRANSPORT << 3) | (1 << 5),
            frame_counter: 7,
            source_address: Some(COORDINATOR_IEEE),
            key_seq_number: None,
        };
        let mut embedded = [0u8; 64];
        let mut embedded_len = embedded_header.serialize(&mut embedded);
        embedded_len += embedded_security.serialize(&mut embedded[embedded_len..]);
        embedded[embedded_len..embedded_len + 8].copy_from_slice(&[0xAB; 8]);
        embedded_len += 8;

        // The outer Tunnel command, unsecured at the APS layer as R22 requires
        // and asking for an acknowledgement.
        let tunnel_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: true,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: TUNNEL_APS_COUNTER,
            extended_header: None,
        };
        let mut aps_frame = [0u8; 96];
        let mut aps_len = tunnel_header.serialize(&mut aps_frame);
        aps_frame[aps_len] = ApsCommandId::Tunnel as u8;
        aps_len += 1;
        aps_frame[aps_len..aps_len + 8].copy_from_slice(&CHILD_IEEE);
        aps_len += 8;
        aps_frame[aps_len..aps_len + embedded_len].copy_from_slice(&embedded[..embedded_len]);
        aps_len += embedded_len;

        let frame = nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Data,
            ShortAddress(OUR_SHORT),
            &aps_frame[..aps_len],
            31,
            true,
        );
        device.mac_mut().clear_tx_history();

        assert!(block_on(device.process_incoming(&indication(frame), &mut [])).is_none());

        // The tunnelled key still reaches the sleepy child's indirect queue.
        assert!(
            device.bdb.zdo().nwk().indirect_queue().has_pending(child),
            "the Tunnel must still be forwarded to the joining child"
        );

        // …and the acknowledgement went out. The child's copy is deliberately
        // *not* NWK-secured and is held for its next poll, so the one frame on
        // the air here is the acknowledgement to the Trust Center.
        let history = device.mac_mut().tx_history();
        assert_eq!(
            history.len(),
            1,
            "the Tunnel command must be acknowledged exactly once"
        );
        let bytes = history[0].payload.as_slice();
        let (nwk_header, nwk_consumed) =
            zigbee_nwk::frames::NwkHeader::parse(bytes).expect("the acknowledgement parses");
        assert_eq!(nwk_header.dst_addr, COORDINATOR);
        assert!(nwk_header.frame_control.security);
        let (aux, aux_len) = zigbee_nwk::security::NwkSecurityHeader::parse(&bytes[nwk_consumed..])
            .expect("the acknowledgement is NWK-secured");
        let aad_len = nwk_consumed + aux_len;
        // Zigbee transmits security level 0 but authenticates the real level 5.
        let mut aad = [0u8; 64];
        aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
        aad[nwk_consumed] = (aad[nwk_consumed] & !0x07) | 0x05;
        let plaintext = zigbee_nwk::security::NwkSecurity::new()
            .decrypt(&aad[..aad_len], &bytes[aad_len..], &NETWORK_KEY, &aux)
            .expect("the acknowledgement decrypts under the network key");
        let (ack, _) = ApsHeader::parse(&plaintext).expect("the APS acknowledgement parses");
        assert_eq!(ack.frame_control.frame_type, ApsFrameType::Ack as u8);
        assert!(
            ack.frame_control.ack_format,
            "an APS command is acknowledged in the command format"
        );
        assert_eq!(
            ack.aps_counter, TUNNEL_APS_COUNTER,
            "the acknowledgement echoes the Tunnel's own APS counter"
        );
    }

    #[test]
    fn end_device_resume_does_not_start_router_operation() {
        let device = resumed_device(DeviceType::EndDevice);

        assert_eq!(
            block_on(
                device
                    .bdb
                    .zdo()
                    .nwk()
                    .mac()
                    .mlme_get(PibAttribute::MacRxOnWhenIdle)
            )
            .unwrap(),
            PibValue::Bool(false),
            "end-device resume behaviour must be unchanged"
        );
    }

    // ── R22 End Device Timeout client lifecycle ──────────────

    use zigbee_nwk::frames::{
        ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_REQUESTED, NwkCommandId,
        ed_timeout_enum_to_seconds,
    };

    /// Payloads of every End Device Timeout Request (0x0B) this device sent.
    ///
    /// Transmissions on a commissioned network are NWK-secured, so the frame
    /// is decrypted with the same network key the test installed before the
    /// command body is inspected.
    fn ed_timeout_requests<R: crate::role::DeviceRole>(
        device: &ZigbeeDevice<MockMac, R>,
    ) -> std::vec::Vec<[u8; 2]> {
        device
            .mac()
            .tx_history()
            .iter()
            .filter_map(|record| {
                let frame = record.payload.as_slice();
                let (header, header_len) = zigbee_nwk::frames::NwkHeader::parse(frame)?;
                let body = if header.frame_control.security {
                    let (sec_hdr, sec_len) =
                        zigbee_nwk::security::NwkSecurityHeader::parse(&frame[header_len..])?;
                    let aad_len = header_len + sec_len;
                    let mut aad = [0u8; 64];
                    aad[..aad_len].copy_from_slice(&frame[..aad_len]);
                    // The over-the-air security level bits are zero; the AAD
                    // uses the real level, exactly like the receive path.
                    aad[header_len] = (aad[header_len] & !0x07) | 0x05;
                    zigbee_nwk::security::NwkSecurity::new().decrypt(
                        &aad[..aad_len],
                        &frame[aad_len..],
                        &NETWORK_KEY,
                        &sec_hdr,
                    )?
                } else {
                    let mut plain = heapless::Vec::<u8, 128>::new();
                    plain.extend_from_slice(frame.get(header_len..)?).ok()?;
                    plain
                };
                if body.first().copied()? != NwkCommandId::EdTimeoutRequest as u8 {
                    return None;
                }
                Some([*body.get(1)?, *body.get(2)?])
            })
            .collect()
    }

    /// Commissioned record whose stored negotiation matches `(info, valid)`.
    fn negotiated_state(
        parent_information: u8,
        valid: bool,
        timeout: u8,
    ) -> PersistentSecurityState {
        let mut state = commissioned_state();
        state.parent_information = parent_information;
        state.parent_information_valid = valid;
        state.end_device_timeout = timeout;
        state
    }

    fn resumed_end_device(
        state: PersistentSecurityState,
    ) -> (ZigbeeDevice<MockMac>, RamSecurityStateStore) {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        (device, store)
    }

    /// A resumed *sleepy* end device whose automatic poll cadence is
    /// `poll_interval_ms`, so `run_sleepy_poll` polls on the application (not
    /// forced-keepalive) path once the interval elapses.
    fn resumed_sleepy_end_device(
        state: PersistentSecurityState,
        poll_interval_ms: u32,
    ) -> (ZigbeeDevice<MockMac>, RamSecurityStateStore) {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .power_mode(crate::power::PowerMode::Sleepy {
                poll_interval_ms,
                wake_duration_ms: 0,
            })
            .build();
        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        (device, store)
    }

    /// A secured NWK End Device Timeout Response from the coordinator.
    fn ed_timeout_response(status: u8, parent_info: u8, counter: u32) -> zigbee_mac::MacFrame {
        nwk_frame(
            zigbee_nwk::frames::NwkFrameType::Command,
            ShortAddress(OUR_SHORT),
            &[NwkCommandId::EdTimeoutResponse as u8, status, parent_info],
            counter,
            true,
        )
    }

    #[test]
    fn a_fresh_join_negotiation_sends_exactly_one_long_timeout_request() {
        // `finish_join()` is the single choke point every fresh join and
        // secured rejoin passes through, so proving it here proves `start()`,
        // `start_with_security_store()`, the steering branch of
        // `start_or_resume_with_security_store()`, `secure_rejoin()` and the
        // `UserAction::Join`/`Toggle` handlers, which have no request of their
        // own. A full BDB steering join cannot be driven to completion against
        // `MockMac` — nothing transports the network key — so the assertion is
        // made at the shared choke point rather than through a fake join.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.mac_mut().clear_tx_history();
        device.reset_end_device_timeout_state();
        // A real join renegotiates from scratch at the NWK parent-assignment
        // point.
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .reset_end_device_timeout_negotiation();

        block_on(device.finish_join()).unwrap();

        assert_eq!(
            ed_timeout_requests(&device),
            std::vec![[ED_TIMEOUT_ENUM_REQUESTED, 0x00]],
            "a fresh join sends exactly one request, with the reserved byte clear"
        );
        assert!(
            device.ed_timeout().forced_poll,
            "the indirect response needs a forced poll on the next tick"
        );
    }

    #[test]
    fn a_join_still_succeeds_when_the_initial_request_cannot_be_transmitted() {
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.mac_mut().clear_tx_history();
        device.mac_mut().set_tx_failures(64);
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .reset_end_device_timeout_negotiation();

        let address = block_on(device.finish_join())
            .expect("a join must not fail because a keepalive frame did not go out");

        assert_eq!(address, OUR_SHORT);
        assert!(device.is_joined());
        assert!(
            !device.secure_rejoin_pending(),
            "a single keepalive failure must not schedule a rejoin"
        );
        assert!(
            device.ed_timeout().keepalive_remaining_secs.is_some(),
            "the recurring keepalive owns recovery after a failed request"
        );
    }

    /// Current neighbour-cache age for the entry with this IEEE address.
    fn neighbour_age(device: &ZigbeeDevice<MockMac>, ieee: &[u8; 8]) -> Option<u16> {
        device
            .bdb
            .zdo()
            .nwk()
            .neighbor_table()
            .find_by_ieee(ieee)
            .map(|entry| entry.age)
    }

    #[test]
    fn end_device_tick_runs_common_nwk_maintenance_and_the_ed_timeout_client() {
        // A non-routing end device links no parent/router maintenance, but the
        // joined tick must still run the role-independent NWK maintenance
        // (neighbour-cache aging via `run_common_nwk_maintenance`) and keep the
        // End Device Timeout *client* alive. This proves the maintenance split
        // preserved the mandatory common end-device tick work.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.mac_mut().clear_tx_history();

        // A neighbour learned from a Device_annce (not our parent) starts at
        // age 0 and is never reset by parent traffic, so its age tracks exactly
        // how many seconds of common maintenance the tick applied.
        const NEIGHBOUR: [u8; 8] = [0xC7; 8];
        device
            .bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .update_neighbor_address(ShortAddress(0x2345), NEIGHBOUR);
        assert_eq!(neighbour_age(&device, &NEIGHBOUR), Some(0));

        // The End Device Timeout client is armed after a resume — the common,
        // non-router keepalive path the split must preserve.
        assert!(
            device.ed_timeout().keepalive_remaining_secs.is_some(),
            "a resumed end device tracks its keepalive countdown"
        );

        let _ = block_on(device.tick(5, &mut []));

        assert_eq!(
            neighbour_age(&device, &NEIGHBOUR),
            Some(5),
            "run_common_nwk_maintenance ages the end-device neighbour cache once per elapsed second"
        );
        assert!(
            device.ed_timeout().keepalive_remaining_secs.is_some(),
            "the End Device Timeout client stays armed across the tick"
        );
    }

    #[test]
    fn silent_resume_forces_a_poll_for_a_poll_aging_parent() {
        for (info, valid) in [(0x01, true), (0x03, true), (0x00, true)] {
            let (mut device, _store) = resumed_end_device(negotiated_state(info, valid, 14));

            assert!(
                ed_timeout_requests(&device).is_empty(),
                "info=0x{info:02X}: a poll-aging parent needs no fresh negotiation"
            );
            let polls_before = device.mac().poll_count();
            // A default-built device is AlwaysOn and would never poll
            // automatically, so any poll here proves the forced-poll bypass.
            assert!(!device.is_sleepy());
            let _ = block_on(device.tick(1, &mut []));
            assert_eq!(
                device.mac().poll_count(),
                polls_before + 1,
                "info=0x{info:02X}: the resume keepalive must force one poll"
            );
        }
    }

    #[test]
    fn silent_resume_requests_a_timeout_for_request_only_or_unknown_parents() {
        for (info, valid) in [(0x02, true), (0x00, false)] {
            let (device, _store) = resumed_end_device(negotiated_state(info, valid, 8));

            assert_eq!(
                ed_timeout_requests(&device).len(),
                1,
                "info=0x{info:02X} valid={valid}: a poll would not refresh the parent timer"
            );
        }
    }

    #[test]
    fn a_forced_poll_retrieves_the_indirect_response_and_persists_it() {
        let (mut device, mut store) = resumed_end_device(negotiated_state(0x00, false, 8));
        assert_eq!(ed_timeout_requests(&device).len(), 1);

        device
            .mac_mut()
            .enqueue_poll_response(ed_timeout_response(0x00, 0x01, 1));
        let _ = block_on(device.tick(1, &mut []));

        let nib = device.bdb.zdo().nwk().nib();
        assert!(nib.parent_information_valid);
        assert_eq!(nib.parent_information, 0x01);
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
        assert!(device.state_dirty());

        assert!(device.refresh_security_state(&mut store).unwrap());
        let stored = store.load().unwrap().unwrap();
        assert!(stored.parent_information_valid);
        assert_eq!(stored.parent_information, 0x01);
        assert_eq!(stored.end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);

        // A reboot restores the negotiated relationship and then only polls.
        let (rebooted, _store) = resumed_end_device(stored);
        assert!(ed_timeout_requests(&rebooted).is_empty());
        let nib = rebooted.bdb.zdo().nwk().nib();
        assert!(nib.parent_information_valid);
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
    }

    #[test]
    fn a_refused_timeout_is_retried_one_step_lower_and_never_below_the_default() {
        let (mut device, _store) = resumed_end_device(negotiated_state(0x00, false, 8));
        device.mac_mut().clear_tx_history();

        // Enough refusals to walk the request all the way to the floor and
        // then several more. Each response needs a fresh NWK frame counter so
        // the replay window accepts it.
        for counter in 1..=u32::from(ED_TIMEOUT_ENUM_REQUESTED - ED_TIMEOUT_ENUM_DEFAULT + 3) {
            device
                .mac_mut()
                .enqueue_poll_response(ed_timeout_response(0x01, 0x02, counter));
            let _ = block_on(device.tick(1, &mut []));
        }

        let requests = ed_timeout_requests(&device);
        assert!(!requests.is_empty());
        let lowest = requests.iter().map(|request| request[0]).min().unwrap();
        assert_eq!(
            lowest, ED_TIMEOUT_ENUM_DEFAULT,
            "refusals must never retry below the default enumeration"
        );
        assert!(
            requests.iter().all(|request| request[1] == 0),
            "the reserved End Device Configuration byte stays clear"
        );
        assert!(
            !device.bdb.zdo().nwk().nib().parent_information_valid,
            "a refusal never validates parent information"
        );
    }

    #[test]
    fn an_unanswered_negotiation_retries_twice_and_falls_back_to_the_default() {
        let (mut device, _store) = resumed_end_device(negotiated_state(0x00, false, 8));
        device.mac_mut().clear_tx_history();

        // Each tick advances well past the bounded response wait.
        for _ in 0..6 {
            let _ = block_on(device.tick(30, &mut []));
        }

        assert_eq!(
            ed_timeout_requests(&device).len(),
            2,
            "exactly the bounded retransmission budget is spent"
        );
        let nib = device.bdb.zdo().nwk().nib();
        assert!(!nib.parent_information_valid);
        assert_eq!(
            nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT,
            "the recurring fallback is the default enumeration"
        );
        assert!(device.is_joined(), "a silent parent must not undo the join");

        device
            .mac_mut()
            .enqueue_poll_response(ed_timeout_response(0x00, 0x03, 1));
        device.force_end_device_poll();
        let _ = block_on(device.tick(1, &mut []));
        assert!(
            !device.bdb.zdo().nwk().nib().parent_information_valid,
            "a response after the retry round was abandoned must be ignored"
        );
    }

    #[test]
    fn a_repeat_acceptance_cancels_the_response_wait_without_retransmitting() {
        // A bit1-only parent is kept alive by the request itself, so every
        // keepalive re-confirms an unchanged timeout. That must still count as
        // an answer, or each keepalive would burn the whole retransmission
        // budget.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x02, true, 14));
        device.mac_mut().clear_tx_history();
        assert!(device.ed_timeout().response_remaining_secs.is_some());

        device
            .mac_mut()
            .enqueue_poll_response(ed_timeout_response(0x00, 0x02, 1));
        let _ = block_on(device.tick(1, &mut []));

        assert_eq!(
            device.ed_timeout().response_remaining_secs,
            None,
            "an acceptance must cancel the response wait"
        );
        assert!(ed_timeout_requests(&device).is_empty());
        assert_eq!(
            device.ed_timeout().keepalive_remaining_secs,
            Some(device.end_device_keepalive_interval_secs())
        );

        // Several further ticks must not retransmit anything.
        for _ in 0..4 {
            let _ = block_on(device.tick(30, &mut []));
        }
        assert!(ed_timeout_requests(&device).is_empty());
    }

    #[test]
    fn a_poll_for_a_request_only_parent_never_postpones_the_next_request() {
        let (mut device, _store) = resumed_end_device(negotiated_state(0x02, true, 14));
        let interval = device.end_device_keepalive_interval_secs();
        device.ed_timeout_mut().keepalive_remaining_secs = Some(7);

        block_on(device.poll()).unwrap();

        assert_eq!(
            device.ed_timeout().keepalive_remaining_secs,
            Some(7),
            "a bit1-only parent does not age its children on polls"
        );

        // Every advertisement that explicitly supports poll aging refreshes
        // the parent timer.
        for (info, valid) in [(0x01, true), (0x03, true), (0x00, true)] {
            let (mut device, _store) = resumed_end_device(negotiated_state(info, valid, 14));
            device.ed_timeout_mut().keepalive_remaining_secs = Some(7);
            block_on(device.poll()).unwrap();
            assert_eq!(
                device.ed_timeout().keepalive_remaining_secs,
                Some(device.end_device_keepalive_interval_secs()),
                "info=0x{info:02X} valid={valid}"
            );
        }

        let (mut device, _store) = resumed_end_device(negotiated_state(0x00, false, 8));
        device.ed_timeout_mut().keepalive_remaining_secs = Some(7);
        block_on(device.poll()).unwrap();
        assert_eq!(
            device.ed_timeout().keepalive_remaining_secs,
            Some(7),
            "an unknown parent may be request-only, so a poll cannot postpone 0x0B"
        );
        let _ = interval;
    }

    #[test]
    fn the_keepalive_interval_stays_below_the_negotiated_timeout() {
        let (device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        let timeout = ed_timeout_enum_to_seconds(ED_TIMEOUT_ENUM_REQUESTED).unwrap();
        let interval = device.end_device_keepalive_interval_secs();
        assert!(interval < timeout);
        assert_eq!(interval, timeout / 3);

        let (device, _store) = resumed_end_device(negotiated_state(0x00, false, 8));
        let fallback = ed_timeout_enum_to_seconds(ED_TIMEOUT_ENUM_DEFAULT).unwrap();
        assert_eq!(device.end_device_keepalive_interval_secs(), fallback / 3);
    }

    #[test]
    fn a_due_keepalive_polls_or_requests_by_advertised_method() {
        // Poll-aging parent: the keepalive is a forced poll, not a request.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.mac_mut().clear_tx_history();
        device.ed_timeout_mut().keepalive_remaining_secs = Some(0);
        let polls_before = device.mac().poll_count();
        let _ = block_on(device.tick(0, &mut []));
        let _ = block_on(device.tick(0, &mut []));
        assert!(ed_timeout_requests(&device).is_empty());
        assert!(device.mac().poll_count() > polls_before);
        assert_eq!(
            device.ed_timeout().keepalive_remaining_secs,
            Some(device.end_device_keepalive_interval_secs())
        );

        // Request-only parent: the keepalive is a fresh 0x0B.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x02, true, 14));
        device.mac_mut().clear_tx_history();
        device.ed_timeout_mut().keepalive_remaining_secs = Some(0);
        let _ = block_on(device.tick(0, &mut []));
        assert_eq!(ed_timeout_requests(&device).len(), 1);
    }

    #[test]
    fn repeated_keepalive_failures_schedule_the_secure_rejoin_retry() {
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.mac_mut().set_poll_failures(64);
        assert!(!device.secure_rejoin_pending());

        for _ in 0..8 {
            device.ed_timeout_mut().keepalive_remaining_secs = Some(0);
            let _ = block_on(device.tick(0, &mut []));
            let _ = block_on(device.tick(0, &mut []));
            if device.secure_rejoin_pending() {
                break;
            }
        }

        assert!(
            device.secure_rejoin_pending(),
            "a parent that stops answering polls must trigger the rejoin path"
        );
    }

    #[test]
    fn application_driven_polls_that_lose_the_parent_ack_trigger_recovery() {
        // An OTA fast poll is an *application-driven* poll: the app calls the
        // public `poll()` directly (see `service_joined_polls`), bypassing the
        // forced-keepalive gate entirely. A parent that has silently stopped
        // MAC-ACKing Data Requests surfaces as `Err(NoAck)` from the MAC, which
        // must now advance the bounded failure counter in the single `poll()`
        // choke point and hand recovery to the secure-rejoin retry path —
        // exactly like a missed keepalive.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        let _ = device.take_forced_poll();
        device.mac_mut().set_poll_failures(64);
        assert!(!device.secure_rejoin_pending());

        // Mirror the application OTA fast-poll loop: call `poll()` directly.
        for _ in 0..10 {
            let _ = block_on(device.poll());
            if device.secure_rejoin_pending() {
                break;
            }
        }

        assert!(
            device.secure_rejoin_pending(),
            "no-ACK application/OTA polls must drive the same recovery as keepalive loss"
        );
    }

    #[test]
    fn application_driven_empty_polls_are_normal_and_never_trigger_recovery() {
        // With nothing queued the MAC returns `Ok(None)` — an ACKed-but-empty
        // poll. The parent is reachable, so ordinary empty application polls
        // must neither advance the failure counter nor schedule a rejoin, even
        // when driven from the application `poll()` fast-poll loop.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        let _ = device.take_forced_poll();
        assert!(!device.secure_rejoin_pending());

        for _ in 0..10 {
            let _ = block_on(device.poll());
        }

        assert!(
            !device.secure_rejoin_pending(),
            "an ACKed-empty poll is normal and must not be conflated with parent loss"
        );
        assert_eq!(
            device.ed_timeout().failures,
            0,
            "successful empty polls keep the failure counter clear"
        );
    }

    #[test]
    fn a_recovered_parent_ack_clears_accumulated_poll_failures() {
        // A few no-ACK polls that stop short of the threshold must not leave a
        // latent count behind once the parent answers again: any acknowledged
        // poll (empty or not) clears the counter through the same choke point.
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        let _ = device.take_forced_poll();

        device.mac_mut().set_poll_failures(2);
        let _ = block_on(device.poll());
        let _ = block_on(device.poll());
        assert_eq!(device.ed_timeout().failures, 2);
        assert!(!device.secure_rejoin_pending());

        // Parent answers (empty) — counter resets before it can trip recovery.
        let _ = block_on(device.poll());
        assert_eq!(device.ed_timeout().failures, 0);
        assert!(!device.secure_rejoin_pending());
    }

    #[test]
    fn sleepy_tick_polls_also_participate_in_failure_accounting() {
        // The automatic sleepy-poll path (`run_sleepy_poll`) shares the same
        // `poll()` choke point, so a silent parent recovers there too.
        let (mut device, _store) =
            resumed_sleepy_end_device(negotiated_state(0x01, true, 14), 1_000);
        let _ = device.take_forced_poll();
        device.mac_mut().set_poll_failures(64);

        let mut now_ms = 0u32;
        for _ in 0..10 {
            now_ms = now_ms.wrapping_add(1_000);
            let _ = block_on(device.run_sleepy_poll(now_ms, &mut []));
            if device.secure_rejoin_pending() {
                break;
            }
        }

        assert!(
            device.secure_rejoin_pending(),
            "automatic sleepy polls must also drive recovery on parent loss"
        );
    }

    #[test]
    fn a_pending_rejoin_suppresses_further_failure_accounting() {
        // Once a secure rejoin is scheduled, a persistently silent parent —
        // which under an OTA fast-poll cadence can fail many times per second —
        // must not re-arm the retry or churn the counter (a recovery storm).
        let (mut device, _store) = resumed_end_device(negotiated_state(0x01, true, 14));
        device.schedule_secure_rejoin_retry();
        assert!(device.secure_rejoin_pending());

        for _ in 0..20 {
            device.record_end_device_keepalive_failure();
        }

        assert!(
            device.secure_rejoin_pending(),
            "the already-scheduled rejoin must still be pending"
        );
        assert_eq!(
            device.ed_timeout().failures,
            0,
            "no counter churn while a rejoin is already pending"
        );
    }

    #[test]
    fn resuming_a_router_never_negotiates_a_timeout() {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        store.store(&commissioned_state()).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();

        // A `Router` is not an `EndDeviceRole`, so it carries no End Device
        // Timeout *client* state at all — `ed_timeout()`/`take_forced_poll()`
        // do not even exist on it (see `role_split_removes_ed_client_from_*`).
        // The observable guarantee here is behavioural: resuming a router
        // transmits no client 0x0B request.
        assert!(
            ed_timeout_requests(&device).is_empty(),
            "a router must never transmit an End Device Timeout client request"
        );
    }
}

/// Byte budget of a queued ZCL response payload
/// ([`PendingZclResponse::zcl_data`]).
///
/// Shared with `zcl_dispatch` so its compile-time proof that a cluster-specific
/// response can never overflow this buffer (and therefore never hits
/// `queue_frame`'s drop branch) stays tied to the real capacity.
pub(crate) const PENDING_ZCL_DATA_CAP: usize = 128;

/// A queued ZCL response to be sent in the next tick().
///
/// Because `process_incoming()` is sync but sending requires async MAC access,
/// we queue responses here and drain them in `tick()`.
pub(crate) struct PendingZclResponse {
    pub(crate) dst_addr: ShortAddress,
    pub(crate) dst_endpoint: u8,
    pub(crate) src_endpoint: u8,
    pub(crate) cluster_id: u16,
    #[cfg(feature = "router")]
    pub(crate) zcl_data: heapless::Vec<u8, PENDING_ZCL_DATA_CAP>,
    #[cfg(not(feature = "router"))]
    pub(crate) zcl_data: heapless::Vec<u8, PENDING_ZCL_DATA_CAP>,
}

pub(crate) struct EndpointIdentifyCluster {
    pub(crate) endpoint: u8,
    pub(crate) cluster: IdentifyCluster,
}

/// Maximum number of endpoints on a device (endpoint 0 is ZDO, 1-240 are application)
#[cfg(feature = "router")]
pub const MAX_ENDPOINTS: usize = 8;
#[cfg(not(feature = "router"))]
pub const MAX_ENDPOINTS: usize = 4;
/// Maximum clusters per endpoint
#[cfg(feature = "router")]
pub const MAX_CLUSTERS_PER_ENDPOINT: usize = 16;
#[cfg(not(feature = "router"))]
pub const MAX_CLUSTERS_PER_ENDPOINT: usize = 8;

/// Endpoint configuration.
#[derive(Debug, Clone)]
pub struct EndpointConfig {
    pub endpoint: u8,
    pub profile_id: u16,
    pub device_id: DeviceId,
    pub device_version: u8,
    pub server_clusters: heapless::Vec<ClusterId, MAX_CLUSTERS_PER_ENDPOINT>,
    pub client_clusters: heapless::Vec<ClusterId, MAX_CLUSTERS_PER_ENDPOINT>,
}

/// A reference to a cluster instance, tagged with its endpoint.
///
/// Pass a slice of these to `tick()` and `process_incoming()` so the runtime
/// can dispatch commands, read/write attributes, and send reports automatically.
/// Basic and Identify are owned by `ZigbeeDevice`; only application-owned
/// sensor and actuator clusters belong in this slice.
pub struct ClusterRef<'a> {
    pub endpoint: u8,
    pub cluster: &'a mut dyn Cluster,
}

/// User-initiated actions, triggered by button presses or application logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UserAction {
    /// Join a network (BDB commissioning).
    Join,
    /// Rejoin a previously-joined network using stored NWK key.
    /// Use after `restore_state()` succeeds — skips full BDB commissioning
    /// and performs NWK-level rejoin on the last-known channel.
    Rejoin,
    /// Leave the current network.
    Leave,
    /// Toggle join/leave based on current state.
    Toggle,
    /// Open permit joining (coordinator/router only).
    PermitJoin(u8),
    /// Factory reset — leave network and clear all state.
    FactoryReset,
}

/// Outcome of one bounded, nonblocking parent-command maintenance step.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ParentCommandStep {
    pub processed: u8,
    pub failures: u8,
}

#[derive(Debug, Clone, Copy)]
struct PendingChildUpdate {
    poll_address: ShortAddress,
    device_address: IeeeAddress,
    device_short_address: ShortAddress,
    status: zigbee_aps::apsme::ApsUpdateDeviceStatus,
    expires_at_us: u32,
}

/// A *parent-only* NWK command outcome, extracted from
/// [`NwkCommandOutcome`](zigbee_nwk::nlde::NwkCommandOutcome) so it can be
/// dispatched statically through the role type.
///
/// Only a [`Router`](crate::role::Router) acts on these; [`EndDevice`] and
/// [`RelayRouter`] ignore them (see
/// [`DeviceRole::service_parent_nwk_outcome`](crate::role::DeviceRole::service_parent_nwk_outcome)),
/// which is what stops a relay — whose `NwkLayer::can_route` is `true` — from
/// answering a child Rejoin Request or serving End Device Timeout.
///
/// Public and `#[doc(hidden)]`: it only needs to be nameable by the role trait
/// method's signature, not part of the supported surface.
#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub enum ParentNwkOutcome {
    /// A child asked to rejoin; answer with a Rejoin Response (and coupled
    /// Trust Center Update-Device).
    ChildRejoinRequest {
        src: ShortAddress,
        ieee: IeeeAddress,
        capability_info: u8,
        secured: bool,
    },
    /// A child requested an End Device Timeout; apply policy and transmit the
    /// 0x0C response.
    EndDeviceTimeoutRequest {
        src: ShortAddress,
        ieee: IeeeAddress,
        requested_timeout: u8,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrustCenterMode {
    Unknown,
    Centralized,
    Distributed,
}

/// Keepalive method selected from the negotiated `nwkParentInformation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KeepaliveMethod {
    /// The parent keeps the child alive on MAC Data Poll. Used when the
    /// parent advertised bit 0, and when it answered with no bits at all
    /// (pre-R22 parents only implement poll-based aging).
    MacDataPoll,
    /// The parent only advertised End Device Timeout Request keepalive, or
    /// never answered at all — a poll would not refresh the child timer, so a
    /// fresh 0x0B has to be sent.
    TimeoutRequest,
}

/// R22 End Device Timeout client lifecycle state.
///
/// All timers are in whole seconds driven by the `elapsed_secs` a tick
/// reports, using saturating subtraction. A negotiated timeout can be days
/// long, which does not fit the microsecond monotonic clock used elsewhere in
/// the runtime.
///
/// This value is the entire runtime payload of the **client** lifecycle, and
/// it now lives *only* inside an [`EndDevice`](crate::role::EndDevice)'s inline
/// [`EndDeviceState`](crate::role::EndDeviceState) role state — a
/// [`RelayRouter`](crate::role::RelayRouter) or [`Router`](crate::role::Router)
/// carries none of it (see [`crate::role`]). Reached through the
/// [`EndDeviceRole`](crate::role::EndDeviceRole) accessors so only an
/// end-device monomorphization can name it.
///
/// Exposed as `pub` (opaque — its fields are `pub(crate)`) only so it can name
/// the return type of the sealed [`EndDeviceRole`](crate::role::EndDeviceRole)
/// accessors, exactly as [`ParentState`](crate::role::ParentState) is; it is
/// hidden from the public docs and carries no externally usable surface.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndDeviceTimeoutState {
    /// Seconds until the next keepalive is due. `None` while no keepalive is
    /// scheduled (not joined, or not an end device).
    pub(crate) keepalive_remaining_secs: Option<u32>,
    /// Seconds still allowed for an outstanding End Device Timeout Response.
    /// `Some(0)` means the wait expired and has not been serviced yet.
    pub(crate) response_remaining_secs: Option<u16>,
    /// Remaining no-response retransmissions of the current request.
    pub(crate) retries_left: u8,
    /// Consecutive forced keepalive TX/poll failures.
    pub(crate) failures: u8,
    /// A forced MAC poll is scheduled for the next runtime tick.
    pub(crate) forced_poll: bool,
}

impl EndDeviceTimeoutState {
    pub(crate) const fn new() -> Self {
        Self {
            keepalive_remaining_secs: None,
            response_remaining_secs: None,
            retries_left: 0,
            failures: 0,
            forced_poll: false,
        }
    }

    /// Age both bounded timers by one tick.
    ///
    /// Saturating subtraction keeps `Some(0)` as the "expired but not yet
    /// serviced" state, which is what distinguishes a lapsed response wait
    /// from `None` (no request outstanding).
    fn advance(&mut self, elapsed_secs: u16) {
        let elapsed = elapsed_secs as u32;
        if let Some(remaining) = self.keepalive_remaining_secs.as_mut() {
            *remaining = remaining.saturating_sub(elapsed);
        }
        if let Some(remaining) = self.response_remaining_secs.as_mut() {
            *remaining = remaining.saturating_sub(elapsed_secs);
        }
    }
}

/// NIB fields that describe the End Device Timeout negotiation.
///
/// Compared around NWK receive processing to detect an accepted or refused End
/// Device Timeout Response without adding a public NWK command outcome.
///
/// Exposed as `pub` (opaque — its fields are private) only so it can name the
/// argument of the sealed, `#[doc(hidden)]`
/// [`DeviceRole::ed_apply_timeout_change`](crate::role::DeviceRole) hook; it is
/// hidden from the public docs and carries no externally usable surface.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndDeviceTimeoutSnapshot {
    parent_information: u8,
    parent_information_valid: bool,
    end_device_timeout: u8,
    requested_end_device_timeout: u8,
    accepts: u8,
}

/// The running Zigbee device — owns the full BDB→ZDO→APS→NWK→MAC stack.
///
/// The `R` type parameter is the compile-time logical role (see
/// [`crate::role`]): it
/// defaults to [`EndDevice`](crate::role::EndDevice) so existing
/// `ZigbeeDevice<M>` source keeps building an end device unchanged. A
/// [`Router`](crate::role::Router)-typed device can only be constructed from a
/// [`ParentMacDriver`](zigbee_mac::ParentMacDriver) backend and gains the
/// parent-only operational APIs.
///
/// The parent operational surface is bounded on
/// [`ParentRole`](crate::role::ParentRole), so a leaf end device cannot even
/// name it — the following does not compile:
///
/// ```compile_fail
/// use zigbee_runtime::ZigbeeDevice;
/// use zigbee_mac::mock::MockMac;
///
/// let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8])).build();
/// // `permit_joining` lives behind `ParentRole`; an `EndDevice` has no such
/// // method, so this is a compile error rather than a silent no-op.
/// let _ = device.permit_joining(0);
/// ```
///
/// The same holds for `announce_parent`, `service_parent_commands` and the
/// child-table persistence APIs:
///
/// ```compile_fail
/// use zigbee_runtime::ZigbeeDevice;
/// use zigbee_mac::mock::MockMac;
///
/// let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8])).build();
/// let _ = device.announce_parent();
/// ```
///
/// Symmetrically, the R22 End Device Timeout **client** API is bounded on
/// [`EndDeviceRole`](crate::role::EndDeviceRole), so a router-typed device
/// cannot name it — the following does not compile (rather than being a
/// success-shaped no-op on a router):
///
/// ```compile_fail
/// use zigbee_runtime::ZigbeeDevice;
/// use zigbee_runtime::role::Router;
/// use zigbee_mac::mock::MockMac;
///
/// let mut device: ZigbeeDevice<_, Router> =
///     ZigbeeDevice::builder(MockMac::new([0x11; 8])).build_router();
/// // `send_ed_timeout_request` is the leaf-only client obligation; a `Router`
/// // has no such method.
/// let _ = device.send_ed_timeout_request();
/// ```
pub struct ZigbeeDevice<M: MacDriver, R: crate::role::DeviceRole = crate::role::EndDevice> {
    /// BDB layer (transitively owns ZDO → APS → NWK → MAC).
    bdb: BdbLayer<M>,
    /// Application endpoint configurations.
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    /// ZCL attribute reporting engine.
    ///
    /// Holds *every* reporting configuration — the product's own defaults as
    /// well as anything a remote client configured. Interview completion is
    /// therefore tracked separately in `remote_reporting`.
    reporting: ReportingEngine,
    /// Clusters a remote ZCL client successfully configured reporting for.
    ///
    /// See [`remote_reporting`](crate::remote_reporting) — deliberately
    /// distinct from `reporting` so a locally installed default can never be
    /// mistaken for a completed coordinator interview.
    remote_reporting: remote_reporting::RemoteReportingState,
    /// Power management.
    power: PowerManager,
    /// Monotonic millisecond clock accumulated from `tick()` deltas.
    power_now_ms: u32,
    /// Whether `tick()` owns sleepy-end-device parent polling.
    automatic_polling: bool,
    /// Pending user action (set by button press, consumed by tick).
    pending_action: Option<UserAction>,
    /// ZCL transaction sequence counter.
    zcl_seq: u8,
    /// Standard clusters owned and configured by DeviceBuilder.
    basic_cluster: BasicCluster,
    identify_clusters: heapless::Vec<EndpointIdentifyCluster, MAX_ENDPOINTS>,
    /// Channel mask for network scanning.
    channel_mask: ChannelMask,
    /// Queued ZCL responses to send in next tick().
    #[cfg(feature = "router")]
    pending_responses: heapless::Vec<PendingZclResponse, 4>,
    #[cfg(not(feature = "router"))]
    pending_responses: heapless::Vec<PendingZclResponse, 2>,
    /// Per-instance receive/decrypt/serialize scratch storage.
    scratch: RuntimeScratch,
    /// Flag: network state has changed and should be persisted.
    state_dirty: bool,
    /// Earliest monotonic time for the next automatic secure-rejoin attempt.
    secure_rejoin_retry_at: Option<u32>,
    /// Per-role runtime state (see [`crate::role::RoleState`]).
    ///
    /// This is where every role-specific runtime field now lives, keeping each
    /// role's RAM off the others: a [`RelayRouter`](crate::role::RelayRouter)
    /// selects the zero-sized [`NonParentState`](crate::role::NonParentState),
    /// an [`EndDevice`](crate::role::EndDevice) selects
    /// [`EndDeviceState`](crate::role::EndDeviceState) (the R22 End Device
    /// Timeout *client* lifecycle), and a [`Router`](crate::role::Router)
    /// selects [`ParentState`](crate::role::ParentState) (the deferred
    /// child-update queue and Parent Announce flag). So a relay carries no role
    /// RAM at all, an end device carries only the client timeout state, and a
    /// router carries only the parent/server state. Role-only helpers reach the
    /// concrete state through [`ParentRole`](crate::role::ParentRole) /
    /// [`EndDeviceRole`](crate::role::EndDeviceRole).
    role_state: R::State,
    /// Zero-sized compile-time logical role marker.
    _role: core::marker::PhantomData<R>,
}

/// Role-agnostic constructor entry point.
///
/// Pinned to the default [`EndDevice`](crate::role::EndDevice) role so
/// `ZigbeeDevice::builder(mac)` resolves without a role annotation; the actual
/// role is chosen when the returned builder is finalized with
/// [`build`](crate::builder::DeviceBuilder::build) (end device) or
/// [`build_router`](crate::builder::DeviceBuilder::build_router) (router).
impl<M: MacDriver> ZigbeeDevice<M, crate::role::EndDevice> {
    /// Create a new device builder.
    pub fn builder(mac: M) -> builder::DeviceBuilder<M> {
        builder::DeviceBuilder::new(mac)
    }
}

/// Parent-only operational API, available only on a router/parent-role device.
///
/// These operations are meaningless on a leaf end device, so they live behind
/// the [`ParentRole`](crate::role::ParentRole) bound instead of being exposed
/// as success-shaped no-ops on every device. A router-role device can only be
/// constructed from a [`ParentMacDriver`](zigbee_mac::ParentMacDriver) backend
/// (see [`DeviceBuilder::build_router`](crate::builder::DeviceBuilder::build_router)),
/// so reaching this API already implies genuine parent capability.
impl<M: MacDriver, R: crate::role::ParentRole> ZigbeeDevice<M, R> {
    /// Open or close child joining through this coordinator/router.
    ///
    /// `0` closes immediately, `0xFF` is indefinite, and `1..=254` is a
    /// duration in seconds aged by [`tick`](Self::tick).
    pub async fn permit_joining(&mut self, duration: u8) -> Result<(), zigbee_nwk::NwkStatus> {
        self.bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .nlme_permit_joining(duration)
            .await
    }

    /// Broadcast a R22 Parent Announce for this router/coordinator's children.
    ///
    /// Explicit runtime hook. A router product calls it after
    /// [`restore_child_table`](Self::restore_child_table) (and after the
    /// network is up) so any former parent of a child that has since moved
    /// prunes its stale entry. Also fired automatically by the joined tick once
    /// after a child table is restored. Only available on a parent role, so a
    /// leaf device cannot emit it.
    pub async fn announce_parent(&mut self) -> Result<(), zigbee_zdo::ZdoError> {
        self.send_parent_annce_inner().await
    }

    /// Drain a bounded number of already-received MAC parent-management events
    /// (beacon requests, association requests, child data requests).
    ///
    /// Only available on a parent role. A structural no-op until the device is
    /// a joined, child-capable parent.
    pub async fn service_parent_commands(&mut self) -> ParentCommandStep {
        self.service_parent_commands_inner().await
    }

    /// Snapshot this router/coordinator's authenticated child table into a
    /// durable [`ChildTableStore`](crate::child_store::ChildTableStore).
    ///
    /// Only available on a parent role — a leaf device has no children to
    /// persist.
    pub fn save_child_table<S: child_store::ChildTableStore>(
        &self,
        store: &mut S,
    ) -> Result<(), child_store::ChildStoreError> {
        self.save_child_table_inner(store)
    }

    /// Restore the authenticated child table from durable persistence.
    ///
    /// Only available on a parent role.
    pub fn restore_child_table<S: child_store::ChildTableStore>(
        &mut self,
        store: &mut S,
    ) -> Result<usize, child_store::ChildStoreError> {
        self.restore_child_table_inner(store)
    }
}

/// Test-only accessors for the parent runtime state, so parent tests reach the
/// role-specific [`ParentState`](crate::role::ParentState) through the
/// [`ParentRole`](crate::role::ParentRole) accessors rather than a common
/// field (which no longer exists on a leaf/relay device).
#[cfg(all(test, feature = "router"))]
impl<M: MacDriver, R: crate::role::ParentRole> ZigbeeDevice<M, R> {
    /// Whether a R22 Parent Announce is currently marked due.
    fn parent_annce_due(&self) -> bool {
        R::parent_state(&self.role_state).parent_annce_due
    }

    /// Force the Parent Announce due flag (mirrors a post-restore state).
    fn set_parent_annce_due(&mut self, due: bool) {
        R::parent_state_mut(&mut self.role_state).parent_annce_due = due;
    }

    /// Number of queued deferred Trust Center Update-Device notifications.
    fn pending_child_update_count(&self) -> usize {
        R::parent_state(&self.role_state)
            .pending_child_updates
            .len()
    }
}

/// End-device-only access to the R22 End Device Timeout **client** lifecycle
/// state.
///
/// The client state lives in [`EndDeviceState`](crate::role::EndDeviceState),
/// reached through the [`EndDeviceRole`](crate::role::EndDeviceRole) accessors,
/// so a leaf `EndDevice` is the only role that can name it — a relay/router
/// carries neither the state nor these helpers.
impl<M: MacDriver, R: crate::role::EndDeviceRole> ZigbeeDevice<M, R> {
    /// Shared access to the client End Device Timeout state.
    #[inline]
    pub(crate) fn ed_timeout(&self) -> &EndDeviceTimeoutState {
        R::ed_timeout(&self.role_state)
    }

    /// Exclusive access to the client End Device Timeout state.
    #[inline]
    pub(crate) fn ed_timeout_mut(&mut self) -> &mut EndDeviceTimeoutState {
        R::ed_timeout_mut(&mut self.role_state)
    }

    /// Reset the client lifecycle to its fresh, unjoined state.
    ///
    /// Used by Leave / factory reset via the [`DeviceRole::ed_reset`] hook and
    /// at the start of a fresh negotiation / resume.
    #[inline]
    pub(crate) fn reset_end_device_timeout_state(&mut self) {
        *self.ed_timeout_mut() = EndDeviceTimeoutState::new();
    }
}

impl<M: MacDriver, R: crate::role::DeviceRole> ZigbeeDevice<M, R> {
    const SECURE_REJOIN_RETRY_DELAY_US: u32 = 5_000_000;
    #[cfg(feature = "router")]
    const PARENT_RX_SLICE_US: u32 = 20_000;
    const MAX_PARENT_COMMANDS_PER_STEP: u8 = 4;
    #[cfg(feature = "router")]
    const PENDING_CHILD_UPDATE_TIMEOUT_US: u32 = 10_000_000;
    /// Poll cadence hint while the event-driven commissioning security
    /// handshake is still running after network-up.
    ///
    /// The handshake advances one bounded step per tick and its shortest
    /// per-message response window is 1.5 s, so the application has to come
    /// back well inside that window for retransmissions and the overall
    /// 15 s deadline to be honoured.
    const COMMISSIONING_POLL_MS: u32 = 50;
    /// How long an End Device Timeout Response may take to arrive. The
    /// response is delivered indirectly, so this has to cover the parent's
    /// transaction persistence plus one forced poll.
    const ED_TIMEOUT_RESPONSE_WAIT_SECS: u16 = 5;
    /// Retransmissions of an End Device Timeout Request that drew no response.
    const ED_TIMEOUT_MAX_RETRIES: u8 = 2;
    /// Consecutive forced keepalive TX/poll failures tolerated before the
    /// existing secured-rejoin retry path is scheduled.
    const ED_TIMEOUT_MAX_FAILURES: u8 = 3;

    /// Allocate the next ZCL sequence number.
    fn next_zcl_seq(&mut self) -> u8 {
        let s = self.zcl_seq;
        self.zcl_seq = self.zcl_seq.wrapping_add(1);
        s
    }

    // ── Network lifecycle ───────────────────────────────────

    /// Initialize and join a Zigbee network via BDB commissioning.
    ///
    /// Performs BDB initialize and the pre-network part of Network Steering.
    /// Returns the assigned short address once the network is up. The
    /// post-network unique Trust Center link-key exchange is advanced by
    /// [`Self::tick`], which emits `CommissioningComplete` on its terminal
    /// transition.
    #[inline(never)]
    pub async fn start(&mut self) -> Result<u16, event_loop::StartError> {
        self.remote_reporting.clear();
        rt_trace!("[RT] start: init");
        let r = self.bdb.initialize();
        rt_trace!("[RT] bdb_init={}", if r.is_ok() { "ok" } else { "ERR" });
        if r.is_err() {
            return Err(event_loop::StartError::InitFailed);
        }
        rt_trace!("[RT] start: commission");
        let r = self.bdb.network_steering().await;
        rt_trace!("[RT] bdb_comm={}", if r.is_ok() { "ok" } else { "ERR" });
        if let Err(status) = r {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        rt_trace!("[RT] start: finish");
        self.finish_join().await
    }

    /// Initialize and join while durably reserving all security counters.
    ///
    /// The pre-network work (scan → join → initial Transport-Key → reserve
    /// network security → `Device_annce`) is awaited here. The post-network
    /// unique Trust Center link-key handshake is **not** awaited: on success
    /// the device is on the network and the caller must drive commissioning
    /// security to completion by calling [`Self::tick_with_security_store`]
    /// each tick/poll (GSDK network-steering / update-tc-link-key split).
    /// `CommissioningComplete { success: true }` is emitted from the tick loop
    /// only after Verify/Confirm (or a pre-R21 determination).
    #[inline(never)]
    pub async fn start_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<u16, event_loop::StartError> {
        self.remote_reporting.clear();
        let r = self.bdb.initialize();
        if r.is_err() {
            return Err(event_loop::StartError::InitFailed);
        }

        let mut persistence = CommissioningSecurityPersistence::new(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        let result = self
            .bdb
            .network_steering_with_persistence(&mut persistence)
            .await;
        if let Some(error) = persistence.take_error() {
            return Err(event_loop::StartError::PersistenceFailed(error));
        }
        if let Err(status) = result {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        self.finish_join().await
    }

    /// Resume a committed network when available, otherwise commission a new
    /// one while using the same durable security store.
    ///
    /// Like [`Self::start_with_security_store`], commissioning of a new network
    /// leaves the unique-TCLK handshake pending; drive it via
    /// [`Self::tick_with_security_store`].
    #[inline(never)]
    pub async fn start_or_resume_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<u16, event_loop::StartError> {
        self.remote_reporting.clear();
        if self.bdb.initialize().is_err() {
            return Err(event_loop::StartError::InitFailed);
        }

        if self
            .restore_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?
        {
            if self.secure_rejoin_pending() {
                self.configure_restored_network().await?;
                return self.secure_rejoin_with_security_store(store).await;
            }
            return self.rejoin().await;
        }

        let mut persistence = CommissioningSecurityPersistence::new(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        let result = self
            .bdb
            .network_steering_with_persistence(&mut persistence)
            .await;
        if let Some(error) = persistence.take_error() {
            return Err(event_loop::StartError::PersistenceFailed(error));
        }
        if let Err(status) = result {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        self.finish_join().await
    }

    #[inline(never)]
    async fn finish_join(&mut self) -> Result<u16, event_loop::StartError> {
        let addr = self.bdb.zdo().nwk().nib().network_address.0;
        let ieee = self.bdb.zdo().nwk().nib().ieee_address;
        log::info!("[Runtime] Joined network as 0x{:04X}", addr);

        self.bdb.zdo_mut().set_local_nwk_addr(ShortAddress(addr));
        self.bdb.zdo_mut().set_local_ieee_addr(ieee);

        self.state_dirty = !self.bdb.tclk_exchange_active();
        self.secure_rejoin_retry_at = None;
        // Single choke point for the R22 End Device Timeout negotiation: every
        // real join and secured rejoin passes through here, so the initial
        // request cannot be duplicated or forgotten by an individual entry
        // point. The silent persisted resume does *not* call this and uses
        // `resume_end_device_timeout` instead. Dispatched statically so only an
        // `EndDevice` runs (and links) the client negotiation.
        R::ed_begin_negotiation(self).await;
        Ok(addr)
    }

    /// Rejoin a previously-joined network using stored NWK credentials.
    ///
    /// Uses a silent resume approach: restores MAC-layer addresses (PAN ID,
    /// short address, channel) so the device can immediately start polling
    /// its parent and responding to frames without scanning, associating, or
    /// broadcasting Device_annce.
    ///
    /// This avoids the NWK Rejoin Request/Response exchange which some
    /// coordinators (e.g. ZHA/EZSP) handle unreliably. If the parent has
    /// disappeared, the caller should fall back to `start()` for a full
    /// BDB steering join.
    ///
    /// Call `restore_security_state()` first — it sets up NIB, security keys,
    /// and marks `node_is_on_a_network = true`.
    #[inline(never)]
    pub async fn rejoin(&mut self) -> Result<u16, event_loop::StartError> {
        self.remote_reporting.clear();
        log::info!("[Runtime] Resuming on previous network…");
        let addr = self.configure_restored_network().await?;

        // Mark as joined so NWK/ZDO accept frames
        self.bdb.zdo_mut().nwk_mut().set_joined(true);

        // A router must resume MAC router operation before announcing itself:
        // otherwise it advertises a routing device that never starts its
        // receiver or accepts children.
        self.restore_router_operation().await?;

        // Silent resume keeps the stored parent relationship, so the cheapest
        // keepalive the parent advertised is enough to refresh the child
        // timer — no fresh negotiation is needed. Dispatched statically so only
        // an `EndDevice` runs (and links) the client resume.
        R::ed_resume(self).await;

        self.state_dirty = true;
        self.secure_rejoin_retry_at = None;
        Ok(addr)
    }

    async fn configure_restored_network(&mut self) -> Result<u16, event_loop::StartError> {
        let nib = self.bdb.zdo().nwk().nib();
        let addr = nib.network_address.0;
        let channel = nib.logical_channel;
        let pan_id = nib.pan_id;
        let parent = nib.parent_address;

        log::info!(
            "[Runtime] Resume: addr=0x{:04X} PAN=0x{:04X} ch={} parent=0x{:04X}",
            addr,
            pan_id.0,
            channel,
            parent.0
        );

        let mac = self.bdb.zdo_mut().nwk_mut().mac_mut();
        mac.mlme_set(
            zigbee_mac::PibAttribute::PhyCurrentChannel,
            zigbee_mac::PibValue::U8(channel),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacPanId,
            zigbee_mac::PibValue::PanId(PanId(pan_id.0)),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacShortAddress,
            zigbee_mac::PibValue::ShortAddress(ShortAddress(addr)),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacCoordShortAddress,
            zigbee_mac::PibValue::ShortAddress(parent),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacAssociatedPanCoord,
            zigbee_mac::PibValue::Bool(true),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;

        let hw_ieee = match self
            .bdb
            .zdo_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_get(zigbee_mac::PibAttribute::MacExtendedAddress)
            .await
        {
            Ok(zigbee_mac::PibValue::ExtendedAddress(address)) => address,
            _ => return Err(event_loop::StartError::InitFailed),
        };
        let restored_ieee = self.bdb.zdo().nwk().nib().ieee_address;
        if restored_ieee != [0; 8] && restored_ieee != hw_ieee {
            return Err(event_loop::StartError::PersistenceFailed(
                SecurityStoreError::Corrupt,
            ));
        }
        self.bdb.zdo_mut().nwk_mut().nib_mut().ieee_address = hw_ieee;
        self.bdb.zdo_mut().set_local_nwk_addr(ShortAddress(addr));
        self.bdb.zdo_mut().set_local_ieee_addr(hw_ieee);
        log::info!("[Runtime] NIB IEEE set from MAC: {:02X?}", hw_ieee);
        Ok(addr)
    }

    /// Re-establish the parent relationship with a secured NWK rejoin.
    ///
    /// Unlike [`Self::rejoin`], this performs the over-the-air Rejoin
    /// Request/Response exchange. Use it when a parent sends a Leave command
    /// with the rejoin bit set or when silent resume can no longer poll.
    #[inline(never)]
    pub async fn secure_rejoin(&mut self) -> Result<u16, event_loop::StartError> {
        self.remote_reporting.clear();
        self.bdb.zdo_mut().nwk_mut().set_joined(false);
        let mut result = match self.bdb.rejoin_previous_network().await {
            Ok(()) => self.finish_join().await,
            Err(status) => Err(event_loop::StartError::CommissioningFailed(status)),
        };
        if result.is_ok()
            && let Err(error) = self.restore_router_operation().await
        {
            result = Err(error);
        }
        if result.is_ok() {
            self.secure_rejoin_retry_at = None;
        } else {
            self.schedule_secure_rejoin_retry();
        }
        result
    }

    /// Resume MAC router operation after a resume or secured rejoin.
    ///
    /// Fresh BDB steering already issues NLME-START-ROUTER; the persisted
    /// resume and rejoin paths skip commissioning entirely, so the router has
    /// to be restarted here. End devices and coordinators are unaffected.
    ///
    /// A router that cannot start is reported as a start failure instead of
    /// being left half-operational: the device would otherwise look like a
    /// router to the network while never enabling its receiver or accepting
    /// children. On failure the joined flag is cleared so the stack stops
    /// transmitting as if it were on the network; the commissioned
    /// credentials are kept so the caller can retry a resume or fall back to
    /// steering.
    ///
    /// `StartError` deliberately gains no new variant here — downstream
    /// applications match it exhaustively — so the NWK status is logged and
    /// mapped onto [`event_loop::StartError::InitFailed`].
    async fn restore_router_operation(&mut self) -> Result<(), event_loop::StartError> {
        if self.bdb.zdo().nwk().device_type() != zigbee_nwk::DeviceType::Router {
            return Ok(());
        }
        match self.bdb.zdo_mut().nwk_mut().nlme_start_router().await {
            Ok(()) => {
                log::info!("[Runtime] Router operation restored after resume");
                Ok(())
            }
            Err(status) => {
                rt_trace!("[RT] start_router=err {:?}", status);
                log::error!(
                    "[Runtime] NLME-START-ROUTER failed after resume: {:?}",
                    status
                );
                self.bdb.zdo_mut().nwk_mut().set_joined(false);
                Err(event_loop::StartError::InitFailed)
            }
        }
    }

    /// Perform a secured NWK rejoin while preserving crash-safe counters and
    /// persisting any new short address or parent selected by the network.
    #[inline(never)]
    pub async fn secure_rejoin_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<u16, event_loop::StartError> {
        self.refresh_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        self.persist_rejoin_pending(store, true)
            .map_err(event_loop::StartError::PersistenceFailed)?;

        let addr = match self.secure_rejoin().await {
            Ok(addr) => addr,
            Err(error) => {
                self.refresh_security_state(store)
                    .map_err(event_loop::StartError::PersistenceFailed)?;
                return Err(error);
            }
        };
        let mut state = store
            .load()
            .map_err(event_loop::StartError::PersistenceFailed)?
            .ok_or(event_loop::StartError::PersistenceFailed(
                SecurityStoreError::NotFound,
            ))?;
        state
            .validate()
            .map_err(event_loop::StartError::PersistenceFailed)?;

        let nib = self.bdb.zdo().nwk().nib();
        state.extended_pan_id = nib.extended_pan_id;
        state.pan_id = nib.pan_id.0;
        state.short_address = nib.network_address.0;
        state.channel = nib.logical_channel;
        state.depth = nib.depth;
        state.parent_address = nib.parent_address.0;
        // Round-trip the NIB's own notion of validity: a rejoin that started
        // from an unknown update state and has not yet learned one must not be
        // persisted as an authoritative `0`.
        match nib.nwk_update_id() {
            Some(update_id) => {
                state.update_id = update_id;
                state.update_id_valid = true;
            }
            None => {
                state.update_id = 0;
                state.update_id_valid = false;
            }
        }
        // A secured rejoin re-selects a parent, so the negotiation the NWK
        // layer just reset (and whatever the new parent has already answered)
        // is committed together with the new parent address.
        state.parent_information = nib.parent_information;
        state.parent_information_valid = nib.parent_information_valid;
        state.end_device_timeout = nib.end_device_timeout;
        state.rejoin_pending = false;
        store
            .store(&state)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        self.refresh_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;

        Ok(addr)
    }

    /// Leave the current Zigbee network.
    pub async fn leave(&mut self) -> Result<(), event_loop::StartError> {
        log::info!("[Runtime] Leaving network…");
        if self
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nlme_leave(false)
            .await
            .is_err()
        {
            log::warn!("[Runtime] Leave notification failed; clearing local state");
            let _ = self.bdb.zdo_mut().nlme_reset(false);
        }
        self.mark_left();
        log::info!("[Runtime] Left network");
        Ok(())
    }

    fn mark_left(&mut self) {
        self.bdb.attributes_mut().node_is_on_a_network = false;
        self.bdb.zdo_mut().nwk_mut().set_joined(false);
        let aps = self.bdb.zdo_mut().aps_mut();
        aps.binding_table_mut().clear();
        aps.group_table_mut().clear();
        aps.security_mut().clear_keys();
        self.reset_identify_clusters();
        // The interview belongs to the network that was just left; the next
        // coordinator re-runs it from scratch.
        self.remote_reporting.clear();
        self.secure_rejoin_retry_at = None;
        R::ed_reset(self);
        self.state_dirty = true;
    }

    /// Factory reset: leave network, clear all state, wipe NV.
    ///
    /// After this the device is in a "fresh out of box" state and
    /// must be commissioned again.
    pub async fn factory_reset(&mut self, nv: Option<&mut dyn NvStorage>) {
        log::info!("[Runtime] Factory reset…");

        // BDB factory_reset handles leave + state clearing
        let _ = self.bdb.factory_reset().await;

        // Clear NV storage if provided
        if let Some(nv) = nv {
            let items = [
                NvItemId::NwkPanId,
                NvItemId::NwkChannel,
                NvItemId::NwkShortAddress,
                NvItemId::NwkExtendedPanId,
                NvItemId::NwkIeeeAddress,
                NvItemId::NwkKey,
                NvItemId::NwkKeySeqNum,
                NvItemId::NwkFrameCounter,
                NvItemId::NwkDepth,
                NvItemId::NwkParentAddress,
                NvItemId::NwkUpdateId,
                NvItemId::BdbNodeIsOnNetwork,
                NvItemId::BdbCommissioningMode,
                NvItemId::BdbPrimaryChannelSet,
                NvItemId::BdbSecondaryChannelSet,
                NvItemId::BdbCommissioningGroupId,
                NvItemId::ApsBindingTable,
                NvItemId::ApsGroupTable,
            ];
            for id in &items {
                let _ = nv.delete(*id);
            }
        }

        self.basic_cluster.reset_to_factory_defaults();
        self.reset_identify_clusters();
        self.remote_reporting.clear();
        self.secure_rejoin_retry_at = None;
        R::ed_reset(self);
        log::info!("[Runtime] Factory reset complete");
    }

    // ── User action API ─────────────────────────────────────

    /// Queue a user action (e.g., from a button press).
    /// Will be processed on the next call to `tick()`.
    pub fn user_action(&mut self, action: UserAction) {
        self.pending_action = Some(action);
    }

    // ── Query state ─────────────────────────────────────────

    /// Whether the device is currently joined to a network.
    pub fn is_joined(&self) -> bool {
        self.bdb.is_on_network() && self.bdb.zdo().nwk().is_joined()
    }

    /// Whether a coordinator-requested secure rejoin still needs retrying.
    pub fn secure_rejoin_pending(&self) -> bool {
        self.secure_rejoin_retry_at.is_some()
    }

    pub(crate) fn secure_rejoin_retry_due(&self) -> bool {
        let Some(deadline) = self.secure_rejoin_retry_at else {
            return false;
        };
        let now = self.bdb.zdo().nwk().mac().monotonic_micros();
        now.wrapping_sub(deadline) < 0x8000_0000
    }

    fn schedule_secure_rejoin_retry(&mut self) {
        let now = self.bdb.zdo().nwk().mac().monotonic_micros();
        self.secure_rejoin_retry_at = Some(now.wrapping_add(Self::SECURE_REJOIN_RETRY_DELAY_US));
    }

    /// The device's NWK short address (0xFFFF if not joined).
    pub fn short_address(&self) -> u16 {
        self.bdb.zdo().nwk().nib().network_address.0
    }

    /// The current operating channel (0 if not joined).
    pub fn channel(&self) -> u8 {
        self.bdb.zdo().nwk().nib().logical_channel
    }

    /// The current PAN ID (0xFFFF if not joined).
    pub fn pan_id(&self) -> u16 {
        self.bdb.zdo().nwk().nib().pan_id.0
    }

    /// The device type (coordinator / router / end device).
    pub fn device_type(&self) -> zigbee_nwk::DeviceType {
        self.bdb.zdo().nwk().device_type()
    }

    /// The configured application endpoints.
    pub fn endpoints(&self) -> &[EndpointConfig] {
        &self.endpoints
    }

    /// The manufacturer name.
    pub fn manufacturer_name(&self) -> &str {
        self.basic_cluster.manufacturer_name()
    }

    /// The model identifier.
    pub fn model_identifier(&self) -> &str {
        self.basic_cluster.model_identifier()
    }

    /// The configured channel mask.
    pub fn channel_mask(&self) -> ChannelMask {
        self.channel_mask
    }

    pub fn steering_diagnostics(&self) -> zigbee_bdb::SteeringDiagnostics {
        self.bdb.steering_diagnostics()
    }

    pub fn nwk_rx_security_stats(&self) -> zigbee_nwk::NwkRxSecurityStats {
        self.bdb.zdo().aps().nwk().rx_security_stats()
    }

    pub fn aps_security_handshake_stats(&self) -> zigbee_aps::ApsSecurityHandshakeStats {
        self.bdb.zdo().aps().security_handshake_stats()
    }

    pub fn zdo_diagnostics(&self) -> zigbee_zdo::ZdoDiagnostics {
        self.bdb.zdo().diagnostics()
    }

    /// The software build identifier.
    pub fn sw_build_id(&self) -> &str {
        self.basic_cluster.sw_build_id()
    }

    /// The date code (Basic cluster attribute).
    pub fn date_code(&self) -> &str {
        self.basic_cluster.date_code()
    }

    /// Whether the Identify cluster is active on the given endpoint.
    pub fn is_identifying(&self, endpoint: u8) -> bool {
        self.identify_clusters
            .iter()
            .find(|entry| entry.endpoint == endpoint)
            .is_some_and(|entry| entry.cluster.is_identifying())
    }

    /// Consume a pending Identify trigger effect for an endpoint.
    pub fn take_identify_effect(&mut self, endpoint: u8) -> Option<(u8, u8)> {
        self.identify_clusters
            .iter_mut()
            .find(|entry| entry.endpoint == endpoint)
            .and_then(|entry| entry.cluster.take_pending_effect())
    }

    fn tick_identify_clusters(&mut self, elapsed_secs: u16) {
        for entry in &mut self.identify_clusters {
            entry.cluster.tick(elapsed_secs);
        }
    }

    fn reset_identify_clusters(&mut self) {
        for entry in &mut self.identify_clusters {
            entry.cluster = IdentifyCluster::new();
        }
    }

    fn endpoint_has_server_cluster(&self, endpoint: u8, cluster_id: ClusterId) -> bool {
        self.endpoints.iter().any(|configured| {
            configured.endpoint == endpoint && configured.server_clusters.contains(&cluster_id)
        })
    }

    // Immutable cluster lookup. The receive path's read-only ZCL dispatch now
    // lives in the `MacDriver`-independent `zcl_dispatch::LocalZclCtx`, so this
    // is only exercised by the cluster-routing unit tests below; the mutable
    // twin `with_cluster_mut` is still used by the Identify tick in
    // `event_loop`.
    #[cfg(test)]
    fn with_cluster<T>(
        &self,
        endpoint: u8,
        cluster_id: ClusterId,
        clusters: &[ClusterRef<'_>],
        access: impl FnOnce(&dyn Cluster) -> T,
    ) -> Option<T> {
        if !self.endpoint_has_server_cluster(endpoint, cluster_id) {
            return None;
        }
        match cluster_id {
            ClusterId::BASIC => Some(access(&self.basic_cluster)),
            ClusterId::IDENTIFY => self
                .identify_clusters
                .iter()
                .find(|entry| entry.endpoint == endpoint)
                .map(|entry| access(&entry.cluster)),
            _ => clusters
                .iter()
                .find(|cluster| {
                    cluster.endpoint == endpoint && cluster.cluster.cluster_id() == cluster_id
                })
                .map(|cluster| access(&*cluster.cluster)),
        }
    }

    fn with_cluster_mut<T>(
        &mut self,
        endpoint: u8,
        cluster_id: ClusterId,
        clusters: &mut [ClusterRef<'_>],
        access: impl FnOnce(&mut dyn Cluster) -> T,
    ) -> Option<T> {
        if !self.endpoint_has_server_cluster(endpoint, cluster_id) {
            return None;
        }
        match cluster_id {
            ClusterId::BASIC => Some(access(&mut self.basic_cluster)),
            ClusterId::IDENTIFY => self
                .identify_clusters
                .iter_mut()
                .find(|entry| entry.endpoint == endpoint)
                .map(|entry| access(&mut entry.cluster)),
            _ => clusters
                .iter_mut()
                .find(|cluster| {
                    cluster.endpoint == endpoint && cluster.cluster.cluster_id() == cluster_id
                })
                .map(|cluster| access(&mut *cluster.cluster)),
        }
    }

    /// Access the power manager (for sleep decisions).
    pub fn power(&self) -> &PowerManager {
        &self.power
    }

    /// Access the power manager mutably.
    pub fn power_mut(&mut self) -> &mut PowerManager {
        &mut self.power
    }

    /// Whether this device is configured as a sleepy end device.
    pub fn is_sleepy(&self) -> bool {
        !matches!(self.power.mode(), power::PowerMode::AlwaysOn)
    }

    /// Whether the network state has changed since last save.
    ///
    /// Check this after `tick()` returns — if true, call `save_state(nv)`
    /// and then `clear_state_dirty()` to persist the new state.
    pub fn state_dirty(&self) -> bool {
        self.state_dirty
    }

    /// Clear the dirty flag after saving state.
    pub fn clear_state_dirty(&mut self) {
        self.state_dirty = false;
    }

    // ── Reporting / Interview Detection ────────────────────

    /// Check whether this device has any reporting configuration for a
    /// specific cluster.
    ///
    /// This includes locally installed defaults, so it answers whether the
    /// reporting engine can produce reports for the cluster, not whether a
    /// remote client completed its interview. Use
    /// [`is_cluster_remotely_configured`](Self::is_cluster_remotely_configured)
    /// for remote Configure Reporting state.
    pub fn is_cluster_reporting_configured(&self, endpoint: u8, cluster_id: u16) -> bool {
        self.reporting.has_cluster_configured(endpoint, cluster_id)
    }

    /// Count how many distinct clusters have *any* reporting configuration on
    /// an endpoint.
    ///
    /// This includes the product's own defaults installed through
    /// [`ApplicationProfile::configure_default_reporting`](crate::profile::ApplicationProfile::configure_default_reporting),
    /// so it answers "will this device send reports?", **not** "has a remote
    /// client finished configuring us?". Use
    /// [`remote_reporting_cluster_count`](Self::remote_reporting_cluster_count)
    /// for interview completion.
    pub fn configured_cluster_count(&self, endpoint: u8) -> usize {
        self.reporting.configured_cluster_count(endpoint)
    }

    // ── Remote (client-configured) reporting ───────────────

    /// The clusters a remote ZCL client has successfully configured reporting
    /// for, tracked separately from local defaults.
    ///
    /// See [`crate::remote_reporting`] for the exact recording rule.
    pub const fn remote_reporting(&self) -> &remote_reporting::RemoteReportingState {
        &self.remote_reporting
    }

    /// Whether a remote client fully configured reporting for this cluster.
    ///
    /// `true` only after a non-empty, well-formed Configure Reporting command
    /// for `(endpoint, cluster_id)` made entirely of Send-direction records in
    /// which every record returned `Success`. A locally configured default
    /// never sets this.
    pub fn is_cluster_remotely_configured(&self, endpoint: u8, cluster_id: u16) -> bool {
        self.remote_reporting.contains(endpoint, cluster_id)
    }

    /// Number of distinct clusters a remote client configured on `endpoint`.
    ///
    /// This is generic diagnostic state and may include unrelated server
    /// clusters. Do not use a bare count comparison for profile interview
    /// completion; use [`remote_reporting_covers`](Self::remote_reporting_covers)
    /// with the profile's exact expected cluster IDs, or use
    /// [`ZigbeeNode::remote_reporting_is_complete`](crate::node::ZigbeeNode::remote_reporting_is_complete).
    pub fn remote_reporting_cluster_count(&self, endpoint: u8) -> usize {
        self.remote_reporting.cluster_count(endpoint)
    }

    /// Number of cluster IDs from `expected` that a remote client has fully
    /// configured for outbound reporting on `endpoint`.
    ///
    /// Unlike [`remote_reporting_cluster_count`](Self::remote_reporting_cluster_count),
    /// unrelated clusters retained by the generic state do not inflate this
    /// profile/application progress count.
    pub fn remote_reporting_coverage(&self, endpoint: u8, expected: &[u16]) -> usize {
        expected
            .iter()
            .filter(|&&cluster_id| self.remote_reporting.contains(endpoint, cluster_id))
            .count()
    }

    /// Whether a remote client has fully configured reporting for *every*
    /// cluster in `expected` on `endpoint`.
    ///
    /// Exact-membership interview completion: unlike a bare
    /// [`remote_reporting_cluster_count`](Self::remote_reporting_cluster_count)
    /// comparison, a coordinator that configured an unrelated cluster cannot
    /// substitute for a missing expected one. Applications built on the
    /// profile/node contract get this via
    /// [`ZigbeeNode::remote_reporting_is_complete`](crate::node::ZigbeeNode::remote_reporting_is_complete);
    /// this device-level entry point serves products that compose clusters
    /// directly and pass their own expected cluster-ID list.
    pub fn remote_reporting_covers(&self, endpoint: u8, expected: &[u16]) -> bool {
        self.remote_reporting_coverage(endpoint, expected) == expected.len()
    }

    /// Forget every remotely configured cluster.
    ///
    /// Network start, resume, secured rejoin, leave, and factory-reset paths
    /// clear this automatically. This explicit API is available to
    /// applications that begin an equivalent product-specific lifecycle
    /// outside those entry points.
    pub fn reset_remote_reporting(&mut self) {
        self.remote_reporting.clear();
    }

    // ── NV Persistence ─────────────────────────────────────

    /// Snapshot this router/coordinator's authenticated child table into a
    /// durable [`ChildTableStore`](crate::child_store::ChildTableStore).
    ///
    /// Persists every authenticated child (identity, short address,
    /// capability/configuration and accepted End Device Timeout enumeration)
    /// so a reboot can restore live child state before Parent Announce. An end
    /// device or non-routing build has no children and stores an empty table.
    ///
    /// This store is independent of the security journal: it never reads or
    /// writes NWK/APS frame counters, so persisting the child table can never
    /// disturb the crash-safe counter reservation.
    ///
    /// This is the private implementation. The public
    /// [`save_child_table`](Self::save_child_table) lives behind
    /// [`ParentRole`](crate::role::ParentRole) so a leaf device never exposes
    /// child-table persistence.
    fn save_child_table_inner<S: child_store::ChildTableStore>(
        &self,
        store: &mut S,
    ) -> Result<(), child_store::ChildStoreError> {
        use zigbee_nwk::neighbor::{NeighborDeviceType, Relationship};

        let mut table = child_store::PersistentChildTable::new();
        for entry in self.bdb.zdo().nwk().neighbor_table().iter() {
            if entry.relationship != Relationship::Child {
                continue;
            }
            let child = child_store::PersistentChild {
                ieee_address: entry.ieee_address,
                short_address: entry.network_address.0,
                rx_on_when_idle: entry.rx_on_when_idle,
                security_capable: entry.security_capable,
                is_router: entry.device_type == NeighborDeviceType::Router,
                end_device_timeout: entry.end_device_timeout,
            };
            if table.push(child).is_err() {
                // The neighbour table cannot hold more children than the
                // persisted table, so this is unreachable, but never silently
                // drop children — report the overflow.
                return Err(child_store::ChildStoreError::Full);
            }
        }
        store.store(&table)
    }

    /// Restore the authenticated child table from durable persistence.
    ///
    /// Must run **before** Parent Announce so the announced child list is
    /// authoritative. Re-installs each persisted child as an authenticated
    /// neighbour and re-arms an end-device child's End Device Timeout deadline
    /// to a fresh full window. Returns the number of children restored (`0` if
    /// nothing was persisted or this is not a routing device).
    ///
    /// A corrupt or unreadable store is surfaced as an error rather than
    /// silently treated as "no children", so a caller can distinguish "no
    /// child table yet" (`Ok(0)`) from a real persistence fault.
    ///
    /// This is the private implementation. The public
    /// [`restore_child_table`](Self::restore_child_table) lives behind
    /// [`ParentRole`](crate::role::ParentRole).
    fn restore_child_table_inner<S: child_store::ChildTableStore>(
        &mut self,
        store: &mut S,
    ) -> Result<usize, child_store::ChildStoreError>
    where
        R: crate::role::ParentRole,
    {
        let Some(table) = store.load()? else {
            return Ok(0);
        };
        table.validate()?;
        if !self.bdb.zdo().nwk().can_route() {
            return Ok(0);
        }
        let mut restored = 0;
        let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
        for child in table.children() {
            if nwk.restore_child(
                child.ieee_address,
                ShortAddress(child.short_address),
                child.rx_on_when_idle,
                child.security_capable,
                child.is_router,
                child.end_device_timeout,
            ) {
                restored += 1;
            }
        }
        // Restored child state is now authoritative, so a R22 Parent Announce
        // is due once the network is up. This is tied to the product's
        // explicit restore call rather than invented automatically.
        if restored > 0 {
            R::parent_state_mut(&mut self.role_state).parent_annce_due = true;
        }
        log::info!("[Runtime] Restored {restored} children from durable child table");
        Ok(restored)
    }

    /// Broadcast a R22 Parent Announce for this router/coordinator's children.
    ///
    /// Private implementation of the Parent Announce transmit. Bounded on
    /// [`ParentRole`](crate::role::ParentRole) since it clears the parent-only
    /// due flag; the event loop's due-announce servicing reaches it through the
    /// static role dispatch, so a leaf device never links it.
    async fn send_parent_annce_inner(&mut self) -> Result<(), zigbee_zdo::ZdoError>
    where
        R: crate::role::ParentRole,
    {
        R::parent_state_mut(&mut self.role_state).parent_annce_due = false;
        self.bdb.zdo_mut().send_parent_annce().await
    }

    /// Send a due Parent Announce once the restored child table is
    /// authoritative and the network is up. Dispatched from the joined tick
    /// only for a [`Router`](crate::role::Router) role (see
    /// [`DeviceRole::run_role_nwk_maintenance`](crate::role::DeviceRole::run_role_nwk_maintenance)),
    /// so a non-parent role never links the Parent Announce transmit code.
    #[cfg(feature = "router")]
    pub(crate) async fn service_due_parent_annce(&mut self)
    where
        R: crate::role::ParentRole,
    {
        if !R::parent_state(&self.role_state).parent_annce_due {
            return;
        }
        if !self.is_joined() || !self.bdb.zdo().nwk().can_route() {
            return;
        }
        if let Err(error) = self.send_parent_annce_inner().await {
            // Keep the flag set so a later tick retries once the path is up.
            R::parent_state_mut(&mut self.role_state).parent_annce_due = true;
            log::warn!("[Runtime] Parent_annce send failed: {:?}", error);
        }
    }

    /// Restore a fully commissioned network and reserve fresh counter ranges
    /// before any secured rejoin traffic can be sent.
    pub fn restore_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, SecurityStoreError> {
        let Some(mut state) = store.load()? else {
            return Ok(false);
        };
        state.validate()?;
        if !state.commissioned {
            return Ok(false);
        }
        let configured_ieee = self.bdb.zdo().nwk().nib().ieee_address;
        if configured_ieee != [0; 8] && configured_ieee != state.ieee_address {
            return Err(SecurityStoreError::Corrupt);
        }

        let global_current = state.global_counter_limit;
        let global_limit = global_current
            .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
            .ok_or(SecurityStoreError::CounterExhausted)?;
        let tclk_current = state.tclk_counter_limit;
        let tclk_limit = tclk_current
            .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
            .ok_or(SecurityStoreError::CounterExhausted)?;

        state.global_counter_limit = global_limit;
        state.tclk_counter_limit = tclk_limit;
        store.store(&state)?;

        {
            let nwk = self.bdb.zdo_mut().nwk_mut();
            nwk.security_mut()
                .set_network_key(state.network_key, state.key_sequence);
            if state.staged_network_key_present
                && !nwk
                    .security_mut()
                    .stage_network_key(state.staged_network_key, state.staged_key_sequence)
            {
                return Err(SecurityStoreError::Corrupt);
            }
            let nib = nwk.nib_mut();
            nib.extended_pan_id = state.extended_pan_id;
            nib.pan_id = PanId(state.pan_id);
            nib.network_address = ShortAddress(state.short_address);
            nib.ieee_address = state.ieee_address;
            nib.logical_channel = state.channel;
            nib.depth = state.depth;
            nib.parent_address = ShortAddress(state.parent_address);
            // Record version 4 stores `nwkUpdateId` validity explicitly, so a
            // record migrated from a format that never held the value (a
            // legacy log-structured NV region, say) restores as *unknown*
            // rather than as an authoritative `0`. Records written by
            // versions 1..=3 always decode as valid, so an existing
            // installation keeps the update state it had.
            nib.restore_nwk_update_id(if state.update_id_valid {
                Some(state.update_id)
            } else {
                None
            });
            nib.active_key_seq_number = state.key_sequence;
            nib.security_enabled = true;
            // The stored parent relationship is still in force after a silent
            // resume, so the negotiated keepalive method and timeout are
            // restored with it. A record that fails validation here has
            // already been rejected by `state.validate()`.
            if !nib.restore_end_device_timeout(
                state.parent_information,
                state.parent_information_valid,
                state.end_device_timeout,
            ) {
                return Err(SecurityStoreError::Corrupt);
            }
            if !nib.set_frame_counter_reservation(global_current, global_limit) {
                return Err(SecurityStoreError::Corrupt);
            }
        }

        if state.tclk_present {
            let aps = self.bdb.zdo_mut().aps_mut();
            aps.aib_mut().aps_trust_center_address = state.trust_center_address;
            aps.security_mut()
                .add_key(zigbee_aps::security::ApsLinkKeyEntry {
                    partner_address: state.trust_center_address,
                    key: state.trust_center_link_key,
                    key_type: zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter: tclk_current,
                    outgoing_frame_counter_limit: tclk_limit,
                    incoming_frame_counter: state.tclk_incoming_counter,
                    incoming_frame_counter_valid: state.tclk_incoming_counter_valid,
                })
                .map_err(|_| SecurityStoreError::Full)?;
        }
        // Otherwise this is a `legacy_default_tclk` network (see
        // `PersistentSecurityState`): no unique Trust Center link key was ever
        // persisted, so none is invented here. APS link-key traffic falls back
        // to the default global key, which draws its outgoing counter from the
        // NWK reservation installed above, and the Trust Center address stays
        // unset until the network transports a real key.

        self.bdb.attributes_mut().node_is_on_a_network = true;
        self.bdb.attributes_mut().primary_channel_set = ChannelMask(1u32 << state.channel);
        self.bdb.attributes_mut().secondary_channel_set = ChannelMask(0);
        self.state_dirty = false;
        let now = self.bdb.zdo().nwk().mac().monotonic_micros();
        self.secure_rejoin_retry_at = state.rejoin_pending.then_some(now);
        Ok(true)
    }

    /// Persist updated incoming counters and extend low outgoing reservations.
    ///
    /// Call before and after runtime operations that may send or accept secured
    /// frames. Storage is committed before in-memory limits are extended.
    pub fn refresh_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, SecurityStoreError> {
        const LOW_WATER: u32 = 32;

        /// The counter fields of the installed unique Trust Center link key.
        /// Copied out so the APS key table is not borrowed while the state is
        /// updated and committed.
        #[derive(Clone, Copy)]
        struct TclkCounters {
            outgoing: u32,
            limit: u32,
            incoming: u32,
            incoming_valid: bool,
        }

        let Some(mut state) = store.load()? else {
            return Ok(false);
        };
        state.validate()?;
        if !state.commissioned {
            return Ok(false);
        }

        let nwk = self.bdb.zdo().nwk();
        let nib = nwk.nib();
        let active_network_key = nwk
            .security()
            .active_key()
            .ok_or(SecurityStoreError::Corrupt)?;
        let staged_network_key = nwk
            .security()
            .staged_key()
            .map(|entry| (entry.key, entry.seq_number));
        if nib.ieee_address != state.ieee_address
            || nib.pan_id.0 != state.pan_id
            || nib.outgoing_frame_counter > nib.outgoing_frame_counter_limit
            || nib.outgoing_frame_counter_limit != state.global_counter_limit
            || nib.active_key_seq_number != active_network_key.seq_number
        {
            return Err(SecurityStoreError::Corrupt);
        }
        let network_key_changed = state.network_key != active_network_key.key
            || state.key_sequence != active_network_key.seq_number;
        if network_key_changed {
            state.network_key = active_network_key.key;
            state.key_sequence = active_network_key.seq_number;
        }
        let staged_key_changed = match staged_network_key {
            Some((key, sequence)) => {
                let changed = !state.staged_network_key_present
                    || state.staged_network_key != key
                    || state.staged_key_sequence != sequence;
                state.staged_network_key_present = true;
                state.staged_network_key = key;
                state.staged_key_sequence = sequence;
                changed
            }
            None => {
                let changed = state.staged_network_key_present;
                state.staged_network_key_present = false;
                state.staged_network_key = [0; 16];
                state.staged_key_sequence = 0;
                changed
            }
        };

        // A device restored with an unknown `nwkUpdateId` — a migrated record,
        // say — learns one from its first successful rejoin or from an
        // accepted `Mgmt_NWK_Update`. Adopt that durably so the unknown state
        // is not re-read on every boot. The converse never happens here: an
        // unknown NIB value leaves the stored record untouched, so this path
        // can never promote a placeholder into an authoritative `0`.
        let update_id_changed = match nib.nwk_update_id() {
            Some(update_id) => {
                let changed = !state.update_id_valid || state.update_id != update_id;
                state.update_id = update_id;
                state.update_id_valid = true;
                changed
            }
            None => false,
        };

        // R22 End Device Timeout negotiation result. Persisting it is what
        // lets a silent resume pick the cheap MAC-poll keepalive instead of
        // renegotiating on every reboot; the NIB is authoritative here because
        // both the NWK receive path and a parent change write it directly.
        let end_device_timeout_changed = state.parent_information != nib.parent_information
            || state.parent_information_valid != nib.parent_information_valid
            || state.end_device_timeout != nib.end_device_timeout;
        if end_device_timeout_changed {
            state.parent_information = nib.parent_information;
            state.parent_information_valid = nib.parent_information_valid;
            state.end_device_timeout = nib.end_device_timeout;
        }

        // Set when a unique Trust Center link key transported at runtime is
        // adopted into the durable store below; the live APS entry then has to
        // start from the reserved floor rather than its counter of zero.
        let mut adopted_current: Option<u32> = None;

        let tclk = if state.tclk_present {
            let tclk = self
                .bdb
                .zdo()
                .aps()
                .security()
                .find_key(
                    &state.trust_center_address,
                    zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                )
                .ok_or(SecurityStoreError::Corrupt)?;
            if tclk.key != state.trust_center_link_key {
                let current = tclk.outgoing_frame_counter.max(state.tclk_counter_limit);
                let limit = current
                    .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                    .ok_or(SecurityStoreError::CounterExhausted)?;
                state.trust_center_link_key = tclk.key;
                state.tclk_counter_limit = limit;
                state.tclk_incoming_counter = tclk.incoming_frame_counter;
                state.tclk_incoming_counter_valid = tclk.incoming_frame_counter_valid;
                adopted_current = Some(current);
                Some(TclkCounters {
                    outgoing: current,
                    limit,
                    incoming: tclk.incoming_frame_counter,
                    incoming_valid: tclk.incoming_frame_counter_valid,
                })
            } else {
                if tclk.outgoing_frame_counter > tclk.outgoing_frame_counter_limit
                    || tclk.outgoing_frame_counter_limit != state.tclk_counter_limit
                {
                    return Err(SecurityStoreError::Corrupt);
                }
                Some(TclkCounters {
                    outgoing: tclk.outgoing_frame_counter,
                    limit: tclk.outgoing_frame_counter_limit,
                    incoming: tclk.incoming_frame_counter,
                    incoming_valid: tclk.incoming_frame_counter_valid,
                })
            }
        } else {
            // A `legacy_default_tclk` network has no unique key to maintain:
            // the default global Trust Center link key draws from the NWK
            // counter space handled above, so there is no separate APS
            // reservation to extend. If the Trust Center transports a unique
            // key to such a node, though, the APS layer installs it with no
            // reservation and an outgoing counter of zero — a reboot would
            // replay it. Adopt it durably here instead, which also retires the
            // transitional representation.
            let adopted = self
                .bdb
                .zdo()
                .aps()
                .security()
                .key_table()
                .iter()
                .find(|entry| {
                    entry.key_type == zigbee_aps::security::ApsKeyType::TrustCenterLinkKey
                })
                .map(|entry| {
                    (
                        entry.partner_address,
                        entry.key,
                        entry.outgoing_frame_counter,
                        entry.incoming_frame_counter,
                        entry.incoming_frame_counter_valid,
                    )
                });
            match adopted {
                Some((partner, key, outgoing, incoming, incoming_valid)) => {
                    let current = outgoing.max(state.tclk_counter_limit);
                    let limit = current
                        .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                        .ok_or(SecurityStoreError::CounterExhausted)?;
                    state.tclk_present = true;
                    state.legacy_default_tclk = false;
                    state.trust_center_address = partner;
                    state.trust_center_link_key = key;
                    state.tclk_counter_limit = limit;
                    state.tclk_incoming_counter = incoming;
                    state.tclk_incoming_counter_valid = incoming_valid;
                    state.validate()?;
                    adopted_current = Some(current);
                    Some(TclkCounters {
                        outgoing: current,
                        limit,
                        incoming,
                        incoming_valid,
                    })
                }
                None => None,
            }
        };

        let mut changed = adopted_current.is_some()
            || network_key_changed
            || staged_key_changed
            || update_id_changed
            || end_device_timeout_changed;
        let mut new_global_limit = nib.outgoing_frame_counter_limit;
        if nib
            .outgoing_frame_counter_limit
            .saturating_sub(nib.outgoing_frame_counter)
            <= LOW_WATER
        {
            new_global_limit = nib
                .outgoing_frame_counter_limit
                .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                .ok_or(SecurityStoreError::CounterExhausted)?;
            state.global_counter_limit = new_global_limit;
            changed = true;
        }

        let mut new_tclk_limit = state.tclk_counter_limit;
        if let Some(tclk) = tclk {
            if tclk.limit.saturating_sub(tclk.outgoing) <= LOW_WATER {
                new_tclk_limit = tclk
                    .limit
                    .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                    .ok_or(SecurityStoreError::CounterExhausted)?;
                state.tclk_counter_limit = new_tclk_limit;
                changed = true;
            }

            if state.tclk_incoming_counter != tclk.incoming
                || state.tclk_incoming_counter_valid != tclk.incoming_valid
            {
                state.tclk_incoming_counter = tclk.incoming;
                state.tclk_incoming_counter_valid = tclk.incoming_valid;
                changed = true;
            }
        }

        if !changed {
            return Ok(false);
        }
        store.store(&state)?;

        self.bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .outgoing_frame_counter_limit = new_global_limit;
        if tclk.is_some() {
            let entry = self
                .bdb
                .zdo_mut()
                .aps_mut()
                .security_mut()
                .find_key_mut(
                    &state.trust_center_address,
                    zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                )
                .ok_or(SecurityStoreError::Corrupt)?;
            entry.outgoing_frame_counter_limit = new_tclk_limit;
            if let Some(current) = adopted_current {
                entry.outgoing_frame_counter = current;
            }
        }
        Ok(true)
    }

    /// Clear commissioned state while preserving outgoing counter bounds.
    pub fn factory_reset_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<(), SecurityStoreError> {
        let (global_counter_limit, tclk_counter_limit) = store
            .load()?
            .map(|state| (state.global_counter_limit, state.tclk_counter_limit))
            .unwrap_or((0, 0));
        let mut state = PersistentSecurityState::empty();
        state.global_counter_limit = global_counter_limit;
        state.tclk_counter_limit = tclk_counter_limit;
        store.store(&state)
    }

    /// Clear persisted network identity when firmware selects a different EUI.
    ///
    /// Outgoing counter reservations are preserved, so reflashing one board
    /// with a different device role cannot reuse a prior key/counter pair.
    pub fn reset_security_state_if_identity_changed<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
        configured_ieee: IeeeAddress,
    ) -> Result<bool, SecurityStoreError> {
        let Some(state) = store.load()? else {
            return Ok(false);
        };
        if state.ieee_address == [0; 8] || state.ieee_address == configured_ieee {
            return Ok(false);
        }
        self.factory_reset_security_state(store)?;
        Ok(true)
    }

    /// Factory-reset the stack while retaining outgoing counter bounds that
    /// prevent key/counter reuse on a later commissioning attempt.
    pub async fn factory_reset_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<(), event_loop::StartError> {
        self.factory_reset_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        self.bdb
            .factory_reset()
            .await
            .map_err(event_loop::StartError::CommissioningFailed)?;
        self.basic_cluster.reset_to_factory_defaults();
        self.reset_identify_clusters();
        self.remote_reporting.clear();
        self.state_dirty = false;
        self.secure_rejoin_retry_at = None;
        R::ed_reset(self);
        Ok(())
    }

    /// Process an incoming frame with crash-safe counter maintenance.
    pub async fn process_incoming_with_security_store<S: SecurityStateStore>(
        &mut self,
        indication: &McpsDataIndication,
        clusters: &mut [ClusterRef<'_>],
        store: &mut S,
    ) -> Result<Option<event_loop::StackEvent>, SecurityStoreError> {
        self.refresh_security_state(store)?;
        let event = self.process_incoming(indication, clusters).await;
        match &event {
            Some(event_loop::StackEvent::RejoinRequested) => {
                self.persist_rejoin_pending(store, true)?;
            }
            Some(event_loop::StackEvent::Left | event_loop::StackEvent::LeaveRequested) => {
                self.factory_reset_security_state(store)?;
            }
            _ => {}
        }
        self.refresh_security_state(store)?;
        Ok(event)
    }

    fn persist_rejoin_pending<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
        pending: bool,
    ) -> Result<(), SecurityStoreError> {
        let Some(mut state) = store.load()? else {
            return Err(SecurityStoreError::NotFound);
        };
        state.validate()?;
        if !state.commissioned {
            return Err(SecurityStoreError::Corrupt);
        }
        if state.rejoin_pending != pending {
            state.rejoin_pending = pending;
            store.store(&state)?;
        }
        Ok(())
    }

    /// Tick reporting and pending responses with crash-safe counter
    /// maintenance.
    pub async fn tick_with_security_store<S: SecurityStateStore>(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [ClusterRef<'_>],
        store: &mut S,
    ) -> Result<event_loop::TickResult, SecurityStoreError> {
        self.refresh_security_state(store)?;
        self.tick_identify_clusters(elapsed_secs);
        let security_reset_action = matches!(
            self.pending_action,
            Some(UserAction::Leave | UserAction::FactoryReset)
        ) || (matches!(self.pending_action, Some(UserAction::Toggle))
            && self.is_joined());
        let recovery_action = self.secure_rejoin_pending()
            && matches!(
                self.pending_action,
                Some(UserAction::Join | UserAction::Rejoin | UserAction::Toggle)
            );
        let result = if security_reset_action {
            self.pending_action = None;
            self.factory_reset_with_security_store(store)
                .await
                .map_err(|error| match error {
                    event_loop::StartError::PersistenceFailed(error) => error,
                    _ => SecurityStoreError::Hardware,
                })?;
            event_loop::TickResult::Event(event_loop::StackEvent::Left)
        } else if recovery_action {
            self.pending_action = None;
            self.retry_secure_rejoin_with_security_store(store).await?
        } else if self.pending_action.is_none() && self.secure_rejoin_retry_due() {
            self.retry_secure_rejoin_with_security_store(store).await?
        } else {
            if let Some(action) = self.pending_action.take() {
                self.handle_action(action).await
            } else {
                self.flush_pending_responses().await;
                if !self.is_joined() {
                    event_loop::TickResult::Idle
                } else {
                    // Keep the durable path direct too: another async wrapper
                    // adds several KiB of transient stack on Series-1 devices.
                    self.run_aps_maintenance().await;
                    self.run_nwk_maintenance(elapsed_secs).await;
                    R::ed_advance_timers(self, elapsed_secs);

                    self.reporting.tick(elapsed_secs);
                    self.apply_fb_target_request(clusters);
                    self.run_finding_binding_tick(elapsed_secs).await;
                    self.send_due_reports(clusters).await;
                    self.update_pending_tx_flag();

                    // Advance commissioning before polling/result generation.
                    // A terminal transition wins immediately; otherwise the
                    // poll still runs and any application event it produces is
                    // returned without being replaced by commissioning.
                    if self.bdb.tclk_exchange_active()
                        && let Some(event) = self
                            .advance_commissioning_with_security_store(store)
                            .await?
                    {
                        self.refresh_security_state(store)?;
                        return Ok(event_loop::TickResult::Event(event));
                    }

                    let now_ms = self.advance_power_clock(elapsed_secs);
                    let poll_event = self.run_sleepy_poll(now_ms, clusters).await;
                    R::ed_service(self).await;

                    if let Some(event) = poll_event {
                        event_loop::TickResult::Event(event)
                    } else if R::CAN_ROUTE {
                        self.tick_power_state(now_ms)
                    } else {
                        event_loop::sleep_decision_to_tick(self.power.decide(now_ms))
                    }
                }
            }
        };
        let result = self.commissioning_tick_hint(result);
        self.refresh_security_state(store)?;
        Ok(result)
    }

    /// Advance the event-driven unique Trust Center link-key exchange by one
    /// bounded step using the durable security store for persistence.
    ///
    /// Network security was reserved before `Device_annce`; here the unique
    /// TCLK/counter is reserved before Verify-Key and the commissioned network
    /// is committed only after a successful Confirm-Key. Returns a
    /// `CommissioningComplete` event on a terminal transition.
    async fn advance_commissioning_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<Option<event_loop::StackEvent>, SecurityStoreError> {
        if !self.bdb.tclk_exchange_active() {
            return Ok(None);
        }
        let progress = {
            let mut persistence = CommissioningSecurityPersistence::new(store)?;
            let progress = self.bdb.advance_tclk_exchange(Some(&mut persistence)).await;
            if let Some(error) = persistence.take_error() {
                return Err(error);
            }
            progress
        };
        match progress {
            zigbee_bdb::TclkProgress::InProgress => Ok(None),
            zigbee_bdb::TclkProgress::Complete => {
                self.state_dirty = true;
                log::info!("[Runtime] Commissioning security complete — network committed");
                Ok(Some(event_loop::StackEvent::CommissioningComplete {
                    success: true,
                }))
            }
            zigbee_bdb::TclkProgress::Failed(status) => {
                log::warn!("[Runtime] Commissioning security failed: {:?}", status);
                // BDB already reset NWK/MAC and cleared the on-network flag;
                // mirror that in the runtime so we stop servicing consistently.
                self.mark_left();
                Ok(Some(event_loop::StackEvent::CommissioningComplete {
                    success: false,
                }))
            }
        }
    }

    async fn retry_secure_rejoin_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<event_loop::TickResult, SecurityStoreError> {
        log::info!("[Runtime] Retrying secure rejoin with security store");
        match self.secure_rejoin_with_security_store(store).await {
            Ok(addr) => Ok(event_loop::TickResult::Event(
                event_loop::StackEvent::Joined {
                    short_address: addr,
                    channel: self.channel(),
                    pan_id: self.pan_id(),
                },
            )),
            Err(event_loop::StartError::PersistenceFailed(error)) => Err(error),
            Err(_) => Ok(event_loop::TickResult::Event(
                event_loop::StackEvent::CommissioningComplete { success: false },
            )),
        }
    }

    /// Save critical network state to non-volatile storage.
    ///
    /// Call after: join, key update, bind/unbind, group changes, or before sleep.
    ///
    /// This legacy item-by-item format is not crash-safe for Zigbee security
    /// counters or unique Trust Center link keys. New secured devices must use
    /// `SecurityStateStore` and `start_or_resume_with_security_store()`.
    pub fn save_state(&self, nv: &mut dyn NvStorage) {
        let nib = self.bdb.zdo().nwk().nib();

        // Network identity
        let _ = nv.write(NvItemId::NwkPanId, &nib.pan_id.0.to_le_bytes());
        let _ = nv.write(NvItemId::NwkChannel, &[nib.logical_channel]);
        let _ = nv.write(
            NvItemId::NwkShortAddress,
            &nib.network_address.0.to_le_bytes(),
        );
        let _ = nv.write(NvItemId::NwkExtendedPanId, &nib.extended_pan_id);
        let _ = nv.write(NvItemId::NwkIeeeAddress, &nib.ieee_address);
        let _ = nv.write(NvItemId::NwkDepth, &[nib.depth]);
        let _ = nv.write(
            NvItemId::NwkParentAddress,
            &nib.parent_address.0.to_le_bytes(),
        );
        // Only a known-good `nwkUpdateId` may be written. Persisting the
        // placeholder of an unknown state would let the next boot read back a
        // "known" 0 and start rejecting every beacon in 0x81..=0xFF as stale,
        // so the item is removed instead and the unknown state survives.
        match nib.nwk_update_id() {
            Some(update_id) => {
                let _ = nv.write(NvItemId::NwkUpdateId, &[update_id]);
            }
            None => {
                let _ = nv.delete(NvItemId::NwkUpdateId);
            }
        }

        // NWK security — active key + frame counter
        if let Some(key_entry) = self.bdb.zdo().nwk().security().active_key() {
            let _ = nv.write(NvItemId::NwkKey, &key_entry.key);
            let _ = nv.write(NvItemId::NwkKeySeqNum, &[key_entry.seq_number]);
        }
        let fc = nib.outgoing_frame_counter;
        let _ = nv.write(NvItemId::NwkFrameCounter, &fc.to_le_bytes());

        // BDB state
        let on_network: u8 = if self.bdb.is_on_network() { 1 } else { 0 };
        let _ = nv.write(NvItemId::BdbNodeIsOnNetwork, &[on_network]);
        let _ = nv.write(
            NvItemId::BdbCommissioningMode,
            &[self.bdb.attributes().commissioning_mode.0],
        );
        let _ = nv.write(
            NvItemId::BdbPrimaryChannelSet,
            &self.bdb.attributes().primary_channel_set.0.to_le_bytes(),
        );
        let _ = nv.write(
            NvItemId::BdbSecondaryChannelSet,
            &self.bdb.attributes().secondary_channel_set.0.to_le_bytes(),
        );
        let _ = nv.write(
            NvItemId::BdbCommissioningGroupId,
            &self.bdb.attributes().commissioning_group_id.to_le_bytes(),
        );

        log::debug!(
            "[NV] Saved network state (PAN=0x{:04X}, ch={}, addr=0x{:04X})",
            nib.pan_id.0,
            nib.logical_channel,
            nib.network_address.0
        );
    }

    /// Restore network state from non-volatile storage.
    ///
    /// Call on startup before `start()`. If state is found, the device can
    /// attempt rejoin instead of full commissioning.
    /// Returns `true` if valid state was restored.
    ///
    /// This legacy format is not suitable for production secured restore; use
    /// `restore_security_state()` through
    /// `start_or_resume_with_security_store()` instead.
    pub fn restore_state(&mut self, nv: &mut dyn NvStorage) -> bool {
        let mut buf = [0u8; 16];

        // Check if we have stored network state
        let on_network = match nv.read(NvItemId::BdbNodeIsOnNetwork, &mut buf) {
            Ok(1) => buf[0] != 0,
            _ => return false,
        };
        if !on_network {
            return false;
        }

        // Restore network identity
        let pan_id = match nv.read(NvItemId::NwkPanId, &mut buf) {
            Ok(2) => PanId(u16::from_le_bytes([buf[0], buf[1]])),
            _ => return false,
        };
        let channel = match nv.read(NvItemId::NwkChannel, &mut buf) {
            Ok(1) => buf[0],
            _ => return false,
        };
        let short_addr = match nv.read(NvItemId::NwkShortAddress, &mut buf) {
            Ok(2) => ShortAddress(u16::from_le_bytes([buf[0], buf[1]])),
            _ => return false,
        };
        let mut epid = [0u8; 8];
        if nv.read(NvItemId::NwkExtendedPanId, &mut epid).is_err() {
            return false;
        }
        let depth = match nv.read(NvItemId::NwkDepth, &mut buf) {
            Ok(1) => buf[0],
            _ => 1,
        };
        let parent = match nv.read(NvItemId::NwkParentAddress, &mut buf) {
            Ok(2) => ShortAddress(u16::from_le_bytes([buf[0], buf[1]])),
            _ => ShortAddress(0x0000),
        };
        // A record written before this item existed — or one saved while the
        // update state was unknown — says nothing about the network's update
        // state. Restore "unknown" rather than a known 0.
        let update_id = match nv.read(NvItemId::NwkUpdateId, &mut buf) {
            Ok(1) => Some(buf[0]),
            _ => {
                log::debug!("[NV] No NwkUpdateId item — nwkUpdateId restored as unknown");
                None
            }
        };

        // Apply to NIB
        {
            let nib = self.bdb.zdo_mut().nwk_mut().nib_mut();
            nib.pan_id = pan_id;
            nib.logical_channel = channel;
            nib.network_address = short_addr;
            nib.extended_pan_id = epid;
            nib.depth = depth;
            nib.parent_address = parent;
            nib.restore_nwk_update_id(update_id);
            // Restore IEEE address (critical for NWK security headers)
            let mut ieee_buf = [0u8; 8];
            if let Ok(8) = nv.read(NvItemId::NwkIeeeAddress, &mut ieee_buf) {
                nib.ieee_address = ieee_buf;
            }
        }

        // Restore NWK security key
        let mut key_buf = [0u8; 16];
        if let Ok(16) = nv.read(NvItemId::NwkKey, &mut key_buf) {
            let seq = match nv.read(NvItemId::NwkKeySeqNum, &mut buf) {
                Ok(1) => buf[0],
                _ => 0,
            };
            let fc = match nv.read(NvItemId::NwkFrameCounter, &mut buf) {
                Ok(4) => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
                _ => 0,
            };
            self.bdb
                .zdo_mut()
                .nwk_mut()
                .security_mut()
                .set_network_key(key_buf, seq);
            {
                let nib = self.bdb.zdo_mut().nwk_mut().nib_mut();
                nib.active_key_seq_number = seq;
                nib.security_enabled = true;
            }
            // Restore frame counter with safety margin: frames may have been
            // sent after the last NV save, so the coordinator's expected counter
            // is higher than what we saved. Add 1000 to avoid replay rejection.
            const FC_SAFETY_MARGIN: u32 = 1000;
            let fc_safe = fc.saturating_add(FC_SAFETY_MARGIN);
            log::info!(
                "[NV] Restored NWK key seq={}, fc={} (saved={} +{})",
                seq,
                fc_safe,
                fc,
                FC_SAFETY_MARGIN
            );
            self.bdb
                .zdo_mut()
                .nwk_mut()
                .nib_mut()
                .outgoing_frame_counter = fc_safe;
        }

        // Mark as on-network in BDB
        self.bdb.attributes_mut().node_is_on_a_network = true;

        // Restore BDB attributes
        if let Ok(1) = nv.read(NvItemId::BdbCommissioningMode, &mut buf) {
            self.bdb.attributes_mut().commissioning_mode = zigbee_bdb::CommissioningMode(buf[0]);
        }
        if let Ok(4) = nv.read(NvItemId::BdbPrimaryChannelSet, &mut buf) {
            self.bdb.attributes_mut().primary_channel_set =
                ChannelMask(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]));
        }
        if let Ok(4) = nv.read(NvItemId::BdbSecondaryChannelSet, &mut buf) {
            self.bdb.attributes_mut().secondary_channel_set =
                ChannelMask(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]));
        }
        if let Ok(2) = nv.read(NvItemId::BdbCommissioningGroupId, &mut buf) {
            self.bdb.attributes_mut().commissioning_group_id = u16::from_le_bytes([buf[0], buf[1]]);
        }

        log::info!(
            "[NV] Restored network state (PAN=0x{:04X}, ch={}, addr=0x{:04X})",
            pan_id.0,
            channel,
            short_addr.0
        );
        true
    }

    // ── MAC proxy ───────────────────────────────────────────

    fn parent_mode_active(&self) -> bool {
        // A device whose logical role is not a parent never serves children,
        // even if it is a routing FFD — this is the typed parent invariant.
        if !R::IS_PARENT {
            return false;
        }
        let nwk = self.bdb.zdo().aps().nwk();
        let capabilities = nwk.mac().capabilities();
        nwk.can_route()
            && match nwk.device_type() {
                zigbee_nwk::DeviceType::Coordinator => capabilities.coordinator,
                zigbee_nwk::DeviceType::Router => capabilities.router,
                zigbee_nwk::DeviceType::EndDevice => false,
            }
    }

    fn parent_beacon_response(&self) -> MlmeBeaconResponse {
        let nwk = self.bdb.zdo().aps().nwk();
        let nib = nwk.nib();
        let capacity = nwk.child_capacity();
        let mut nwk_info = u16::from(nib.stack_profile & 0x0F) | (2u16 << 4);
        if capacity.router {
            nwk_info |= 1 << 10;
        }
        nwk_info |= u16::from(nib.depth.min(15)) << 11;
        if capacity.end_device {
            nwk_info |= 1 << 15;
        }
        let mut bytes = [0u8; 15];
        bytes[0] = 0; // Zigbee protocol ID
        bytes[1..3].copy_from_slice(&nwk_info.to_le_bytes());
        bytes[3..11].copy_from_slice(&nib.extended_pan_id);
        bytes[11..14].fill(0xFF); // undefined TX offset in a non-beacon network
        bytes[14] = nib.update_id;

        let mut response =
            MlmeBeaconResponse::new(PibPayload::from_slice(&bytes).expect("fixed beacon payload"));
        response.pan_coordinator = nwk.device_type() == zigbee_nwk::DeviceType::Coordinator;
        response.association_permit =
            nib.permit_joining && (capacity.router || capacity.end_device);
        for child in nwk.indirect_queue().pending_children() {
            if !response.pending_short_addresses.contains(&child)
                && response.pending_short_addresses.push(child).is_err()
            {
                break;
            }
        }
        response
    }

    fn trust_center_mode(&self) -> TrustCenterMode {
        let address = self.bdb.zdo().aps().aib().aps_trust_center_address;
        if address == [0; 8] {
            TrustCenterMode::Unknown
        } else if address == [0xFF; 8] {
            TrustCenterMode::Distributed
        } else {
            TrustCenterMode::Centralized
        }
    }

    fn child_update_status(
        security_capable: bool,
        secured_rejoin: Option<bool>,
    ) -> zigbee_aps::apsme::ApsUpdateDeviceStatus {
        use zigbee_aps::apsme::ApsUpdateDeviceStatus;

        match (security_capable, secured_rejoin) {
            (false, None) => ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
            (true, None) => ApsUpdateDeviceStatus::HighSecurityDeviceUnsecuredJoin,
            (false, Some(true)) => ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
            (true, Some(true)) => ApsUpdateDeviceStatus::HighSecurityDeviceSecuredRejoin,
            (false, Some(false)) => ApsUpdateDeviceStatus::StandardDeviceUnsecuredRejoin,
            (true, Some(false)) => ApsUpdateDeviceStatus::HighSecurityDeviceUnsecuredRejoin,
        }
    }

    async fn notify_trust_center_of_child(
        &mut self,
        device_address: IeeeAddress,
        device_short_address: ShortAddress,
        status: zigbee_aps::apsme::ApsUpdateDeviceStatus,
    ) -> Result<(), zigbee_aps::ApsStatus> {
        match self.trust_center_mode() {
            TrustCenterMode::Distributed => Ok(()),
            TrustCenterMode::Unknown => Err(zigbee_aps::ApsStatus::InvalidParameter),
            TrustCenterMode::Centralized => {
                self.bdb
                    .zdo_mut()
                    .aps_mut()
                    .send_update_device(&device_address, device_short_address, status)
                    .await
            }
        }
    }

    fn prune_pending_child_updates(&mut self)
    where
        R: crate::role::ParentRole,
    {
        let now = self.bdb.zdo().nwk().mac().monotonic_micros();
        R::parent_state_mut(&mut self.role_state)
            .pending_child_updates
            .retain(|pending| now.wrapping_sub(pending.expires_at_us) >= 0x8000_0000);
    }

    /// Drop runtime-owned state coupled to a child the NWK layer just evicted.
    ///
    /// The NWK layer already cleaned the neighbour entry, indirect queue,
    /// routing entry, replay counters and MAC Frame Pending for `child`; a
    /// deferred Trust Center Update-Device is the runtime's own coupled state,
    /// so it is dropped here to keep the two consistent. A child that later
    /// re-associates re-runs the notification from scratch.
    ///
    /// Only a parent evicts children, so this is compiled only in `router`
    /// builds; a sensor image never links End Device Timeout child eviction.
    #[cfg(feature = "router")]
    fn forget_evicted_child(&mut self, child: ShortAddress)
    where
        R: crate::role::ParentRole,
    {
        R::parent_state_mut(&mut self.role_state)
            .pending_child_updates
            .retain(|pending| {
                pending.poll_address != child && pending.device_short_address != child
            });
    }

    #[cfg(feature = "router")]
    fn queue_pending_child_update(
        &mut self,
        poll_address: ShortAddress,
        device_address: IeeeAddress,
        device_short_address: ShortAddress,
        status: zigbee_aps::apsme::ApsUpdateDeviceStatus,
    ) -> Result<(), MacError>
    where
        R: crate::role::ParentRole,
    {
        self.prune_pending_child_updates();
        let expires_at_us = self
            .bdb
            .zdo()
            .nwk()
            .mac()
            .monotonic_micros()
            .wrapping_add(Self::PENDING_CHILD_UPDATE_TIMEOUT_US);
        let pending = PendingChildUpdate {
            poll_address,
            device_address,
            device_short_address,
            status,
            expires_at_us,
        };
        let updates = &mut R::parent_state_mut(&mut self.role_state).pending_child_updates;
        if let Some(existing) = updates.iter_mut().find(|existing| {
            existing.poll_address == poll_address || existing.device_address == device_address
        }) {
            *existing = pending;
            return Ok(());
        }
        updates
            .push(pending)
            .map_err(|_| MacError::TransactionOverflow)
    }

    async fn complete_pending_child_update(
        &mut self,
        poll_address: ShortAddress,
    ) -> Result<(), MacError>
    where
        R: crate::role::ParentRole,
    {
        self.prune_pending_child_updates();
        let Some(index) = R::parent_state(&self.role_state)
            .pending_child_updates
            .iter()
            .position(|pending| pending.poll_address == poll_address)
        else {
            return Ok(());
        };
        let pending = R::parent_state_mut(&mut self.role_state)
            .pending_child_updates
            .swap_remove(index);
        self.notify_trust_center_of_child(
            pending.device_address,
            pending.device_short_address,
            pending.status,
        )
        .await
        .map_err(|_| MacError::SecurityError)
    }

    async fn handle_parent_command(&mut self, event: MacCommandEvent) -> Result<(), MacError>
    where
        R: crate::role::ParentRole,
    {
        match event {
            MacCommandEvent::BeaconRequest(_) => {
                let response = self.parent_beacon_response();
                self.bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .mac_mut()
                    .mlme_beacon_response(response)
                    .await
            }
            MacCommandEvent::AssociationRequest(indication) => {
                let (was_existing, association) = {
                    let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
                    let was_existing = nwk
                        .known_child_by_ieee(&indication.device_address)
                        .is_some();
                    let association = nwk.handle_child_association(
                        indication.device_address,
                        indication.capability_info.to_byte(),
                    );
                    (was_existing, association)
                };
                let (short_address, status) = match association {
                    Ok(address) => (address, AssociationStatus::Success),
                    Err(zigbee_nwk::NwkStatus::NeighborTableFull) => {
                        (ShortAddress(0xFFFF), AssociationStatus::PanAtCapacity)
                    }
                    Err(_) => (ShortAddress(0xFFFF), AssociationStatus::PanAccessDenied),
                };
                let response = MlmeAssociateResponse {
                    device_address: indication.device_address,
                    short_address,
                    status,
                };
                let result = self
                    .bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .mac_mut()
                    .mlme_associate_response(response)
                    .await;
                if result.is_err() && status == AssociationStatus::Success && !was_existing {
                    self.bdb
                        .zdo_mut()
                        .aps_mut()
                        .nwk_mut()
                        .remove_neighbor(short_address);
                }
                result
            }
            MacCommandEvent::AssociationResponseDelivery(delivery) => {
                if delivery.status != AssociationStatus::Success {
                    return Ok(());
                }
                if let Err(error) = delivery.result {
                    self.bdb
                        .zdo_mut()
                        .aps_mut()
                        .nwk_mut()
                        .remove_unauthenticated_child(
                            &delivery.device_address,
                            delivery.short_address,
                        );
                    return Err(error);
                }

                let security_capable = self
                    .bdb
                    .zdo()
                    .nwk()
                    .child_security_capable(&delivery.device_address)
                    .ok_or(MacError::InvalidParameter)?;
                let status = Self::child_update_status(security_capable, None);
                self.notify_trust_center_of_child(
                    delivery.device_address,
                    delivery.short_address,
                    status,
                )
                .await
                .map_err(|_| MacError::SecurityError)
            }
            MacCommandEvent::DataRequest(indication) => {
                let outcome = self
                    .bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .service_child_data_request(indication.source_address)
                    .await?;
                if let zigbee_nwk::ChildPollOutcome::Delivered { child, .. } = outcome {
                    self.complete_pending_child_update(child).await?;
                }
                Ok(())
            }
        }
    }

    /// Drain a bounded number of already-received MAC management events.
    ///
    /// The zero-timeout poll never starts a long radio window. Limiting each
    /// call to four events prevents beacon/association traffic from starving
    /// normal MCPS data.
    ///
    /// Private implementation. The public
    /// [`service_parent_commands`](Self::service_parent_commands) is bounded on
    /// [`ParentRole`](crate::role::ParentRole); this inert helper is likewise
    /// parent-only and is dispatched into a router monomorphization by the
    /// static role hooks, so a non-parent role never links it. It is also a
    /// structural no-op whenever [`parent_mode_active`](Self::parent_mode_active)
    /// is false.
    pub(crate) async fn service_parent_commands_inner(&mut self) -> ParentCommandStep
    where
        R: crate::role::ParentRole,
    {
        let mut outcome = ParentCommandStep::default();
        if !self.parent_mode_active() {
            return outcome;
        }
        self.prune_pending_child_updates();
        for _ in 0..Self::MAX_PARENT_COMMANDS_PER_STEP {
            let event = self
                .bdb
                .zdo_mut()
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .mac_command_event_timeout(0)
                .await;
            let event = match event {
                Ok(event) => event,
                Err(MacError::NoData | MacError::Unsupported) => break,
                Err(_) => {
                    outcome.failures = outcome.failures.saturating_add(1);
                    break;
                }
            };
            outcome.processed = outcome.processed.saturating_add(1);
            if self.handle_parent_command(event).await.is_err() {
                outcome.failures = outcome.failures.saturating_add(1);
            }
        }
        outcome
    }

    /// Wait for an incoming MAC frame. Blocks until a frame arrives.
    ///
    /// Use with `select!` and a timer for non-blocking operation:
    /// ```rust,ignore
    /// select! {
    ///     frame = device.receive() => { device.process_incoming(&frame.unwrap()); }
    ///     _ = Timer::after(Duration::from_secs(1)) => { device.tick(1).await; }
    /// }
    /// ```
    pub async fn receive(&mut self) -> Result<McpsDataIndication, MacError> {
        // Parent RX slicing (interleaving MAC parent-command servicing with the
        // receive window) exists only in router builds. A non-routing device
        // waits on the MAC directly and never links the parent-command path.
        #[cfg(feature = "router")]
        if self.parent_mode_active() {
            return self.receive_timeout(Self::PARENT_RX_SLICE_US).await;
        }
        self.bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mcps_data_indication()
            .await
    }

    /// Wait for an incoming MAC frame for at most `timeout_us`.
    ///
    /// Parent-mode command servicing runs before and after the bounded RX
    /// window, matching [`Self::receive`] while allowing an application to
    /// shorten the slice when [`event_loop::TickResult::RunAgain`] requests an
    /// earlier runtime deadline. The parent servicing is dispatched statically
    /// through the role, so only a router monomorphization links (and runs) it.
    pub async fn receive_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        // Parent-command servicing is dispatched statically through the role:
        // only a `Router` monomorphization materializes the servicing future,
        // and it self-gates on `parent_mode_active`. A relay/end device is a
        // compile-time no-op here, so no parent future is instantiated.
        R::run_role_parent_servicing(self).await;
        let result = self
            .bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mcps_data_indication_timeout(timeout_us)
            .await;
        R::run_role_parent_servicing(self).await;
        result
    }

    /// Poll the parent for pending data (Sleepy End Device).
    ///
    /// Sends a MAC Data Request to the coordinator/parent and returns
    /// any queued frame. Returns `None` if no data is pending.
    /// After calling this, feed the result into `process_incoming()`.
    ///
    /// A completed poll also refreshes the R22 End Device Timeout keepalive
    /// deadline — unless the parent advertised *only* End Device Timeout
    /// Request keepalive, in which case a poll does not reset its child timer
    /// and must not postpone the next request either.
    pub async fn poll(&mut self) -> Result<Option<McpsDataIndication>, MacError> {
        let frame = match self
            .bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_poll()
            .await
        {
            Ok(frame) => frame,
            Err(error) => {
                // The MAC now distinguishes an ACKed-but-empty poll (`Ok(None)`)
                // from a poll that exhausted retries with no MAC ACK (this
                // `Err`, e.g. `NoAck`): the parent is silent. Feed the bounded
                // consecutive-failure counter — this is the single choke point
                // every poll passes through (forced keepalive, automatic sleepy,
                // and application-driven OTA fast polls via the public API), so
                // a silently-stalled parent drives the existing storm-guarded
                // secure-rejoin recovery regardless of what issued the poll. The
                // hook is inert on non-end-device roles.
                log::debug!("[Runtime] Poll to parent failed: {:?}", error);
                R::ed_note_forced_poll_result(self, false);
                return Err(error);
            }
        };
        self.power.record_poll(self.power_now_ms);
        R::ed_note_poll(self);
        // An acknowledged poll (empty or not) proves the parent is reachable and
        // clears the consecutive-failure counter.
        R::ed_note_forced_poll_result(self, true);
        match frame {
            Some(frame) => {
                self.power.record_activity(self.power_now_ms);
                // Wrap the raw poll response in a McpsDataIndication.
                // The parent address comes from NIB; LQI is unknown from poll.
                let parent = self.bdb.zdo().nwk().nib().parent_address;
                let pan_id = self.bdb.zdo().nwk().nib().pan_id;
                let our_addr = self.bdb.zdo().nwk().nib().network_address;
                Ok(Some(McpsDataIndication {
                    src_address: zigbee_types::MacAddress::Short(pan_id, parent),
                    dst_address: zigbee_types::MacAddress::Short(pan_id, our_addr),
                    lqi: 0, // not available from poll
                    payload: frame,
                    security_use: false,
                }))
            }
            None => Ok(None),
        }
    }

    // ── Incoming frame processing ───────────────────────────

    /// Answer a child's Rejoin Request and notify the Trust Center.
    ///
    /// Only a parent accepts children, so this is bounded on
    /// [`ParentRole`](crate::role::ParentRole) and compiled only in `router`
    /// builds. It is reached through the static role dispatch of
    /// [`ParentNwkOutcome::ChildRejoinRequest`], so a sensor/relay build
    /// neither observes it nor links this rejoin / Update-Device path.
    #[cfg(feature = "router")]
    async fn handle_child_rejoin_request(
        &mut self,
        request_address: ShortAddress,
        device_address: IeeeAddress,
        capability_info: u8,
        secured: bool,
    ) where
        R: crate::role::ParentRole,
    {
        let capability = CapabilityInfo::from_byte(capability_info);
        let trust_center_mode = self.trust_center_mode();
        let was_existing = self
            .bdb
            .zdo()
            .nwk()
            .known_child_by_ieee(&device_address)
            .is_some();
        let admission = if trust_center_mode == TrustCenterMode::Unknown
            || (!secured && trust_center_mode == TrustCenterMode::Distributed)
        {
            Err(zigbee_nwk::NwkStatus::NotPermitted)
        } else {
            self.bdb.zdo_mut().aps_mut().nwk_mut().handle_child_rejoin(
                request_address,
                device_address,
                capability_info,
                secured,
            )
        };
        let (assigned_address, rejoin_status) = match admission {
            Ok(address) => (address, 0x00),
            Err(zigbee_nwk::NwkStatus::NeighborTableFull) => (ShortAddress(0xFFFF), 0x01),
            Err(_) => (ShortAddress(0xFFFF), 0x02),
        };

        let delivery = self
            .bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .send_rejoin_response(
                request_address,
                device_address,
                assigned_address,
                rejoin_status,
                secured,
                capability.rx_on_when_idle,
            )
            .await;
        let delivery = match delivery {
            Ok(delivery) => delivery,
            Err(error) => {
                log::warn!(
                    "[Runtime] Rejoin Response to 0x{:04X} failed: {:?}",
                    request_address.0,
                    error
                );
                if rejoin_status == 0x00 && !was_existing {
                    self.bdb
                        .zdo_mut()
                        .aps_mut()
                        .nwk_mut()
                        .remove_neighbor(assigned_address);
                }
                return;
            }
        };
        if rejoin_status != 0x00 || trust_center_mode != TrustCenterMode::Centralized {
            return;
        }

        let update_status = Self::child_update_status(capability.security_capable, Some(secured));
        match delivery {
            zigbee_nwk::RejoinResponseDelivery::Direct => {
                if let Err(error) = self
                    .notify_trust_center_of_child(device_address, assigned_address, update_status)
                    .await
                {
                    log::warn!("[Runtime] Rejoin Update-Device failed: {:?}", error);
                }
            }
            zigbee_nwk::RejoinResponseDelivery::Indirect => {
                if let Err(error) = self.queue_pending_child_update(
                    request_address,
                    device_address,
                    assigned_address,
                    update_status,
                ) {
                    log::warn!("[Runtime] Cannot defer Rejoin Update-Device: {:?}", error);
                    if !was_existing {
                        self.bdb
                            .zdo_mut()
                            .aps_mut()
                            .nwk_mut()
                            .remove_neighbor(assigned_address);
                    }
                }
            }
        }
    }

    /// Apply the runtime policy for a NWK command reported by the NWK layer.
    ///
    /// The NWK layer owns validation (security, addressing and parent
    /// authorization) and has already dropped its joined state. The runtime
    /// owns rejoin scheduling and the application-visible event.
    async fn handle_nwk_command_outcome(
        &mut self,
        command: zigbee_nwk::nlde::NwkCommandOutcome,
    ) -> Option<event_loop::StackEvent> {
        match command {
            zigbee_nwk::nlde::NwkCommandOutcome::ParentLeft { src } => {
                rt_trace!("[RT] parent_left src=0x{:04X}", src.0);
                log::warn!("[RX] Parent 0x{:04X} left the network", src.0);
                // The interview belongs to the network we just lost the parent
                // for; drop it now rather than waiting for the eventual
                // rejoin/leave lifecycle action, so a stale record can never be
                // read as "interview complete" during the rejoin window.
                self.remote_reporting.clear();
                let now = self.bdb.zdo().nwk().mac().monotonic_micros();
                self.secure_rejoin_retry_at = Some(now);
                Some(event_loop::StackEvent::RejoinRequested)
            }
            zigbee_nwk::nlde::NwkCommandOutcome::LeaveRequested {
                src,
                rejoin,
                remove_children,
            } => {
                rt_trace!(
                    "[RT] leave src=0x{:04X} remove_children={} rejoin={}",
                    src.0,
                    remove_children,
                    rejoin
                );
                log::warn!(
                    "[RX] NWK Leave request from 0x{:04X} (remove_children={}, rejoin={})",
                    src.0,
                    remove_children,
                    rejoin
                );
                // Whether or not the coordinator asked us to rejoin, the
                // current network's interview is over; clear the remote
                // reporting record immediately on this accepted inbound leave
                // rather than deferring to a later `mark_left`/factory-reset.
                self.remote_reporting.clear();
                if rejoin {
                    let now = self.bdb.zdo().nwk().mac().monotonic_micros();
                    self.secure_rejoin_retry_at = Some(now);
                    Some(event_loop::StackEvent::RejoinRequested)
                } else {
                    self.secure_rejoin_retry_at = None;
                    Some(event_loop::StackEvent::LeaveRequested)
                }
            }
            zigbee_nwk::nlde::NwkCommandOutcome::ChildRejoinRequest {
                src,
                ieee,
                capability_info,
                secured,
            } => {
                // Only a parent answers a child's Rejoin Request. Dispatched
                // statically through the role type so a relay (whose
                // `NwkLayer::can_route` is true) or an end device never answers
                // it and never materializes the rejoin / Update-Device future.
                R::service_parent_nwk_outcome(
                    self,
                    ParentNwkOutcome::ChildRejoinRequest {
                        src,
                        ieee,
                        capability_info,
                        secured,
                    },
                )
                .await;
                None
            }
            zigbee_nwk::nlde::NwkCommandOutcome::EndDeviceTimeoutRequest {
                src,
                ieee,
                requested_timeout,
            } => {
                // The NWK layer validated the request came from an
                // authenticated attached child; a parent applies the policy and
                // transmits the 0x0C response (indirectly for a sleepy child).
                // Dispatched statically through the role type so a relay/end
                // device never answers it and never links the End Device
                // Timeout *server*.
                R::service_parent_nwk_outcome(
                    self,
                    ParentNwkOutcome::EndDeviceTimeoutRequest {
                        src,
                        ieee,
                        requested_timeout,
                    },
                )
                .await;
                None
            }
        }
    }

    /// Perform a router's response to a parent-only NWK command outcome.
    ///
    /// Bounded on [`ParentRole`](crate::role::ParentRole) and reached only from
    /// [`Router`](crate::role::Router)'s
    /// [`service_parent_nwk_outcome`](crate::role::DeviceRole::service_parent_nwk_outcome)
    /// hook, so a relay/end device build neither observes nor links the child
    /// rejoin / Update-Device / End Device Timeout server subgraph.
    #[cfg(feature = "router")]
    pub(crate) async fn dispatch_parent_nwk_outcome(&mut self, outcome: ParentNwkOutcome)
    where
        R: crate::role::ParentRole,
    {
        match outcome {
            ParentNwkOutcome::ChildRejoinRequest {
                src,
                ieee,
                capability_info,
                secured,
            } => {
                self.handle_child_rejoin_request(src, ieee, capability_info, secured)
                    .await;
            }
            ParentNwkOutcome::EndDeviceTimeoutRequest {
                src,
                ieee,
                requested_timeout,
            } => {
                if let Err(error) = self
                    .bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .respond_to_end_device_timeout_request(src, ieee, requested_timeout)
                    .await
                {
                    log::warn!(
                        "[Runtime] ED Timeout Response to 0x{:04X} failed: {:?}",
                        src.0,
                        error
                    );
                }
            }
        }
    }

    /// Process an incoming MAC frame through the full stack.
    ///
    /// MAC → NWK → APS → ZDO (endpoint 0) or ZCL (app endpoints).
    /// Async because ZDO handling sends responses directly through the stack.
    ///
    /// Pass registered cluster instances so the runtime can automatically:
    /// - Handle Read/Write/Discover Attributes using cluster attribute stores
    /// - Dispatch cluster-specific commands to `Cluster::handle_command()`
    /// - Sync Groups cluster actions to the APS group table
    #[inline(never)]
    pub async fn process_incoming(
        &mut self,
        indication: &McpsDataIndication,
        clusters: &mut [ClusterRef<'_>],
    ) -> Option<event_loop::StackEvent> {
        let mac_payload = indication.payload.as_slice();

        // NWK layer: header parsing, broadcast eligibility, BTR/relay for
        // routing devices, replay checks, decryption and NWK command
        // dispatch all live in the shared NWK implementation. The runtime
        // only routes the outcome onwards — it must not duplicate the
        // security-counter check/commit sequence.
        // The MAC source address is the neighbour that actually transmitted
        // this frame. NWK routing state is hop by hop — a propagated Route
        // Request still carries the *originator* in its NWK header — so the
        // previous hop has to travel with the frame into the NWK layer or the
        // next hops installed from it would name a device several hops away.
        let previous_hop = match indication.src_address {
            zigbee_types::MacAddress::Short(_, addr) if addr.0 < 0xFFF8 => Some(addr),
            // Extended or broadcast MAC source: unknown short address, so the
            // NWK layer falls back to the NWK source address.
            _ => None,
        };

        let negotiates_timeout = self.negotiates_end_device_timeout();
        let timeout_before = negotiates_timeout.then(|| self.end_device_timeout_snapshot());

        let (nwk_indication, command_outcome) = {
            let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
            let nwk_indication = nwk
                .process_incoming_nwk_frame_from(mac_payload, indication.lqi, previous_hop)
                .await;
            // Collected in the same borrow as the call that produced it: the
            // NWK layer clears this slot per frame, so nothing stale can leak
            // into a later frame.
            (nwk_indication, nwk.take_command_outcome())
        };

        // An End Device Timeout Response (0x0C) changes NIB state only and
        // deliberately reports no lifecycle outcome, so the client lifecycle
        // is driven from the before/after difference. This also covers the
        // negotiation reset a Leave performs. Dispatched statically so only an
        // `EndDevice` runs (and links) the client-side apply.
        if let Some(before) = timeout_before {
            R::ed_apply_timeout_change(self, before).await;
        }

        // NWK commands never carry an NLDE-DATA payload; a lifecycle outcome
        // is the whole result of the frame.
        if let Some(command) = command_outcome {
            return self.handle_nwk_command_outcome(command).await;
        }

        let (dst, src, nwk_security, nwk_security_source, len) = {
            let scratch_nwk = unsafe { &mut *self.scratch.nwk.get() };
            match unpack_nwk_indication(scratch_nwk, nwk_indication) {
                Some(v) => v,
                None => {
                    rt_trace!("[RT] nwk_rx=dropped len={}", mac_payload.len());
                    return None;
                }
            }
        };

        let buf = unsafe { &*self.scratch.nwk.get() };

        rt_trace!(
            "[RT] nwk src=0x{:04X} dst=0x{:04X} sec={} len={}",
            src.0,
            dst.0,
            nwk_security as u8,
            len
        );
        log::info!(
            "[RX] NWK data src=0x{:04X} dst=0x{:04X} sec={} len={}",
            src.0,
            dst.0,
            nwk_security,
            len
        );

        // APS decryption buffer (for APS-secured frames like Transport Key)
        let aps_decrypt_buf = unsafe { &mut *self.scratch.aps.get() };

        // APS layer: parse APS header
        let (aps_indication, pending_tunnel) = {
            let aps = self.bdb.zdo_mut().aps_mut();
            let indication = aps.process_incoming_aps_frame(
                &buf[..len],
                src,
                dst,
                indication.lqi,
                zigbee_aps::apsde::IncomingNwkSecurity::new(nwk_security, nwk_security_source),
                aps_decrypt_buf,
            );
            let tunnel = aps.take_pending_tunnel();
            (indication, tunnel)
        };
        if let Some(tunnel) = pending_tunnel {
            // A Tunnel command is an APS *command* frame like any other, so if
            // it asked for an acknowledgement the APS layer queued one. Flush
            // it before forwarding: the acknowledgement precedes the resulting
            // application action (R22 §2.2.5.1), and this path returns early,
            // so leaving the single pending-ACK slot occupied would either
            // acknowledge the Tunnel with the APS counter of an unrelated later
            // frame or drop the acknowledgement entirely — either way the Trust
            // Center keeps retransmitting the tunnelled key. `send_pending_aps_ack`
            // takes the slot, so this can never send it twice.
            let _ = self.bdb.zdo_mut().aps_mut().send_pending_aps_ack().await;
            if let Err(error) = self.bdb.zdo_mut().aps_mut().forward_tunnel(&tunnel).await {
                log::warn!("[Runtime] APS Tunnel forwarding failed: {:?}", error);
            }
            return None;
        }
        let aps_indication = match aps_indication {
            Some(v) => v,
            None => {
                rt_trace!("[RT] aps_process=none");
                // An APS *command* frame never produces a data indication, and
                // neither does a duplicate that was dropped after its
                // acknowledgement was regenerated (R22 §2.2.4.1.3). Either way
                // the acknowledgement that was queued has to be flushed here or
                // it would be left for an unrelated later frame.
                let _ = self.bdb.zdo_mut().aps_mut().send_pending_aps_ack().await;
                return None;
            }
        };

        // Route by destination endpoint. Reading the metadata and emitting the
        // RX trace/log line is a synchronous, `MacDriver`-independent step, so
        // it runs in the shared `#[inline(never)]` `aps_route_metadata` helper
        // kept out of the per-backend receive future.
        let (dst_ep, cluster_id, src_addr) = aps_route_metadata(&aps_indication);

        // Send APS ACK now if the incoming frame requested one. This must
        // happen for *every* endpoint (ZDO and application clusters alike),
        // not just for ZDO — otherwise the coordinator/TC retransmits ZCL
        // Read Attributes (e.g. Basic Manufacturer/Model) until the ZHA
        // interview times out, leaving the device as `unk_manufacturer /
        // unk_model` with empty endpoints. Spec: APS sub-layer ACKs precede
        // any application-level response (ZB R22 §2.2.5.1).
        let _ = self.bdb.zdo_mut().aps_mut().send_pending_aps_ack().await;

        if dst_ep == 0x00 {
            // ZDO endpoint — dispatch to ZDP handler which sends responses
            // directly through the APS layer.
            rt_trace!(
                "[RT] zdo_req cluster=0x{:04X} from=0x{:04X} len={}",
                cluster_id,
                src_addr,
                aps_indication.payload.len()
            );
            log::info!(
                "[Runtime] ZDO request: cluster=0x{:04X} from 0x{:04X} len={}",
                cluster_id,
                src_addr,
                aps_indication.payload.len(),
            );
            if cluster_id == zigbee_zdo::MGMT_LEAVE_REQ
                && self.bdb.zdo().nwk().nib().security_enabled
                && !nwk_security
            {
                log::warn!("[Runtime] Ignoring unsecured Mgmt_Leave_req");
                return None;
            }
            // Classify Mgmt_Leave acceptance before attempting its response.
            // Response delivery is independent: a valid authorized local
            // leave remains accepted even when Mgmt_Leave_rsp cannot be sent.
            let accepted_mgmt_leave = if cluster_id == zigbee_zdo::MGMT_LEAVE_REQ {
                aps_indication.payload.get(1..).and_then(|payload| {
                    self.bdb
                        .zdo()
                        .classify_mgmt_leave_request(ShortAddress(src_addr), payload)
                        .ok()
                        .flatten()
                })
            } else {
                None
            };
            match self.bdb.zdo_mut().handle_indication(&aps_indication).await {
                Ok(()) => {
                    rt_trace!("[RT] zdo_ok cluster=0x{:04X}", cluster_id);
                    log::info!("[Runtime] ZDO OK cluster=0x{:04X}", cluster_id);
                }
                Err(e) => {
                    rt_trace!("[RT] zdo_fail cluster=0x{:04X} err={:?}", cluster_id, e);
                    log::warn!("[Runtime] ZDO FAIL cluster=0x{:04X}: {:?}", cluster_id, e,);
                }
            }

            // Execute every accepted local Mgmt_Leave after its response
            // attempt, whether that attempt succeeded or failed.
            if let Some(request) = accepted_mgmt_leave {
                if request.remove_children {
                    log::info!(
                        "[Runtime] Mgmt_Leave remove-children requested; local leave clears child state"
                    );
                }
                // The accepted request ends this network's interview
                // immediately. Clear before awaiting the leave notification so
                // neither a slow/failed transmission nor the later lifecycle
                // action can leave stale completion visible.
                self.remote_reporting.clear();
                log::info!("[Runtime] Executing NLME-LEAVE after Mgmt_Leave response attempt");
                let leave_result = self
                    .bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .nlme_leave(request.rejoin)
                    .await;
                if request.rejoin {
                    self.bdb.zdo_mut().nwk_mut().set_joined(false);
                    self.reset_identify_clusters();
                    let now = self.bdb.zdo().nwk().mac().monotonic_micros();
                    self.secure_rejoin_retry_at = Some(now);
                    return Some(event_loop::StackEvent::RejoinRequested);
                }
                if leave_result.is_err() {
                    log::warn!(
                        "[Runtime] Mgmt_Leave notification failed; clearing local NWK state"
                    );
                    let _ = self.bdb.zdo_mut().nlme_reset(false);
                }
                self.mark_left();
                return Some(event_loop::StackEvent::Left);
            }

            return None;
        }

        // Application endpoint — local ZCL work runs in a synchronous,
        // `MacDriver`-independent dispatcher so the full ZCL command/attribute
        // engine is not monomorphised into every backend's async receive
        // future. It parses the ZCL frame, runs foundation and cluster-specific
        // handling against runtime-local state, and enqueues responses into the
        // shared pending-response queue (drained by `flush_pending_responses`).
        // The only `M`-generic side effects — APS group-table updates and the
        // Finding & Binding Identify collection — are returned as actions and
        // applied here, after the borrow of runtime-local state is released.
        let zcl_scratch = unsafe { &mut *self.scratch.zcl.get() };
        let outcome = zcl_dispatch::LocalZclCtx::new(
            &self.endpoints,
            &mut self.basic_cluster,
            &mut self.identify_clusters,
            &mut self.reporting,
            &mut self.remote_reporting,
            &mut self.pending_responses,
            clusters,
            zcl_scratch,
        )
        .dispatch(
            dst_ep,
            aps_indication.src_endpoint,
            cluster_id,
            src_addr,
            aps_indication.payload,
        );

        if let Some(action) = outcome.group_action {
            let aps = self.bdb.zdo_mut().aps_mut();
            match action {
                zcl_dispatch::GroupTableAction::Add { group, endpoint } => {
                    let _ = aps.apsme_add_group(&zigbee_aps::apsme::ApsmeAddGroupRequest {
                        group_address: group,
                        endpoint,
                    });
                }
                zcl_dispatch::GroupTableAction::Remove { group, endpoint } => {
                    let _ = aps.apsme_remove_group(&zigbee_aps::apsme::ApsmeRemoveGroupRequest {
                        group_address: group,
                        endpoint,
                    });
                }
                zcl_dispatch::GroupTableAction::RemoveAll { endpoint } => {
                    let _ = aps.apsme_remove_all_groups(
                        &zigbee_aps::apsme::ApsmeRemoveAllGroupsRequest { endpoint },
                    );
                }
            }
        }

        if let Some((addr, ep)) = outcome.fb_identify_target {
            let _ = self.bdb.fb_identify_responses.push((addr, ep));
        }

        outcome.event
    }

    /// Send a raw ZCL frame via APS→NWK→MAC.
    pub async fn send_zcl_frame(
        &mut self,
        dst_addr: ShortAddress,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        zcl_data: &[u8],
    ) -> Result<(), event_loop::SendError> {
        if !self.is_joined() {
            return Err(event_loop::SendError::NotJoined);
        }

        let req = zigbee_aps::apsde::ApsdeDataRequest {
            dst_addr_mode: zigbee_aps::ApsAddressMode::Short,
            dst_address: ApsAddress::Short(dst_addr),
            dst_endpoint,
            profile_id: 0x0104, // Home Automation
            cluster_id,
            src_endpoint,
            payload: zcl_data,
            tx_options: zigbee_aps::ApsTxOptions {
                use_nwk_key: true,
                ..zigbee_aps::ApsTxOptions::default()
            },
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };

        match self.bdb.zdo_mut().aps_mut().apsde_data_request(&req).await {
            Ok(_) => Ok(()),
            Err(e) => {
                log::warn!("[Runtime] ZCL frame send failed: {:?}", e);
                Err(event_loop::SendError::Aps(e))
            }
        }
    }

    // ── Reporting ───────────────────────────────────────────

    /// Access the reporting engine (e.g., to configure reports).
    pub fn reporting(&self) -> &ReportingEngine {
        &self.reporting
    }

    /// Mutable access to the reporting engine.
    pub fn reporting_mut(&mut self) -> &mut ReportingEngine {
        &mut self.reporting
    }

    /// Access the underlying MAC driver (e.g., for platform-specific power management).
    pub fn mac_mut(&mut self) -> &mut M {
        self.bdb.zdo_mut().nwk_mut().mac_mut()
    }

    /// Shared access to the underlying MAC driver.
    pub fn mac(&self) -> &M {
        self.bdb.zdo().nwk().mac()
    }

    /// Check if any attribute reports are due for a cluster and send them.
    ///
    /// Call this after updating cluster attributes (e.g., after reading sensors).
    /// The reporting engine checks configured min/max intervals and value changes,
    /// then sends a ZCL Report Attributes (0x0A) frame if needed.
    ///
    /// Returns `true` if a report was sent.
    ///
    /// # Example
    /// ```rust,no_run,ignore
    /// temp_cluster.set_temperature(2350);
    /// let sent = device.check_and_send_cluster_reports(
    ///     1,          // endpoint
    ///     0x0402,     // Temperature Measurement cluster
    ///     temp_cluster.attributes(),
    /// ).await;
    /// ```
    pub async fn check_and_send_cluster_reports(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        store: &dyn zigbee_zcl::clusters::AttributeStoreAccess,
    ) -> bool {
        // We need to work through the reporting engine, which requires AttributeStore<N>.
        // Since we have a trait object, we build reports manually by checking each config.
        use zigbee_zcl::foundation::reporting::{
            AttributeReport, MAX_REPORT_CONFIGS, ReportAttributes,
        };

        let mut reports: heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS> = heapless::Vec::new();
        self.reporting
            .check_and_collect_dyn(endpoint, cluster_id, store, &mut reports);

        if reports.is_empty() {
            return false;
        }

        let report = ReportAttributes { reports };
        self.send_report(endpoint, cluster_id, &report)
            .await
            .is_ok()
    }

    // ── ZCL global command response helpers ──────────────────

    /// Queue a ZCL global command response for sending in the next tick.
    ///
    /// Used by applications to respond to Read Attributes (0x00→0x01),
    /// Write Attributes (0x02→0x04), and Discover Attributes (0x0C→0x0D).
    #[allow(clippy::too_many_arguments)]
    pub fn queue_global_response(
        &mut self,
        dst_addr: u16,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        seq: u8,
        response_cmd: u8,
        payload: &[u8],
    ) {
        zcl_dispatch::queue_global_response_inner(
            &mut self.pending_responses,
            dst_addr,
            dst_endpoint,
            src_endpoint,
            cluster_id,
            seq,
            response_cmd,
            payload,
        );
    }

    // ── Layer access (for advanced use) ─────────────────────

    /// Access the BDB layer.
    pub fn bdb(&self) -> &BdbLayer<M> {
        &self.bdb
    }

    /// Mutable access to the BDB layer.
    pub fn bdb_mut(&mut self) -> &mut BdbLayer<M> {
        &mut self.bdb
    }

    /// Re-send Device_annce broadcast. Useful after join to retry if
    /// the coordinator missed the initial announcement.
    pub async fn send_device_annce(&mut self) -> Result<(), zigbee_zdo::ZdpStatus> {
        let nwk_addr = self.bdb.zdo().local_nwk_addr();
        let ieee_addr = self.bdb.zdo().local_ieee_addr();
        self.bdb.zdo_mut().device_annce(nwk_addr, ieee_addr).await
    }

    /// Send an End Device Timeout Request (0x0B) to the parent and arm the
    /// client-side response handling.
    ///
    /// A successful transmission schedules a forced MAC poll for the next
    /// runtime tick (the response is delivered indirectly, so a sleepy device
    /// has to fetch it within the parent's transaction persistence), starts a
    /// bounded response wait, and resets the recurring keepalive interval.
    ///
    /// Never fails a join, resume or tick: a transmission failure only feeds
    /// the bounded keepalive-failure counter.
    ///
    /// Bounded on [`EndDeviceRole`](crate::role::EndDeviceRole): the End Device
    /// Timeout *client* is a leaf-only obligation, so this API exists only on an
    /// end-device-typed device rather than as a success-shaped no-op on a
    /// router/relay.
    pub async fn send_ed_timeout_request(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        self.send_ed_timeout_request_tracked().await;
    }

    /// Whether the local device negotiates an End Device Timeout at all.
    fn negotiates_end_device_timeout(&self) -> bool {
        self.bdb.zdo().nwk().device_type() == zigbee_nwk::DeviceType::EndDevice
    }

    fn end_device_timeout_snapshot(&self) -> EndDeviceTimeoutSnapshot {
        let nib = self.bdb.zdo().nwk().nib();
        EndDeviceTimeoutSnapshot {
            parent_information: nib.parent_information,
            parent_information_valid: nib.parent_information_valid,
            end_device_timeout: nib.end_device_timeout,
            requested_end_device_timeout: nib.requested_end_device_timeout,
            accepts: nib.end_device_timeout_accepts,
        }
    }

    /// Keepalive method implied by the negotiated `nwkParentInformation`.
    ///
    /// A parent that answered with no bits set is a pre-R22 parent: it ages
    /// children on MAC Data Poll only, so polling is the right keepalive.
    /// An unanswered negotiation has to assume nothing and re-run the
    /// request instead.
    fn end_device_keepalive_method(&self) -> KeepaliveMethod {
        let nib = self.bdb.zdo().nwk().nib();
        if !nib.parent_information_valid {
            return KeepaliveMethod::TimeoutRequest;
        }
        if nib.parent_information == 0
            || nib.parent_information & zigbee_nwk::frames::PARENT_INFO_MAC_DATA_POLL_KEEPALIVE != 0
        {
            KeepaliveMethod::MacDataPoll
        } else {
            KeepaliveMethod::TimeoutRequest
        }
    }

    /// Recurring keepalive interval in seconds.
    ///
    /// Strictly below the timeout currently in effect — a third of it — so
    /// two consecutive missed keepalives still leave a margin before the
    /// parent ages the child out of its table.
    fn end_device_keepalive_interval_secs(&self) -> u32 {
        (self.bdb.zdo().nwk().nib().end_device_timeout_seconds() / 3).max(1)
    }

    fn reset_end_device_keepalive(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        let interval = self.end_device_keepalive_interval_secs();
        self.ed_timeout_mut().keepalive_remaining_secs = Some(interval);
    }

    /// Schedule a MAC poll on the next tick regardless of the sleepy-poll
    /// gates, so an indirect response or command can be retrieved.
    fn force_end_device_poll(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        self.ed_timeout_mut().forced_poll = true;
    }

    /// Consume the forced-poll request, if one is scheduled.
    pub(crate) fn take_forced_poll(&mut self) -> bool
    where
        R: crate::role::EndDeviceRole,
    {
        core::mem::take(&mut self.ed_timeout_mut().forced_poll)
    }

    /// Whether a MAC Data Poll refreshes this device's entry in the parent's
    /// child table.
    ///
    /// True only after the parent explicitly advertised MAC Data Poll
    /// keepalive, or answered with no bits (the pre-R22 poll-aging behavior).
    ///
    /// While parent information is unknown, ordinary application polls must
    /// not postpone the next 0x0B: the parent may support only End Device
    /// Timeout Request keepalive, in which case successful polls do not reset
    /// its child-aging timer.
    fn mac_poll_refreshes_parent_timer(&self) -> bool {
        let nib = self.bdb.zdo().nwk().nib();
        nib.parent_information_valid
            && (nib.parent_information == 0
                || nib.parent_information & zigbee_nwk::frames::PARENT_INFO_MAC_DATA_POLL_KEEPALIVE
                    != 0)
    }

    /// Note a completed MAC poll for the End Device Timeout lifecycle.
    pub(crate) fn note_end_device_poll(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        if !self.negotiates_end_device_timeout() || !self.mac_poll_refreshes_parent_timer() {
            return;
        }
        self.reset_end_device_keepalive();
    }

    /// Count one failed poll to the parent (forced keepalive *or*
    /// application-driven, e.g. an OTA fast poll) and, once a small bounded
    /// threshold is reached, hand recovery to the existing secured-rejoin retry
    /// path.
    ///
    /// A recovery is only *scheduled* once: while a secure rejoin is already
    /// pending, further no-ACK polls neither advance the counter nor re-arm the
    /// retry deadline. This prevents a persistently silent parent — which under
    /// an OTA fast-poll cadence can produce many failures per second — from
    /// repeatedly pushing the retry out (a recovery storm / livelock) and lets
    /// the scheduled rejoin actually fire.
    pub(crate) fn record_end_device_keepalive_failure(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        if self.secure_rejoin_pending() {
            return;
        }
        let failures = self.ed_timeout().failures.saturating_add(1);
        self.ed_timeout_mut().failures = failures;
        if failures < Self::ED_TIMEOUT_MAX_FAILURES {
            return;
        }
        self.ed_timeout_mut().failures = 0;
        log::warn!("[Runtime] Polls to parent failing — scheduling secure rejoin");
        self.schedule_secure_rejoin_retry();
    }

    pub(crate) fn record_end_device_keepalive_success(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        self.ed_timeout_mut().failures = 0;
    }

    /// Transmit an End Device Timeout Request and arm the response handling.
    /// Returns whether the frame was actually transmitted.
    #[inline(never)]
    async fn send_ed_timeout_request_tracked(&mut self) -> bool
    where
        R: crate::role::EndDeviceRole,
    {
        if !self.negotiates_end_device_timeout() {
            return false;
        }
        match self.bdb.zdo_mut().nwk_mut().send_ed_timeout_request().await {
            Ok(()) => {
                self.ed_timeout_mut().response_remaining_secs =
                    Some(Self::ED_TIMEOUT_RESPONSE_WAIT_SECS);
                // The parent answers with an indirect frame, so a sleepy
                // device must poll for it before transaction persistence
                // expires.
                self.force_end_device_poll();
                self.reset_end_device_keepalive();
                self.record_end_device_keepalive_success();
                true
            }
            Err(status) => {
                log::warn!("[Runtime] ED Timeout Request failed: {:?}", status);
                // Retry through the normal keepalive cadence rather than
                // spinning here; a join must not fail because of this frame.
                self.reset_end_device_keepalive();
                self.record_end_device_keepalive_failure();
                false
            }
        }
    }

    /// Begin a fresh End Device Timeout negotiation after a real join or a
    /// secured rejoin.
    ///
    /// The NWK layer already reset the negotiation at the authoritative
    /// parent-assignment point, so this only clears the client-side timers and
    /// sends exactly one initial request. Called from every path that
    /// establishes a new parent relationship; the silent persisted resume uses
    /// [`Self::resume_end_device_timeout`] instead.
    #[inline(never)]
    async fn begin_end_device_timeout_negotiation(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        self.reset_end_device_timeout_state();
        if !self.negotiates_end_device_timeout() {
            return;
        }
        self.ed_timeout_mut().retries_left = Self::ED_TIMEOUT_MAX_RETRIES;
        self.reset_end_device_keepalive();
        // A transmission failure is deliberately ignored here: the join or
        // rejoin has already succeeded and the recurring keepalive owns
        // recovery.
        self.send_ed_timeout_request_tracked().await;
    }

    /// Choose the first keepalive after a silent persisted resume.
    ///
    /// The stored parent relationship is still in force, so a device whose
    /// parent advertised MAC Data Poll keepalive (or answered with no bits, a
    /// pre-R22 parent) refreshes it with a forced poll. That poll also
    /// retrieves any frame the parent already has queued for us, so choosing
    /// the cheap keepalive never drops a pending indirect frame. Only a
    /// bit1-only parent, or a relationship that was never negotiated, needs a
    /// fresh End Device Timeout Request.
    #[inline(never)]
    async fn resume_end_device_timeout(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        self.reset_end_device_timeout_state();
        if !self.negotiates_end_device_timeout() {
            return;
        }
        self.ed_timeout_mut().retries_left = Self::ED_TIMEOUT_MAX_RETRIES;
        self.reset_end_device_keepalive();
        match self.end_device_keepalive_method() {
            KeepaliveMethod::MacDataPoll => self.force_end_device_poll(),
            KeepaliveMethod::TimeoutRequest => {
                self.send_ed_timeout_request_tracked().await;
            }
        }
    }

    /// Age the End Device Timeout timers by one tick.
    ///
    /// Pure state, run before the tick's polling so a response that arrives in
    /// the same tick can still cancel an expired wait.
    fn advance_end_device_timeout(&mut self, elapsed_secs: u16)
    where
        R: crate::role::EndDeviceRole,
    {
        self.ed_timeout_mut().advance(elapsed_secs);
    }

    /// Run the due End Device Timeout work for this tick.
    ///
    /// Ordered after the tick's polling: a forced poll scheduled by the
    /// previous tick has already run, so an outstanding response wait that is
    /// still armed here really did expire.
    #[inline(never)]
    async fn service_end_device_timeout(&mut self)
    where
        R: crate::role::EndDeviceRole,
    {
        if !self.negotiates_end_device_timeout() || !self.is_joined() {
            return;
        }

        if self.ed_timeout().response_remaining_secs == Some(0) {
            self.ed_timeout_mut().response_remaining_secs = None;
            if self.ed_timeout().retries_left > 0 {
                self.ed_timeout_mut().retries_left -= 1;
                log::warn!("[Runtime] ED Timeout Response missing — retransmitting");
                self.send_ed_timeout_request_tracked().await;
                return;
            }
            // Give up on this negotiation round: the parent information stays
            // invalid and the recurring keepalive falls back to the default
            // enumeration a R22 parent applies anyway. The join is untouched.
            log::warn!("[Runtime] ED Timeout negotiation unanswered — using default timeout");
            self.bdb
                .zdo_mut()
                .nwk_mut()
                .cancel_ed_timeout_response_wait();
            self.reset_end_device_keepalive();
            return;
        }

        if self.ed_timeout().keepalive_remaining_secs != Some(0) {
            return;
        }
        self.ed_timeout_mut().retries_left = Self::ED_TIMEOUT_MAX_RETRIES;
        match self.end_device_keepalive_method() {
            KeepaliveMethod::MacDataPoll => {
                // Schedule the poll rather than performing it here: the
                // shared forced-poll path routes any frame the parent had
                // queued through `process_incoming`, so choosing the cheap
                // keepalive never drops a pending indirect frame.
                self.reset_end_device_keepalive();
                self.force_end_device_poll();
            }
            KeepaliveMethod::TimeoutRequest => {
                self.send_ed_timeout_request_tracked().await;
            }
        }
    }

    /// Apply the client-side effect of an End Device Timeout Response that the
    /// NWK layer accepted while processing an incoming frame.
    ///
    /// The NWK layer deliberately reports no lifecycle outcome for 0x0C, so
    /// the change is detected by comparing the NIB negotiation fields around
    /// NWK processing.
    #[inline(never)]
    async fn apply_end_device_timeout_change(&mut self, before: EndDeviceTimeoutSnapshot)
    where
        R: crate::role::EndDeviceRole,
    {
        let after = self.end_device_timeout_snapshot();
        if after == before {
            return;
        }

        // Only the durably persisted fields make the application state dirty;
        // the accept counter is a local observation, not persisted state.
        if after.parent_information != before.parent_information
            || after.parent_information_valid != before.parent_information_valid
            || after.end_device_timeout != before.end_device_timeout
        {
            self.state_dirty = true;
        }

        if after.requested_end_device_timeout < before.requested_end_device_timeout {
            // Refused: retry immediately with the lowered enumeration. The NWK
            // layer floors the walk at the default, so this can never retry
            // below it. A refusal that could not lower the enumeration any
            // further leaves the bounded retransmission budget to run out.
            self.ed_timeout_mut().response_remaining_secs = None;
            self.send_ed_timeout_request_tracked().await;
            return;
        }

        if after.accepts != before.accepts {
            // Accepted: the negotiation is complete for this parent, including
            // a recurring keepalive that re-confirmed an unchanged timeout.
            self.ed_timeout_mut().response_remaining_secs = None;
            self.ed_timeout_mut().retries_left = Self::ED_TIMEOUT_MAX_RETRIES;
            self.reset_end_device_keepalive();
            self.record_end_device_keepalive_success();
        }
    }
}

#[cfg(all(test, feature = "router"))]
mod parent_router_tests {
    use super::role::Router;
    use super::{ClusterRef, ZigbeeDevice};
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use zigbee_mac::frames::parse_zigbee_beacon;
    use zigbee_mac::mock::MockMac;
    use zigbee_mac::{
        AssociationStatus, CapabilityInfo, MacCommandEvent, MacError, MacFrame, McpsDataIndication,
        MlmeAssociateIndication, MlmeAssociateResponseDelivery, MlmeBeaconRequestIndication,
        MlmeDataRequestIndication,
    };
    use zigbee_nwk::DeviceType;
    use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

    const PAN: PanId = PanId(0x1234);
    const ROUTER: ShortAddress = ShortAddress(0x0001);
    const CHILD_IEEE: IeeeAddress = [0x22; 8];
    const ROUTER_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
    const TC_IEEE: IeeeAddress = [0x44; 8];
    const NETWORK_KEY: [u8; 16] = [0x55; 16];

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(Waker::noop());
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    fn router() -> ZigbeeDevice<MockMac, Router> {
        let mut device = ZigbeeDevice::builder(MockMac::new(ROUTER_IEEE))
            .device_type(DeviceType::Router)
            .build_router();
        device.bdb_mut().attributes_mut().node_is_on_a_network = true;
        {
            let aps = device.bdb_mut().zdo_mut().aps_mut();
            aps.aib_mut().aps_trust_center_address = [0xFF; 8];
            let nwk = aps.nwk_mut();
            nwk.set_joined(true);
            let nib = nwk.nib_mut();
            nib.pan_id = PAN;
            nib.network_address = ROUTER;
            nib.extended_pan_id = [0xA5; 8];
            nib.depth = 3;
            // A router on a network holds an authoritative update state.
            nib.set_nwk_update_id(7);
        }
        device
    }

    fn centralized_router() -> ZigbeeDevice<MockMac, Router> {
        let mut device = router();
        let aps = device.bdb_mut().zdo_mut().aps_mut();
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        let nwk = aps.nwk_mut();
        nwk.security_mut().set_network_key(NETWORK_KEY, 0);
        let nib = nwk.nib_mut();
        nib.ieee_address = ROUTER_IEEE;
        nib.parent_address = ShortAddress::COORDINATOR;
        nib.security_enabled = true;
        nib.active_key_seq_number = 0;
        device
    }

    fn beacon_request() -> MacCommandEvent {
        MacCommandEvent::BeaconRequest(MlmeBeaconRequestIndication {
            destination_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
            lqi: 200,
            security_use: false,
        })
    }

    /// A joined forwarding-only [`RelayRouter`], mirroring [`router`] but built
    /// with `build_relay()` so it can route without being a parent.
    fn relay() -> ZigbeeDevice<MockMac, super::role::RelayRouter> {
        let mut device = ZigbeeDevice::builder(MockMac::new(ROUTER_IEEE)).build_relay();
        device.bdb_mut().attributes_mut().node_is_on_a_network = true;
        {
            let aps = device.bdb_mut().zdo_mut().aps_mut();
            aps.aib_mut().aps_trust_center_address = [0xFF; 8];
            let nwk = aps.nwk_mut();
            nwk.set_joined(true);
            let nib = nwk.nib_mut();
            nib.pan_id = PAN;
            nib.network_address = ROUTER;
            nib.extended_pan_id = [0xA5; 8];
            nib.depth = 3;
            // A router on a network holds an authoritative update state.
            nib.set_nwk_update_id(7);
        }
        device
    }

    #[test]
    fn a_joined_relay_tick_runs_routing_but_not_parent_maintenance() {
        // A relay routes (permit-join expiry / router maintenance run) but is
        // not a parent: it never services MAC parent commands, ages children,
        // or sends a Parent Announce. This is the routing/parent split enforced
        // by the typed role rather than the `router` feature alone.
        let mut relay = relay();
        {
            let nib = relay.bdb_mut().zdo_mut().aps_mut().nwk_mut().nib_mut();
            nib.permit_joining = true;
            nib.permit_joining_duration = 1;
        }
        relay.mac_mut().enqueue_command_event(beacon_request());
        // A relay's role state is the zero-sized `NonParentState`: it has no
        // `parent_annce_due` field at all, so the type system already forbids a
        // relay from ever queuing (or servicing) a Parent Announce.
        relay.mac_mut().clear_tx_history();

        let mut clusters: [ClusterRef<'_>; 0] = [];
        block_on(relay.tick(1, &mut clusters));

        // Routing maintenance ran: the finite permit window expired.
        assert!(
            !relay.bdb().zdo().aps().nwk().nib().permit_joining,
            "a relay runs permit-join expiry (routing maintenance)"
        );
        // Parent-command servicing did NOT run: the queued beacon request is
        // left untouched (no beacon response emitted).
        assert!(
            relay.mac().beacon_responses().is_empty(),
            "a relay never services MAC parent commands"
        );
        // No parent broadcast (Parent Announce) went out on the tick.
        assert!(
            relay.mac().tx_history().is_empty(),
            "a relay emits no parent broadcast on the tick"
        );
    }

    fn association_request(ieee: IeeeAddress, capability_info: CapabilityInfo) -> MacCommandEvent {
        MacCommandEvent::AssociationRequest(MlmeAssociateIndication {
            device_address: ieee,
            coordinator_address: MacAddress::Short(PAN, ROUTER),
            capability_info,
            lqi: 180,
            security_use: false,
        })
    }

    fn sleepy_child_capabilities() -> CapabilityInfo {
        CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        }
    }

    fn data_request(source_address: MacAddress) -> MacCommandEvent {
        MacCommandEvent::DataRequest(MlmeDataRequestIndication {
            source_address,
            destination_address: MacAddress::Short(PAN, ROUTER),
            lqi: 170,
            security_use: false,
        })
    }

    fn open_for_joining(device: &mut ZigbeeDevice<MockMac, Router>, duration: u8) {
        block_on(device.permit_joining(duration)).unwrap();
    }

    fn associate_sleepy_child(device: &mut ZigbeeDevice<MockMac, Router>) -> ShortAddress {
        open_for_joining(device, 0xFF);
        device
            .mac_mut()
            .enqueue_command_event(association_request(CHILD_IEEE, sleepy_child_capabilities()));
        assert_eq!(block_on(device.service_parent_commands()).processed, 2);
        let response = device.mac().association_responses().last().unwrap();
        assert_eq!(response.status, AssociationStatus::Success);
        response.short_address
    }

    #[test]
    fn beacon_uses_nib_capacity_and_join_state() {
        let mut device = router();
        {
            let nib = device.bdb_mut().zdo_mut().aps_mut().nwk_mut().nib_mut();
            nib.max_routers = 0;
            nib.max_children = 1;
        }
        open_for_joining(&mut device, 0xFF);
        device.mac_mut().enqueue_command_event(beacon_request());
        assert_eq!(block_on(device.service_parent_commands()).processed, 1);

        let response = &device.mac().beacon_responses()[0];
        let beacon = parse_zigbee_beacon(response.beacon_payload.as_slice());
        assert_eq!(beacon.protocol_id, 0);
        assert_eq!(beacon.stack_profile, 2);
        assert_eq!(beacon.protocol_version, 2);
        assert!(!beacon.router_capacity);
        assert!(beacon.end_device_capacity);
        assert_eq!(beacon.device_depth, 3);
        assert_eq!(beacon.extended_pan_id, [0xA5; 8]);
        assert_eq!(beacon.tx_offset, [0xFF; 3]);
        assert_eq!(beacon.update_id, 7);
        assert!(response.association_permit);

        let _ = associate_sleepy_child(&mut device);
        device.mac_mut().enqueue_command_event(beacon_request());
        block_on(device.service_parent_commands());
        let full = device.mac().beacon_responses().last().unwrap();
        let full_payload = parse_zigbee_beacon(full.beacon_payload.as_slice());
        assert!(!full_payload.end_device_capacity);
        assert!(!full.association_permit);
    }

    #[test]
    fn association_is_accepted_or_denied_by_policy_and_capacity() {
        let mut accepted = router();
        let short = associate_sleepy_child(&mut accepted);
        assert_ne!(short, ShortAddress(0xFFFF));
        assert_eq!(
            accepted
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .known_child_by_ieee(&CHILD_IEEE),
            Some(short)
        );

        let mut closed = router();
        closed
            .mac_mut()
            .enqueue_command_event(association_request(CHILD_IEEE, sleepy_child_capabilities()));
        block_on(closed.service_parent_commands());
        assert_eq!(
            closed.mac().association_responses()[0].status,
            AssociationStatus::PanAccessDenied
        );

        let mut full = router();
        full.bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .nib_mut()
            .max_children = 0;
        open_for_joining(&mut full, 0xFF);
        full.mac_mut()
            .enqueue_command_event(association_request(CHILD_IEEE, sleepy_child_capabilities()));
        block_on(full.service_parent_commands());
        assert_eq!(
            full.mac().association_responses()[0].status,
            AssociationStatus::PanAtCapacity
        );

        let mut invalid = router();
        open_for_joining(&mut invalid, 0xFF);
        let mut capabilities = sleepy_child_capabilities();
        capabilities.allocate_address = false;
        invalid
            .mac_mut()
            .enqueue_command_event(association_request(CHILD_IEEE, capabilities));
        block_on(invalid.service_parent_commands());
        assert_eq!(
            invalid.mac().association_responses()[0].status,
            AssociationStatus::PanAccessDenied
        );
    }

    #[test]
    fn tick_expires_finite_permit_but_not_indefinite_permit() {
        let mut finite = router();
        open_for_joining(&mut finite, 2);
        let mut clusters: [ClusterRef<'_>; 0] = [];
        block_on(finite.tick(1, &mut clusters));
        assert!(finite.mac().association_permit());
        block_on(finite.tick(1, &mut clusters));
        assert!(!finite.mac().association_permit());
        assert!(!finite.bdb().zdo().aps().nwk().nib().permit_joining);

        let mut indefinite = router();
        open_for_joining(&mut indefinite, 0xFF);
        block_on(indefinite.tick(u16::MAX, &mut clusters));
        assert!(indefinite.mac().association_permit());
        assert_eq!(
            indefinite
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .nib()
                .permit_joining_duration,
            0xFF
        );
    }

    #[test]
    fn child_poll_sends_one_frame_and_tracks_remaining_pending_data() {
        let mut device = router();
        let child = associate_sleepy_child(&mut device);
        {
            let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
            nwk.enqueue_indirect_for_child(child, &[1, 2]).unwrap();
            nwk.enqueue_indirect_for_child(child, &[3, 4]).unwrap();
        }
        device.mac_mut().clear_tx_history();
        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Short(PAN, child)));
        block_on(device.service_parent_commands());

        assert_eq!(device.mac().tx_history().len(), 1);
        assert!(device.mac().tx_history()[0].indirect);
        assert_eq!(device.mac().tx_history()[0].payload.as_slice(), &[1, 2]);
        assert_eq!(
            device.mac().indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), true))
        );

        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Short(PAN, child)));
        block_on(device.service_parent_commands());
        assert_eq!(device.mac().tx_history().len(), 2);
        assert_eq!(
            device.mac().indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), false))
        );
    }

    #[test]
    fn unknown_and_extended_polls_do_not_dequeue_nwk_indirect_data() {
        let mut device = router();
        let child = associate_sleepy_child(&mut device);
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .enqueue_indirect_for_child(child, &[9])
            .unwrap();
        device.mac_mut().clear_tx_history();
        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Short(PAN, ShortAddress(0x7788))));
        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Extended(PAN, CHILD_IEEE)));
        block_on(device.service_parent_commands());
        assert!(device.mac().tx_history().is_empty());
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(child)
        );
    }

    #[test]
    fn expired_indirect_transaction_clears_frame_pending() {
        let mut device = router();
        let child = associate_sleepy_child(&mut device);
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .enqueue_indirect_for_child(child, &[9])
            .unwrap();
        let mut clusters: [ClusterRef<'_>; 0] = [];
        block_on(device.tick(8, &mut clusters));

        assert!(
            !device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(child)
        );
        assert_eq!(
            device.mac().indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), false))
        );
    }

    #[test]
    fn command_drain_is_bounded_to_four_events() {
        let mut device = router();
        for _ in 0..5 {
            device.mac_mut().enqueue_command_event(beacon_request());
        }
        let first = block_on(device.service_parent_commands());
        assert_eq!(first.processed, 4);
        assert_eq!(device.mac().beacon_responses().len(), 4);
        let second = block_on(device.service_parent_commands());
        assert_eq!(second.processed, 1);
        assert_eq!(device.mac().beacon_responses().len(), 5);
    }

    #[test]
    fn update_device_waits_for_association_response_delivery() {
        let mut device = centralized_router();
        open_for_joining(&mut device, 0xFF);

        block_on(
            device.handle_parent_command(association_request(
                CHILD_IEEE,
                sleepy_child_capabilities(),
            )),
        )
        .unwrap();
        let response = device.mac().association_responses()[0].clone();
        assert!(
            device.mac().tx_history().is_empty(),
            "queueing the Association Response must not notify the Trust Center"
        );
        assert!(
            !device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .child_is_authorized(&CHILD_IEEE)
        );

        let step = block_on(device.service_parent_commands());
        assert_eq!(step.processed, 1);
        assert_eq!(step.failures, 0);
        assert_eq!(
            device.mac().tx_history().len(),
            2,
            "the global TC key produces encrypted and NWK-only Update-Device copies"
        );
        assert_eq!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .known_child_by_ieee(&CHILD_IEEE),
            Some(response.short_address)
        );
    }

    #[test]
    fn failed_association_response_delivery_rolls_back_a_provisional_child() {
        let mut device = centralized_router();
        open_for_joining(&mut device, 0xFF);
        block_on(
            device.handle_parent_command(association_request(
                CHILD_IEEE,
                sleepy_child_capabilities(),
            )),
        )
        .unwrap();
        let response = device.mac().association_responses()[0].clone();

        assert_eq!(
            block_on(
                device.handle_parent_command(MacCommandEvent::AssociationResponseDelivery(
                    MlmeAssociateResponseDelivery {
                        device_address: CHILD_IEEE,
                        short_address: response.short_address,
                        status: AssociationStatus::Success,
                        result: Err(MacError::NoAck),
                    },
                ),)
            ),
            Err(MacError::NoAck)
        );
        assert_eq!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .known_child_by_ieee(&CHILD_IEEE),
            None
        );
    }

    #[test]
    fn sleepy_rejoin_notifies_the_trust_center_only_after_the_response_poll() {
        let mut device = centralized_router();
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .nib_mut()
            .permit_joining = true;
        let old_address = ShortAddress(0x3344);

        block_on(device.handle_child_rejoin_request(
            old_address,
            CHILD_IEEE,
            sleepy_child_capabilities().to_byte(),
            false,
        ));
        assert!(device.mac().tx_history().is_empty());
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(old_address)
        );
        assert_eq!(device.pending_child_update_count(), 1);

        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Short(PAN, old_address)));
        let step = block_on(device.service_parent_commands());
        assert_eq!(step.processed, 1);
        assert_eq!(step.failures, 0);
        assert_eq!(device.pending_child_update_count(), 0);
        let history = device.mac().tx_history();
        assert_eq!(history.len(), 3);
        assert!(history[0].indirect);
        let response = history[0].payload.as_slice();
        let (header, consumed) = zigbee_nwk::frames::NwkHeader::parse(response).unwrap();
        assert_eq!(header.dst_addr, old_address);
        assert!(!header.frame_control.security);
        assert_eq!(
            &response[consumed..],
            &[
                zigbee_nwk::frames::NwkCommandId::RejoinResponse as u8,
                old_address.0 as u8,
                (old_address.0 >> 8) as u8,
                0,
            ]
        );
    }

    #[test]
    fn distributed_security_rejects_an_unsecured_trust_center_rejoin() {
        let mut device = router();
        let old_address = ShortAddress(0x4455);
        let mut capability = sleepy_child_capabilities();
        capability.rx_on_when_idle = true;

        block_on(device.handle_child_rejoin_request(
            old_address,
            CHILD_IEEE,
            capability.to_byte(),
            false,
        ));

        assert_eq!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .known_child_by_ieee(&CHILD_IEEE),
            None
        );
        let history = device.mac().tx_history();
        assert_eq!(history.len(), 1);
        let response = history[0].payload.as_slice();
        let (_, consumed) = zigbee_nwk::frames::NwkHeader::parse(response).unwrap();
        assert_eq!(
            &response[consumed..],
            &[
                zigbee_nwk::frames::NwkCommandId::RejoinResponse as u8,
                0xFF,
                0xFF,
                0x02,
            ]
        );
    }

    #[test]
    fn runtime_forwards_a_tunneled_apdu_unchanged_to_a_sleepy_child() {
        let mut device = centralized_router();
        {
            let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
            nwk.nib_mut().permit_joining = true;
            nwk.handle_child_association(CHILD_IEEE, sleepy_child_capabilities().to_byte())
                .unwrap();
        }
        let child = device
            .bdb()
            .zdo()
            .aps()
            .nwk()
            .known_child_by_ieee(&CHILD_IEEE)
            .unwrap();

        let embedded_header = zigbee_aps::frames::ApsHeader {
            frame_control: zigbee_aps::frames::ApsFrameControl {
                frame_type: zigbee_aps::frames::ApsFrameType::Command as u8,
                delivery_mode: zigbee_aps::frames::ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: true,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: 7,
            extended_header: None,
        };
        let mut embedded = [0u8; 64];
        let embedded_header_len = embedded_header.serialize(&mut embedded);
        let embedded_security = zigbee_aps::security::ApsSecurityHeader {
            security_control: (zigbee_aps::security::KEY_ID_KEY_TRANSPORT << 3) | (1 << 5),
            frame_counter: 9,
            source_address: Some(TC_IEEE),
            key_seq_number: None,
        };
        let embedded_security_len =
            embedded_security.serialize(&mut embedded[embedded_header_len..]);
        let embedded_len = embedded_header_len + embedded_security_len + 5;
        embedded[embedded_header_len + embedded_security_len..embedded_len]
            .copy_from_slice(&[1, 2, 3, 4, 5]);

        let outer_header = zigbee_aps::frames::ApsHeader {
            frame_control: zigbee_aps::frames::ApsFrameControl {
                frame_type: zigbee_aps::frames::ApsFrameType::Command as u8,
                delivery_mode: zigbee_aps::frames::ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: 8,
            extended_header: None,
        };
        let mut outer = [0u8; 96];
        let outer_header_len = outer_header.serialize(&mut outer);
        outer[outer_header_len] = zigbee_aps::frames::ApsCommandId::Tunnel as u8;
        outer[outer_header_len + 1..outer_header_len + 9].copy_from_slice(&CHILD_IEEE);
        outer[outer_header_len + 9..outer_header_len + 9 + embedded_len]
            .copy_from_slice(&embedded[..embedded_len]);
        let outer_len = outer_header_len + 9 + embedded_len;

        let nwk_header = zigbee_nwk::frames::NwkHeader {
            frame_control: zigbee_nwk::frames::NwkFrameControl {
                frame_type: zigbee_nwk::frames::NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: true,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: ROUTER,
            src_addr: ShortAddress::COORDINATOR,
            radius: 5,
            seq_number: 9,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let mut nwk_frame = [0u8; 128];
        let nwk_header_len = nwk_header.serialize(&mut nwk_frame);
        let nwk_security = zigbee_nwk::security::NwkSecurityHeader {
            security_control: zigbee_nwk::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 1,
            source_address: TC_IEEE,
            key_seq_number: 0,
        };
        let nwk_security_len = nwk_security.serialize(&mut nwk_frame[nwk_header_len..]);
        let aad_len = nwk_header_len + nwk_security_len;
        let ciphertext = zigbee_nwk::security::NwkSecurity::new()
            .encrypt(
                &nwk_frame[..aad_len],
                &outer[..outer_len],
                &NETWORK_KEY,
                &nwk_security,
            )
            .unwrap();
        nwk_frame[aad_len..aad_len + ciphertext.len()].copy_from_slice(&ciphertext);
        nwk_frame[nwk_header_len] &= !0x07;
        let frame_len = aad_len + ciphertext.len();
        let indication = McpsDataIndication {
            src_address: MacAddress::Short(PAN, ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PAN, ROUTER),
            lqi: 200,
            payload: MacFrame::from_slice(&nwk_frame[..frame_len]).unwrap(),
            security_use: false,
        };

        let mut clusters: [ClusterRef<'_>; 0] = [];
        assert!(block_on(device.process_incoming(&indication, &mut clusters)).is_none());
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(child)
        );

        device
            .mac_mut()
            .enqueue_command_event(data_request(MacAddress::Short(PAN, child)));
        block_on(device.service_parent_commands());
        let record = &device.mac().tx_history()[0];
        assert!(record.indirect);
        let bytes = record.payload.as_slice();
        let (forward_header, consumed) = zigbee_nwk::frames::NwkHeader::parse(bytes).unwrap();
        assert!(!forward_header.frame_control.security);
        assert_eq!(&bytes[consumed..], &embedded[..embedded_len]);
    }

    // ── R22 End Device Timeout server + persistence (runtime) ─

    /// Build a secured NWK command frame sent by an authenticated child to the
    /// router, produced with the shared network key like real over-the-air
    /// traffic.
    fn child_secured_command(
        child: ShortAddress,
        command: &[u8],
        frame_counter: u32,
    ) -> McpsDataIndication {
        let header = zigbee_nwk::frames::NwkHeader {
            frame_control: zigbee_nwk::frames::NwkFrameControl {
                frame_type: zigbee_nwk::frames::NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: true,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr: ROUTER,
            src_addr: child,
            radius: 1,
            seq_number: frame_counter as u8,
            dst_ieee: None,
            src_ieee: Some(CHILD_IEEE),
            multicast_control: None,
            source_route: None,
        };
        let mut frame = [0u8; 128];
        let header_len = header.serialize(&mut frame);
        let security = zigbee_nwk::security::NwkSecurityHeader {
            security_control: zigbee_nwk::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter,
            source_address: CHILD_IEEE,
            key_seq_number: 0,
        };
        let security_len = security.serialize(&mut frame[header_len..]);
        let aad_len = header_len + security_len;
        let ciphertext = zigbee_nwk::security::NwkSecurity::new()
            .encrypt(&frame[..aad_len], command, &NETWORK_KEY, &security)
            .unwrap();
        frame[aad_len..aad_len + ciphertext.len()].copy_from_slice(&ciphertext);
        frame[header_len] &= !0x07;
        let frame_len = aad_len + ciphertext.len();
        McpsDataIndication {
            src_address: MacAddress::Short(PAN, child),
            dst_address: MacAddress::Short(PAN, ROUTER),
            lqi: 200,
            payload: MacFrame::from_slice(&frame[..frame_len]).unwrap(),
            security_use: false,
        }
    }

    /// Admit `CHILD_IEEE` at `child` as a secured, authenticated sleepy child.
    fn authenticated_sleepy_child(device: &mut ZigbeeDevice<MockMac, Router>, child: ShortAddress) {
        let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
        let assigned = nwk
            .handle_child_rejoin(
                child,
                CHILD_IEEE,
                sleepy_child_capabilities().to_byte(),
                true,
            )
            .expect("the child is admitted");
        assert_eq!(assigned, child);
    }

    #[test]
    fn the_runtime_answers_a_child_ed_timeout_request() {
        let mut device = centralized_router();
        const CHILD: ShortAddress = ShortAddress(0x89AB);
        authenticated_sleepy_child(&mut device, CHILD);
        device.mac_mut().clear_tx_history();

        let request = child_secured_command(
            CHILD,
            &[
                zigbee_nwk::frames::NwkCommandId::EdTimeoutRequest as u8,
                14,
                0,
            ],
            1,
        );
        let mut clusters: [ClusterRef<'_>; 0] = [];
        assert!(block_on(device.process_incoming(&request, &mut clusters)).is_none());

        let nwk = device.bdb().zdo().aps().nwk();
        assert_eq!(
            nwk.neighbor_table()
                .find_by_short(CHILD)
                .unwrap()
                .end_device_timeout,
            14,
            "the runtime applied the accepted timeout"
        );
        assert!(
            nwk.indirect_queue().has_pending(CHILD),
            "the response is queued indirectly for the sleepy child"
        );
    }

    #[test]
    fn the_child_table_survives_a_reboot_through_the_durable_store() {
        let mut device = centralized_router();
        const CHILD: ShortAddress = ShortAddress(0x89AB);
        authenticated_sleepy_child(&mut device, CHILD);
        // Negotiate a non-default timeout so the persisted enum is meaningful.
        block_on(
            device
                .bdb_mut()
                .zdo_mut()
                .aps_mut()
                .nwk_mut()
                .respond_to_end_device_timeout_request(CHILD, CHILD_IEEE, 14),
        )
        .unwrap();

        let mut store = crate::child_store::RamChildTableStore::new();
        device.save_child_table(&mut store).unwrap();

        // A fresh router restores the child table before any Parent Announce.
        let mut rebooted = centralized_router();
        assert_eq!(rebooted.restore_child_table(&mut store).unwrap(), 1);
        assert!(
            rebooted.parent_annce_due(),
            "a restore schedules a Parent Announce"
        );

        let nwk = rebooted.bdb().zdo().aps().nwk();
        assert_eq!(nwk.known_child_by_ieee(&CHILD_IEEE), Some(CHILD));
        let entry = nwk.neighbor_table().find_by_short(CHILD).unwrap();
        assert_eq!(
            entry.end_device_timeout, 14,
            "the accepted timeout is restored"
        );
        assert_eq!(
            entry.keepalive_remaining_secs,
            zigbee_nwk::frames::ed_timeout_enum_to_seconds(14).unwrap(),
            "a restored child gets a fresh full window"
        );
        assert!(
            !entry.keepalive_confirmed,
            "a restored child is unconfirmed until heard from"
        );
    }

    #[test]
    fn a_joined_router_tick_ages_out_a_silent_child() {
        let mut device = centralized_router();
        const CHILD: ShortAddress = ShortAddress(0x89AB);
        authenticated_sleepy_child(&mut device, CHILD);
        let window = zigbee_nwk::frames::ed_timeout_enum_to_seconds(
            zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        )
        .unwrap() as u16;
        let mut clusters: [ClusterRef<'_>; 0] = [];
        block_on(device.tick(window, &mut clusters));
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .neighbor_table()
                .find_by_short(CHILD)
                .is_none(),
            "a silent child ages out via the joined tick"
        );
    }

    #[test]
    fn a_joined_router_tick_runs_the_full_parent_maintenance() {
        // One joined-router tick must drive the whole parent maintenance
        // subgraph a sensor build compiles out: finite permit-join expiry, MAC
        // parent-command servicing, End Device Timeout child aging, and — the
        // path this split moves onto the tick via `run_nwk_maintenance` — a due
        // Parent Announce broadcast.
        let mut device = centralized_router();
        const CHILD: ShortAddress = ShortAddress(0x89AB);
        authenticated_sleepy_child(&mut device, CHILD);

        // A finite permit window, a queued MAC parent command, and a Parent
        // Announce marked due (as `restore_child_table` does after a reboot).
        open_for_joining(&mut device, 1);
        device.mac_mut().enqueue_command_event(beacon_request());
        device.set_parent_annce_due(true);
        device.mac_mut().clear_tx_history();

        let mut clusters: [ClusterRef<'_>; 0] = [];
        block_on(device.tick(1, &mut clusters));

        // Permit-join expiry (run_parent_nwk_maintenance → tick_permit_joining).
        assert!(
            !device.mac().association_permit(),
            "the finite permit window expired on the tick"
        );
        assert!(!device.bdb().zdo().aps().nwk().nib().permit_joining);

        // MAC parent-command servicing (run_parent_nwk_maintenance →
        // service_parent_commands): the queued beacon request was answered.
        assert!(
            !device.mac().beacon_responses().is_empty(),
            "a queued MAC parent command is serviced by the tick"
        );

        // Parent Announce sending (run_nwk_maintenance → service_parent_annce).
        assert!(
            !device.parent_annce_due(),
            "the due Parent Announce was consumed"
        );
        assert!(
            device
                .mac()
                .tx_history()
                .iter()
                .any(|record| matches!(record.dst, MacAddress::Short(_, addr) if addr.0 == 0xFFFF)),
            "a due Parent Announce broadcast goes out on the tick"
        );

        // The child is still within its End Device Timeout window after 1 s.
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .neighbor_table()
                .find_by_short(CHILD)
                .is_some(),
            "a child kept within its window survives the tick"
        );

        // Child End Device Timeout aging (run_parent_nwk_maintenance →
        // age_end_device_children): a tick past the window evicts the child.
        let window = zigbee_nwk::frames::ed_timeout_enum_to_seconds(
            zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        )
        .unwrap() as u16;
        block_on(device.tick(window, &mut clusters));
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .neighbor_table()
                .find_by_short(CHILD)
                .is_none(),
            "the silent child ages out via the joined tick"
        );
    }

    #[test]
    fn announce_parent_broadcasts_only_when_there_are_children() {
        // A childless router has nothing to reconcile.
        let mut empty = centralized_router();
        block_on(empty.announce_parent()).unwrap();
        assert!(empty.mac().tx_history().is_empty());

        // With a child, exactly one broadcast Parent Announce goes out.
        let mut device = centralized_router();
        const CHILD: ShortAddress = ShortAddress(0x89AB);
        authenticated_sleepy_child(&mut device, CHILD);
        device.mac_mut().clear_tx_history();
        block_on(device.announce_parent()).unwrap();

        let tx = device.mac().tx_history();
        assert_eq!(tx.len(), 1, "one Parent_annce broadcast frame");
        assert!(
            matches!(tx[0].dst, MacAddress::Short(_, addr) if addr.0 == 0xFFFF),
            "Parent_annce is a NWK broadcast"
        );
    }

    #[test]
    fn a_relay_ignores_a_synthetic_parent_nwk_outcome() {
        // The correctness hole this slice closes: `NwkLayer::can_route` is true
        // for a relay, so a parent-only NWK outcome could previously reach the
        // shared handler and be answered. The static role dispatch now makes a
        // relay ignore it — no response, no state mutation.
        let old_address = ShortAddress(0x3344);

        let mut relay = relay();
        relay.mac_mut().clear_tx_history();
        let event = block_on(relay.handle_nwk_command_outcome(
            zigbee_nwk::nlde::NwkCommandOutcome::ChildRejoinRequest {
                src: old_address,
                ieee: CHILD_IEEE,
                capability_info: sleepy_child_capabilities().to_byte(),
                secured: true,
            },
        ));
        assert!(event.is_none());
        assert!(
            relay.mac().tx_history().is_empty(),
            "a relay never answers a child Rejoin Request"
        );
        assert!(
            !relay
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(old_address),
            "a relay queues no Rejoin Response for a sleepy child"
        );

        // Likewise, a relay must not serve the End Device Timeout request.
        let event = block_on(relay.handle_nwk_command_outcome(
            zigbee_nwk::nlde::NwkCommandOutcome::EndDeviceTimeoutRequest {
                src: old_address,
                ieee: CHILD_IEEE,
                requested_timeout: 14,
            },
        ));
        assert!(event.is_none());
        assert!(
            relay.mac().tx_history().is_empty(),
            "a relay never serves an End Device Timeout Request"
        );
    }

    #[test]
    fn a_router_answers_a_synthetic_child_rejoin_outcome() {
        // The same synthetic outcome that a relay ignores is acted on by a
        // router: it queues the indirect Rejoin Response for the sleepy child
        // and records the deferred Trust Center Update-Device.
        let mut device = centralized_router();
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .nib_mut()
            .permit_joining = true;
        let old_address = ShortAddress(0x3344);
        device.mac_mut().clear_tx_history();

        let event = block_on(device.handle_nwk_command_outcome(
            zigbee_nwk::nlde::NwkCommandOutcome::ChildRejoinRequest {
                src: old_address,
                ieee: CHILD_IEEE,
                capability_info: sleepy_child_capabilities().to_byte(),
                secured: false,
            },
        ));
        assert!(event.is_none());
        assert!(
            device
                .bdb()
                .zdo()
                .aps()
                .nwk()
                .indirect_queue()
                .has_pending(old_address),
            "a router queues the Rejoin Response indirectly for a sleepy child"
        );
        assert_eq!(
            device.pending_child_update_count(),
            1,
            "a router defers the coupled Trust Center Update-Device"
        );
    }
}

/// Typed-role and parent-capability boundary tests.
///
/// These compile-time and runtime checks prove that:
/// - an ordinary [`MacDriver`] backend constructs and uses an end-device-role
///   device (the default role), and
/// - a router-role device is type-bounded by
///   [`ParentMacDriver`](zigbee_mac::ParentMacDriver), so it cannot be built on
///   a MAC backend that cannot parent.
#[cfg(test)]
mod role_tests {
    use super::ZigbeeDevice;
    use super::role::{DeviceRole, EndDevice, ParentRole, RelayRouter, Router};
    use crate::builder::{BuildError, DeviceBuilder};
    use zigbee_mac::mock::MockMac;
    use zigbee_mac::{MacDriver, ParentMacDriver};
    use zigbee_nwk::DeviceType;

    const IEEE: [u8; 8] = [0x11; 8];

    /// Compile-time proof that router construction is bounded on a genuine
    /// parent MAC: this only type-checks because `MockMac: ParentMacDriver`.
    fn requires_parent_mac<M: ParentMacDriver>(_mac: &M) {}

    /// Compile-time proof that the ordinary end-device path needs only
    /// `MacDriver`.
    fn requires_mac_only<M: MacDriver>(_mac: &M) {}

    // Compile-time role invariants (evaluated in const context so clippy does
    // not flag them as constant runtime assertions).
    const _: () = assert!(!EndDevice::IS_PARENT);
    const _: () = assert!(!EndDevice::CAN_ROUTE);
    const _: () = assert!(RelayRouter::CAN_ROUTE);
    const _: () = assert!(!RelayRouter::IS_PARENT);
    const _: () = assert!(Router::IS_PARENT);
    const _: () = assert!(Router::CAN_ROUTE);

    #[test]
    fn role_markers_report_names() {
        assert_eq!(EndDevice::NAME, "end-device");
        assert_eq!(RelayRouter::NAME, "relay-router");
        assert_eq!(Router::NAME, "router");
    }

    #[test]
    fn ordinary_mac_builds_an_end_device() {
        let mac = MockMac::new(IEEE);
        requires_mac_only(&mac);
        // Default role: `ZigbeeDevice<MockMac>` == `ZigbeeDevice<MockMac, EndDevice>`.
        let device: ZigbeeDevice<MockMac> = ZigbeeDevice::builder(mac)
            .device_type(DeviceType::EndDevice)
            .build();
        // The end-device monomorphization is the default role.
        fn assert_end_device<M: MacDriver>(_d: &ZigbeeDevice<M, EndDevice>) {}
        assert_end_device(&device);
        assert_eq!(device.device_type(), DeviceType::EndDevice);
    }

    #[test]
    fn parent_mac_builds_a_router() {
        let mac = MockMac::new(IEEE);
        requires_parent_mac(&mac);
        let device: ZigbeeDevice<MockMac, Router> = ZigbeeDevice::builder(mac)
            .device_type(DeviceType::Router)
            .build_router();
        // The router role satisfies the parent bound used to gate parent-only
        // operational APIs (e.g. `permit_joining`).
        fn assert_parent_role<M: MacDriver, R: ParentRole>(_d: &ZigbeeDevice<M, R>) {}
        assert_parent_role(&device);
        assert_eq!(device.device_type(), DeviceType::Router);
    }

    #[test]
    fn any_mac_builds_a_relay_router_without_the_parent_bound() {
        // A relay is forwarding-only, so it needs only `MacDriver` — no parent
        // capability. It builds as `DeviceType::Router`, can route, but is not a
        // parent.
        let device: ZigbeeDevice<MockMac, RelayRouter> =
            ZigbeeDevice::builder(MockMac::new(IEEE)).build_relay();
        assert_eq!(device.device_type(), DeviceType::Router);
        const { assert!(RelayRouter::CAN_ROUTE) };
        const { assert!(!RelayRouter::IS_PARENT) };
        // A relay is *not* a `ParentRole`, so parent-only APIs are not present;
        // that absence is enforced by the type system (see the `compile_fail`
        // doctests on `ZigbeeDevice`).
        fn is_routing<M: MacDriver, R: super::role::RoutingRole>(_d: &ZigbeeDevice<M, R>) {}
        is_routing(&device);
    }

    #[test]
    fn coordinator_builds_as_a_parent_router() {
        let device = ZigbeeDevice::builder(MockMac::new(IEEE)).build_coordinator();
        fn assert_parent_role<M: MacDriver, R: ParentRole>(_d: &ZigbeeDevice<M, R>) {}
        assert_parent_role(&device);
        assert_eq!(device.device_type(), DeviceType::Coordinator);
    }

    #[test]
    #[should_panic(expected = "device_type conflicts")]
    fn coordinator_builder_rejects_an_explicit_router_type() {
        let _ = ZigbeeDevice::builder(MockMac::new(IEEE))
            .device_type(DeviceType::Router)
            .build_coordinator();
    }

    #[test]
    fn build_rejects_a_routing_device_type() {
        // `build()` yields an end device; a routing/coordinator device type is a
        // misconfiguration surfaced explicitly (not a success-shaped fallback).
        let error = DeviceBuilder::new(MockMac::new(IEEE))
            .device_type(DeviceType::Router)
            .try_build()
            .err()
            .expect("router device_type must be rejected by build()");
        assert_eq!(
            error,
            BuildError::RoleRejectsDeviceType {
                role: "end-device",
                device_type: DeviceType::Router,
            }
        );
        assert!(matches!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::Coordinator)
                .try_build(),
            Err(BuildError::RoleRejectsDeviceType { .. })
        ));
        // The matching device type builds cleanly.
        assert!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::EndDevice)
                .try_build()
                .is_ok()
        );
    }

    #[test]
    fn relay_rejects_a_non_router_device_type() {
        // A relay is only ever a router; an end-device or coordinator type is a
        // misconfiguration.
        assert!(matches!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::EndDevice)
                .try_build_relay(),
            Err(BuildError::RoleRejectsDeviceType { .. })
        ));
        assert!(matches!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::Coordinator)
                .try_build_relay(),
            Err(BuildError::RoleRejectsDeviceType { .. })
        ));
        assert!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::Router)
                .try_build_relay()
                .is_ok()
        );
    }

    #[test]
    fn router_rejects_an_end_device_type() {
        // `build_router` accepts router or coordinator, but not end device.
        assert!(matches!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::EndDevice)
                .try_build_router(),
            Err(BuildError::RoleRejectsDeviceType { .. })
        ));
        assert!(
            DeviceBuilder::new(MockMac::new(IEEE))
                .device_type(DeviceType::Coordinator)
                .try_build_router()
                .is_ok()
        );
    }

    #[test]
    #[should_panic(expected = "device_type conflicts")]
    fn ergonomic_build_panics_on_a_mismatch() {
        // The ergonomic `build()` turns the misconfiguration into an explicit
        // panic rather than silently producing a role/device-type mismatch.
        let _ = DeviceBuilder::new(MockMac::new(IEEE))
            .device_type(DeviceType::Router)
            .build();
    }

    #[test]
    fn typed_node_composes_a_router_device() {
        use crate::node::ZigbeeNode;
        use crate::profile::{DeviceProfile, RangeExtender};
        use crate::security_store::RamSecurityStateStore;
        use zigbee_aps::PROFILE_HOME_AUTOMATION;
        use zigbee_zcl::DeviceId;

        let mut device: ZigbeeDevice<MockMac, Router> = ZigbeeDevice::builder(MockMac::new(IEEE))
            .device_type(DeviceType::Router)
            .build_router();
        let mut store = RamSecurityStateStore::new();
        let mut profile = DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::RANGE_EXTENDER,
            RangeExtender,
        );
        // `ZigbeeNode` infers and carries the `Router` role type parameter, so a
        // router product needs no bespoke wrapper.
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        fn assert_router_node<'a, S, P>(_n: &ZigbeeNode<'a, MockMac, S, P, Router>)
        where
            S: crate::security_store::SecurityStateStore,
            P: crate::profile::ApplicationProfile,
        {
        }
        assert_router_node(&node);
    }

    #[test]
    fn router_role_exposes_permit_joining() {
        use core::future::Future;
        use core::task::{Context, Poll, Waker};

        fn block_on<F: Future>(future: F) -> F::Output {
            let mut context = Context::from_waker(Waker::noop());
            let mut future = std::pin::pin!(future);
            loop {
                if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                    return output;
                }
                std::thread::yield_now();
            }
        }

        // Parent-only APIs (`permit_joining`, `announce_parent`,
        // `save_child_table`) live behind `ParentRole`; that they resolve here
        // on a router-role device (and not on an end-device- or relay-role
        // device) is the capability boundary. On a not-yet-joined router they
        // are structurally successful no-ops.
        let mut device: ZigbeeDevice<MockMac, Router> = ZigbeeDevice::builder(MockMac::new(IEEE))
            .device_type(DeviceType::Router)
            .build_router();
        assert!(block_on(device.permit_joining(0)).is_ok());
        assert!(block_on(device.announce_parent()).is_ok());
        let mut child_store = crate::child_store::RamChildTableStore::new();
        assert!(device.save_child_table(&mut child_store).is_ok());
    }

    /// The role-state split gives each role a *distinct* inline runtime state,
    /// so no role pays for another's RAM:
    /// - a [`RelayRouter`] holds the zero-sized `NonParentState`,
    /// - an [`EndDevice`] holds `EndDeviceState` (the R22 End Device Timeout
    ///   *client* lifecycle only), and
    /// - a [`Router`] holds `ParentState` (the deferred child-update queue and
    ///   Parent Announce flag only).
    ///
    /// So a relay is the smallest of the three, an end device is larger by its
    /// client state, and a router is larger by its (bigger) parent state — and,
    /// crucially, a router no longer carries the End Device Timeout *client*
    /// state at all (it moved out of the common struct into `EndDeviceState`).
    #[test]
    fn each_role_carries_only_its_own_inline_state() {
        use super::EndDeviceTimeoutState;
        use super::role::{EndDeviceState, NonParentState, ParentState};
        use core::mem::size_of;

        // A relay carries no role RAM; an end device carries exactly the client
        // timeout state; a router carries the (strictly larger) parent state.
        assert_eq!(size_of::<NonParentState>(), 0);
        assert_eq!(
            size_of::<EndDeviceState>(),
            size_of::<EndDeviceTimeoutState>(),
            "the end-device role state is exactly the client timeout lifecycle"
        );
        assert!(size_of::<EndDeviceState>() > 0);
        assert!(
            size_of::<ParentState>() > size_of::<EndDeviceState>(),
            "the parent state (queue + flag) is larger than the client state"
        );

        let relay = size_of::<ZigbeeDevice<MockMac, RelayRouter>>();
        let end_device = size_of::<ZigbeeDevice<MockMac, EndDevice>>();
        let router = size_of::<ZigbeeDevice<MockMac, Router>>();

        // The relay is the leanest role: strictly smaller than an end device
        // (which adds the client state) and than a router (which adds the
        // parent state). Use `>=`/alignment-tolerant bounds so struct-field
        // ordering / padding cannot make the assertions brittle.
        let slack = size_of::<usize>() - 1;
        assert!(
            end_device > relay,
            "an end device carries the client timeout state a relay does not"
        );
        assert!(
            end_device - relay >= size_of::<EndDeviceState>() - slack,
            "the end-device/relay gap ({}) must account for the client state ({})",
            end_device - relay,
            size_of::<EndDeviceState>()
        );
        assert!(
            router > relay,
            "a router carries the parent state a relay does not"
        );
        assert!(
            router - relay >= size_of::<ParentState>() - slack,
            "the router/relay gap ({}) must account for the parent state ({})",
            router - relay,
            size_of::<ParentState>()
        );
        // And a router is larger than an end device, because its parent state is
        // larger than the client state it no longer carries.
        assert!(
            router > end_device,
            "a router (parent state) is larger than an end device (client state)"
        );
    }

    /// Compile-time proof that the R22 End Device Timeout **client** lifecycle
    /// belongs to the end-device role alone: only [`EndDevice`] implements
    /// [`EndDeviceRole`](super::role::EndDeviceRole), so only an
    /// end-device-typed device can name the client state or its helpers. A
    /// [`RelayRouter`] or [`Router`] carries neither the state nor the client
    /// code.
    #[test]
    fn only_the_end_device_role_owns_the_ed_timeout_client() {
        use super::role::EndDeviceRole;

        fn assert_end_device_role<M: MacDriver, R: EndDeviceRole>(_d: &ZigbeeDevice<M, R>) {}

        let end_device = ZigbeeDevice::builder(MockMac::new(IEEE)).build();
        assert_end_device_role(&end_device);

        // The negative side is enforced at compile time by the `compile_fail`
        // doctests on `ZigbeeDevice` and by `EndDeviceRole` being implemented
        // for `EndDevice` only; a relay/router simply has no `ed_timeout()` or
        // `send_ed_timeout_request` method to call.
    }
}

/// Legacy item-by-item NV persistence of `nwkUpdateId` (R22 §3.6.1.4.1).
///
/// The item is optional in that format: records written before it existed
/// carry no update state at all. Restoring such a record as a *known* `0`
/// would arm the rejoin staleness gate against a value the device never
/// learned, and every beacon in `0x81..=0xFF` would then look stale.
#[cfg(test)]
mod legacy_nv_update_id_tests {
    use super::ZigbeeDevice;
    use crate::nv_storage::{NvItemId, NvStorage, RamNvStorage};
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::DeviceType;

    const IEEE: [u8; 8] = [0x21; 8];

    fn new_device() -> ZigbeeDevice<MockMac> {
        ZigbeeDevice::builder(MockMac::new(IEEE))
            .device_type(DeviceType::EndDevice)
            .build()
    }

    /// A minimal legacy record that `restore_state` accepts, without the
    /// `NwkUpdateId` item.
    fn write_legacy_record(nv: &mut RamNvStorage) {
        nv.write(NvItemId::BdbNodeIsOnNetwork, &[1]).unwrap();
        nv.write(NvItemId::NwkPanId, &0x1A2Bu16.to_le_bytes())
            .unwrap();
        nv.write(NvItemId::NwkChannel, &[15]).unwrap();
        nv.write(NvItemId::NwkShortAddress, &0x4321u16.to_le_bytes())
            .unwrap();
        nv.write(NvItemId::NwkExtendedPanId, &[0xAA; 8]).unwrap();
        nv.write(NvItemId::NwkIeeeAddress, &IEEE).unwrap();
        nv.write(NvItemId::NwkDepth, &[1]).unwrap();
        nv.write(NvItemId::NwkParentAddress, &0x0000u16.to_le_bytes())
            .unwrap();
    }

    #[test]
    fn legacy_restore_without_the_item_leaves_the_update_id_unknown() {
        let mut nv = RamNvStorage::new();
        write_legacy_record(&mut nv);
        assert!(!nv.exists(NvItemId::NwkUpdateId).unwrap());

        let mut device = new_device();
        assert!(device.restore_state(&mut nv));

        let nib = device.bdb.zdo().nwk().nib();
        assert_eq!(
            nib.nwk_update_id(),
            None,
            "an absent NwkUpdateId item must restore as unknown, never as a known 0"
        );
        assert!(!nib.update_id_valid);

        // The rest of the record still restored.
        assert_eq!(nib.logical_channel, 15);
        assert_eq!(nib.network_address.0, 0x4321);
    }

    #[test]
    fn legacy_restore_with_the_item_marks_the_update_id_known() {
        let mut nv = RamNvStorage::new();
        write_legacy_record(&mut nv);
        nv.write(NvItemId::NwkUpdateId, &[0x2A]).unwrap();

        let mut device = new_device();
        assert!(device.restore_state(&mut nv));
        assert_eq!(device.bdb.zdo().nwk().nib().nwk_update_id(), Some(0x2A));

        // Including a genuine, authoritative 0.
        let mut nv = RamNvStorage::new();
        write_legacy_record(&mut nv);
        nv.write(NvItemId::NwkUpdateId, &[0]).unwrap();
        let mut device = new_device();
        assert!(device.restore_state(&mut nv));
        assert_eq!(device.bdb.zdo().nwk().nib().nwk_update_id(), Some(0));
    }

    /// A malformed item is not authoritative either.
    #[test]
    fn legacy_restore_ignores_a_wrong_length_item() {
        let mut nv = RamNvStorage::new();
        write_legacy_record(&mut nv);
        nv.write(NvItemId::NwkUpdateId, &[0x07, 0x08]).unwrap();

        let mut device = new_device();
        assert!(device.restore_state(&mut nv));
        assert_eq!(device.bdb.zdo().nwk().nib().nwk_update_id(), None);
    }

    #[test]
    fn saving_an_unknown_update_id_removes_the_item_instead_of_writing_zero() {
        let mut nv = RamNvStorage::new();
        // A stale item from a previous commissioning.
        nv.write(NvItemId::NwkUpdateId, &[0x2A]).unwrap();

        let device = new_device();
        assert_eq!(device.bdb.zdo().nwk().nib().nwk_update_id(), None);
        device.save_state(&mut nv);

        assert!(
            !nv.exists(NvItemId::NwkUpdateId).unwrap(),
            "an unknown update state must not be persisted as a known value"
        );
    }

    #[test]
    fn saving_a_known_update_id_writes_the_item() {
        let mut nv = RamNvStorage::new();
        let mut device = new_device();
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .set_nwk_update_id(0x2A);
        device.save_state(&mut nv);

        let mut buf = [0u8; 4];
        assert_eq!(nv.read(NvItemId::NwkUpdateId, &mut buf).unwrap(), 1);
        assert_eq!(buf[0], 0x2A);
    }

    /// Save/reboot/restore round trip: an unknown state stays unknown across a
    /// reboot, and cannot be promoted to a known `0`.
    #[test]
    fn an_unknown_update_id_survives_a_save_and_restore_cycle_as_unknown() {
        let mut nv = RamNvStorage::new();
        write_legacy_record(&mut nv);
        nv.write(NvItemId::NwkUpdateId, &[0x2A]).unwrap();

        let mut device = new_device();
        assert!(device.restore_state(&mut nv));
        assert_eq!(device.bdb.zdo().nwk().nib().nwk_update_id(), Some(0x2A));

        // Something clears the update state (a leave, say) and the record is
        // saved again.
        device
            .bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .clear_nwk_update_id();
        write_legacy_record(&mut nv);
        device.save_state(&mut nv);

        let mut rebooted = new_device();
        assert!(rebooted.restore_state(&mut nv));
        assert_eq!(rebooted.bdb.zdo().nwk().nib().nwk_update_id(), None);
    }
}
