//! A router proxy without an entry for a GPD searches for its pairing; a
//! sink answers with GP Pairing and the proxy tunnels the GPD from then on.

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_gp::cluster::{PairingSearch, client_cmd, server_cmd};
use zigbee_gp::frame::{FrameType, GpdfParams};
use zigbee_gp::security;
use zigbee_gp::sink::SinkEntry;
use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel};
use zigbee_mac::mock::MockMac;
use zigbee_mac::{MacDriver, MacFrame, McpsDataIndication, PlatformServices};
use zigbee_nwk::DeviceType;
use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

use super::imp::{SEARCH_GAP_MS, SEARCH_REPEAT_MS};
use crate::ZigbeeDevice;
use crate::role::Router;

const PAN: PanId = PanId(0x1234);
const PROXY: ShortAddress = ShortAddress(0x0001);
const PROXY_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
const SINK_IEEE: IeeeAddress = [0x44; 8];
const NETWORK_KEY: [u8; 16] = [0x55; 16];
const SRC: u32 = 0x7A6B_9879;
const KEY: [u8; 16] = [0xC5; 16];

fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        std::thread::yield_now();
    }
}

fn network(device: &mut ZigbeeDevice<MockMac, Router>, nwk: ShortAddress, ieee: IeeeAddress) {
    device.bdb_mut().attributes_mut().node_is_on_a_network = true;
    let aps = device.bdb_mut().zdo_mut().aps_mut();
    aps.aib_mut().aps_trust_center_address = SINK_IEEE;
    let nwk_layer = aps.nwk_mut();
    nwk_layer.set_joined(true);
    nwk_layer.security_mut().set_network_key(NETWORK_KEY, 0);
    let nib = nwk_layer.nib_mut();
    nib.pan_id = PAN;
    nib.network_address = nwk;
    nib.ieee_address = ieee;
    nib.extended_pan_id = [0xA5; 8];
    nib.depth = u8::from(nwk != ShortAddress::COORDINATOR);
    nib.security_enabled = true;
    nib.active_key_seq_number = 0;
    nib.outgoing_frame_counter_limit = 0x400;
    nib.logical_channel = 15;
}

fn new_proxy() -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(PROXY_IEEE))
        .device_type(DeviceType::Router)
        .build_router();
    network(&mut device, PROXY, PROXY_IEEE);
    device
        .bdb_mut()
        .zdo_mut()
        .aps_mut()
        .nwk_mut()
        .nib_mut()
        .parent_address = ShortAddress::COORDINATOR;
    device
}

/// The coordinator's sink with the GPD paired (derived groupcast, L3).
fn new_sink() -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(SINK_IEEE)).build_coordinator();
    let mac = device.mac_mut();
    for (attribute, value) in [
        (
            zigbee_mac::PibAttribute::MacPanId,
            zigbee_mac::PibValue::PanId(PAN),
        ),
        (
            zigbee_mac::PibAttribute::MacShortAddress,
            zigbee_mac::PibValue::ShortAddress(ShortAddress::COORDINATOR),
        ),
    ] {
        block_on(mac.mlme_set(attribute, value)).unwrap();
    }
    network(&mut device, ShortAddress::COORDINATOR, SINK_IEEE);
    device
        .bdb_mut()
        .zdo_mut()
        .aps_mut()
        .nwk_mut()
        .update_neighbor_address(PROXY, PROXY_IEEE);
    let mut e = SinkEntry::new(GpdId::SrcId(SRC));
    e.communication_mode = CommunicationMode::DerivedGroupcast;
    e.device_id = 0x02;
    e.security_level = SecurityLevel::Encrypted;
    e.key_type = KeyType::IndividualKey;
    e.key = Some(KEY);
    e.frame_counter = 4;
    device.gp_sink_mut().import(e, false).unwrap();
    device
}

fn gpdf(src: u32, counter: u32) -> McpsDataIndication {
    let mut raw = [0u8; 127];
    let n = security::build_gpdf(
        &mut MockMac::new([0; 8]),
        Some(&KEY),
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(src),
            security_level: SecurityLevel::Encrypted,
            individual_key: true,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: counter,
        },
        counter as u8,
        0x22,
        &[],
        &mut raw,
    )
    .unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        lqi: 180,
        payload: MacFrame::from_slice(&raw[7..n])
            .unwrap()
            .with_mac_sequence(counter as u8),
        security_use: false,
    }
}

/// The NWK frames `from` sent since the last call, as `to` receives them.
fn deliver(
    from: &mut ZigbeeDevice<MockMac, Router>,
    src: ShortAddress,
) -> heapless::Vec<McpsDataIndication, 4> {
    let mut out = heapless::Vec::new();
    for tx in from.mac().tx_history() {
        let _ = out.push(McpsDataIndication {
            src_address: MacAddress::Short(PAN, src),
            dst_address: MacAddress::Short(PAN, ShortAddress(0xFFFF)),
            lqi: 200,
            payload: MacFrame::from_slice(tx.payload.as_slice()).unwrap(),
            security_use: false,
        });
    }
    from.mac_mut().clear_tx_history();
    out
}

/// GP cluster commands (ZCL frame control, command, payload) in `frames`.
fn gp_commands(
    reader: &mut ZigbeeDevice<MockMac, Router>,
    frames: &[McpsDataIndication],
) -> heapless::Vec<(u8, u8, heapless::Vec<u8, 96>), 4> {
    let mut out = heapless::Vec::new();
    for f in frames {
        let nwk: heapless::Vec<u8, 128> = heapless::Vec::from_slice(f.payload.as_slice()).unwrap();
        let Some(zigbee_nwk::nlde::NwkIndication::Owned(aps)) = block_on(
            reader
                .bdb
                .zdo_mut()
                .aps_mut()
                .nwk_mut()
                .process_incoming_nwk_frame(&nwk, 200),
        ) else {
            continue;
        };
        let p = &aps.payload;
        let (header, cluster) = match p[0] & 0x0C {
            0x0C => (9, u16::from_le_bytes([p[3], p[4]])),
            _ => (8, u16::from_le_bytes([p[2], p[3]])),
        };
        if cluster == 0x0021 {
            let z = &p[header..];
            let _ = out.push((z[0], z[2], heapless::Vec::from_slice(&z[3..]).unwrap()));
        }
    }
    out
}

fn wait(device: &mut ZigbeeDevice<MockMac, Router>, ms: u32) {
    block_on(device.mac_mut().delay_micros(ms * 1_000));
}

#[test]
fn a_proxy_without_an_entry_finds_the_pairing() {
    let mut proxy = new_proxy();
    let mut sink = new_sink();
    // Commissioning frames are not searched for (outside commissioning
    // mode they are simply dropped); a data frame of an unknown GPD is.
    assert!(block_on(proxy.process_incoming(&gpdf(SRC, 5), &mut [])).is_none());
    assert!(proxy.gp_proxy().entries().is_empty());
    block_on(proxy.gp_poll());
    let sent = deliver(&mut proxy, PROXY);
    // A second reader decodes what the sink will receive.
    let mut reader = new_sink();
    let commands = gp_commands(&mut reader, &sent);
    assert_eq!(commands.len(), 1);
    let (fc, command, payload) = &commands[0];
    assert_eq!(*fc & 0x0F, 0x01, "cluster specific, client to server");
    assert_eq!(*command, server_cmd::PAIRING_SEARCH);
    let search = PairingSearch::decode(payload).unwrap();
    assert_eq!(search.gpd_id, GpdId::SrcId(SRC));
    assert!(search.request_derived_groupcast && search.request_key);

    // The sink answers to the proxy with GP Pairing.
    for frame in &sent {
        let _ = block_on(sink.process_incoming(frame, &mut []));
    }
    block_on(sink.gp_poll());
    let answer = deliver(&mut sink, ShortAddress::COORDINATOR);
    let commands = gp_commands(&mut new_proxy(), &answer);
    assert_eq!(commands.len(), 1);
    assert_eq!(commands[0].1, client_cmd::PAIRING);
    for frame in &answer {
        let _ = block_on(proxy.process_incoming(frame, &mut []));
    }
    assert_eq!(proxy.gp_proxy().entries().len(), 1);
    assert_eq!(proxy.gp_proxy().entries()[0].frame_counter, 4);

    // The next GPDF is tunnelled to the sinks.
    assert!(block_on(proxy.process_incoming(&gpdf(SRC, 6), &mut [])).is_none());
    wait(&mut proxy, 150);
    block_on(proxy.gp_poll());
    let tunnelled = deliver(&mut proxy, PROXY);
    let commands = gp_commands(&mut new_sink(), &tunnelled);
    assert_eq!(commands.len(), 1);
    assert_eq!(commands[0].1, server_cmd::NOTIFICATION);
}

#[test]
fn searches_are_rate_limited() {
    let mut proxy = new_proxy();
    let searches = |proxy: &mut ZigbeeDevice<MockMac, Router>| {
        block_on(proxy.gp_poll());
        let sent = deliver(proxy, PROXY);
        gp_commands(&mut new_sink(), &sent)
            .iter()
            .filter(|(_, c, _)| *c == server_cmd::PAIRING_SEARCH)
            .count()
    };
    let _ = block_on(proxy.process_incoming(&gpdf(0x0055_0001, 1), &mut []));
    assert_eq!(searches(&mut proxy), 1);
    // Another unknown GPD within the gap: none.
    let _ = block_on(proxy.process_incoming(&gpdf(0x0055_0002, 1), &mut []));
    assert_eq!(searches(&mut proxy), 0);
    wait(&mut proxy, SEARCH_GAP_MS + 1);
    let _ = block_on(proxy.process_incoming(&gpdf(0x0055_0002, 2), &mut []));
    assert_eq!(searches(&mut proxy), 1);
    // The first GPD again: after the gap but within the repeat interval.
    wait(&mut proxy, SEARCH_GAP_MS + 1);
    let _ = block_on(proxy.process_incoming(&gpdf(0x0055_0001, 3), &mut []));
    assert_eq!(searches(&mut proxy), 0);
    wait(&mut proxy, SEARCH_REPEAT_MS);
    let _ = block_on(proxy.process_incoming(&gpdf(0x0055_0001, 4), &mut []));
    assert_eq!(searches(&mut proxy), 1);
}
