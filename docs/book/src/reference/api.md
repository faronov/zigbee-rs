# API Quick Reference

One-page cheat sheet for the `zigbee-rs` public API, organized by crate.

---

## `zigbee-types` — Core Addressing Types

| Type | Description |
|------|-------------|
| `IeeeAddress` = `[u8; 8]` | 64-bit IEEE/EUI-64 address |
| `ShortAddress(u16)` | 16-bit network address. Constants: `BROADCAST` (0xFFFF), `UNASSIGNED` (0xFFFE), `COORDINATOR` (0x0000) |
| `PanId(u16)` | PAN identifier. Constant: `BROADCAST` (0xFFFF) |
| `MacAddress` | Either `Short(PanId, ShortAddress)` or `Extended(PanId, IeeeAddress)` |
| `Channel` | 2.4 GHz channels Ch11–Ch26. `from_number(u8)`, `number() → u8` |
| `ChannelMask(u32)` | Bitmask of channels. Constants: `ALL_2_4GHZ` (0x07FFF800), `PREFERRED`. Methods: `contains(Channel)`, `iter()` |
| `TxPower(i8)` | Transmit power in dBm |

---

## `zigbee-mac` — MAC Layer Driver

### `MacDriver` Trait

| Method | Description |
|--------|-------------|
| `mlme_scan(MlmeScanRequest) → Result<MlmeScanConfirm>` | Active/energy/orphan scan |
| `mlme_associate(MlmeAssociateRequest) → Result<MlmeAssociateConfirm>` | Associate with coordinator |
| `mlme_associate_response(MlmeAssociateResponse) → Result<()>` | Respond to association request (ZC/ZR) |
| `mlme_disassociate(MlmeDisassociateRequest) → Result<()>` | Leave the network |
| `mlme_reset(set_default_pib: bool) → Result<()>` | Reset MAC sublayer |
| `mlme_start(MlmeStartRequest) → Result<()>` | Start network as coordinator |
| `mlme_get(PibAttribute) → Result<PibValue>` | Read a PIB attribute |
| `mlme_set(PibAttribute, PibValue) → Result<()>` | Write a PIB attribute |
| `mlme_poll() → Result<Option<MacFrame>>` | Poll parent for pending data (ZED) |
| `mcps_data(McpsDataRequest) → Result<McpsDataConfirm>` | Transmit a MAC frame |
| `mcps_data_indication() → Result<McpsDataIndication>` | Receive a MAC frame |
| `capabilities() → MacCapabilities` | Query radio capabilities |

### `MacCapabilities`

| Field | Type | Description |
|-------|------|-------------|
| `coordinator` | `bool` | Can act as PAN coordinator |
| `router` | `bool` | Can route frames |
| `hardware_security` | `bool` | Autonomous MAC-level security offload (not mere silicon AES; a stack that runs CCM* itself, even via a hardware block cipher, reports `false`) |
| `max_payload` | `u16` | Max MAC payload bytes |
| `tx_power_min` / `tx_power_max` | `TxPower` | TX power range |

### `MacError` Variants

`NoBeacon`, `InvalidParameter`, `RadioError`, `ChannelAccessFailure`, `NoAck`, `FrameTooLong`, `Unsupported`, `SecurityError`, `TransactionOverflow`, `TransactionExpired`, `ScanInProgress`, `TrackingOff`, `AssociationDenied`, `PanAtCapacity`, `Other`, `NoData`

### Platform Drivers

Feature-gated MAC implementations: `esp` (ESP32-C6/H2), `nrf` (nRF52840/52833), `bl702`, `cc2340`, `telink`, `phy6222`, `mock` (testing)

---

## `zigbee-nwk` — Network Layer

### `NwkLayer<M: MacDriver>`

| Method | Description |
|--------|-------------|
| `new(mac, device_type) → Self` | Create NWK layer |
| `set_rx_on_when_idle(bool)` | Set RX-on-when-idle (router=true, sleepy ZED=false) |
| `rx_on_when_idle() → bool` | Query RX-on-when-idle |
| `nib() → &Nib` | Read Network Information Base |
| `nib_mut() → &mut Nib` | Write Network Information Base |
| `is_joined() → bool` | Whether device has joined a network |
| `device_type() → DeviceType` | Coordinator / Router / EndDevice |
| `mac() → &M` / `mac_mut() → &mut M` | Access underlying MAC driver |
| `security() → &NwkSecurity` | Read network security state |
| `security_mut() → &mut NwkSecurity` | Write network security state |
| `neighbor_table() → &NeighborTable` | Read neighbor table |
| `routing_table() → &RoutingTable` | Read routing table |
| `find_short_by_ieee(&IeeeAddress) → Option<ShortAddress>` | Resolve IEEE → short address |
| `find_ieee_by_short(ShortAddress) → Option<IeeeAddress>` | Resolve short → IEEE address |
| `update_neighbor_address(ShortAddress, IeeeAddress)` | Update address mapping in neighbor table |

### `DeviceType`

`Coordinator`, `Router`, `EndDevice`

---

## `zigbee-aps` — Application Support Sub-layer

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `ZDO_ENDPOINT` | `0x00` | ZDO endpoint |
| `MIN_APP_ENDPOINT` | `0x01` | First application endpoint |
| `MAX_APP_ENDPOINT` | `0xF0` | Last application endpoint |
| `BROADCAST_ENDPOINT` | `0xFF` | Broadcast to all endpoints |
| `PROFILE_HOME_AUTOMATION` | `0x0104` | HA profile ID |
| `PROFILE_SMART_ENERGY` | `0x0109` | SE profile ID |
| `PROFILE_ZLL` | `0xC05E` | ZLL profile ID |

### `ApsLayer<M: MacDriver>`

| Method | Description |
|--------|-------------|
| `new(nwk) → Self` | Create APS layer wrapping NWK |
| `next_aps_counter() → u8` | Get next APS frame counter |
| `is_aps_duplicate(src_addr, counter) → bool` | Detect duplicate APS frames |
| `age_dup_table()` | Age out old duplicate entries |
| `register_ack_pending(counter, dst, frame) → Option<usize>` | Track pending APS ACK |
| `confirm_ack(src, counter) → bool` | Confirm ACK received |
| `take_ack_status(counter) → Option<bool>` | Consume ACK status |
| `age_ack_table() → Vec<Vec<u8>>` | Age ACKs, return retransmit candidates |
| `nwk() → &NwkLayer<M>` / `nwk_mut()` | Access NWK layer |
| `aib() → &Aib` / `aib_mut()` | Access APS Information Base |
| `binding_table() → &BindingTable` / `binding_table_mut()` | Access binding table |
| `group_table() → &GroupTable` / `group_table_mut()` | Access group table |
| `security() → &ApsSecurity` / `security_mut()` | Access APS security |
| `fragment_rx() → &FragmentReassembly` / `fragment_rx_mut()` | Access fragment reassembly |

### `BindingTable`

| Method | Description |
|--------|-------------|
| `new() → Self` | Create empty binding table |
| `add(entry) → Result<(), BindingEntry>` | Add a binding entry |
| `remove(src, ep, cluster, dst) → bool` | Remove a binding |
| `find_by_source(src, ep, cluster) → Iterator<&BindingEntry>` | Find bindings for a source |
| `find_by_cluster(cluster_id) → Iterator<&BindingEntry>` | Find bindings for a cluster |
| `find_by_endpoint(ep) → Iterator<&BindingEntry>` | Find bindings for an endpoint |
| `entries() → &[BindingEntry]` | All entries |
| `len()`, `is_empty()`, `is_full()`, `clear()` | Capacity management |

### `BindingEntry`

| Constructor | Description |
|-------------|-------------|
| `unicast(src_addr, src_ep, cluster, dst_addr, dst_ep) → Self` | Unicast binding |
| `group(src_addr, src_ep, cluster, group_addr) → Self` | Group binding |

### `GroupTable`

| Method | Description |
|--------|-------------|
| `new() → Self` | Create empty group table |
| `add_group(group_addr, endpoint) → bool` | Add endpoint to group |
| `remove_group(group_addr, endpoint) → bool` | Remove endpoint from group |
| `remove_all_groups(endpoint)` | Remove endpoint from all groups |
| `find(group_addr) → Option<&GroupEntry>` | Find a group entry |
| `is_member(group_addr, endpoint) → bool` | Check group membership |
| `groups() → &[GroupEntry]` | All groups |

---

## `zigbee-zdo` — Zigbee Device Object

### `ZdoLayer<M: MacDriver>`

#### Device & Service Discovery

| Method | Description |
|--------|-------------|
| `simple_desc_req(dst, ep) → Result<SimpleDescriptor>` | Query endpoint descriptor |
| `active_ep_req(dst) → Result<Vec<u8>>` | Query active endpoints |
| `match_desc_req(dst, profile, in_clusters, out_clusters) → Result<Vec<u8>>` | Find matching endpoints |
| `device_annce(nwk_addr, ieee_addr) → Result<()>` | Announce device on network |

#### Binding Management

| Method | Description |
|--------|-------------|
| `bind_req(dst, entry) → Result<()>` | Create remote binding |
| `unbind_req(dst, entry) → Result<()>` | Remove remote binding |

#### Network Management

| Method | Description |
|--------|-------------|
| `mgmt_permit_joining_req(dst, duration, tc) → Result<()>` | Open/close joining |
| `nlme_network_discovery(mask, duration) → Result<Vec<NetworkDescriptor>>` | Scan for networks |
| `nlme_join(network) → Result<ShortAddress>` | Join a network |
| `nlme_rejoin(network) → Result<ShortAddress>` | Rejoin a network |
| `nlme_network_formation(mask, duration) → Result<()>` | Form a new network (ZC) |
| `nlme_permit_joining(duration) → Result<()>` | Set local permit join |
| `nlme_start_router() → Result<()>` | Start router functionality |
| `nlme_reset(warm_start) → Result<()>` | Reset network layer |

#### Descriptor Management

| Method | Description |
|--------|-------------|
| `register_endpoint(SimpleDescriptor) → Result<()>` | Register a local endpoint |
| `endpoints() → &[SimpleDescriptor]` | List registered endpoints |
| `find_endpoint(ep) → Option<&SimpleDescriptor>` | Find endpoint descriptor |
| `set_node_descriptor(NodeDescriptor)` / `node_descriptor()` | Node descriptor access |
| `set_power_descriptor(PowerDescriptor)` / `power_descriptor()` | Power descriptor access |
| `set_local_nwk_addr(ShortAddress)` / `local_nwk_addr()` | Local network address |
| `set_local_ieee_addr(IeeeAddress)` / `local_ieee_addr()` | Local IEEE address |

#### Internal

| Method | Description |
|--------|-------------|
| `new(aps) → Self` | Create ZDO layer wrapping APS |
| `next_seq() → u8` | Next ZDP sequence number |
| `deliver_response(cluster, tsn, payload) → bool` | Deliver a ZDP response |
| `aps() → &ApsLayer<M>` / `aps_mut()` | Access APS layer |
| `nwk() → &NwkLayer<M>` / `nwk_mut()` | Access NWK layer |

---

## `zigbee-bdb` — Base Device Behavior

### `BdbLayer<M: MacDriver>`

| Method | Description |
|--------|-------------|
| `new(zdo) → Self` | Create BDB layer wrapping ZDO |
| `zdo() → &ZdoLayer<M>` / `zdo_mut()` | Access ZDO layer |
| `attributes() → &BdbAttributes` / `attributes_mut()` | BDB commissioning attributes |
| `state() → &BdbState` | Current BDB state machine state |
| `is_on_network() → bool` | Whether device has joined |
| `reset_attributes()` | Reset BDB attributes to defaults |

### `BdbStatus`

`Success`, `InProgress`, `NotOnNetwork`, `NotPermitted`, `NoScanResponse`, `FormationFailure`, `SteeringFailure`, `NoIdentifyResponse`, `BindingTableFull`, `TouchlinkFailure`, `TargetFailure`, `Timeout`

---

## `zigbee-zcl` — Zigbee Cluster Library

### `Cluster` Trait

```rust
pub trait Cluster {
    fn cluster_id(&self) -> ClusterId;
    fn handle_command(&mut self, cmd_id: CommandId, payload: &[u8])
        -> Result<Vec<u8, 64>, ZclStatus>;
    fn attributes(&self) -> &dyn AttributeStoreAccess;
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess;
    fn received_commands(&self) -> Vec<u8, 32>;   // optional
    fn generated_commands(&self) -> Vec<u8, 32>;  // optional
}
```

### `AttributeStoreAccess` / `AttributeStoreMutAccess` Traits

| Method | Description |
|--------|-------------|
| `get(AttributeId) → Option<&ZclValue>` | Read attribute value |
| `set(AttributeId, ZclValue) → Result<()>` | Write attribute with validation |
| `set_raw(AttributeId, ZclValue) → Result<()>` | Write attribute without validation |
| `find(AttributeId) → Option<&AttributeDefinition>` | Find attribute metadata |
| `all_ids() → Vec<AttributeId, 32>` | List all attribute IDs |

### Key Cluster IDs

| ID | Constant | Name |
|----|----------|------|
| `0x0000` | `BASIC` | Basic |
| `0x0001` | `POWER_CONFIG` | Power Configuration |
| `0x0003` | `IDENTIFY` | Identify |
| `0x0004` | `GROUPS` | Groups |
| `0x0005` | `SCENES` | Scenes |
| `0x0006` | `ON_OFF` | On/Off |
| `0x0008` | `LEVEL_CONTROL` | Level Control |
| `0x0019` | `OTA_UPGRADE` | OTA Upgrade |
| `0x0020` | `POLL_CONTROL` | Poll Control |
| `0x0300` | `COLOR_CONTROL` | Color Control |
| `0x0402` | `TEMPERATURE` | Temperature Measurement |
| `0x0405` | `HUMIDITY` | Relative Humidity |
| `0x0406` | `OCCUPANCY` | Occupancy Sensing |
| `0x0500` | `IAS_ZONE` | IAS Zone |
| `0x0702` | `METERING` | Metering |
| `0x0B04` | `ELECTRICAL_MEASUREMENT` | Electrical Measurement |

> See [ZCL Cluster Table](zcl-table.md) for the complete list of all 46 clusters.

### `ReportingEngine`

| Method | Description |
|--------|-------------|
| `new() → Self` | Create empty engine |
| `configure(ReportingConfig) → Result<()>` | Add/update a reporting configuration |
| `configure_for_cluster(ep, cluster, config) → Result<()>` | Configure for specific cluster |
| `tick(elapsed_secs)` | Advance timers |
| `check_and_report(store) → Option<ReportAttributes>` | Check if any reports are due |
| `check_and_report_cluster(ep, cluster, store) → Option<ReportAttributes>` | Check for specific cluster |
| `get_config(ep, cluster, direction, attr) → Option<&ReportingConfig>` | Read reporting config |

---

## `zigbee-runtime` — Device Runtime & Event Loop

### `ZigbeeDevice<M: MacDriver>`

#### Lifecycle

| Method | Description |
|--------|-------------|
| `builder(mac) → DeviceBuilder<M>` | Start building a device |
| `start() → Result<u16, StartError>` | Join network, returns short address |
| `leave() → Result<()>` | Leave the network |
| `factory_reset(nv)` | Factory reset, optionally clear NV storage |
| `user_action(UserAction)` | Inject a user action (Join/Leave/Toggle/PermitJoin/FactoryReset) |

#### State Queries

| Method | Description |
|--------|-------------|
| `is_joined() → bool` | Network join status |
| `short_address() → u16` | Current short address |
| `channel() → u8` | Current channel |
| `pan_id() → u16` | Current PAN ID |
| `device_type() → DeviceType` | Coordinator/Router/EndDevice |
| `endpoints() → &[EndpointConfig]` | Registered endpoints |
| `manufacturer_name() → &str` | Manufacturer string |
| `model_identifier() → &str` | Model string |
| `channel_mask() → ChannelMask` | Configured channel mask |
| `sw_build_id() → &str` | Software build ID |
| `date_code() → &str` | Date code |
| `is_sleepy() → bool` | Whether device is a sleepy end device |

#### Data Path

| Method | Description |
|--------|-------------|
| `receive() → Result<McpsDataIndication>` | Wait for incoming frame |
| `poll() → Result<Option<McpsDataIndication>>` | Poll parent (ZED) |
| `process_incoming(indication, clusters) → Option<StackEvent>` | Process a received frame through the stack |
| `send_zcl_frame(dst, dst_ep, src_ep, cluster, data) → Result<()>` | Send a ZCL frame |

#### Reporting & Persistence

| Method | Description |
|--------|-------------|
| `reporting() → &ReportingEngine` / `reporting_mut()` | Access reporting engine |
| `check_and_send_cluster_reports(ep, cluster, store) → bool` | Check and transmit due reports |
| `save_state(nv)` | Persist device state to NV storage |
| `restore_state(nv) → bool` | Restore state from NV storage |
| `power() → &PowerManager` / `power_mut()` | Access power manager |
| `bdb() → &BdbLayer<M>` / `bdb_mut()` | Access BDB layer |

### `DeviceBuilder<M: MacDriver>`

| Method | Description |
|--------|-------------|
| `new(mac) → Self` | Create builder with MAC driver |
| `device_type(DeviceType) → Self` | Set device type |
| `manufacturer(&'static str) → Self` | Set manufacturer name |
| `model(&'static str) → Self` | Set model identifier |
| `sw_build(&'static str) → Self` | Set software build ID |
| `date_code(&'static str) → Self` | Set date code |
| `channels(ChannelMask) → Self` | Set channel mask |
| `power_mode(PowerMode) → Self` | Set power mode (AlwaysOn/Sleepy/DeepSleep) |
| `endpoint(ep, profile, device_id, configure_fn) → Self` | Add an endpoint |
| `build() → ZigbeeDevice<M>` | Build the device |

### `EndpointBuilder`

| Method | Description |
|--------|-------------|
| `cluster_server(cluster_id) → Self` | Add a server cluster |
| `cluster_client(cluster_id) → Self` | Add a client cluster |
| `device_version(version) → Self` | Set device version |

### Device Templates

Pre-configured `DeviceBuilder` shortcuts in `zigbee_runtime::templates`:

| Template | Description |
|----------|-------------|
| `temperature_sensor(mac)` | Temperature sensor (0x0402) |
| `temperature_humidity_sensor(mac)` | Temp + humidity (0x0402, 0x0405) |
| `on_off_light(mac)` | On/Off light (0x0006) |
| `dimmable_light(mac)` | Dimmable light (0x0006, 0x0008) |
| `color_temperature_light(mac)` | Color temp light (0x0006, 0x0008, 0x0300) |
| `contact_sensor(mac)` | Contact sensor (IAS Zone 0x0500) |
| `occupancy_sensor(mac)` | Occupancy sensor (0x0406) |
| `smart_plug(mac)` | Smart plug (0x0006, 0x0B04, 0x0702) |
| `thermostat(mac)` | Thermostat (0x0201) |

### `StackEvent` Enum

| Variant | Description |
|---------|-------------|
| `Joined { short_address, channel, pan_id }` | Successfully joined network |
| `Left` | Left the network |
| `AttributeReport { src_addr, endpoint, cluster_id, attr_id }` | Received an attribute report |
| `CommandReceived { src_addr, endpoint, cluster_id, command_id, seq_number, payload }` | Received a cluster command |
| `CommissioningComplete { success }` | BDB commissioning finished |
| `DefaultResponse { src_addr, endpoint, cluster_id, command_id, status }` | Received a default response |
| `PermitJoinChanged { open }` | Permit join state changed |
| `ReportSent` | An attribute report was transmitted |
| `OtaImageAvailable { version, size }` | OTA image available |
| `OtaProgress { percent }` | OTA download progress |
| `OtaComplete` / `OtaFailed` | OTA finished |
| `OtaDelayedActivation { delay_secs }` | OTA activation delayed |
| `BasicResetToFactoryDefaults` | Basic cluster application attributes were reset; network/security state is preserved |

### `PowerManager`

| Method | Description |
|--------|-------------|
| `new(PowerMode) → Self` | Create power manager |
| `mode() → PowerMode` | Current power mode |
| `record_activity(now_ms)` | Record activity to prevent premature sleep |
| `record_poll(now_ms)` | Record data poll time |
| `set_pending_tx(bool)` | Mark pending transmission |
| `set_pending_reports(bool)` | Mark pending reports |
| `decide(now_ms) → SleepDecision` | Decide: StayAwake / LightSleep(ms) / DeepSleep(ms) |
| `should_poll(now_ms) → bool` | Whether it's time to poll parent |

### `PowerMode` / `SleepDecision`

| Variant | Description |
|---------|-------------|
| `PowerMode::AlwaysOn` | ZC/ZR — never sleep |
| `PowerMode::Sleepy { poll_interval_ms, wake_duration_ms }` | ZED with periodic polling |
| `PowerMode::DeepSleep { wake_interval_s }` | ZED with deep sleep cycles |
| `SleepDecision::StayAwake` | Don't sleep |
| `SleepDecision::LightSleep(ms)` | Light sleep for N ms |
| `SleepDecision::DeepSleep(ms)` | Deep sleep for N ms |

### `UserAction`

`Join`, `Leave`, `Toggle`, `PermitJoin(u8)`, `FactoryReset`

---

## `zigbee` — Top-Level Crate

Re-exports all sub-crates and provides high-level coordinator/router/trust center components.

### Re-exports

```rust
pub use zigbee_runtime::ZigbeeDevice;
pub use zigbee_types::{Channel, ChannelMask, IeeeAddress, MacAddress, PanId, ShortAddress};
pub use zigbee_aps as aps;
pub use zigbee_bdb as bdb;
pub use zigbee_mac as mac;
pub use zigbee_nwk as nwk;
pub use zigbee_runtime as runtime;
pub use zigbee_types as types;
pub use zigbee_zcl as zcl;
pub use zigbee_zdo as zdo;
```

### `Coordinator`

| Method | Description |
|--------|-------------|
| `new(CoordinatorConfig) → Self` | Create coordinator |
| `generate_network_key()` | Generate random 128-bit network key |
| `network_key() → &[u8; 16]` / `set_network_key(key)` | Network key access |
| `allocate_address() → ShortAddress` | Allocate address for joining device |
| `can_accept_child() → bool` | Check child capacity |
| `is_formed() → bool` / `mark_formed()` | Network formation state |
| `next_frame_counter() → u32` | Next NWK frame counter |

### `CoordinatorConfig`

| Field | Type | Description |
|-------|------|-------------|
| `channel_mask` | `ChannelMask` | Channels to form on |
| `extended_pan_id` | `IeeeAddress` | Extended PAN ID |
| `centralized_security` | `bool` | Use centralized Trust Center |
| `require_install_codes` | `bool` | Require install codes for joining |
| `max_children` | `u8` | Max direct children |
| `max_depth` | `u8` | Max network depth |
| `initial_permit_join_duration` | `u8` | Permit join duration at startup (seconds) |

### `Router`

| Method | Description |
|--------|-------------|
| `new(RouterConfig) → Self` | Create router |
| `add_child(ieee, short, is_ffd, rx_on) → Result<()>` | Register a child device |
| `remove_child(addr)` | Remove a child |
| `find_child(addr) → Option<&ChildDevice>` | Find child by short address |
| `find_child_by_ieee(&ieee) → Option<&ChildDevice>` | Find child by IEEE address |
| `is_child(addr) → bool` | Check if address is a child |
| `can_accept_child() → bool` | Check capacity |
| `age_children(elapsed_seconds)` | Age children, detect timeouts |
| `child_activity(addr)` | Record child activity |
| `child_count() → u8` | Number of children |
| `is_started() → bool` / `mark_started()` | Router started state |

### `TrustCenter`

| Method | Description |
|--------|-------------|
| `new(network_key) → Self` | Create Trust Center with network key |
| `network_key() → &[u8; 16]` / `set_network_key(key)` | Network key access |
| `key_seq_number() → u8` | Current key sequence number |
| `set_require_install_codes(bool)` | Enable/disable install code requirement |
| `link_key_for_device(&ieee) → [u8; 16]` | Get link key for device |
| `set_link_key(ieee, key, TcKeyType) → Result<()>` | Set device-specific link key |
| `remove_link_key(&ieee)` | Remove a link key |
| `mark_key_verified(&ieee)` | Mark key as verified |
| `should_accept_join(&ieee) → bool` | Check join policy for device |
| `update_frame_counter(&ieee, counter) → bool` | Update incoming frame counter |
| `next_frame_counter() → u32` | Next outgoing frame counter |
| `device_count() → usize` | Number of registered devices |
