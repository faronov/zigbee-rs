# Glossary

Zigbee and IEEE 802.15.4 terminology used throughout zigbee-rs.

| Term | Definition |
|------|------------|
| **APS** | Application Support Sub-layer. Provides addressing, binding, group management, and reliable delivery between application endpoints. Implemented in the `zigbee-aps` crate. |
| **BDB** | Base Device Behavior. Defines standard commissioning procedures such as steering, formation, Finding & Binding, and optional Touchlink. Required capabilities depend on the logical device role and advertised commissioning support. Implemented in the `zigbee-bdb` crate. |
| **Binding** | A persistent link in the APS binding table that maps a local cluster to a remote device or group. Bindings enable indirect addressing so an application can send data without knowing the destination address at compile time. |
| **Channel** | One of the sixteen IEEE 802.15.4 radio channels (11–26) in the 2.4 GHz band. Zigbee PRO primarily uses channels 11, 15, 20, and 25 for network formation. |
| **Cluster** | A ZCL construct grouping related attributes and commands (e.g. On/Off, Temperature Measurement). Each cluster has a 16-bit ID and is hosted on an endpoint. Defined and parsed in the `zigbee-zcl` crate. |
| **Commissioning** | The process of getting a device onto a network and configured. BDB defines four methods: network steering, network formation, Finding & Binding, and Touchlink. |
| **Coordinator** | The device that forms the Zigbee network, assigns the PAN ID, and often acts as the Trust Center. It always has short address `0x0000`. |
| **ED (End Device)** | A Zigbee device that does not route traffic. It communicates only through its parent (a router or the coordinator) and may sleep to save power. See also *SED*. |
| **Endpoint** | A numbered application-level port (1–240) on a Zigbee node. Each endpoint hosts a set of input and output clusters. Endpoint 0 is reserved for the ZDO. |
| **Extended PAN ID** | A 64-bit IEEE address used to uniquely identify a Zigbee network, distinguishing it from other PANs that may share the same 16-bit PAN ID. |
| **FFD** | Full-Function Device. An IEEE 802.15.4 device capable of acting as a PAN coordinator or router. Zigbee coordinators and routers are FFDs. |
| **Formation** | The BDB commissioning step where a coordinator creates a new Zigbee network by selecting a channel and PAN ID, then starting the network. |
| **Green Power** | A Zigbee feature allowing ultra-low-power devices (e.g. energy-harvesting switches) to transmit without joining the network. Green Power frames are proxied by nearby routers. |
| **Group** | A 16-bit multicast address. Devices that belong to the same group all receive frames sent to that group ID, enabling one-to-many communication within a cluster. |
| **IEEE Address** | The globally unique 64-bit hardware address of a Zigbee device (also called the EUI-64 or extended address). Used during joining and for link-key establishment. |
| **Install Code** | A per-device secret (typically printed on the device or packaging) used to derive a unique link key during commissioning. Provides out-of-band security for Trust Center joining. |
| **Link Key** | A 128-bit AES key shared between two specific devices for APS-level encryption. Can be derived from an install code or provisioned by the Trust Center. |
| **MAC** | Medium Access Control. The lowest layer in zigbee-rs, responsible for frame formatting, CSMA-CA channel access, acknowledgements, and scanning. Implemented in the `zigbee-mac` crate with per-platform radio backends. |
| **Network Key** | A 128-bit AES key shared by all devices in the Zigbee network. Provides NWK-layer encryption and is distributed (encrypted) by the Trust Center. |
| **NIB** | NWK Information Base. A set of attributes maintained by the NWK layer (e.g. short address, PAN ID, security material). Accessed via `NwkGet` / `NwkSet` primitives. |
| **NWK** | Network layer. Handles mesh routing, 16-bit address assignment, broadcast, and network-level security. Implemented in the `zigbee-nwk` crate. |
| **OTA** | Over-The-Air upgrade. A ZCL cluster (0x0019) that allows firmware images to be distributed wirelessly. Parsed by `zigbee-zcl` and applied via `zigbee-runtime`'s firmware writer. |
| **PAN** | Personal Area Network. The logical Zigbee network formed by a coordinator and all devices that have joined it. |
| **PAN ID** | A 16-bit identifier for a PAN, used in MAC frame headers to distinguish traffic from overlapping networks. |
| **PIB** | PAN Information Base. A set of MAC-layer attributes (e.g. current channel, short address, frame counter) defined by IEEE 802.15.4. |
| **Poll Control** | A ZCL cluster (0x0020) that lets a server manage when a sleepy end device polls its parent. Useful for battery-powered devices that need timely command delivery. |
| **Profile** | A Zigbee application profile defining which clusters a device type must support. Zigbee 3.0 uses a single unified profile (HA profile `0x0104`). |
| **RFD** | Reduced-Function Device. An IEEE 802.15.4 device that cannot route or act as a coordinator — it can only be an end device. |
| **Router** | A Zigbee device that participates in mesh routing, relays frames for other devices, and can allow new devices to join through it. Routers are always powered on. |
| **SED (Sleepy End Device)** | An end device that spends most of its time asleep to conserve battery. It wakes periodically to poll its parent for pending messages. |
| **Short Address** | A 16-bit network address assigned to a device when it joins the network. Used in NWK and MAC frame headers for compact addressing. |
| **Steering** | The BDB commissioning step where a device scans for open networks, joins one, and authenticates with the Trust Center. |
| **Touchlink** | A proximity-based commissioning mechanism (BDB chapter 8). A device physically close to a Touchlink initiator can be commissioned without an existing network. |
| **Trust Center** | The device (usually the coordinator) responsible for network security policy: distributing the network key, authorising joins, and managing link keys. |
| **ZCL** | Zigbee Cluster Library. Defines the standard set of clusters, attributes, commands, and data types used by Zigbee applications. Implemented in the `zigbee-zcl` crate. |
| **ZDO** | Zigbee Device Object. The management entity on endpoint 0 that handles device and service discovery, binding, and network management. Implemented in the `zigbee-zdo` crate. |
| **ZDP** | Zigbee Device Profile. The protocol (request/response commands) used to communicate with the ZDO on a remote device. `ZdpStatus` codes are returned in every ZDP response. |
| **Zigbee PRO** | The Zigbee PRO feature set, including mesh networking, frequency agility, and stochastic addressing. This branch targets Zigbee Core R22 (`05-3474-22`); later R23/BDB 3.1 material is future guidance, not the current baseline. |
