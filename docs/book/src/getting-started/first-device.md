# Building Your First Device

This chapter shows how to add a product without creating another platform
application state machine.

## Choose the role first

- Battery sensor: `sensor_sed_app::SensorApp`.
- Forwarding-only mains device: `router_app::RelayRouterApp`.
- Child-admitting router: `router_app::ParentRouterApp` and a real
  `ParentMacDriver`.
- Coordinator: `router_app::CoordinatorApp` and a real `ParentMacDriver`.

Do not select a router role for a battery sensor. Do not select a parent role
for a MAC backend that lacks association responses, pending transactions, and
indirect delivery.

## Create the four ownership layers

```text
<chip>-hal/                 generic chip mechanisms
boards/<board>/             fitted pins and peripherals
products/<product>/         identity, profile, policy, layout, storage, OTA
examples/<product-role>/    composition root
```

### 1. Chip HAL and MAC

Supply:

- a real monotonic clock;
- bounded delays and interrupt handling;
- exclusive GPIO/bus/flash resources;
- raw IEEE 802.15.4 TX/RX with valid FCS handling;
- a `MacDriver` implementation;
- `ParentMacDriver` only if the backend really supports parent operations.

Bring up raw TX/RX, scan, association, ACK timing, polling, and addressed data
before debugging the full Zigbee stack.

### 2. Board resources

The board crate maps physical wiring:

```rust,ignore
pub struct BoardResources {
    pub led0: Led0,
    pub button0: Button0,
    pub sensor_bus: SensorBus,
    pub storage: OnChipFlash,
}
```

It should consume peripheral tokens once and expose typed ownership. It does
not choose manufacturer identity, clusters, battery chemistry, persistence
addresses, or OTA policy.

### 3. Product profile and policy

The product crate owns:

- manufacturer/model and software version;
- endpoint/profile/cluster composition;
- reporting defaults and measurement conversion;
- battery chemistry;
- fast/slow poll and wait-depth policy;
- protected flash partitions and linker script;
- persistence and OTA/bootloader selection.

For a non-OTA environmental profile, explicitly implement the non-OTA marker
through one of the shared `NonOtaComponent` profile archetypes. Do not pair
`NoOta` with a profile that advertises the OTA cluster.

### 4. Narrow adapters

Implement only the capabilities the shared application requests:

```rust,ignore
SensorSedParts {
    wake: MyWakeController::new(timer, button),
    status: MyStatusLed::new(led),
    environment: MySensor::new(i2c),
    battery: MyBattery::new(adc),
    ota: NoOta,
    actions: product::policy::USER_ACTIONS,
    supervisor: MySupervisor,
    diagnostics: MyDiagnostics,
}
```

`WakeController::wait` must preserve the atomic transition:

1. reject unsupported depth or prepare the MAC/radio;
2. enter the bounded wait;
3. restore required clocks, calibration, radio, timers, and errata state;
4. return only when normal MAC operations are safe again.

Do not add a broad platform provider or runtime peripheral lookup.

## Compose the sensor

```rust,ignore
let mut profile = product::profile::sensor_profile();
let mut device = ZigbeeDevice::builder(mac)
    .power_mode(product::policy::SENSOR_POLICY.power_mode())
    .automatic_polling(false)
    .manufacturer(product::MANUFACTURER)
    .model(product::MODEL)
    .endpoint(
        profile.endpoint(),
        profile.profile_id(),
        profile.device_id(),
        |endpoint| profile.configure_endpoint(endpoint),
    )
    .build();

let mut security_store = product::storage::security_store(flash)?;
device.reset_security_state_if_identity_changed(&mut security_store, ieee)?;

let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
let mut app = SensorApp::new(node, &product::policy::SENSOR_POLICY, parts)?;
app.run().await
```

For an outer scheduler or reset-on-wake target, call `initialize()` once and
then `step()` explicitly.

## Compose a router

```rust,ignore
let mut app = RelayRouterApp::new(
    ZigbeeNode::new(&mut relay_device, &mut security_store, &mut profile),
    NoChildren,
    &product::policy::ROUTER_POLICY,
    RouterParts::new(status, supervisor, diagnostics),
)?;
```

Use `ParentRouterApp` only after constructing a `ZigbeeDevice<_, Router>` from
a `ParentMacDriver` and a separate child-table store. Use `CoordinatorApp`
when formation and persisted-PAN restart are the intended product behavior.

## Persistence checklist

- The product linker script and Rust partition wrapper must describe the same
  bounds.
- Preserve bootloader, factory identity/calibration, security journal,
  child-table, and OTA regions.
- Use `SecurityStateJournal` for keys and outgoing counter reservations.
- Keep child-table persistence separate from security counters.
- Test erased flash, invalid CRC, interrupted writes, rollover, reset/resume,
  and identity change.

## Validation gates

1. startup, clocks, timer, identity, linker, and SRAM;
2. raw IEEE 802.15.4 TX/RX;
3. scan and beacons;
4. association, ACKs, poll, and addressed data;
5. reusable `MacDriver`;
6. Zigbee security and BDB commissioning;
7. ZDO interview and ZCL reporting;
8. persistence and reset/rejoin;
9. low-power operation with measured current;
10. OTA install, reboot, and retained commissioned state.

Document the highest gate actually passed. Do not describe a compiled
retention path, flash driver, coordinator, or OTA writer as hardware-supported
before its acceptance test.
