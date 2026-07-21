# Mock Dimmable Light

Simulates a mains-powered dimmable light (Zigbee router) using `MockMac`.
Demonstrates On/Off and Level Control cluster command handling with visual
brightness feedback. No hardware required.

## What It Demonstrates

- **Network join as router** — FFD, mains-powered, rx-on-when-idle
- **Device template** — `templates::dimmable_light()` (HA Device ID `0x0101`)
- **On/Off cluster** — `CMD_ON`, `CMD_OFF`, `CMD_TOGGLE` command handling
- **Level Control cluster** — `CMD_MOVE_TO_LEVEL` with transition times, `CMD_STEP` (up/down)
- **Visual state** — brightness bar rendered after each command (💡 / ⚫)
- **Attribute reading** — On/Off state, GlobalSceneControl, CurrentLevel, MinLevel, MaxLevel

## Build & Run

```sh
cargo run -p mock-light
```

## Expected Output

1. Joins network on channel 15, PAN `0x1A62`, assigned short address `0x5E3D`
2. Builds dimmable light device (endpoint 1: Basic, Identify, Groups, Scenes, On/Off, Level Control)
3. Executes On/Off commands: ON → TOGGLE → TOGGLE → OFF
4. Executes Level Control commands: MoveToLevel(50) → MoveToLevel(254) → Step(up, 30) → Step(down, 100) → MoveToLevel(10)
5. Reads all On/Off and Level Control cluster attributes
6. Prints MAC capabilities (coordinator, router, max_payload)

## Project Structure

```
mock-light/
├── Cargo.toml      # Dependencies: zigbee-*, pollster
└── src/
    └── main.rs     # Light simulation (~290 lines)
```

## Dependencies

| Crate | Purpose |
|---|---|
| `zigbee-mac` (mock) | MockMac, scan + associate |
| `zigbee-zcl` | OnOffCluster, LevelControlCluster, typed cluster/device IDs |
| `zigbee-runtime` | DeviceBuilder, templates |
| `zigbee-types` | IeeeAddress, PanId, ChannelMask |
| `pollster` | Block on async MAC calls |
