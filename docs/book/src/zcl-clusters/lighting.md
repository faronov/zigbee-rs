# Lighting Clusters

Lighting clusters control color and ballast configuration for smart lights.

---

## Color Control (0x0300)

The most complex cluster in zigbee-rs. Supports three color models (Hue/Saturation, CIE XY, and Color Temperature) plus enhanced hue and color loop functionality. All color transitions are managed by a built-in `TransitionManager`.

### Attributes

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| CurrentHue | `0x0000` | U8 | Report | Hue (0–254) |
| CurrentSaturation | `0x0001` | U8 | Report | Saturation (0–254) |
| RemainingTime | `0x0002` | U16 | Read | Transition time remaining (1/10s) |
| CurrentX | `0x0003` | U16 | Report | CIE x chromaticity (0–65279) |
| CurrentY | `0x0004` | U16 | Report | CIE y chromaticity (0–65279) |
| ColorTemperatureMireds | `0x0007` | U16 | Report | Color temp in mireds |
| ColorMode | `0x0008` | Enum8 | Read | Active mode (0=Hue/Sat, 1=XY, 2=Temp) |
| Options | `0x000F` | Bitmap8 | R/W | Processing flags |
| EnhancedCurrentHue | `0x4000` | U16 | Read | 16-bit enhanced hue |
| EnhancedColorMode | `0x4001` | Enum8 | Read | Enhanced mode indicator |
| ColorLoopActive | `0x4002` | U8 | Read | Color loop running (0/1) |
| ColorLoopDirection | `0x4003` | U8 | Read | Loop direction (0=decrement, 1=increment) |
| ColorLoopTime | `0x4004` | U16 | Read | Loop period in seconds |
| ColorCapabilities | `0x400A` | Bitmap16 | Read | Supported features bitmask |
| ColorTempPhysicalMinMireds | `0x400B` | U16 | Read | Min supported mireds (e.g. 153 = 6500K) |
| ColorTempPhysicalMaxMireds | `0x400C` | U16 | Read | Max supported mireds (e.g. 500 = 2000K) |

### Color Modes

```rust
pub const COLOR_MODE_HUE_SAT: u8 = 0x00;
pub const COLOR_MODE_XY: u8 = 0x01;
pub const COLOR_MODE_TEMPERATURE: u8 = 0x02;
```

### Commands

| ID | Command | Description |
|----|---------|-------------|
| `0x00` | MoveToHue | Transition to target hue |
| `0x01` | MoveHue | Continuous hue movement |
| `0x02` | StepHue | Step hue by increment |
| `0x03` | MoveToSaturation | Transition to target saturation |
| `0x04` | MoveSaturation | Continuous saturation movement |
| `0x05` | StepSaturation | Step saturation by increment |
| `0x06` | MoveToHueAndSaturation | Transition both |
| `0x07` | MoveToColor | Transition to XY color |
| `0x08` | MoveColor | Continuous XY movement |
| `0x09` | StepColor | Step XY by increments |
| `0x0A` | MoveToColorTemperature | Transition to color temp |
| `0x40` | EnhancedMoveToHue | 16-bit hue transition |
| `0x41` | EnhancedMoveHue | 16-bit continuous hue |
| `0x42` | EnhancedStepHue | 16-bit hue step |
| `0x43` | EnhancedMoveToHueAndSaturation | 16-bit hue + sat |
| `0x44` | ColorLoopSet | Start/stop color loop |
| `0x47` | StopMoveStep | Stop all transitions |
| `0x4B` | MoveColorTemperature | Continuous temp movement |
| `0x4C` | StepColorTemperature | Step color temp |

### Usage

```rust
use zigbee_zcl::clusters::color_control::ColorControlCluster;

let mut color = ColorControlCluster::new();
// Default: XY mode, color temp 250 mireds (~4000K), ColorCapabilities 0x001F (all)

// In your timer callback (call every 100ms):
color.tick(1); // advance transitions by 1 decisecond

// Read current state for LED driver:
let hue = color.attributes().get(AttributeId(0x0000)); // CurrentHue
let sat = color.attributes().get(AttributeId(0x0001)); // CurrentSaturation
let temp = color.attributes().get(AttributeId(0x0007)); // ColorTemperature
```

### The Transition Engine

Color Control uses a `TransitionManager<4>` supporting 4 concurrent transitions (hue, saturation, X, Y or color temperature). When a command like `MoveToColor` arrives:

1. The cluster calculates start value, target value, and transition time
2. Starts a `Transition` in the `TransitionManager`
3. Each `tick()` call interpolates the current value linearly
4. Attribute store is updated with the interpolated value
5. `RemainingTime` attribute reflects time left

### Color Loop

The Color Loop engine (`ColorLoopSet` command 0x44) continuously cycles the hue:

- **Active**: ColorLoopActive attribute (0 = off, 1 = running)
- **Direction**: 0 = decrement hue, 1 = increment hue  
- **Time**: Full cycle period in seconds
- `tick()` advances the hue based on elapsed time and loop parameters

---

## Ballast Configuration (0x0301)

Configuration attributes for fluorescent lamp ballasts.

| Attribute | ID | Type | Access | Description |
|-----------|----|------|--------|-------------|
| PhysicalMinLevel | `0x0000` | U8 | Read | Minimum light output |
| PhysicalMaxLevel | `0x0001` | U8 | Read | Maximum light output |
| BallastStatus | `0x0002` | U8 | Read | Status flags |
| MinLevel | `0x0010` | U8 | R/W | Configured minimum |
| MaxLevel | `0x0011` | U8 | R/W | Configured maximum |
| PowerOnLevel | `0x0012` | U8 | R/W | Level at power-on |
| PowerOnFadeTime | `0x0013` | U16 | R/W | Fade time at power-on |
| IntrinsicBallastFactor | `0x0014` | U8 | R/W | Ballast factor |
| BallastFactorAdjustment | `0x0015` | U8 | R/W | Factor adjustment |
| LampQuantity | `0x0020` | U8 | Read | Number of lamps |
| LampRatedHours | `0x0032` | U24 | R/W | Rated lamp life |
| LampBurnHours | `0x0033` | U24 | R/W | Accumulated burn hours |
| LampAlarmMode | `0x0034` | U8 | R/W | Alarm enable bits |
| LampBurnHoursTripPoint | `0x0035` | U24 | R/W | Burn-hours alarm threshold |

`LampType` (0x0030) and `LampManufacturer` (0x0031) have ID constants but are
not registered in the attribute store.

No cluster-specific commands — configuration is done via Write Attributes.

```rust
use zigbee_zcl::clusters::ballast_config::BallastConfigCluster;

let mut ballast = BallastConfigCluster::new();
ballast.set_lamp_burn_hours(1200); // masked to 24 bits
```

---

## Putting It Together: Dimmable Color Light

A typical color light uses On/Off + Level Control + Color Control together:

```rust
use zigbee_zcl::clusters::on_off::OnOffCluster;
use zigbee_zcl::clusters::level_control::LevelControlCluster;
use zigbee_zcl::clusters::color_control::ColorControlCluster;

let mut on_off = OnOffCluster::new();
let mut level = LevelControlCluster::new();
let mut color = ColorControlCluster::new();

// In the 100ms timer:
on_off.tick();
level.tick(1);
color.tick(1);

// Drive the LED:
if on_off.is_on() {
    let brightness = level.current_level();
    // Read color temp from attribute store
    // Apply to LED driver
}
```
