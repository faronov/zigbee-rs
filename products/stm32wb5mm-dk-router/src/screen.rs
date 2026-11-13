//! Text pages of the DK's 128x64 OLED (21 columns x 6 rows of a 6x10
//! font). The composition root takes a [`NetworkSnapshot`] and a
//! [`SensorSnapshot`] once per second and draws the [`render`]ed lines;
//! nothing here touches the stack or the display, so the layout is host
//! tested. Integer formatting only: no float formatting code is linked.

use core::fmt::Write;

use router_app::RouterStatus;

pub const COLUMNS: usize = 21;
pub const ROWS: usize = 6;

/// One display row, truncated at [`COLUMNS`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Line {
    bytes: [u8; COLUMNS],
    len: usize,
}

impl Line {
    pub const EMPTY: Self = Self {
        bytes: [b' '; COLUMNS],
        len: 0,
    };

    pub fn as_str(&self) -> &str {
        // Only ASCII is ever written (see `write_str`).
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Debug for Line {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            if self.len == COLUMNS {
                break;
            }
            self.bytes[self.len] = if c.is_ascii() && !c.is_ascii_control() {
                c as u8
            } else {
                b'?'
            };
            self.len += 1;
        }
        Ok(())
    }
}

/// Network state for the display, filled from the NWK NIB, the neighbour
/// and routing tables and the MAC counters.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct NetworkSnapshot {
    pub state: State,
    pub short_address: u16,
    pub pan_id: u16,
    pub channel: u8,
    /// Extended PAN ID as stored (little-endian octets).
    pub extended_pan_id: [u8; 8],
    /// Parent short address and its link quality, if any.
    pub parent: Option<(u16, u8)>,
    pub neighbours: u8,
    pub children: u8,
    pub routes: u8,
    /// Remaining local permit-join seconds; `Some(0xFF)` is indefinite.
    pub permit_join_secs: Option<u8>,
    pub nwk_frame_counter: u32,
    pub tx_ok: u32,
    pub tx_no_ack: u32,
    pub tx_cca: u32,
    pub tx_other: u32,
    pub rx_dropped: u32,
    pub indirect_delivered: u32,
    pub indirect_handed_off: u32,
    pub uptime_secs: u32,
}

impl NetworkSnapshot {
    pub const EMPTY: Self = Self {
        state: State::Booting,
        short_address: 0,
        pan_id: 0,
        channel: 0,
        extended_pan_id: [0; 8],
        parent: None,
        neighbours: 0,
        children: 0,
        routes: 0,
        permit_join_secs: None,
        nwk_frame_counter: 0,
        tx_ok: 0,
        tx_no_ack: 0,
        tx_cca: 0,
        tx_other: 0,
        rx_dropped: 0,
        indirect_delivered: 0,
        indirect_handed_off: 0,
        uptime_secs: 0,
    };
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum State {
    #[default]
    Booting,
    Searching,
    Online,
    Resetting,
    Fault,
}

impl State {
    pub const fn from_status(status: RouterStatus) -> Self {
        match status {
            RouterStatus::Starting { .. } => Self::Booting,
            RouterStatus::Commissioning { .. }
            | RouterStatus::Recommissioning { .. }
            | RouterStatus::Rejoining { .. } => Self::Searching,
            RouterStatus::Online { .. } => Self::Online,
            RouterStatus::Resetting { .. } => Self::Resetting,
            RouterStatus::Fault { .. } => Self::Fault,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Booting => "BOOT",
            Self::Searching => "SEARCH",
            Self::Online => "ONLINE",
            Self::Resetting => "RESET",
            Self::Fault => "FAULT",
        }
    }
}

/// Latest sensor values. `None` = sensor not enabled, not responding, or
/// no reading yet.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SensorSnapshot {
    pub temperature_centi_c: Option<i16>,
    /// ToF sensor: `Some(None)` = working, but no target within range.
    pub distance_mm: Option<Option<u16>>,
    /// Inclination from vertical; `None` also while the board accelerates.
    pub inclination_centi_deg: Option<u16>,
    /// IMU motion; `None` = IMU not working.
    pub occupied: Option<bool>,
    pub sound_centi_db: Option<i16>,
}

impl SensorSnapshot {
    pub const EMPTY: Self = Self {
        temperature_centi_c: None,
        distance_mm: None,
        inclination_centi_deg: None,
        occupied: None,
        sound_centi_db: None,
    };
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    #[default]
    Network,
    Traffic,
    Sensors,
}

impl Page {
    pub const fn next(self) -> Self {
        match self {
            Self::Network => Self::Traffic,
            Self::Traffic => Self::Sensors,
            Self::Sensors => Self::Network,
        }
    }
}

pub fn render(page: Page, net: &NetworkSnapshot, sensors: &SensorSnapshot) -> [Line; ROWS] {
    let mut lines = [Line::EMPTY; ROWS];
    let _ = match page {
        Page::Network => network(&mut lines, net),
        Page::Traffic => traffic(&mut lines, net),
        Page::Sensors => sensor_page(&mut lines, sensors),
    };
    lines
}

fn network(l: &mut [Line; ROWS], net: &NetworkSnapshot) -> core::fmt::Result {
    write!(
        l[0],
        "ZR {:<6} 0x{:04X}",
        net.state.label(),
        net.short_address
    )?;
    write!(l[1], "PAN 0x{:04X} ch {}", net.pan_id, net.channel)?;
    l[2].write_str("X ")?;
    for octet in net.extended_pan_id.iter().rev() {
        write!(l[2], "{octet:02X}")?;
    }
    match net.parent {
        Some((address, lqi)) => write!(l[3], "par 0x{address:04X} lqi {lqi}")?,
        None => l[3].write_str("par -")?,
    }
    write!(
        l[4],
        "nb {} ch {} rt {}",
        net.neighbours, net.children, net.routes
    )?;
    match net.permit_join_secs {
        Some(0xFF) => l[5].write_str("pj on  ")?,
        Some(secs) => write!(l[5], "pj {secs:<3}s ")?,
        None => l[5].write_str("pj off ")?,
    }
    uptime(&mut l[5], net.uptime_secs)
}

fn traffic(l: &mut [Line; ROWS], net: &NetworkSnapshot) -> core::fmt::Result {
    write!(l[0], "TRAFFIC  0x{:04X}", net.short_address)?;
    write!(l[1], "tx ok {}", net.tx_ok)?;
    write!(
        l[2],
        "noack {} cca {} o {}",
        net.tx_no_ack, net.tx_cca, net.tx_other
    )?;
    write!(
        l[3],
        "ind {}/{} rxdrop {}",
        net.indirect_delivered, net.indirect_handed_off, net.rx_dropped
    )?;
    write!(l[4], "nwk fc {}", net.nwk_frame_counter)?;
    l[5].write_str("up ")?;
    uptime(&mut l[5], net.uptime_secs)
}

fn sensor_page(l: &mut [Line; ROWS], s: &SensorSnapshot) -> core::fmt::Result {
    l[0].write_str("SENSORS")?;
    l[1].write_str("temp ")?;
    match s.temperature_centi_c {
        Some(t) => {
            centi(&mut l[1], i32::from(t))?;
            l[1].write_str(" C")?
        }
        None => l[1].write_str("---")?,
    }
    match s.distance_mm {
        Some(Some(mm)) => write!(l[2], "dist {mm} mm")?,
        Some(None) => l[2].write_str("dist --- no target")?,
        None => l[2].write_str("dist ---")?,
    }
    l[3].write_str("tilt ")?;
    match s.inclination_centi_deg {
        Some(t) => {
            let t = u32::from(t) + 5;
            write!(l[3], "{}.{} deg", t / 100, (t / 10) % 10)?
        }
        None => l[3].write_str("---")?,
    }
    match s.occupied {
        Some(true) => l[4].write_str("motion yes")?,
        Some(false) => l[4].write_str("motion no")?,
        None => l[4].write_str("motion ---")?,
    }
    l[5].write_str("sound ")?;
    match s.sound_centi_db {
        Some(db) => {
            let db = i32::from(db);
            let tenths = (db + if db < 0 { -5 } else { 5 }) / 10;
            let sign = if tenths < 0 { "-" } else { "" };
            write!(
                l[5],
                "{sign}{}.{} dB",
                tenths.unsigned_abs() / 10,
                tenths.unsigned_abs() % 10
            )?
        }
        None => l[5].write_str("---")?,
    }
    Ok(())
}

fn centi(line: &mut Line, value: i32) -> core::fmt::Result {
    let sign = if value < 0 { "-" } else { "" };
    let abs = value.unsigned_abs();
    write!(line, "{sign}{}.{:02}", abs / 100, abs % 100)
}

fn uptime(line: &mut Line, secs: u32) -> core::fmt::Result {
    let (d, h, m, s) = (
        secs / 86_400,
        (secs / 3_600) % 24,
        (secs / 60) % 60,
        secs % 60,
    );
    if d > 0 {
        write!(line, "{d}d{h:02}:{m:02}")
    } else {
        write!(line, "{h:02}:{m:02}:{s:02}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text(lines: &[Line; ROWS]) -> [&str; ROWS] {
        core::array::from_fn(|i| lines[i].as_str())
    }

    fn online() -> NetworkSnapshot {
        NetworkSnapshot {
            state: State::Online,
            short_address: 0xC1E6,
            pan_id: 0xDFE9,
            channel: 15,
            extended_pan_id: [0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11],
            parent: Some((0x0000, 255)),
            neighbours: 3,
            children: 1,
            routes: 2,
            permit_join_secs: Some(120),
            nwk_frame_counter: 123_456,
            tx_ok: 442,
            tx_no_ack: 0,
            tx_cca: 8,
            tx_other: 0,
            rx_dropped: 1,
            indirect_delivered: 4,
            indirect_handed_off: 5,
            uptime_secs: 93_784,
        }
    }

    #[test]
    fn network_page() {
        let lines = render(Page::Network, &online(), &SensorSnapshot::default());
        assert_eq!(
            text(&lines),
            [
                "ZR ONLINE 0xC1E6",
                "PAN 0xDFE9 ch 15",
                "X 1122334455667788",
                "par 0x0000 lqi 255",
                "nb 3 ch 1 rt 2",
                "pj 120s 1d02:03",
            ]
        );
    }

    #[test]
    fn traffic_page() {
        let lines = render(Page::Traffic, &online(), &SensorSnapshot::default());
        assert_eq!(
            text(&lines),
            [
                "TRAFFIC  0xC1E6",
                "tx ok 442",
                "noack 0 cca 8 o 0",
                "ind 4/5 rxdrop 1",
                "nwk fc 123456",
                "up 1d02:03",
            ]
        );
    }

    #[test]
    fn sensor_page_with_and_without_values() {
        let sensors = SensorSnapshot {
            temperature_centi_c: Some(-512),
            distance_mm: Some(Some(123)),
            inclination_centi_deg: Some(8_893),
            occupied: Some(true),
            sound_centi_db: Some(6_214),
        };
        assert_eq!(
            text(&render(Page::Sensors, &online(), &sensors)),
            [
                "SENSORS",
                "temp -5.12 C",
                "dist 123 mm",
                "tilt 88.9 deg",
                "motion yes",
                "sound 62.1 dB",
            ]
        );
        assert_eq!(
            text(&render(
                Page::Sensors,
                &online(),
                &SensorSnapshot::default()
            )),
            [
                "SENSORS",
                "temp ---",
                "dist ---",
                "tilt ---",
                "motion ---",
                "sound ---",
            ]
        );
        let no_target = SensorSnapshot {
            distance_mm: Some(None),
            ..SensorSnapshot::EMPTY
        };
        assert_eq!(
            render(Page::Sensors, &online(), &no_target)[2].as_str(),
            "dist --- no target"
        );
    }

    #[test]
    fn offline_and_closed_states_and_short_uptime() {
        let net = NetworkSnapshot {
            state: State::Searching,
            parent: None,
            permit_join_secs: None,
            uptime_secs: 3_725,
            ..NetworkSnapshot::EMPTY
        };
        let lines = render(Page::Network, &net, &SensorSnapshot::default());
        assert_eq!(lines[0].as_str(), "ZR SEARCH 0x0000");
        assert_eq!(lines[3].as_str(), "par -");
        assert_eq!(lines[5].as_str(), "pj off 01:02:05");
    }

    #[test]
    fn every_line_fits_and_pages_cycle() {
        let net = NetworkSnapshot {
            tx_ok: u32::MAX,
            tx_no_ack: u32::MAX,
            tx_cca: u32::MAX,
            nwk_frame_counter: u32::MAX,
            uptime_secs: u32::MAX,
            ..online()
        };
        for page in [Page::Network, Page::Traffic, Page::Sensors] {
            for line in render(page, &net, &SensorSnapshot::default()) {
                assert!(line.as_str().len() <= COLUMNS);
            }
        }
        assert_eq!(Page::Network.next().next().next(), Page::Network);
        let mut line = Line::EMPTY;
        let _ = line.write_str("°ü");
        assert_eq!(line.as_str(), "??");
    }
}
