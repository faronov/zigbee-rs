//! Register-level plan of the nRF52 FEM (external PA/LNA) pin control.
//!
//! The pins are switched by the RADIO's own events through PPI and GPIOTE
//! task-mode channels — never by the CPU on the radio's timeline:
//!
//! ```text
//! RADIO CCAIDLE ─PPI a─▶ GPIOTE PA on   ┬ FORK LNA off   (CCA passed; TXEN via SHORT)
//! TIMER0 CC[0]  ─PPI 20 (fixed)─▶ RADIO TXEN ┬ FORK PA on  (ISR PHY Imm-Ack, TXEN instant)
//! RADIO PHYEND  ─PPI b─▶ GPIOTE PA off                   (last TX bit on air)
//! RADIO RXREADY ─PPI c─▶ GPIOTE LNA on  ┬ FORK PA off    (receiver ramped up)
//! RADIO DISABLED─PPI d─▶ GPIOTE LNA off                  (any path to DISABLED)
//! ```
//!
//! The PA is enabled when TX ramp-up *starts* (CCAIDLE → TXEN short, or
//! the TIMER0 compare that triggers TXEN for an Imm-Ack), so it settles for
//! the whole TX ramp-up — 40 µs with the ISR PHY's fast ramp-up, 130 µs with
//! Embassy's default — before the first preamble symbol. The PA is not
//! dropped on DISABLED because the RX→TX switch of a CCA transmission
//! passes through DISABLED (observed by the ISR PHY) after CCAIDLE; it is
//! dropped on PHYEND, on RXREADY, and in software by every forced-disable
//! path of both nRF PHYs. The LNA is enabled at RXREADY (zero lead: the
//! receiver starts listening at RXREADY) and dropped at DISABLED, CCAIDLE
//! and PHY sleep.
//!
//! Everything here is pure address/value computation so the wiring is
//! host-tested; `nrf::fem` writes the plan to the hardware.

use crate::fem::{FemConfigError, FemTimingLimits};

const RADIO: u32 = 0x4000_1000;
/// RADIO EVENTS_DISABLED.
pub const RADIO_EVENTS_DISABLED: u32 = RADIO + 0x110;
/// RADIO EVENTS_CCAIDLE.
pub const RADIO_EVENTS_CCAIDLE: u32 = RADIO + 0x144;
/// RADIO EVENTS_RXREADY.
pub const RADIO_EVENTS_RXREADY: u32 = RADIO + 0x158;
/// RADIO EVENTS_PHYEND.
pub const RADIO_EVENTS_PHYEND: u32 = RADIO + 0x16C;

const GPIOTE: u32 = 0x4000_6000;
const GPIOTE_TASKS_SET: u32 = GPIOTE + 0x030;
const GPIOTE_TASKS_CLR: u32 = GPIOTE + 0x060;
const GPIOTE_CONFIG: u32 = GPIOTE + 0x510;
/// GPIOTE channels on nRF52833/nRF52840.
pub const GPIOTE_CHANNELS: u8 = 8;

const PPI: u32 = 0x4001_F000;
/// PPI CHENSET.
pub const PPI_CHENSET: u32 = PPI + 0x504;
/// PPI CHENCLR.
pub const PPI_CHENCLR: u32 = PPI + 0x508;
/// Programmable PPI channels (20..=31 are pre-programmed).
pub const PPI_CONFIGURABLE_CHANNELS: u8 = 20;
/// Pre-programmed channel used by the ISR PHY: `TIMER0 COMPARE[0]` → `TXEN`.
pub const PPI_CH_ACK_TXEN: u8 = 20;

const P0: u32 = 0x5000_0000;
const P1: u32 = 0x5000_0300;
const GPIO_OUTSET: u32 = 0x508;
const GPIO_OUTCLR: u32 = 0x50C;
const GPIO_PIN_CNF: u32 = 0x700;
/// PIN_CNF: DIR = output, INPUT = disconnect, no pull, S0S1, no sense.
pub const PIN_CNF_OUTPUT: u32 = 0b11;

/// GPIOTE CONFIG MODE = Task.
const GPIOTE_MODE_TASK: u32 = 3;

/// Timing the PPI scheme honours (see the module docs).
///
/// The PA lead is the TX ramp-up; 40 µs is the shorter (fast ramp-up, ISR
/// PHY) of the two nRF PHYs. The LNA has no lead: it is enabled when the
/// receiver starts listening, so a settle time above zero costs at most the
/// start of a preamble that began within it; one O-QPSK symbol (16 µs) of
/// the 8-symbol preamble is accepted.
pub const NRF_FEM_TIMING: FemTimingLimits = FemTimingLimits {
    max_pa_settle_us: 40,
    max_lna_settle_us: 16,
};

/// IEEE 802.15.4 energy units per dB on nRF52 (`ED_RSSISCALE`): energy and
/// LQI are `4 × value` where `P[dBm] = ED_RSSIOFFS + value`.
pub const NRF_ED_UNITS_PER_DB: u8 = 4;

/// nRF52833/nRF52840 RADIO TXPOWER steps (dBm), ascending.
pub const NRF_TX_POWER_STEPS: &[i8] = &[-40, -30, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8];

/// One FEM control line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FemPin {
    /// GPIO port (0 or 1).
    pub port: u8,
    /// Pin within the port (0..=31).
    pub pin: u8,
    /// `true` when the FEM function is enabled by a high level.
    pub active_high: bool,
}

impl FemPin {
    fn valid(&self) -> bool {
        self.port <= 1 && self.pin < 32
    }

    const fn port_base(&self) -> u32 {
        if self.port == 0 { P0 } else { P1 }
    }

    /// GPIO OUTSET/OUTCLR register that drives the pin to its active level.
    pub const fn active_register(&self) -> u32 {
        self.port_base()
            + if self.active_high {
                GPIO_OUTSET
            } else {
                GPIO_OUTCLR
            }
    }

    /// GPIO OUTSET/OUTCLR register that drives the pin to its inactive level.
    pub const fn inactive_register(&self) -> u32 {
        self.port_base()
            + if self.active_high {
                GPIO_OUTCLR
            } else {
                GPIO_OUTSET
            }
    }

    /// GPIO PIN_CNF register of the pin.
    pub const fn pin_cnf_register(&self) -> u32 {
        self.port_base() + GPIO_PIN_CNF + 4 * self.pin as u32
    }

    /// Bit of this pin in its port registers.
    pub const fn mask(&self) -> u32 {
        1 << self.pin
    }
}

/// GPIOTE task addresses that switch one FEM line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineTasks {
    pub on: u32,
    pub off: u32,
}

/// One programmed PPI channel. `fork == 0` leaves the fork unused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PpiLink {
    pub channel: u8,
    pub eep: u32,
    pub tep: u32,
    pub fork: u32,
}

impl PpiLink {
    /// `CH[n].EEP` register.
    pub const fn eep_register(&self) -> u32 {
        PPI + 0x510 + 8 * self.channel as u32
    }

    /// `CH[n].TEP` register.
    pub const fn tep_register(&self) -> u32 {
        PPI + 0x514 + 8 * self.channel as u32
    }

    /// `FORK[n].TEP` register.
    pub const fn fork_register(&self) -> u32 {
        fork_register(self.channel)
    }
}

/// `FORK[n].TEP` register of any PPI channel (including pre-programmed ones,
/// whose fork stays programmable).
pub const fn fork_register(channel: u8) -> u32 {
    PPI + 0x910 + 4 * channel as u32
}

/// A GPIOTE CONFIG register write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioteConfig {
    pub register: u32,
    pub value: u32,
}

/// Physical lines and the GPIOTE/PPI channels reserved for them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FemLayout {
    /// CTX (PA enable) + CRX (LNA enable): AT2401C, RFX2401C, SKY66112-11
    /// in two-pin mode. Four PPI channels, two GPIOTE channels.
    TwoPin {
        pa: FemPin,
        lna: FemPin,
        gpiote_pa: u8,
        gpiote_lna: u8,
        ppi: [u8; 4],
    },
    /// PA enable only (TX-only amplifier). Three PPI channels, one GPIOTE
    /// channel.
    SinglePin {
        pa: FemPin,
        gpiote_pa: u8,
        ppi: [u8; 3],
    },
}

/// Complete, validated register plan of one FEM.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FemPlan {
    /// Pins driven to their inactive level and made outputs before GPIOTE
    /// takes them over.
    pub pins: heapless::Vec<FemPin, 3>,
    pub gpiote: heapless::Vec<GpioteConfig, 2>,
    pub links: heapless::Vec<PpiLink, 4>,
    pub pa: LineTasks,
    pub lna: Option<LineTasks>,
    /// Optional CPU-controlled power-down / chip-enable line.
    pub pdn: Option<FemPin>,
    /// CHENSET/CHENCLR mask of `links`.
    pub channel_mask: u32,
}

impl FemPlan {
    /// Task for the FORK of [`PPI_CH_ACK_TXEN`] (ISR PHY Imm-Ack): PA on at
    /// the TXEN instant.
    pub const fn ack_txen_fork(&self) -> u32 {
        self.pa.on
    }
}

const fn line_tasks(channel: u8, active_high: bool) -> LineTasks {
    let set = GPIOTE_TASKS_SET + 4 * channel as u32;
    let clr = GPIOTE_TASKS_CLR + 4 * channel as u32;
    if active_high {
        LineTasks { on: set, off: clr }
    } else {
        LineTasks { on: clr, off: set }
    }
}

/// GPIOTE CONFIG value: task mode on `pin`, OUT task polarity None (only
/// SET/CLR are used), initial output at the inactive level.
pub const fn gpiote_config_value(pin: FemPin) -> u32 {
    let out_init_high = !pin.active_high;
    GPIOTE_MODE_TASK
        | ((pin.pin as u32 & 0x1F) << 8)
        | ((pin.port as u32 & 1) << 13)
        | ((out_init_high as u32) << 20)
}

fn gpiote_config(channel: u8, pin: FemPin) -> GpioteConfig {
    GpioteConfig {
        register: GPIOTE_CONFIG + 4 * u32::from(channel),
        value: gpiote_config_value(pin),
    }
}

fn distinct(values: &[u8]) -> bool {
    values
        .iter()
        .enumerate()
        .all(|(index, value)| !values[..index].contains(value))
}

/// Validate `layout` and compute its register plan.
pub fn plan(layout: FemLayout, pdn: Option<FemPin>) -> Result<FemPlan, FemConfigError> {
    let invalid = FemConfigError::InvalidResources;
    let mut pins: heapless::Vec<FemPin, 3> = heapless::Vec::new();
    let mut gpiote = heapless::Vec::new();
    let mut links = heapless::Vec::new();
    let (pa, lna, ppi_channels): (LineTasks, Option<LineTasks>, &[u8]) = match &layout {
        FemLayout::TwoPin {
            pa,
            lna,
            gpiote_pa,
            gpiote_lna,
            ppi,
        } => {
            if !distinct(&[*gpiote_pa, *gpiote_lna])
                || *gpiote_pa >= GPIOTE_CHANNELS
                || *gpiote_lna >= GPIOTE_CHANNELS
            {
                return Err(invalid);
            }
            pins.push(*pa).map_err(|_| invalid)?;
            pins.push(*lna).map_err(|_| invalid)?;
            gpiote
                .push(gpiote_config(*gpiote_pa, *pa))
                .map_err(|_| invalid)?;
            gpiote
                .push(gpiote_config(*gpiote_lna, *lna))
                .map_err(|_| invalid)?;
            let pa_tasks = line_tasks(*gpiote_pa, pa.active_high);
            let lna_tasks = line_tasks(*gpiote_lna, lna.active_high);
            for (channel, eep, tep, fork) in [
                (ppi[0], RADIO_EVENTS_CCAIDLE, pa_tasks.on, lna_tasks.off),
                (ppi[1], RADIO_EVENTS_PHYEND, pa_tasks.off, 0),
                (ppi[2], RADIO_EVENTS_RXREADY, lna_tasks.on, pa_tasks.off),
                (ppi[3], RADIO_EVENTS_DISABLED, lna_tasks.off, 0),
            ] {
                links
                    .push(PpiLink {
                        channel,
                        eep,
                        tep,
                        fork,
                    })
                    .map_err(|_| invalid)?;
            }
            (pa_tasks, Some(lna_tasks), &ppi[..])
        }
        FemLayout::SinglePin { pa, gpiote_pa, ppi } => {
            if *gpiote_pa >= GPIOTE_CHANNELS {
                return Err(invalid);
            }
            pins.push(*pa).map_err(|_| invalid)?;
            gpiote
                .push(gpiote_config(*gpiote_pa, *pa))
                .map_err(|_| invalid)?;
            let pa_tasks = line_tasks(*gpiote_pa, pa.active_high);
            for (channel, eep, tep) in [
                (ppi[0], RADIO_EVENTS_CCAIDLE, pa_tasks.on),
                (ppi[1], RADIO_EVENTS_PHYEND, pa_tasks.off),
                (ppi[2], RADIO_EVENTS_RXREADY, pa_tasks.off),
            ] {
                links
                    .push(PpiLink {
                        channel,
                        eep,
                        tep,
                        fork: 0,
                    })
                    .map_err(|_| invalid)?;
            }
            (pa_tasks, None, &ppi[..])
        }
    };
    if !distinct(ppi_channels)
        || ppi_channels
            .iter()
            .any(|&channel| channel >= PPI_CONFIGURABLE_CHANNELS)
    {
        return Err(invalid);
    }
    if let Some(pdn) = pdn {
        pins.push(pdn).map_err(|_| invalid)?;
    }
    let psels: heapless::Vec<u8, 3> = pins.iter().map(|pin| pin.port * 32 + pin.pin).collect();
    if !pins.iter().all(FemPin::valid) || !distinct(&psels) {
        return Err(invalid);
    }
    let channel_mask = ppi_channels
        .iter()
        .fold(0u32, |mask, &channel| mask | 1 << channel);
    Ok(FemPlan {
        pins,
        gpiote,
        links,
        pa,
        lna,
        pdn,
        channel_mask,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const CTX: FemPin = FemPin {
        port: 1,
        pin: 4,
        active_high: true,
    };
    const CRX: FemPin = FemPin {
        port: 1,
        pin: 2,
        active_high: true,
    };

    fn at2401c() -> FemLayout {
        FemLayout::TwoPin {
            pa: CTX,
            lna: CRX,
            gpiote_pa: 6,
            gpiote_lna: 7,
            ppi: [16, 17, 18, 19],
        }
    }

    #[test]
    fn two_pin_wiring_matches_the_timing_scheme() {
        let plan = plan(at2401c(), None).unwrap();
        // GPIOTE: TASKS_SET[6] 0x4000_6048, TASKS_CLR[6] 0x4000_6078,
        // TASKS_SET[7] 0x4000_604C, TASKS_CLR[7] 0x4000_607C.
        assert_eq!(
            plan.pa,
            LineTasks {
                on: 0x4000_6048,
                off: 0x4000_6078
            }
        );
        assert_eq!(
            plan.lna,
            Some(LineTasks {
                on: 0x4000_604C,
                off: 0x4000_607C
            })
        );
        let expected = [
            // CH16.EEP = RADIO CCAIDLE, TEP = PA on, FORK = LNA off.
            (16, 0x4000_1144, 0x4000_6048, 0x4000_607C),
            // CH17: PHYEND -> PA off.
            (17, 0x4000_116C, 0x4000_6078, 0),
            // CH18: RXREADY -> LNA on, FORK PA off.
            (18, 0x4000_1158, 0x4000_604C, 0x4000_6078),
            // CH19: DISABLED -> LNA off.
            (19, 0x4000_1110, 0x4000_607C, 0),
        ];
        assert_eq!(plan.links.len(), expected.len());
        for (link, (channel, eep, tep, fork)) in plan.links.iter().zip(expected) {
            assert_eq!(
                *link,
                PpiLink {
                    channel,
                    eep,
                    tep,
                    fork
                }
            );
        }
        let ch16 = plan.links[0];
        assert_eq!(ch16.eep_register(), 0x4001_F590);
        assert_eq!(ch16.tep_register(), 0x4001_F594);
        assert_eq!(ch16.fork_register(), 0x4001_F950);
        assert_eq!(fork_register(PPI_CH_ACK_TXEN), 0x4001_F960);
        assert_eq!(plan.ack_txen_fork(), 0x4000_6048);
        assert_eq!(plan.channel_mask, 0x000F_0000);
        // CONFIG[6] = Task | PSEL 4 | PORT 1 | OUTINIT low.
        assert_eq!(
            plan.gpiote[0],
            GpioteConfig {
                register: 0x4000_6528,
                value: 0x0000_2403
            }
        );
        assert_eq!(
            plan.gpiote[1],
            GpioteConfig {
                register: 0x4000_652C,
                value: 0x0000_2203
            }
        );
        // The PA is never dropped on DISABLED (RX->TX switch of a CCA TX).
        assert!(plan.links.iter().all(|link| {
            link.eep != RADIO_EVENTS_DISABLED
                || (link.tep != plan.pa.off && link.fork != plan.pa.off)
        }));
    }

    #[test]
    fn active_low_lines_swap_set_and_clear() {
        let pa = FemPin {
            port: 0,
            pin: 17,
            active_high: false,
        };
        let plan = plan(
            FemLayout::SinglePin {
                pa,
                gpiote_pa: 0,
                ppi: [0, 1, 2],
            },
            None,
        )
        .unwrap();
        assert_eq!(
            plan.pa,
            LineTasks {
                on: 0x4000_6060,
                off: 0x4000_6030
            }
        );
        // OUTINIT = 1 (inactive high), P0.17.
        assert_eq!(plan.gpiote[0].value, (1 << 20) | (17 << 8) | 3);
        assert_eq!(pa.active_register(), 0x5000_050C);
        assert_eq!(pa.inactive_register(), 0x5000_0508);
        assert_eq!(pa.pin_cnf_register(), 0x5000_0744);
        assert_eq!(plan.lna, None);
        let events: heapless::Vec<u32, 4> = plan.links.iter().map(|link| link.eep).collect();
        assert_eq!(
            events.as_slice(),
            [
                RADIO_EVENTS_CCAIDLE,
                RADIO_EVENTS_PHYEND,
                RADIO_EVENTS_RXREADY
            ]
        );
        assert!(plan.links.iter().all(|link| link.fork == 0));
        assert_eq!(plan.channel_mask, 0b111);
    }

    #[test]
    fn pdn_line_is_cpu_controlled() {
        let pdn = FemPin {
            port: 1,
            pin: 5,
            active_high: true,
        };
        let plan = plan(at2401c(), Some(pdn)).unwrap();
        assert_eq!(plan.pdn, Some(pdn));
        assert_eq!(plan.pins.len(), 3);
        assert_eq!(pdn.active_register(), 0x5000_0808);
        assert_eq!(pdn.inactive_register(), 0x5000_080C);
        assert_eq!(pdn.pin_cnf_register(), 0x5000_0A14);
        assert_eq!(pdn.mask(), 1 << 5);
        // The plan's links never touch the PDN line.
        assert_eq!(plan.gpiote.len(), 2);
    }

    #[test]
    fn shared_or_reserved_resources_are_rejected() {
        let reject = |layout, pdn| plan(layout, pdn).unwrap_err();
        let FemLayout::TwoPin {
            pa,
            lna,
            gpiote_pa,
            gpiote_lna,
            ppi,
        } = at2401c()
        else {
            unreachable!()
        };
        let two = |pa, lna, gpiote_pa, gpiote_lna, ppi| FemLayout::TwoPin {
            pa,
            lna,
            gpiote_pa,
            gpiote_lna,
            ppi,
        };
        let err = FemConfigError::InvalidResources;
        assert_eq!(reject(two(pa, pa, gpiote_pa, gpiote_lna, ppi), None), err);
        assert_eq!(reject(two(pa, lna, 6, 6, ppi), None), err);
        assert_eq!(reject(two(pa, lna, 8, 7, ppi), None), err);
        assert_eq!(
            reject(two(pa, lna, gpiote_pa, gpiote_lna, [16, 17, 18, 16]), None),
            err
        );
        // Pre-programmed channels (the ISR PHY's 20/26/27) are never taken.
        assert_eq!(
            reject(two(pa, lna, gpiote_pa, gpiote_lna, [16, 17, 18, 20]), None),
            err
        );
        assert_eq!(reject(at2401c(), Some(CRX)), err);
        let bad_port = FemPin {
            port: 2,
            pin: 0,
            active_high: true,
        };
        assert_eq!(reject(at2401c(), Some(bad_port)), err);
    }
}
