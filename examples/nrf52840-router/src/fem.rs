//! AT2401C RF front-end composition (`--features fem-at2401c`).
//!
//! Joins the board's description of the add-on (`nrf52840-dk`
//! `fem_at2401c`: pins, gains, settle times) with the product's regulatory
//! output cap (`nrf52840-router-product` `fem`) and reserves the hardware
//! channels the PPI pin control uses: GPIOTE 6/7 and PPI 16..=19. The ISR
//! PHY keeps its pre-programmed PPI 20/26/27; nothing else in either image
//! uses GPIOTE or programmable PPI channels. **Hardware-unverified.**
//!
//! The composition roots reach this module only through `with_fem!` and
//! the role's `Phy`/`Mac` aliases, declared at the end of each file, so
//! images built without the feature are byte-identical (no line in front
//! of a panic location moves).

use embassy_nrf::peripherals;
use embassy_nrf::ppi::ConfigurableChannel;
use nrf52840_dk::fem_at2401c as board;
use nrf52840_router_product::fem as policy;
use zigbee_mac::fem::{FemConfigError, FemParams};
use zigbee_mac::nrf::{FemLine, NrfFem, NrfFemHost, NrfFemPhy};

/// Board gains/timing with the product's output cap.
pub const PARAMS: FemParams = FemParams {
    tx_gain_db: board::TX_GAIN_DB,
    rx_gain_db: board::RX_GAIN_DB,
    pa_settle_us: board::CTX_SETTLE_US,
    lna_settle_us: board::CRX_SETTLE_US,
    max_output_dbm: policy::MAX_OUTPUT_DBM,
    default_output_dbm: policy::DEFAULT_OUTPUT_DBM,
};

/// Hardware channels reserved for the FEM.
pub struct Channels {
    pub gpiote_ctx: peripherals::GPIOTE_CH6,
    pub gpiote_crx: peripherals::GPIOTE_CH7,
    pub ppi: (
        peripherals::PPI_CH16,
        peripherals::PPI_CH17,
        peripherals::PPI_CH18,
        peripherals::PPI_CH19,
    ),
}

/// Program the add-on's pin control (lines idle, PPI armed).
pub fn at2401c(pins: board::Pins, channels: Channels) -> Result<NrfFem<'static>, FemConfigError> {
    let (a, b, c, d) = channels.ppi;
    NrfFem::two_pin(
        PARAMS,
        FemLine::new(pins.ctx, board::CTX_ACTIVE_HIGH),
        FemLine::new(pins.crx, board::CRX_ACTIVE_HIGH),
        None,
        channels.gpiote_ctx,
        channels.gpiote_crx,
        [a.degrade(), b.degrade(), c.degrade(), d.degrade()],
    )
}

/// Attach the add-on to `phy` (either nRF PHY), or halt: a FEM build must
/// never run its radio without the front end it was configured for.
pub fn attach<P: NrfFemHost>(
    phy: P,
    pins: board::Pins,
    channels: Channels,
) -> NrfFemPhy<'static, P> {
    match at2401c(pins, channels) {
        Ok(fem) => NrfFemPhy::new(phy, fem),
        Err(error) => {
            log::error!("[NRF] AT2401C FEM configuration rejected: {error:?}");
            loop {
                cortex_m::asm::wfi();
            }
        }
    }
}

/// `with_fem!(phy, p)`: `phy` wrapped with the add-on, whose pins and
/// channels are taken from the `Peripherals` `p`.
macro_rules! with_fem {
    ($phy:ident, $p:ident) => {
        crate::fem::attach(
            $phy,
            nrf52840_dk::fem_at2401c::pins($p.P1_04, $p.P1_02),
            crate::fem::Channels {
                gpiote_ctx: $p.GPIOTE_CH6,
                gpiote_crx: $p.GPIOTE_CH7,
                ppi: ($p.PPI_CH16, $p.PPI_CH17, $p.PPI_CH18, $p.PPI_CH19),
            },
        )
    };
}
pub(crate) use with_fem;
