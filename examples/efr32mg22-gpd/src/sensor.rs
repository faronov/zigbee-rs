//! BRD4183A Green Power temperature/humidity sensor (GPD DeviceID 0x33,
//! model 0xF300). See `gpd_main` for the lifecycle.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod gpd_main;
mod vcom_log;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    gpd_main::run(efr32mg22_gpd_product::SENSOR).await
}
