// Run directly with rustc --test; no PAC, target hardware, or HAL host stubs.
#[path = "../src/rtc_cntl/sleep/clock_restore.rs"]
mod clock_restore;
#[path = "../src/timer/counter_words.rs"]
mod counter_words;
#[path = "../src/rtc_cntl/rtc/h2_sleep_clock.rs"]
mod h2_clock;
#[path = "../src/rtc_cntl/sleep/retained.rs"]
pub mod retained;
