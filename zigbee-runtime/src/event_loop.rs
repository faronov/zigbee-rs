//! Event loop — drives the Zigbee stack processing pipeline.
//!
//! The event loop is the heartbeat of a Zigbee device. It:
//! 1. Polls the MAC for incoming frames
//! 2. Passes frames up through NWK → APS → ZCL
//! 3. Handles timers (reporting, aging, retries)
//! 4. Processes BDB commissioning state machine
//! 5. Manages sleep/wake for end devices

use zigbee_mac::MacDriver;

/// Events that the stack can generate for the application.
#[derive(Debug)]
pub enum StackEvent {
    /// Device joined the network successfully.
    Joined {
        short_address: u16,
        channel: u8,
        pan_id: u16,
    },
    /// Device left the network.
    Left,
    /// Attribute report received from another device.
    AttributeReport {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        attr_id: u16,
    },
    /// Command received from another device.
    CommandReceived {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        command_id: u8,
    },
    /// BDB commissioning completed.
    CommissioningComplete {
        success: bool,
    },
    /// Permit joining status changed.
    PermitJoinChanged {
        open: bool,
    },
    /// OTA image available.
    OtaImageAvailable {
        version: u32,
        size: u32,
    },
}

/// Stack tick result — tells the application what to do next.
#[derive(Debug)]
pub enum TickResult {
    /// Nothing happened, consider sleeping.
    Idle,
    /// Event(s) occurred — process them.
    Event(StackEvent),
    /// Stack needs to run again soon (within ms).
    RunAgain(u32),
}

/// Run one iteration of the Zigbee stack event loop.
///
/// This is designed for cooperative async scheduling:
/// - Call `tick()` in your main loop
/// - It processes one batch of pending work
/// - Returns quickly, never blocks indefinitely
///
/// For Embassy integration, wrap this in an async task:
/// ```rust,no_run,ignore
/// #[embassy_executor::task]
/// async fn zigbee_task(/* ... */) {
///     loop {
///         let result = stack_tick(&mut device).await;
///         match result {
///             TickResult::Idle => Timer::after(Duration::from_millis(100)).await,
///             TickResult::RunAgain(ms) => Timer::after(Duration::from_millis(ms as u64)).await,
///             TickResult::Event(evt) => handle_event(evt),
///         }
///     }
/// }
/// ```
pub async fn stack_tick<M: MacDriver>(
    _device: &mut crate::ZigbeeDevice<M>,
) -> TickResult {
    // Phase 1: Check MAC for incoming frames
    // TODO: call mac.mcps_data_indication() with timeout

    // Phase 2: Process NWK layer (neighbor aging, route maintenance)
    // TODO: nwk.age_tick()

    // Phase 3: Process APS layer (retries, ack timeouts)
    // TODO: aps.process_pending()

    // Phase 4: Process ZCL reporting engine
    // TODO: zcl.check_reports()

    // Phase 5: BDB commissioning state machine
    // TODO: bdb.process()

    // Phase 6: Power management (sleep decision)
    // TODO: check if we can sleep

    TickResult::Idle
}
