//! nRF52840 Dongle (PCA10059) IEEE 802.15.4 sniffer and raw transmitter.
//!
//! A lab tool, not a shipped image. It speaks the USB CDC text protocol of
//! Nordic's *nRF Sniffer for 802.15.4* (same VID:PID and product string, so
//! the Wireshark extcap and hwlab's `nrf802154` plugin find it) and adds
//! `transmit`, `power`, `stats` and `version`; see `radio_lab_protocol`.
//!
//! Tasks: the USB device, a CDC reader that assembles command lines, and
//! the main task, which owns the radio and the CDC writer, executes commands
//! and prints received frames.

#![no_std]
#![no_main]

mod radio;

use core::fmt::{self, Write as _};

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_nrf::config::{Config, Debug, HfclkSource, LfclkSource};
use embassy_nrf::gpio::Output;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embassy_usb::UsbDevice;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use panic_reset as _;
use radio_lab_protocol::{
    self as protocol, Command, LineAssembler, Push, TxRequest, TxResult, TxStatus, TxSummary,
};
use static_cell::StaticCell;

use crate::radio::{RX_SIGNAL, Radio};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    RADIO => radio::RadioIrq;
});

type UsbDriver = usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>;

/// USB identity of Nordic's sniffer, so existing host tools detect the lab.
const USB_VID: u16 = 0x1915;
const USB_PID: u16 = 0x154B;
const USB_PRODUCT: &str = "nRF 802154 Sniffer";
const USB_MANUFACTURER: &str = "zigbee-rs";
const USB_PACKET: usize = 64;

/// One USB write may wait this long for the host before output is dropped.
const WRITE_TIMEOUT: Duration = Duration::from_millis(100);
/// After a write timeout, output is dropped for this long (host not reading).
const STALL_BACKOFF: Duration = Duration::from_secs(1);
/// Heartbeat and clock-extension period (TIMER0 wraps after 71.6 min).
const TICK: Duration = Duration::from_millis(250);

/// One command line handed from the CDC reader to the main task.
struct CommandLine {
    bytes: [u8; protocol::MAX_LINE_LEN],
    len: usize,
    overflow: bool,
}

impl CommandLine {
    fn new(text: &str) -> Self {
        let mut bytes = [0; protocol::MAX_LINE_LEN];
        let len = text.len().min(protocol::MAX_LINE_LEN);
        bytes[..len].copy_from_slice(&text.as_bytes()[..len]);
        Self {
            bytes,
            len,
            overflow: false,
        }
    }

    fn overflow() -> Self {
        Self {
            bytes: [0; protocol::MAX_LINE_LEN],
            len: 0,
            overflow: true,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

static COMMANDS: Channel<CriticalSectionRawMutex, CommandLine, 4> = Channel::new();

/// Formatted output line (longest: a 127-octet `received:` line).
struct LineBuffer {
    bytes: [u8; 400],
    len: usize,
}

impl LineBuffer {
    const fn new() -> Self {
        Self {
            bytes: [0; 400],
            len: 0,
        }
    }
}

impl fmt::Write for LineBuffer {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn reader_task(mut receiver: Receiver<'static, UsbDriver>) -> ! {
    let mut lines = LineAssembler::<{ protocol::MAX_LINE_LEN }>::new();
    let mut packet = [0u8; USB_PACKET];
    loop {
        receiver.wait_connection().await;
        while let Ok(count) = receiver.read_packet(&mut packet).await {
            for &byte in &packet[..count] {
                match lines.push(byte) {
                    Push::Pending => {}
                    Push::Line => COMMANDS.send(CommandLine::new(lines.line())).await,
                    Push::Overflow => COMMANDS.send(CommandLine::overflow()).await,
                }
            }
        }
    }
}

struct Leds {
    heartbeat: Output<'static>,
    rx: Output<'static>,
    tx: Output<'static>,
}

struct App {
    radio: Radio<'static>,
    sender: Sender<'static, UsbDriver>,
    leds: Leds,
    echo: bool,
    pending: Option<CommandLine>,
    stalled_until: Option<Instant>,
    reported_overflows: u32,
    ticks: u32,
}

impl App {
    async fn write_line(&mut self, line: impl fmt::Display) {
        if !self.sender.dtr() {
            return;
        }
        if let Some(until) = self.stalled_until {
            if Instant::now() < until {
                return;
            }
            self.stalled_until = None;
        }
        let mut buffer = LineBuffer::new();
        if write!(buffer, "{line}\r\n").is_err() {
            return;
        }
        let bytes = &buffer.bytes[..buffer.len];
        for chunk in bytes.chunks(USB_PACKET) {
            if !self.write_packet(chunk).await {
                return;
            }
        }
        if bytes.len().is_multiple_of(USB_PACKET) {
            // Zero-length packet ends the transfer for the host.
            self.write_packet(&[]).await;
        }
    }

    async fn write_packet(&mut self, packet: &[u8]) -> bool {
        match with_timeout(WRITE_TIMEOUT, self.sender.write_packet(packet)).await {
            Ok(Ok(())) => true,
            Ok(Err(_)) => false,
            Err(_) => {
                self.stalled_until = Some(Instant::now() + STALL_BACKOFF);
                false
            }
        }
    }

    /// Print every queued received frame.
    async fn drain_rx(&mut self) {
        while let Some(frame) = self.radio.next_frame() {
            self.leds.rx.toggle();
            self.write_line(protocol::Received {
                psdu: frame.psdu(),
                rssi_dbm: frame.rssi_dbm,
                lqi: frame.lqi,
                time_us: frame.time_us,
            })
            .await;
        }
        let overflows = self.radio.stats().overflow;
        if overflows != self.reported_overflows {
            let lost = overflows.wrapping_sub(self.reported_overflows);
            self.reported_overflows = overflows;
            self.write_line(protocol::RxOverflow(lost)).await;
        }
    }

    fn tick(&mut self) {
        // Keeps the 64-bit clock extension ahead of TIMER0 wrap-around.
        let _ = radio::now_us();
        self.ticks = self.ticks.wrapping_add(1);
        // Nordic's heartbeat: 1 s asleep, 250 ms receiving.
        if self.radio.receiving() || self.ticks.is_multiple_of(4) {
            self.leds.heartbeat.toggle();
        }
    }

    async fn handle(&mut self, line: CommandLine) {
        if line.overflow {
            self.write_line("error: line too long").await;
            return;
        }
        let text = line.as_str();
        if self.echo {
            self.write_line(text).await;
        }
        match protocol::parse(text) {
            Ok(Command::Empty) | Ok(Command::ShellColors(_)) => {}
            Ok(Command::Sleep) => self.radio.sleep(),
            Ok(Command::Receive) => self.radio.receive(),
            Ok(Command::ChannelGet) => {
                let channel = self.radio.channel();
                self.write_line(channel).await;
            }
            Ok(Command::ChannelSet(channel)) => self.radio.set_channel(channel),
            Ok(Command::PowerGet) => {
                let power = self.radio.power_dbm();
                self.write_line(protocol::Power(power)).await;
            }
            Ok(Command::PowerSet(dbm)) => self.radio.set_power(dbm),
            Ok(Command::Version) => {
                self.write_line(protocol::Version {
                    firmware_version: env!("CARGO_PKG_VERSION"),
                })
                .await;
            }
            Ok(Command::Help) => {
                for help in protocol::HELP_LINES {
                    self.write_line(help).await;
                }
            }
            Ok(Command::Stats) => {
                let stats = self.radio.stats();
                self.write_line(stats).await;
            }
            Ok(Command::ShellEcho(on)) => self.echo = on,
            Ok(Command::Transmit(request)) => self.transmit(&request).await,
            Err(error) => self.write_line(error).await,
        }
    }

    async fn transmit(&mut self, request: &TxRequest) {
        self.leds.tx.set_low();
        let mut on_air = [0u8; protocol::MAX_PSDU_LEN];
        let len = request.psdu().len() + protocol::FCS_LEN;
        on_air[..request.psdu().len()].copy_from_slice(request.psdu());
        protocol::append_fcs(&mut on_air[..len]);
        let ack_sequence = if request.ack_wait {
            request.sequence()
        } else {
            None
        };

        let mut summary = TxSummary::default();
        let start = Instant::now();
        'attempts: for attempt in 1..=request.repeat {
            if attempt > 1 {
                let offset = u64::from(request.interval_ms) * u64::from(attempt - 1);
                let deadline = start + Duration::from_millis(offset);
                loop {
                    match select3(Timer::at(deadline), RX_SIGNAL.wait(), COMMANDS.receive()).await {
                        Either3::First(()) => break,
                        Either3::Second(()) => self.drain_rx().await,
                        Either3::Third(line) => {
                            // Any command aborts the burst, then runs.
                            self.pending = Some(line);
                            summary.aborted = true;
                            break 'attempts;
                        }
                    }
                }
            }
            self.drain_rx().await;
            let (status, time_us) = self
                .radio
                .transmit(request.psdu(), request.cca, ack_sequence);
            summary.record(status);
            if status.on_air() {
                self.write_line(protocol::Transmitted {
                    psdu: &on_air[..len],
                    channel: self.radio.channel(),
                    power_dbm: self.radio.power_dbm(),
                    time_us,
                })
                .await;
            }
            // Frames heard during the ACK window precede the result line.
            self.drain_rx().await;
            self.write_line(TxResult {
                attempt,
                total: request.repeat,
                status,
                time_us,
            })
            .await;
            if status == TxStatus::Error {
                break;
            }
        }
        self.write_line(summary).await;
        self.leds.tx.set_high();
    }

    async fn run(mut self) -> ! {
        let mut ticker = Ticker::every(TICK);
        loop {
            let command = match self.pending.take() {
                Some(command) => Some(command),
                None => match select3(COMMANDS.receive(), RX_SIGNAL.wait(), ticker.next()).await {
                    Either3::First(command) => Some(command),
                    Either3::Second(()) => None,
                    Either3::Third(()) => {
                        self.tick();
                        None
                    }
                },
            };
            self.drain_rx().await;
            if let Some(command) = command {
                self.handle(command).await;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    // USB and the radio need the crystal; the dongle has no 32 kHz crystal.
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.lfclk_source = LfclkSource::InternalRC;
    // Leave UICR.APPROTECT exactly as the bootloader left it.
    config.debug = Debug::NotConfigured;
    let p = embassy_nrf::init(config);

    let leds = Leds {
        heartbeat: nrf52840_pca10059::status_led(p.P0_06),
        rx: nrf52840_pca10059::led2_blue(p.P0_12),
        tx: nrf52840_pca10059::led2_red(p.P0_08),
    };
    let radio = Radio::new(p.RADIO, p.TIMER0, p.PPI_CH26, Irqs);

    static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();
    let serial = SERIAL.init(radio::usb_serial());
    let serial = core::str::from_utf8(serial).unwrap_or("0000000000000000");

    let mut usb_config = embassy_usb::Config::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some(USB_MANUFACTURER);
    usb_config.product = Some(USB_PRODUCT);
    usb_config.serial_number = Some(serial);
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUFFER: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([]),
        CONTROL_BUFFER.init([0; 64]),
    );
    let class = CdcAcmClass::new(
        &mut builder,
        CDC_STATE.init(State::new()),
        USB_PACKET as u16,
    );
    let device = builder.build();
    let (sender, receiver) = class.split();

    spawner.must_spawn(usb_task(device));
    spawner.must_spawn(reader_task(receiver));

    App {
        radio,
        sender,
        leds,
        echo: true,
        pending: None,
        stalled_until: None,
        reported_overflows: 0,
        ticks: 0,
    }
    .run()
    .await
}
