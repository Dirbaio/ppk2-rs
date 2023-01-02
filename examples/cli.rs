use anyhow::Result;
use clap::Parser;
use ppk2::{
    measurement::MeasurementMatch,
    try_find_ppk2_port,
    types::{DevicePower, Level, LogicPortPins, MeasurementMode, SourceVoltage},
    Ppk2,
};

use log::{debug, error, info, Level as LogLevel};
use std::{
    sync::mpsc::RecvTimeoutError,
    time::{self, Duration, Instant},
};
use tracing_subscriber::FmtSubscriber;

#[derive(Parser)]
struct Args {
    #[clap(
        env,
        short = 'p',
        long,
        help = "The serial port the PPK2 is connected to. If unspecified, will try to find the PPK2 automatically"
    )]
    serial_port: Option<String>,

    #[clap(
        env,
        short = 'v',
        long,
        help = "The voltage of the device source in mV",
        default_value = "0"
    )]
    voltage: SourceVoltage,

    #[clap(
        env,
        short = 'e',
        long,
        help = "Enable power",
        default_value = "disabled"
    )]
    power: DevicePower,

    #[clap(
        env,
        short = 'm',
        long,
        help = "Measurement mode",
        default_value = "source"
    )]
    mode: MeasurementMode,
}

fn main() -> Result<()> {
    pretty_env_logger::init();

    // Setup stuff
    let args = Args::parse();

    let ppk2_port = match args.serial_port {
        Some(p) => p,
        None => try_find_ppk2_port()?,
    };

    // Connect to PPK2 and initialize
    let mut ppk2 = Ppk2::new(ppk2_port, args.mode)?;
    ppk2.set_source_voltage(args.voltage)?;
    ppk2.set_device_power(args.power)?;

    // Set up pin pattern for matching
    // This particular setup will only
    // match measurements if pin 0 is low.
    let mut levels = [Level::Either; 8];
    let pins = LogicPortPins::with_levels(levels);

    std::thread::sleep(Duration::from_secs(1));

    // Start measuring.
    let meas = ppk2.measure(pins, Duration::from_secs(4))?;
    info!("measurement: {:?}", meas);
    Ok(())
}
