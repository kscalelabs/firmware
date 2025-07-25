#![feature(type_alias_impl_trait)]
#![allow(unused)]

pub mod hiwonder;
pub mod typestate_serial;

pub mod actuator;
pub mod actuator_manager;
pub mod behavior;
pub mod bytestream_fd;
pub mod config;
pub mod imu;
pub mod inference;
pub mod keyboard;
pub mod policy_control;
pub mod robot_description;
pub mod socketcan;
pub mod socketcan2;
pub mod state_machine_utils;
pub mod telemetry;
pub mod typestate_socket;
pub mod typestate_socket2;

pub mod git_hash {
    include!(concat!(env!("OUT_DIR"), "/git_hash.rs"));
}

use std::task::{Context, Poll};

use crate::robstride::{ObtainIdRequest, ObtainIdResponse};

use config::Config;

use socketcan::CanFrame;
pub mod robstride;
pub mod robstride_utils;
use std::pin::Pin;

use futures::stream::Stream;
use futures::stream::StreamExt;

use telemetry::{
    forwarder::{EventRecord, HeaplessForwardLayer},
    telemetry_main::start_pipeline,
};
use tracing::{Level, Metadata, debug, error, info, trace, warn};
use tracing_subscriber::{EnvFilter, Layer, fmt, layer::SubscriberExt};

use std::sync::mpsc;
use std::path::PathBuf;

use clap::Parser;
#[derive(Debug, Parser)]
#[command(name = "faux-rtos", about = "Parse three floats and a path")]
pub struct Args {
    /// scale factor for the policy
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    policy_scale: f64,

    /// proportional gain scale
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    kp_scale: f64,

    /// derivative gain scale
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    kd_scale: f64,

    /// path to log file
    #[arg(long, value_name = "PATH", default_value = "events.log")]
    kinfer_log_path: PathBuf,
}

async fn driver(config: &Config) -> std::io::Result<()> {
    let mut behavior_manager = behavior::BehaviorManager::new(config);
    let mut pinned = unsafe { Pin::new_unchecked(&mut behavior_manager) };
    loop {
        // iterate over each SlowCounter in sc_vec
        // get a future
        let next = pinned.next();
        let to = tokio::time::timeout(std::time::Duration::from_millis(100), next);

        let res = to.await;

        match res {
            Ok(Some(Ok(tag))) => {
                debug!("returned Some: {:?}", tag);
            }
            Ok(Some(Err(e))) => {
                error!("returned Err: {:?}", e);
                break;
            }
            Ok(None) => {
                error!("returned None, continuing...");
            }
            Err(_) => {
                debug!("timed out, continuing...");
            }
        }
    }

    info!("finished looping, dropping ActuatorBus");
    Ok(())
}

fn main() {
    // Parse command line arguments
    let args = Args::parse();

    let config = Config {
        policy_scale: args.policy_scale,
        kp_scale: args.kp_scale,
        kd_scale: args.kd_scale,
        log_path: args.kinfer_log_path,
    };


    // Setup telemetry before we do anything else
    let (tx, rx) = mpsc::sync_channel::<EventRecord>(1024 * 1024);

    // Spawn the thread that will format and log our data
    let jh = start_pipeline(rx, &config.log_path).expect("Failed to start telemetry pipeline");

    let trace_only_filter = tracing_subscriber::filter::FilterFn::new(|metadata: &Metadata| {
        metadata.level() == &Level::TRACE && metadata.target().starts_with("faux_rtos")
    });

    // Prepare tracing to forward to our thread
    let forward_layer = HeaplessForwardLayer { tx }.with_filter(trace_only_filter);

    // Add stdout layer for INFO and above
    let stdout_layer = fmt::layer().with_target(true).with_level(true).with_filter(
        EnvFilter::from_default_env().add_directive("faux_rtos=info".parse().unwrap()),
    );

    let subscriber = tracing_subscriber::registry()
        .with(forward_layer)
        .with(stdout_layer);

    let guard = tracing::subscriber::set_default(subscriber);

    let start_time = std::time::Instant::now();
    info!("Starting faux-rtos version: {}", git_hash::GIT_HASH);
    info!(
        "id: {:?} starting at {:?}",
        std::thread::current().id(),
        start_time
    );

    // Create runtime
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build()
        .unwrap();

    // handle driver and SIGINT
    let drv = async {
        tokio::select! {
            result = driver(&config) => {
                match result {
                    Ok(_) => {
                        info!("Driver finished successfully");
                        Ok(())
                    }
                    Err(e) => {
                        error!("Driver error: {:?}", e);
                        Err(e)
                    }
                }
            }
            // Ctrl+C handler
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl+C, shutting down gracefully...");
                Ok(())
            }
        }
    };

    let start = std::time::Instant::now();
    let _result = rt.block_on(drv);
    let elapsed = start.elapsed();
    info!("Runtime elapsed time: {:?}", elapsed);

    // Cleanup sequence - ORDER MATTERS!
    info!("Starting cleanup...");

    // we wil use println beyond this point as tracing guard is dropped
    println!("Dropped tracing guard");
    drop(guard);

    // wait for the telemetry thread to finish
    match jh.join() {
        Ok(_) => println!("Background thread finished successfully"),
        Err(e) => eprintln!("Background thread panicked: {e:?}"),
    }

    println!("Cleanup complete, exiting");
}
