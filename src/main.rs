#![feature(type_alias_impl_trait)]
#![allow(unused)]

// use std::os::unix::io::AsRawFd;
// use tokio::io::{AsyncReadExt, Interest};
// use tokio::io::unix::AsyncFd;
// use tokio_serial::{SerialPortBuilderExt, SerialStream, SerialPort};
// use tokio::net::{TcpListener, TcpStream};
// use std::io::Result as IoResult;
// use std::time::Duration;
// use tokio::time::sleep;
// use std::pin::Pin;
// use std::io::ErrorKind;

pub mod typestate_serial;
pub mod hiwonder;

pub mod typestate_socket;
pub mod typestate_socket2;
pub mod socketcan2;
pub mod socketcan;
pub mod bytestream_fd;
pub mod actuator;
pub mod actuator_manager;
pub mod behavior;
pub mod state_machine_utils;
pub mod robot_description;
pub mod imu;
pub mod inference;
pub mod telemetry;

use std::task::{Context, Poll};

use crate::robstride::{
    ObtainIdRequest,
    ObtainIdResponse,
};

use socketcan::CanFrame;
pub mod robstride;
pub mod robstride_utils;
use std::pin::Pin;

use futures::stream::Stream;
use futures::stream::StreamExt;

use tracing_subscriber::layer::SubscriberExt;
use telemetry::{
    telemetry::start_pipeline,
    forwarder::{EventRecord, HeaplessForwardLayer},
};

use std::sync::mpsc;



async fn get_one() -> std::io::Result<()> {

    // yield and then return ok
    //
    tokio::task::yield_now().await;
    Ok(())
}

struct SlowCounter {
    count: u8,
}

impl SlowCounter {
    fn new() -> Self {
        SlowCounter { count: 0 }
    }

    async fn increment(&mut self) -> std::io::Result<()> {
        // Simulate a slow operation
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        self.count += 1;

        // Return Ok to indicate success
        Ok(())
    }
}

use clap::Parser;
#[derive(Debug, Parser)]
#[command(
    name = "faux-rtos",
    about = "Parse three floats"
)]
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
}


async fn driver2() -> std::io::Result<()> {

    // let mut slow_counter = SlowCounter::new();
    // let mut exp_state = actuator::ExpState::new(
    //     slow_counter.increment(),
    //     // get_one(),
    // );


    // let port2 = crate::typestate_socket2::Socket::<
    //     socketcan2::SocketCanConfigurator>::new("vcan0")?;

    // println!("Port 2: {:?}", port2);

    // let mut port2 = port2.establish::<socketcan2::SocketCanOperator>().await.map_err(|e| {
    //     std::io::Error::other(
    //         format!("Failed to establish socket: {:?}", e),
    //     )
    // })?;
    // println!("Port 2: {:?}", port2);
    // let req_frame = CanFrame {
    //     can_id: 0x00000066,
    //     len: 4,
    //     pad: 0,
    //     res0: 0,
    //     len8_dlc: 0,
    //     can_data: [0xCA, 0xFE, 0xBA, 0xBE, 0, 0, 0, 0],
    // };
    // let frame_bytes: [u8; 16] = unsafe {
    //     std::mem::transmute::<CanFrame, [u8; 16]>(req_frame.into())
    // };

    // port2.write(&frame_bytes).await?;

    // let mut act_store = actuator::Store::new();

    // let mut act_stores = [
    //     actuator::Store::new(),
    //     actuator::Store::new(),
    //     actuator::Store::new(),
    //     actuator::Store::new(),
    // ];

    // let mut act_manager_store = actuator_manager::Store::new(&mut act_stores);
    // let mut act_manager = actuator_manager::ActuatorManager::new();
    let mut behavior_manager = behavior::BehaviorManager::new();
    // let mut pinned = unsafe { Pin::new_unchecked(&mut act_manager) };
    let mut pinned = unsafe { Pin::new_unchecked(&mut behavior_manager) };
    
      // let port2 = port2.establish().await?;
      // println!("Port 2: {:?}", port2);
    // pinned.await;
    // println!("ActuatorBus is now in operate state: {:#?}", act_bus);

    // // we will try to drive this in increments of 10ms

    // let mut pinned = unsafe { Pin::new_unchecked(&mut exp_state) };

    // use socketcan2::SocketCanConfigurator;
    // use socketcan2::SocketCanOperator;

    // let mut sgraph = typestate_socket2::SocketGraph::<SocketCanConfigurator, SocketCanOperator>::new();

    // // pin it on the stack
    // let mut pinned_sgraph = unsafe { Pin::new_unchecked(&mut sgraph) };
    // pinned_sgraph.as_mut().await;


    // pinned_sgraph.as_mut().await;

    // println!("await1: {:?}", pinned_sgraph.as_mut().await);
    // println!("await2: {:?}", pinned_sgraph.as_mut().await);

    loop {
        // iterate over each SlowCounter in sc_vec
        // get a future
        let next = pinned.next();
        let to = tokio::time::timeout(
            std::time::Duration::from_millis(100),
            next
        );


        // pin it
        // let res = exp_state.await?;
        let res = to.await;

        match res {
            Ok(Some(Ok(tag))) => {
                log::debug!("returned Some: {:?}", tag);
                // if tag == target {
                //     log::info!("Reached target state: {:?}", tag);
                //     break;
                // }
            }
            Ok(Some(Err(e))) => {
                log::error!("returned Err: {:?}", e);
                break;
            }
            Ok(None) => {
                log::error!("returned None, continuing...");
            }
            Err(_) => {
                log::debug!("timed out, continuing...");
            }
        }
    }

        log::info!("finished looping, dropping ActuatorBus");
    // }


    Ok(())
}

// async fn driver() -> std::io::Result<()> {
// 
// 
//     // let port2 = typestate_socket::SocketPort::<
//     //     typestate_socket::Operate,
//     //     socketcan::CanSocket>::new("can0", socketcan_handler).await?;
// 
//     let socketcan_handler = socketcan::CanSocket {};
//     let mut port_vec = vec![];
//     for can_iface in ["can0", "can1", "can2", "can3"] {
//         let port2 = crate::typestate_socket2::Socket::<
//             socketcan2::SocketCanConfigurator>::new(can_iface).await?;
//         println!("Port 2: {:?}", port2);
// 
//         let mut port2 = port2.establish::<socketcan2::SocketCanOperator>().await.map_err(|e| {
//             std::io::Error::other(
//                 format!("Failed to establish socket: {:?}", e),
//             )
//         })?;
//         port_vec.push(port2);
//     }
//     // let port2 = crate::typestate_socket2::Socket::<
//     //     socketcan2::SocketCanConfigurator>::new("can3").await?;
//     // println!("Port 2: {:?}", port2);
// 
//     // let mut port2 = port2.establish::<socketcan2::SocketCanOperator>().await.map_err(|e| {
//     //     std::io::Error::other(
//     //         format!("Failed to establish socket: {:?}", e),
//     //     )
//     // })?;
//     // println!("Port 2: {:?}", port2);
//     // // let port2 = port2.establish().await?;
//     // // println!("Port 2: {:?}", port2);
// 
//     // let req_frame = CanFrame {
//     //     can_id: 0x00000066,
//     //     len: 4,
//     //     pad: 0,
//     //     res0: 0,
//     //     len8_dlc: 0,
//     //     can_data: [0xCA, 0xFE, 0xBA, 0xBE, 0, 0, 0, 0],
//     // };
// 
// 
//     for mut port2 in port_vec {
//         for i in 0..=0xFF {
//             let frame = ObtainIdRequest::new(0xFD, i);
//             // let can_frame: CanFrame = frame.into();
//             // let can_frame = req_frame;
//             let frame_bytes: [u8; 16] = unsafe {
//                 std::mem::transmute::<CanFrame, [u8; 16]>(frame.into())
//             };
// 
//             let mut frame_read = [0u8; 16];
//             // println!("Wrote CAN frame: {:?}", frame_bytes);
//             let fut1 = tokio::time::timeout(
//                 std::time::Duration::from_millis(10),
//                 async {
//                     // println!("Wrote CAN frame: {:?}", can_frame);
//                     let ret = port2.write(&frame_bytes).await;
//                     // println!("Wrote CAN frame: {:?}", can_frame);
//                     // ret
//                     port2.read(&mut frame_read).await
//                     // println!("Wrote CAN frame1: {:?}", frame_bytes);
//                     // let _ = port2.read().await;
//                 }
//             );
// 
//             match fut1.await {
//                 Ok(Ok(n)) => {
//                     println!("Received CAN frame: {:?}", n);
//                 }
//                 Ok(Err(e)) => {
//                     log::error!("Error reading from CAN port: {:?}", e);
//                 }
//                 Err(_) => {
//                     // log::error!("Timeout waiting for CAN response");
//                 }
//             }
//         }
//     }
// 
// 
//     
//     // let fut_vec = vec![
//     //     actuator::ActuatorState::PreWrite,
//     //     actuator::ActuatorState::PreWrite,
//     // ];
// 
//     // futures::future::join_all(fut_vec).await;
//     // let act_fut1 = actuator::ActuatorState::PreWrite;
//     // let act_fut2 = actuator::ActuatorState::PreWrite;
// 
//     // tokio::join!(act_fut1, act_fut2);
// 
//     // let ret = act_fut.await;
//     // println!("Actuator state: {:?}", ret);
//     println!("done");
//     // let id_req = ObtainIdRequest::new(0xFD, 0x21);
// 
//     // let frame_bytes = unsafe {
//     //     std::mem::transmute::<CanFrame, [u8; 16]>(id_req.into())
//     // };
// 
// 
//     // let fut1 = tokio::time::timeout(
//     //     std::time::Duration::from_secs(1),
//     //     async {
//     //         port2.write(&frame_bytes).await?;
//     //         port2.read().await
//     //         // println!("Wrote CAN frame1: {:?}", frame_bytes);
//     //         // let _ = port2.read().await;
//     //     }
//     // );
//     // fut1.await?;
// 
//     // let frame_bytes: [u8; 16] = [
//     // // can_id = 102 → 0x00000066
//     // 0x66, 0x00, 0x00, 0x00,
//     // // can_dlc = 4
//     // 0x04,
//     // // __pad, __res0, __res1
//     // 0x00, 0x00, 0x00,
//     // // data[0..4] = 0xCA 0xFE 0xBA 0xBE
//     // 0xCA, 0xFE, 0xBA, 0xBE,
//     // // data[4..8] = padding
//     // 0x00, 0x00, 0x00, 0x00,
//     // ];
// 
//     // let frame_bytes2: [u8; 16] = [
//     // // can_id = 102 → 0x00000066
//     // 0x67, 0x00, 0x00, 0x00,
//     // // can_dlc = 4
//     // 0x04,
//     // // __pad, __res0, __res1
//     // 0x00, 0x00, 0x00,
//     // // data[0..4] = 0xCA 0xFE 0xBA 0xBE
//     // 0xCA, 0xFE, 0xBA, 0xBE,
//     // // data[4..8] = padding
//     // 0x00, 0x00, 0x00, 0x00,
//     // ];
// 
//     // let _ = port2.write(&frame_bytes).await?;
//     // let _ = port2.write(&frame_bytes2).await?;
//     // let fut2 = tokio::time::timeout(
//     //     std::time::Duration::from_secs(2),
//     //     async {
//     //         port2.write(&frame_bytes2).await?;
//     //         // println!("Wrote CAN frame2: {:?}", frame_bytes2);
//     //         port2.read().await
//     //     }
//     // );
// 
//     // fut2.await?;
// 
//     // run them one after the other
//     
//     Ok(())
// 
//     // loop {
//     //     // make a dummy can message 102#cafebabe
//     //     port2.write(&frame_bytes).await?;
//     //     // tokio::time::sleep(std::time::Duration::from_micros(100)).await;
//     //     // log::info!("Wrote CAN frame: {:?}", frame_bytes);
//     //     port2.read().await?;
//     // }
// }


use std::io::Write;
use log::Level;
use chrono::Local;
use log::LevelFilter;

fn main() {

    // setup telemetry before we do anything else
    let (tx, rx) = mpsc::sync_channel::<EventRecord>(1024 * 1024);

    //spawn the thread that will format and log our data
    let jh = start_pipeline(rx, "events.log")
                .expect("Failed to start telemetry pipeline");

    // prepare tracing to forward to our thread
    let layer = HeaplessForwardLayer { tx };
    let subscriber = tracing_subscriber::registry().with(layer);

    let guard: tracing::subscriber::DefaultGuard = 
        tracing::subscriber::set_default(subscriber);
    
    env_logger::Builder::new()
        // Set a default log level; users can still override with RUST_LOG=…
        .filter_level(log::LevelFilter::Info)

        // Provide our own format closure
        .format(|buf, record| {
            // A) Timestamp (wall‐clock, up to nanoseconds):
            let now = Local::now();
            // e.g. "2025-06-01 14:23:45.123456789"
            let timestamp = format!(
                "{}.{:09}",
                now.format("%Y-%m-%d %H:%M:%S"),
                now.timestamp_subsec_nanos()
            );

            // B) Module path (caller) or target:
            // record.module_path() is an Option<&str>. Fallback to record.target().
            let module = record.module_path().unwrap_or(record.target());

            // C) Colourize level:
            // ANSI codes: 31 = red, 33 = yellow, 0 = reset
            // let level = match record.level() {
            //     Level::Error => format!("\x1b[31mERROR\x1b[0m"), // red
            //     Level::Warn  => format!("\x1b[33mWARN \x1b[0m"), // yellow
            //     Level::Info  => "INFO ".to_string(),             // no colour
            //     Level::Debug => "DEBUG".to_string(),
            //     Level::Trace => "TRACE".to_string(),
            // };
            let level = match record.level() {
                Level::Error => format!("\x1b[31mERROR\x1b[0m"),  // red
                Level::Warn  => format!("\x1b[33mWARN \x1b[0m"),  // yellow
                Level::Info  => format!("\x1b[36mINFO \x1b[0m"),  // light blue (ANSI 94)
                Level::Debug => format!("\x1b[37mDEBUG\x1b[0m"),  // white  (ANSI 37)
                Level::Trace => format!("\x1b[90mTRACE\x1b[0m"),  // gray   (ANSI 90)
            };


            // D) Finally, write the line:
            //   [timestamp] [module] [LEVEL] message
            writeln!(
                buf,
                "{} [{}] [{}] {}",
                timestamp,
                level,
                module,
                record.args()
            )
        })

        // E) Send output to stdout (you can change to stderr if desired)
        // .chain(std::io::stdout())

        // F) Install the logger
        .init();
    let start_time = std::time::Instant::now();
    log::info!("id: {:?} starting at {:?}", std::thread::current().id(), start_time);

    // send_can().unwrap();

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build().unwrap();

    // let rt = tokio::runtime::Builder::new_multi_thread()
    //     .worker_threads(2)
    //     .enable_time()
    //     .build().unwrap();


    // let rt = tokio::runtime::Runtime::new().unwrap();

    let drv = driver2();
    let start = std::time::Instant::now();
    if let Err(e) = rt.block_on(drv) {
        log::error!("Error: {:?}", e);
    } else {
        log::info!("Driver finished successfully");
    }
    let elapsed = start.elapsed();
    log::info!("Elapsed time: {:?}", elapsed);
}

//#[tokio::main]
//async fn main() -> Result<(), Box<dyn std::error::Error>> {
//    // Initialize multiple serial ports
//    // let mut port2 = tokio_serial::new("/dev/ttyUSB0", 230400).open_native_async()?;
//    // let mut port1 = tokio_serial::new("/dev/ttyUSB1", 115200).open_native_async()?;
//
//    // let hiwonder_handler = hiwonder::HiwonderImu {};
//    // let port1 = typestate_serial::SerialPort::<
//    //     typestate_serial::Localize,
//    //     hiwonder::HiwonderImu>::new("/dev/ttyUSB1", hiwonder_handler).await?;
//    // println!("Port 1: {:?}", port1);
//    // let port1 = port1.detect_baud().await?;
//    // println!("Port 1: {:?}", port1);
//
//    let socketcan_handler = socketcan::CanSocket {};
//    let port2 = typestate_socket::SocketPort::<
//        typestate_socket::Localize,
//        socketcan::CanSocket>::new("vcan0", socketcan_handler).await?;
//    println!("Port 2: {:?}", port2);
//    let port2 = port2.establish().await?;
//    println!("Port 2: {:?}", port2);
//
//    // let ethernet_handler = socketcan::EthernetSocket {};
//
//    // let port3 = typestate_socket::SocketPort::<
//    //     typestate_socket::Localize,
//    //     socketcan::EthernetSocket>::new("eno2", ethernet_handler).await?;
//
//    // println!("Port 3: {:?}", port3);
//    // let port3 = port3.establish().await?;
//    // println!("Port 3: {:?}", port3);
//    // let mut serial_fds: Vec<AsyncFd<SerialStream>> = ports
//    //     .iter()
//    //     .map(|p| AsyncFd::with_interest(p.as_raw_fd(), Interest::READABLE).unwrap())
//    //     .collect();
//
//    // Initialize a TCP listener
//    // let listener = TcpListener::bind("0.0.0.0:8080").await?;
//    // println!("Listening on 0.0.0.0:8080 and monitoring {} ports", serial_fds.len());
//
//
//    // driver().await;
//    // return Ok(());
//
//    println!("Starting driver");
//    Ok(())
//    // let mut buf1 = [0u8; 1024];
//    // let mut buf0 = [0u8; 1024];
//    // loop {
//    //     tokio::select! {
//    //         Ok(n) = port1.read(&mut buf0) => {
//    //             buf0[n] = 0;
//    //             // println!("Read {} bytes from port 0", n);
//    //         }
//    //         Ok(n) = port2.read(&mut buf1) => {
//    //             // println!("Read {} bytes from port 1", n);
//    //         }
//    //     }
//    // }
//
//    // loop {
//    //     tokio::select! {
//    //         biased;
//    //         Ok(g) = port1.readable() => {
//    //             let mut buf = [0u8; 1024];
//    //             println!("g is {:?}", g);
//    //             match port1.try_read(&mut buf) {
//    //                 Ok(n) => {
//    //                     println!("Read {} bytes from port 0: {:?}", n, &buf[..n]);
//    //                 }
//    //                 // Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
//    //                 //     // No data right now (stale readiness), just ignore.
//    //                 // }
//    //                 Err(e) => {
//    //                     // eprintln!("Error reading from port 0: {}", e);
//    //                 }
//    //             }
//    //         }
//    //         // _ = port1.readable() => {
//    //         //     println!("Port 0 is readable");
//    //         //     let mut buf = [0u8; 1024];
//    //         //     match port1.try_read(&mut buf) {
//    //         //         Ok(n) => println!("Read {} bytes from port 0", n),
//    //         //         Err(e) => eprintln!("Error reading from port 0: {}", e),
//    //         //     }
//    //         // }
//    //         // _ = ports[1].readable() => {
//    //         //     println!("Port 1 is readable");
//    //         //     let mut buf = [0u8; 1024];
//    //         //     match ports[1].try_read(&mut buf) {
//    //         //         Ok(n) => println!("Read {} bytes from port 1", n),
//    //         //         Err(e) => eprintln!("Error reading from port 1: {}", e),
//    //         //     }
//    //         // }
//    //     }
//    // }
//
//    // loop {
//    //     // Wait for either serial readiness or a new socket connection
//    //     tokio::select! {
//    //         // Handle new TCP connections
//    //         Ok((socket, addr)) = listener.accept() => {
//    //             println!("Accepted connection from {}", addr);
//    //             tokio::spawn(handle_socket(socket));
//    //         }
//
//    //         // Iterate over all serial fds and process the first one ready
//    //         Some(idx) = async {
//    //             // Find first ready serial fd index
//    //             for (i, afd) in serial_fds.iter().enumerate() {
//    //                 if afd.poll_read_ready().is_ready() {
//    //                     return Some(i);
//    //                 }
//    //             }
//    //             None
//    //         } => {
//    //             // Clear readiness and read
//    //             let mut afd = serial_fds[idx].readable().await?;
//    //             // Read from the underlying SerialStream
//    //             let port = &mut ports[idx];
//    //             handle_serial(port).await?;
//    //             afd.clear_ready();
//    //         }
//    //     }
//    // }
//    // Ok(())
//}
