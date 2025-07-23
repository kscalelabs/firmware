use std::fs::OpenOptions;
use std::io;
use std::os::unix::io::{AsFd, AsRawFd, OwnedFd};

use std::net::UdpSocket;
use std::os::unix::fs::OpenOptionsExt;

use std::ptr;
use std::thread;
use std::time::Instant;

use nix::libc;
use std::sync::mpsc::Receiver;
use tracing::error;

use crate::telemetry::forwarder::{EventRecord, FieldValue};
use crate::telemetry::multi_fd_writer::MultiFdWriter;

// 4 pages
const BUF_SIZE: usize = 4 * 4096;

/// spawns a thread that will handle the I/O operations (read events and write to disk)
pub fn start_pipeline(
    rx: Receiver<EventRecord>,
    log_path: &str,
) -> io::Result<thread::JoinHandle<()>> {
    // Open log file for writing
    let file = OpenOptions::new()
        .create(true)
        .truncate(true) // Ensure the file is empty at start
        .write(true)
        .open(log_path)?;

    /**
     * Examples of additional file descriptors
     * let udp = UdpSocket::bind("0.0.0.0:0")?;
     * udp.connect("10.33.10.156:5656")?;
     * ...
     * OwnedFd::from(udp)
     *
     */
    // Spawn worker thread to handle I/O
    let jh = thread::spawn(move || {
        // Keep file alive for the entire thread duration

        // Create array of output file descriptors
        let output_fds = vec![OwnedFd::from(file)];

        let mut last_flush = Instant::now();
        // Create multi-fd writer for zero-copy writes to multiple destinations
        let mut writer = match MultiFdWriter::new(output_fds) {
            Ok(w) => w,
            Err(e) => {
                error!("Failed to create multi-fd writer: {}", e);
                return;
            }
        };

        // TODO use ring buffer that is allocated once
        // do not create a filled vec as we have a limit on max buffer size
        let mut batch_buf: Vec<u8> = Vec::with_capacity(BUF_SIZE);

        while let Ok(evt) = rx.recv() {
            // Serialize event into batch_buf (ensure capacity)
            serialize_event(&evt, &mut batch_buf);

            // Flush when half-buffer or timeout reached
            // Feed buffer data to multi-fd writer
            writer.add_data(batch_buf.clone());

            // Drive the writer until completion
            loop {
                match writer.drive() {
                    Ok(true) => {
                        // More work to do
                        continue;
                    }
                    Ok(false) => {
                        // All operations completed
                        break;
                    }
                    Err(e) => {
                        error!("Multi-fd writer error: {}", e);
                        break;
                    }
                }
            }

            // Reset buffer length without deallocating
            unsafe {
                batch_buf.set_len(0);
            }
            last_flush = Instant::now();
        }

        // print instead of log as at this point, tracing may not exist
        println!("Received all events, flushing remaining data...");

        // Final flush if buffer has data
        if !batch_buf.is_empty() {
            writer.add_data(batch_buf.clone());
            while let Ok(true) = writer.drive() {
                // Drive until completion
            }
        }

        // since we have added all pending data to the writer,
        // we must explicitly call flush to ensure it is written to disk
        writer.flush();

        // Unmap the buffer when done
        println!("Pipeline thread exiting, all data flushed.");
    });

    Ok(jh)
}

/// Simple serialization stub
fn serialize_event(evt: &EventRecord, buf: &mut Vec<u8>) {
    // Look for policy_step field specifically
    for field_record in evt.fields.iter() {
        if field_record.name == "policy_step" {
            match &field_record.value {
                FieldValue::Str(s) => {
                    buf.extend_from_slice(s.as_bytes());
                    buf.extend_from_slice(b"\n");
                    return;
                }
                FieldValue::Debug(s) => {
                    buf.extend_from_slice(s.as_bytes());
                    buf.extend_from_slice(b"\n");
                    return;
                }
                _ => {}
            }
        }
    }
}
