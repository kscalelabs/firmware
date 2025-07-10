use std::fs::OpenOptions;
use std::io;
use std::os::unix::io::{
    AsFd,
    AsRawFd,
    OwnedFd,
};

use std::net::UdpSocket;
use std::os::unix::fs::OpenOptionsExt;

use std::ptr;
use std::thread;
use std::time::Instant;

use std::sync::mpsc::Receiver;
use crate::error;
use nix::libc;

use crate::telemetry::forwarder::EventRecord;
use crate::telemetry::multi_fd_writer::MultiFdWriter;

// 4 pages
const BUF_SIZE: usize = 4 * 4096;

/// A pipelined writer that uses mmap buffer with splice pipeline for zero-copy writes to both disk and network.
// return  join handle
pub fn start_pipeline(rx: Receiver<EventRecord>, log_path: &str) -> io::Result<thread::JoinHandle<()>> {
    // Open log file for writing 
    let file = OpenOptions::new()
        .create(true)
        .truncate(true) // Ensure the file is empty at start
        .write(true)
        .custom_flags(libc::O_DIRECT) // Use O_DIRECT for zero-copy
        .open(log_path)?;

    /* Examples of additional file descriptors
     * Uncomment if you want to use multiple outputs like a second file or a network socket
     */

    // let file2 = OpenOptions::new()
    //     .create(true)
    //     .truncate(true) // Ensure the file is empty at start
    //     .write(true)
    //     .custom_flags(libc::O_DIRECT) // Use O_DIRECT for zero-copy
    //     .open("second.log")?;

    // make udp socket
    // let udp = UdpSocket::bind("0.0.0.0:0")?;
    // udp.connect("10.33.10.156:5656")?;

    // Spawn worker thread to handle I/O
    let jh = thread::spawn(move || {
        // Keep file alive for the entire thread duration

        // Create array of output file descriptors
        let output_fds = vec![
            OwnedFd::from(file),
            // OwnedFd::from(file2),
            // OwnedFd::from(udp),
        ];

        let mut last_flush = Instant::now();
        // Create multi-fd writer for zero-copy writes to multiple destinations
        let mut writer = match MultiFdWriter::new(output_fds) {
            Ok(w) => w,
            Err(e) => {
                error!("Failed to create multi-fd writer: {}", e);
                return;
            }
        };

        // Allocate a 4K-aligned buffer via mmap
        let raw_ptr = unsafe {
            nix::libc::mmap(
                ptr::null_mut(),
                BUF_SIZE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };

        if raw_ptr == libc::MAP_FAILED {
            error!("mmap failed: {}", io::Error::last_os_error());
            panic!("mmap failed");
        }
        let mut batch_buf: Vec<u8> = unsafe { Vec::from_raw_parts(raw_ptr as *mut u8, 0, BUF_SIZE) };

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
            unsafe { batch_buf.set_len(0); }
            last_flush = Instant::now();
        }

        // Final flush if buffer has data
        if !batch_buf.is_empty() {
            writer.add_data(batch_buf.clone());
            while let Ok(true) = writer.drive() {
                // Drive until completion
            }
        }

        // Unmap the buffer when done
        unsafe { libc::munmap(raw_ptr, BUF_SIZE); }
    });

    Ok(jh)
}

/// Simple serialization stub
fn serialize_event(evt: &EventRecord, buf: &mut Vec<u8>) {
    let s = format!("{:?}\n", evt);
    buf.extend_from_slice(s.as_bytes());
}

