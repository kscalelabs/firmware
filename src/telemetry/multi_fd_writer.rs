use crate::telemetry::utils::{Page, RingEntry};
use io_uring::{IoUring, opcode, types};
use nix::libc;
use std::collections::VecDeque;
use std::io;
use std::os::unix::io::{AsRawFd, BorrowedFd, OwnedFd, RawFd};
use tracing::{debug, error, info, trace};

const PAGE_SIZE: usize = 4096;

pub struct IoStats {
    pub total_bytes_written: usize,
    pub total_duration: std::time::Duration,
    pub last_log_time: std::time::Instant,
    pub user_owned: usize,
}

impl Default for IoStats {
    fn default() -> Self {
        IoStats {
            total_bytes_written: 0,
            total_duration: std::time::Duration::new(0, 0),
            last_log_time: std::time::Instant::now(),
            user_owned: 0,
        }
    }
}

pub struct BufferedIoUring {
    ring: IoUring,
    free_list: std::collections::VecDeque<RingEntry>,
    ready_list: std::collections::VecDeque<RingEntry>,
    pending_ops: Vec<u8>,
    base_addr: *const u8,
    num_pages: usize,
}

impl Drop for BufferedIoUring {
    fn drop(&mut self) {
        // Unregister buffers and clean up
        unsafe {
            self.ring.submitter().unregister_buffers().ok();
            libc::munmap(
                self.base_addr as *mut libc::c_void,
                self.num_pages * PAGE_SIZE,
            );
        }
    }
}

impl BufferedIoUring {
    pub fn new(num_pages: usize) -> io::Result<Self> {
        let raw_ptr = unsafe {
            nix::libc::mmap(
                std::ptr::null_mut(),
                num_pages * PAGE_SIZE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };

        if std::ptr::eq(raw_ptr, libc::MAP_FAILED) {
            panic!("mmap failed: {}", io::Error::last_os_error());
        }
        if unsafe { libc::mlock(raw_ptr, num_pages * PAGE_SIZE) } != 0 {
            return Err(io::Error::last_os_error());
        }

        // now we make a vector of iovec and our free list
        let mut io_vec = Vec::<libc::iovec>::with_capacity(num_pages);
        let mut free_list = std::collections::VecDeque::with_capacity(num_pages);

        for i in 0..num_pages {
            let slice = unsafe {
                libc::iovec {
                    iov_base: (raw_ptr as *mut u8).add(i * PAGE_SIZE) as *mut libc::c_void,
                    iov_len: PAGE_SIZE,
                }
            };
            free_list.push_back(RingEntry::new(
                unsafe { Page::new(slice.iov_base as *mut u8, PAGE_SIZE) },
                i,
            ));
            io_vec.push(slice);
        }

        let io_uring = IoUring::builder()
            .setup_sqpoll(10)
            .build(128)
            .map_err(io::Error::other)?;

        info!("Registered {} buffers with io_uring", num_pages);
        unsafe {
            io_uring
                .submitter()
                .register_buffers(io_vec.as_slice())
                .map_err(io::Error::other)?;
        }
        Ok(BufferedIoUring {
            ring: io_uring,
            free_list,
            ready_list: std::collections::VecDeque::new(),
            pending_ops: vec![0; num_pages],
            base_addr: raw_ptr as *const u8,
            num_pages,
            // idx: 0,
        })
    }

    fn user_data_from_entry(base_addr: *const u8, entry: &RingEntry) -> u64 {
        // encode the offset from base addr
        unsafe { entry.as_ptr().offset_from(base_addr) as u64 }
    }

    fn entry_from_user_data(base_addr: *const u8, user_data: u64) -> RingEntry {
        // decode the offset from base addr
        let idx = (user_data as usize) / PAGE_SIZE;
        RingEntry {
            page: unsafe { Page::new(base_addr.add(idx * PAGE_SIZE) as *mut u8, PAGE_SIZE) },
            idx,
        }
    }
}

pub struct MultiFdWriter {
    ring: BufferedIoUring,
    output_fds: Vec<OwnedFd>,
    pending_data: VecDeque<Vec<u8>>,
    io_stats: IoStats,
}

impl MultiFdWriter {
    pub fn new(output_fds: Vec<std::os::unix::io::OwnedFd>) -> io::Result<Self> {
        let num = 4096;
        let ring = BufferedIoUring::new(num)?;
        Ok(MultiFdWriter {
            ring,
            output_fds,
            pending_data: VecDeque::new(),
            io_stats: IoStats {
                total_bytes_written: 0,
                total_duration: std::time::Duration::new(0, 0),
                last_log_time: std::time::Instant::now(),
                user_owned: num,
            },
        })
    }

    pub fn add_data(&mut self, data: Vec<u8>) {
        self.pending_data.push_back(data);
    }

    pub fn flush(&mut self) -> io::Result<()> {
        if !self.ring.free_list.is_empty() {
            // Get the next free iovec
            let entry = self.ring.free_list.front_mut().unwrap();
            if !entry.page.is_empty() {
                // fill the page with zeros as for some reason, io_uring doesn't work if less than
                // a page is written
                entry
                    .page
                    .fill_from(&mut std::io::Cursor::new(vec![0; entry.page.remaining()]))
                    .map_err(std::io::Error::other)?;
                println!(
                    "Flushing entry: idx={}, remaining: {}",
                    entry.idx(),
                    entry.page.remaining()
                );
                self.ring
                    .ready_list
                    .push_back(self.ring.free_list.pop_front().unwrap());
            }
        }

        self.drain_ready_list()?;

        while self.io_stats.user_owned < self.ring.num_pages {
            println!(
                "Flushing MultiFdWriter, user_owned: {}, num_pages: {}",
                self.io_stats.user_owned, self.ring.num_pages
            );
            self.handle_completions();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        Ok(())
    }

    fn handle_completions(&mut self) {
        let ring = &mut self.ring.ring;
        let base_addr = self.ring.base_addr;

        let cq = ring.completion();
        cq.for_each(|cqe| {
            let res = cqe.result();
            let _flags = cqe.flags();

            let user_data = cqe.user_data();

            if res < 0 {
                let err = nix::Error::from_raw(-res);
                error!(
                    "io_uring operation failed: user_data={}, error={}",
                    user_data, err
                );
            }

            let entry = BufferedIoUring::entry_from_user_data(base_addr, user_data);
            self.ring.pending_ops[entry.idx()] -= 1;
            if self.ring.pending_ops[entry.idx()] == 0 {
                self.ring.free_list.push_back(entry);
                self.io_stats.user_owned += 1;
            }
            // println!("ready list size: {}", self.ring.ready_list.len());
            // println!("free list size: {}", self.ring.free_list.len());
        });
    }

    /// Call this repeatedly to drive the pipeline
    pub fn drive(&mut self) -> io::Result<bool> {
        // for every second since start, log the current throughput
        if self.io_stats.last_log_time.elapsed().as_secs() >= 1 {
            debug!(
                "user owned: {}, total duration: {:?}",
                self.io_stats.user_owned, self.io_stats.total_duration
            );
            self.io_stats.last_log_time = std::time::Instant::now();
        }

        self.handle_completions();

        // Try to submit new work if we have pending data
        if let Some(data) = self.pending_data.pop_front() {
            self.submit_buffer(data)?;
            return Ok(true); // More work to do
        }

        Ok(false) // No more work
    }

    fn submit_buffer(&mut self, data: Vec<u8>) -> io::Result<()> {
        // copy vector into buffered io uring
        trace!("Submitting buffer of size: {}", data.len());
        if data.len() > PAGE_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Data exceeds PAGE_SIZE",
            ));
        }

        // read all data into ring buffer of pages
        let mut cursor = std::io::Cursor::new(&data);
        while cursor.position() < data.len() as u64 {
            // Check if we have enough free iovecs
            // println!("Free list size: {}", self.ring.free_list.len());
            if self.ring.free_list.is_empty() {
                return Err(io::Error::other("No ring buffers available"));
            }

            // Get the next free iovec
            let entry = self.ring.free_list.front_mut().unwrap();
            entry
                .fill_from(&mut cursor)
                .map_err(std::io::Error::other)?;
            if entry.page.remaining() == 0 {
                self.ring
                    .ready_list
                    .push_back(self.ring.free_list.pop_front().unwrap());
            }
        }

        // now we push all ready pages into io_uring
        self.drain_ready_list()?;

        // Let data drop naturally after operations complete
        Ok(())
    }

    fn drain_ready_list(&mut self) -> std::io::Result<usize> {
        let mut operations = Vec::new();
        let ring = &mut self.ring.ring;

        // drain ready queue first
        self.ring.ready_list.drain(..).for_each(|entry| {
            self.output_fds.iter().for_each(|fd| {
                // Create a new write operation for each page
                operations.push(
                    opcode::WriteFixed::new(
                        types::Fd(fd.as_raw_fd()),
                        entry.as_ptr() as _,
                        entry.len() as u32,
                        entry.idx() as u16, // fixed buffer index
                    )
                    .offset(u64::MAX) // offset -1 means use the file descriptor's current position
                    .build()
                    .user_data(BufferedIoUring::user_data_from_entry(
                        self.ring.base_addr,
                        &entry,
                    )),
                );
                self.ring.pending_ops[entry.idx()] += 1;
                trace!(
                    "Pushing write operation for entry idx={} with ptr={:p} and len={}",
                    entry.idx(),
                    entry.as_ptr(),
                    entry.len()
                );
            });
            self.io_stats.user_owned -= 1;
        });

        // Submit all operations as a batch
        unsafe {
            for op in &operations {
                ring.submission().push(op).map_err(std::io::Error::other)?;
            }
        }
        // Submit the batch of operations
        ring.submit().map_err(io::Error::other)
    }
}

impl Drop for MultiFdWriter {
    fn drop(&mut self) {
        // OwnedFd objects in _pipe_fds will automatically close the file descriptors
    }
}
