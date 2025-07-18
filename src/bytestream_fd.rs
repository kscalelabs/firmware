use std::os::fd::AsRawFd;
use tokio::io::ReadBuf;
use tokio::io::unix::AsyncFd;
use tokio::io::{AsyncRead, AsyncReadExt};
use tokio::io::{AsyncWrite, AsyncWriteExt};
use tracing::{debug, warn};
// use std::os::fd::{AsFd, AsRawFd};
use std::os::unix::io::RawFd;
use std::pin::Pin;
use std::task::{Context, Poll, ready};

use std::io::{self, ErrorKind};

#[derive(Debug)]
pub struct ByteStreamFd {
    inner: AsyncFd<RawFd>,
}

impl ByteStreamFd {
    pub fn new(fd: RawFd) -> std::io::Result<Self> {
        let inner = AsyncFd::new(fd)?;
        Ok(ByteStreamFd { inner })
    }

    pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        // SAFETY: We're just calling libc::read on a valid fd
        let n = unsafe {
            libc::read(
                self.inner.as_raw_fd(),
                buf.as_mut_ptr() as *mut libc::c_void,
                buf.len(),
            )
        };
        if n < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(n as usize)
        }
    }
}

impl Drop for ByteStreamFd {
    fn drop(&mut self) {
        warn!("ByteStreamFd dropped, fd: {}", self.inner.as_raw_fd());
    }
}

impl AsyncRead for ByteStreamFd {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        let this = self.get_mut();
        loop {
            // 1) Wait (or immediately get) a readiness guard
            let mut guard = ready!(this.inner.poll_read_ready(cx))?;
            // 2) Try the syscall
            let result = guard.try_io(|_| {
                let unfilled = buf.initialize_unfilled();
                let n = unsafe {
                    libc::read(
                        this.inner.as_raw_fd(),
                        unfilled.as_mut_ptr() as *mut libc::c_void,
                        unfilled.len(),
                    )
                };
                if n < 0 {
                    Err(io::Error::last_os_error())
                } else {
                    Ok(n as usize)
                }
            });

            match result {
                // Real EOF
                Ok(Ok(0)) => {
                    return Poll::Ready(Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "socket read returned 0 bytes",
                    )));
                }
                // Successfully read some bytes
                Ok(Ok(n)) => {
                    buf.advance(n);
                    return Poll::Ready(Ok(()));
                }
                // Error from read()
                Ok(Err(e)) => return Poll::Ready(Err(e)),
                // Not actually ready: clear readiness and retry
                Err(_would_block) => {
                    // loop around to re-register waker
                    continue;
                }
            }
        }
    }
}

impl AsyncWrite for ByteStreamFd {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        loop {
            // 1) syscall immediately
            let ret =
                unsafe { libc::write(self.inner.as_raw_fd(), buf.as_ptr() as *const _, buf.len()) };
            if ret >= 0 {
                return Poll::Ready(Ok(ret as usize));
            }
            let err = io::Error::last_os_error();
            if err.kind() != ErrorKind::WouldBlock {
                return Poll::Ready(Err(err));
            }

            // 2) on EWOULDBLOCK, await writable
            let mut guard = ready!(self.inner.poll_write_ready(cx))?;
            // 3) clear the readiness flag so epoll re-arms next time
            guard.clear_ready();
            // and loop to retry the syscall
        }
    }

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        // no buffering, always flushed
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        loop {
            // 1) try shutdown syscall
            let ret = unsafe { libc::shutdown(self.inner.as_raw_fd(), libc::SHUT_WR) };
            if ret == 0 {
                return Poll::Ready(Ok(()));
            }
            let err = io::Error::last_os_error();
            if err.kind() != ErrorKind::WouldBlock {
                return Poll::Ready(Err(err));
            }

            // 2) await writable
            let mut guard = ready!(self.inner.poll_write_ready(cx))?;
            // 3) clear and retry
            guard.clear_ready();
        }
    }
}

/*
impl AsyncWrite for ByteStreamFd {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<std::io::Result<usize>> {
        // 1) Wait for the FD to be writable
        debug!("Poll write ready");
        let mut ready = ready!(self.inner.poll_write_ready(cx))?;
        debug!("Write ready confirmed");

        // 2) Try to do the write without blocking
        match ready.try_io(|_| {
            // SAFETY: We're just calling libc::write on a valid fd
            let ret = unsafe {
                libc::write(
                    self.inner.as_raw_fd(),
                    buf.as_ptr() as *const libc::c_void,
                    buf.len(),
                )
            };
            if ret < 0 {
                Err(std::io::Error::last_os_error())
            } else {
                Ok(ret as usize)
            }
        }) {
            // Successfully wrote `n` bytes
            Ok(Ok(n)) => Poll::Ready(Ok(n)),

            // Error during write
            Ok(Err(e)) => Poll::Ready(Err(e)),

            // Would block — go back to Pending
            Err(_would_block) => Poll::Pending,
        }
    }

    fn poll_flush(
        self: Pin<&mut Self>,
        _cx: &mut Context<'_>,
    ) -> Poll<std::io::Result<()>> {
        // No internal buffer to flush, so we're always "flushed"
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<std::io::Result<()>> {
        // Shutdown only the write side of the socket
        let mut ready = ready!(self.inner.poll_write_ready(cx))?;
        match ready.try_io(|_| {
            // SAFETY: shutting down a valid socket fd
            let ret = unsafe { libc::shutdown(self.inner.as_raw_fd(), libc::SHUT_WR) };
            if ret < 0 {
                Err(std::io::Error::last_os_error())
            } else {
                Ok(())
            }
        }) {
            Ok(Ok(()))      => Poll::Ready(Ok(())),
            Ok(Err(e))      => Poll::Ready(Err(e)),
            Err(_would_block) => Poll::Pending,
        }
    }
}
*/
