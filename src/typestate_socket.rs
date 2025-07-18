use std::io;
use std::marker::PhantomData;
use std::os::fd::{AsFd, AsRawFd};
use std::os::unix::io::RawFd;
use std::time::Duration;
use tokio::io::unix::AsyncFd;
use tokio::io::{AsyncRead, AsyncReadExt};
use tokio::io::{AsyncWrite, AsyncWriteExt};
use tokio::time::timeout;
use tracing::{debug, error, info, warn};

use std::pin::Pin;
use std::task::{Context, Poll, ready};
use tokio::io::ReadBuf;

pub trait BytesHandler {
    fn verify_read(buf: &[u8]) -> io::Result<()>;

    fn socket_domain() -> socket2::Domain;
    fn socket_type() -> socket2::Type;
    fn socket_protocol() -> Option<socket2::Protocol>;

    fn sockaddr(ifname: &str) -> socket2::SockAddr;
}

struct ByteStreamFd {
    inner: AsyncFd<RawFd>,
}

impl AsyncRead for ByteStreamFd {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        // Wait until readable
        let mut ready = ready!(self.inner.poll_read_ready(cx))?;
        // Attempt the actual read
        let result = ready.try_io(|_| {
            // get a pointer & length to unfilled part of ReadBuf
            let unfilled = buf.initialize_unfilled();
            let n = unsafe {
                libc::read(
                    self.inner.as_raw_fd(),
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
            // Successfully read some bytes
            Ok(Ok(0)) => Poll::Ready(Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Socket read returned 0 bytes",
            ))),
            Ok(Ok(n)) => {
                // Fill the ReadBuf with the read bytes
                buf.advance(n);
                Poll::Ready(Ok(()))
            }
            // Would block — try again later
            Err(_would_block) => {
                debug!("Would block");
                Poll::Pending
            }
            Ok(Err(e)) => {
                // An error occurred
                Poll::Ready(Err(e))
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
        // 1) Wait for the FD to be writable
        let mut ready = ready!(self.inner.poll_write_ready(cx))?;

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
                Err(io::Error::last_os_error())
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

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        // No internal buffer to flush, so we're always "flushed"
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        // Shutdown only the write side of the socket
        let mut ready = ready!(self.inner.poll_write_ready(cx))?;
        match ready.try_io(|_| {
            // SAFETY: shutting down a valid socket fd
            let ret = unsafe { libc::shutdown(self.inner.as_raw_fd(), libc::SHUT_WR) };
            if ret < 0 {
                Err(io::Error::last_os_error())
            } else {
                Ok(())
            }
        }) {
            Ok(Ok(())) => Poll::Ready(Ok(())),
            Ok(Err(e)) => Poll::Ready(Err(e)),
            Err(_would_block) => Poll::Pending,
        }
    }
}

#[derive(Debug)]
pub struct Localize;
#[derive(Debug)]
pub struct Configure;
#[derive(Debug)]
pub struct BringUp;
#[derive(Debug)]
pub struct Operate;
#[derive(Debug)]
pub struct Recover;

pub enum SocketState<Handler: BytesHandler> {
    Localize(SocketPort<Localize, Handler>),
    Configure(SocketPort<Configure, Handler>),
    BringUp(SocketPort<BringUp, Handler>),
    Operate(SocketPort<Operate, Handler>),
    Recover(SocketPort<Recover, Handler>),
}

#[derive(Debug)]
pub struct SocketPort<State, Handler: BytesHandler> {
    socket: socket2::Socket,
    handler: Handler,
    _state: PhantomData<State>,
}

impl<Handler: BytesHandler> SocketPort<Localize, Handler> {
    /// Open the socket at a *default* port.  
    /// You’ll probe other ports in `detect_port`.
    pub async fn new(ifname: &str, handler: Handler) -> io::Result<Self> {
        let socket = socket2::Socket::new(
            Handler::socket_domain(),
            Handler::socket_type(),
            Handler::socket_protocol(),
        )?;

        socket.set_nonblocking(true)?;

        info!("Socket created: {:?}", socket);
        let sockaddr = Handler::sockaddr(ifname);
        socket.bind(&sockaddr)?;
        Ok(SocketPort {
            socket,
            handler,
            _state: PhantomData,
        })
    }

    pub async fn establish(self) -> io::Result<SocketPort<Operate, Handler>> {
        // read some bytes
        let mut buf = [0u8; 128];
        let mut bytestream = ByteStreamFd {
            inner: AsyncFd::new(self.socket.as_raw_fd())?,
        };
        match bytestream.read(&mut buf).await {
            Ok(0) => Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Socket read returned 0 bytes",
            )),
            Ok(n) => {
                debug!("Socket read {} bytes", n);
                // verify the read bytes
                if let Err(e) = Handler::verify_read(&buf[..n]) {
                    warn!("Socket read verification failed: {}", e);
                    return Err(e);
                }
                Ok(SocketPort {
                    socket: self.socket,
                    handler: self.handler,
                    _state: PhantomData,
                })
            }
            Err(e) => {
                error!("Socket read failed: {}", e);
                Err(e)
            }
        }
        // match timeout(Duration::from_millis(5000), async {
        //     ByteStreamFd {
        //         inner: AsyncFd::new(self.socket.as_raw_fd())?,
        //     }.read(&mut buf).await
        // }).await {
        //     Err(_) => {
        //         return Err(io::Error::new(io::ErrorKind::TimedOut, "Socket read timed out"));
        //     },
        //     Ok(Err(e)) => Err(e),
        //     Ok(Ok(n)) => {
        //         if n == 0 {
        //             return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "Socket read returned 0 bytes"));
        //         }
        //         println!("Socket read {} bytes", n);
        //         // verify the read bytes
        //         if let Err(e) = Handler::verify_read(&buf[..n]) {
        //             eprintln!("Socket read verification failed: {}", e);
        //             Err(e)
        //         } else {
        //             Ok(SocketPort {
        //                 socket: self.socket,
        //                 handler: self.handler,
        //                 _state: PhantomData,
        //             })
        //         }
        //     }
        // }
    }

    pub async fn verify_read(&self, buf: &[u8]) -> io::Result<()> {
        Handler::verify_read(buf)
    }
}

impl<Handler: BytesHandler> SocketPort<Operate, Handler> {
    pub async fn new(ifname: &str, handler: Handler) -> io::Result<Self> {
        let socket = socket2::Socket::new(
            Handler::socket_domain(),
            Handler::socket_type(),
            Handler::socket_protocol(),
        )?;

        socket.set_nonblocking(true)?;

        info!("Socket created: {:?}", socket);
        let sockaddr = Handler::sockaddr(ifname);
        socket.bind(&sockaddr)?;
        Ok(SocketPort {
            socket,
            handler,
            _state: PhantomData,
        })
    }

    pub async fn read(&self) -> io::Result<usize> {
        // read some bytes
        let mut buf = [0u8; 128];
        let mut bytestream = ByteStreamFd {
            inner: AsyncFd::new(self.socket.as_raw_fd())?,
        };
        match bytestream.read(&mut buf).await {
            Ok(0) => Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Socket read returned 0 bytes",
            )),
            Ok(n) => {
                // verify the read bytes
                debug!("Socket read {} bytes", n);
                // if let Err(e) = Handler::verify_read(&buf[..n]) {
                //     eprintln!("Socket read verification failed: {}", e);
                //     return Err(e);
                // }
                Ok(n)
            }
            Err(e) => {
                error!("Socket read failed: {}", e);
                Err(e)
            }
        }
    }

    pub async fn write(&self, buf: &[u8]) -> io::Result<usize> {
        // self.socket.write(buf).await?;
        // let sockaddr = Handler::sockaddr("vcan0");
        // self.socket.send_to(buf, &sockaddr)
        // write some bytes
        let mut bytestream = ByteStreamFd {
            inner: AsyncFd::new(self.socket.as_raw_fd())?,
        };
        match bytestream.write(buf).await {
            Ok(n) => Ok(n),
            Err(e) => {
                error!("Socket write failed: {}", e);
                Err(e)
            }
        }
    }
}
