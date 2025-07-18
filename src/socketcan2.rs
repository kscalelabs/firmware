use crate::bytestream_fd::ByteStreamFd;
use crate::typestate_socket2::{SocketConfigurator, SocketOperator};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tracing::error;

#[derive(Debug, Default)]
pub struct SocketCanConfigurator;

impl SocketConfigurator for SocketCanConfigurator {
    fn get_domain() -> socket2::Domain {
        socket2::Domain::from(libc::AF_CAN)
    }

    fn get_type() -> socket2::Type {
        socket2::Type::RAW
    }

    fn get_protocol() -> Option<socket2::Protocol> {
        Some(socket2::Protocol::from(libc::CAN_RAW))
    }

    fn get_sockaddr(ifname: &str) -> std::io::Result<socket2::SockAddr> {
        let cstr = std::ffi::CString::new(ifname).unwrap();
        let if_index = unsafe { libc::if_nametoindex(cstr.as_ptr()) };

        if if_index == 0 {
            return Err(std::io::Error::last_os_error());
        }
        let addr = libc::sockaddr_can {
            can_family: libc::AF_CAN as _,
            can_ifindex: if_index as i32,
            can_addr: unsafe { std::mem::zeroed() }, // ignored by the kernel, so we set to 0
        };

        // this can simply be casted to sockaddr_storage on x86 and aarch64 linux gnu
        let mut sockaddr_storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };

        unsafe {
            std::ptr::copy_nonoverlapping(
                &addr as *const _ as *const u8,
                &mut sockaddr_storage as *mut _ as *mut u8,
                std::mem::size_of::<libc::sockaddr_can>(),
            );
        }

        unsafe {
            Ok(socket2::SockAddr::new(
                sockaddr_storage,
                std::mem::size_of::<libc::sockaddr_can>() as u32,
            ))
        }
    }
}

#[derive(Debug)]
pub struct SocketCanOperator {
    bytestream_fd: ByteStreamFd,
}

impl SocketOperator for SocketCanOperator {
    type MsgType = [u8; 16];
    // type MsgType = crate::socketcan::CanFrame;

    fn new(bytestream_fd: ByteStreamFd) -> Self {
        SocketCanOperator { bytestream_fd }
    }

    async fn write(&mut self, msg: &Self::MsgType) -> std::io::Result<usize> {
        self.bytestream_fd.write(msg).await
    }

    fn try_read(&mut self, msg: &mut [u8]) -> std::io::Result<usize> {
        self.bytestream_fd.try_read(msg)
    }

    async fn read(&mut self, msg: &mut Self::MsgType) -> std::io::Result<usize> {
        match self.bytestream_fd.read(msg).await {
            Ok(0) => Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Socket read returned 0 bytes",
            )),
            Ok(n) => {
                // println!("Socket read {} bytes", n);
                Ok(n)
            }
            Err(e) => {
                error!("Socket read failed: {}", e);
                Err(e)
            }
        }
    }
}
