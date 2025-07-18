use crate::typestate_socket::BytesHandler;
use std::io;
use tracing::debug;

use socket2::{Domain, Protocol, Type};

pub const CAN_MAX_DLEN: usize = 8; // Maximum Transmission Unit for CAN frames

/// CAN frame structure taken from linux/include/uapi/linuxcan.h
#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct CanFrame {
    pub can_id: u32,
    pub len: u8,
    pub pad: u8,
    pub res0: u8,
    pub len8_dlc: u8,
    pub can_data: [u8; CAN_MAX_DLEN],
}

impl From<CanFrame> for [u8; 16] {
    fn from(frame: CanFrame) -> Self {
        // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
        bytemuck::cast(frame)
    }
}

impl<'a> From<&'a CanFrame> for &'a [u8] {
    fn from(frame: &'a CanFrame) -> Self {
        // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
        // bytemuck::cast_ref(frame)
        bytemuck::cast_slice(frame.into())
    }
}

impl<'a> From<&'a mut CanFrame> for &'a mut [u8] {
    fn from(frame: &'a mut CanFrame) -> Self {
        // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
        // bytemuck::cast_ref(frame)
        bytemuck::cast_slice_mut(frame.into())
    }
}

// impl From<&'a CanFrame> for &[u8; 16] {
//     fn from(frame: &'a CanFrame) -> Self {
//         // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
//         bytemuck::cast_ref(frame)
//     }
// }
// impl Into<&[u8; 16]> for &CanFrame {
//     fn into(self) -> &[u8; 16] {
//         // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
//         bytemuck::cast_ref(self)
//     }
// }

#[derive(Debug)]
pub struct CanSocket {}

impl BytesHandler for CanSocket {
    fn verify_read(_buf: &[u8]) -> io::Result<()> {
        // Implement your verification logic here
        debug!("Verifying CAN socket data... {:?}", _buf);
        Ok(())
    }

    fn socket_domain() -> Domain {
        Domain::from(libc::AF_CAN)
    }

    fn socket_type() -> Type {
        Type::RAW
    }

    fn socket_protocol() -> Option<Protocol> {
        Some(Protocol::from(libc::CAN_RAW))
    }

    fn sockaddr(ifname: &str) -> socket2::SockAddr {
        let cstr = std::ffi::CString::new(ifname).unwrap();
        let if_index = unsafe { libc::if_nametoindex(cstr.as_ptr()) };
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
            socket2::SockAddr::new(
                sockaddr_storage,
                std::mem::size_of::<libc::sockaddr_can>() as u32,
            )
        }
        // sockaddr
    }
}

// Handler that reads raw Ethernet frames
#[derive(Debug)]
pub struct EthernetSocket;

impl BytesHandler for EthernetSocket {
    fn verify_read(buf: &[u8]) -> io::Result<()> {
        // inspect the first 14 bytes of the Ethernet header, for example:
        if buf.len() < 14 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "frame too short",
            ));
        }
        // You could parse dst/src MAC and EtherType here
        debug!("Received Ethernet frame: {:02x?}", &buf[..14]);
        Ok(())
    }

    fn socket_domain() -> Domain {
        // AF_PACKET lets us read/write raw link‐layer frames
        Domain::from(libc::AF_PACKET)
    }

    fn socket_type() -> Type {
        Type::RAW
    }

    fn socket_protocol() -> Option<Protocol> {
        // ETH_P_ALL to receive all protocols
        // note: need to htons the value
        let eth_p_all = (libc::ETH_P_ALL as u16).to_be();
        Some(Protocol::from(eth_p_all as i32))
    }

    fn sockaddr(ifname: &str) -> socket2::SockAddr {
        // get the interface index
        let cstr = std::ffi::CString::new(ifname).unwrap();
        let if_index = unsafe { libc::if_nametoindex(cstr.as_ptr()) } as i32;
        // build a sockaddr_ll struct
        let sll = libc::sockaddr_ll {
            sll_family: libc::AF_PACKET as libc::c_ushort,
            // protocol in network byte order
            sll_protocol: (libc::ETH_P_ALL as u16).to_be(),
            sll_ifindex: if_index,
            sll_hatype: 0,
            sll_pkttype: 0,
            sll_halen: 0,
            sll_addr: [0; 8],
        };

        // copy into a sockaddr_storage
        let mut storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };
        unsafe {
            std::ptr::copy_nonoverlapping(
                &sll as *const _ as *const u8,
                &mut storage as *mut _ as *mut u8,
                std::mem::size_of::<libc::sockaddr_ll>(),
            );
        }

        // wrap it in socket2::SockAddr
        unsafe { socket2::SockAddr::new(storage, std::mem::size_of::<libc::sockaddr_ll>() as u32) }
    }
}
