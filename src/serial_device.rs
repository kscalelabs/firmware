use std::io;
use tokio::io::{AsyncRead, AsyncWrite, AsyncReadExt, AsyncWriteExt};
use tokio::fs::File;
use tokio_serial::SerialPortBuilderExt;
use std::pin::Pin;
use std::task::{Context, Poll};

/// A wrapper that can handle both real serial devices and PTYs
#[derive(Debug)]
pub enum SerialDevice {
    Serial(tokio_serial::SerialStream),
    Pty(tokio::fs::File),
}

impl SerialDevice {
    pub async fn open(path: &str, baud: u32) -> io::Result<Self> {
        // First try to open as a real serial device
        match tokio_serial::new(path, baud).open_native_async() {
            Ok(serial) => {
                tracing::info!("Opened {} as serial device", path);
                Ok(SerialDevice::Serial(serial))
            }
            Err(e) => {
                let error_msg = e.to_string();
                if error_msg.contains("Not a typewriter") || error_msg.contains("ENOTTY") {
                    tracing::warn!("Device {} appears to be a PTY, opening as file", path);
                    // Try to open as a PTY/file
                    match tokio::fs::OpenOptions::new()
                        .read(true)
                        .write(true)
                        .open(path)
                        .await
                    {
                        Ok(file) => {
                            tracing::info!("Opened {} as PTY device", path);
                            Ok(SerialDevice::Pty(file))
                        }
                        Err(file_err) => {
                            tracing::error!("Failed to open {} as PTY: {}", path, file_err);
                            Err(file_err)
                        }
                    }
                } else {
                    Err(e.into())
                }
            }
        }
    }

    pub fn clear(&mut self, _buffer: tokio_serial::ClearBuffer) -> io::Result<()> {
        match self {
            SerialDevice::Serial(serial) => {
                use tokio_serial::SerialPort;
                serial.clear(_buffer).map_err(|e| io::Error::new(io::ErrorKind::Other, e))
            }
            SerialDevice::Pty(_) => {
                // PTYs don't need buffer clearing
                tracing::debug!("Buffer clear ignored for PTY");
                Ok(())
            }
        }
    }

    pub fn set_baud_rate(&mut self, baud: u32) -> io::Result<()> {
        match self {
            SerialDevice::Serial(serial) => {
                use tokio_serial::SerialPort;
                serial.set_baud_rate(baud).map_err(|e| io::Error::new(io::ErrorKind::Other, e))
            }
            SerialDevice::Pty(_) => {
                // PTYs don't have baud rates
                tracing::debug!("Baud rate setting ignored for PTY");
                Ok(())
            }
        }
    }

    pub async fn read_exact(&mut self, buf: &mut [u8]) -> io::Result<()> {
        match self {
            SerialDevice::Serial(serial) => {
                serial.read_exact(buf).await?;
                Ok(())
            }
            SerialDevice::Pty(file) => {
                let mut total_read = 0;
                while total_read < buf.len() {
                    let n = file.read(&mut buf[total_read..]).await?;
                    if n == 0 {
                        return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "EOF while reading"));
                    }
                    total_read += n;
                }
                Ok(())
            }
        }
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            SerialDevice::Serial(serial) => serial.read(buf).await,
            SerialDevice::Pty(file) => file.read(buf).await,
        }
    }

    pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            SerialDevice::Serial(serial) => {
                use tokio_serial::SerialPort;
                serial.try_read(buf).map_err(|e| io::Error::new(io::ErrorKind::Other, e))
            }
            SerialDevice::Pty(_file) => {
                // For PTYs, try_read is not available on File
                // We'll simulate it by returning WouldBlock for now
                Err(io::Error::new(io::ErrorKind::WouldBlock, "try_read not supported on PTY"))
            }
        }
    }

    pub async fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            SerialDevice::Serial(serial) => serial.write(buf).await,
            SerialDevice::Pty(file) => file.write(buf).await,
        }
    }
}

impl AsyncRead for SerialDevice {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut tokio::io::ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        match self.get_mut() {
            SerialDevice::Serial(serial) => Pin::new(serial).poll_read(cx, buf),
            SerialDevice::Pty(file) => Pin::new(file).poll_read(cx, buf),
        }
    }
}

impl AsyncWrite for SerialDevice {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize, io::Error>> {
        match self.get_mut() {
            SerialDevice::Serial(serial) => Pin::new(serial).poll_write(cx, buf),
            SerialDevice::Pty(file) => Pin::new(file).poll_write(cx, buf),
        }
    }

    fn poll_flush(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), io::Error>> {
        match self.get_mut() {
            SerialDevice::Serial(serial) => Pin::new(serial).poll_flush(cx),
            SerialDevice::Pty(file) => Pin::new(file).poll_flush(cx),
        }
    }

    fn poll_shutdown(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), io::Error>> {
        match self.get_mut() {
            SerialDevice::Serial(serial) => Pin::new(serial).poll_shutdown(cx),
            SerialDevice::Pty(file) => Pin::new(file).poll_shutdown(cx),
        }
    }
}
