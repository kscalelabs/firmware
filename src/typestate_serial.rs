use std::io;
use std::marker::PhantomData;
use std::time::Duration;
#[allow(unused_imports)]
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::time::timeout;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{error, info, warn};

use futures::{Stream, StreamExt};
use std::task::{Context, Poll};

use pin_project::pin_project;
use std::pin::Pin;
use std::task::ready;

use crate::state_machine;
state_machine!(Reset, Operate);

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialBaudRate {
    B4800 = 4800,
    B9600 = 9600,
    B19200 = 19200,
    B38400 = 38400,
    B57600 = 57600,
    B115200 = 115200,
    B230400 = 230400,
    B460800 = 460800,
    B921600 = 921600,
}

#[derive(Debug)]
#[pin_project]
struct Store {
    devpath: String,
    baud: SerialBaudRate,
    port: Option<SerialStream>,
}

impl Store {
    pub fn new(devpath: &str, baud: SerialBaudRate) -> Self {
        Store {
            devpath: devpath.to_string(),
            baud,
            port: None,
        }
    }
}

#[derive(Debug)]
pub struct Reset {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Operate {
    shared_state: Pin<Box<Store>>,
}

impl Reset {
    pub fn new(devpath: &str, baud: SerialBaudRate) -> Self {
        Reset {
            shared_state: Box::pin(Store::new(devpath, baud)),
        }
    }
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let ss = self.shared_state.as_mut().project();
            let builder = tokio_serial::new(ss.devpath.as_str(), *ss.baud as u32);
            match builder.open_native_async() {
                Ok(port) => {
                    {
                        use tokio_serial::SerialPort;
                        port.clear(tokio_serial::ClearBuffer::All);
                    }
                    *ss.port = Some(port);
                    // Transition to the next state
                    StateTransitionResult {
                        state: StateStore::Operate(Operate {
                            shared_state: self.shared_state,
                        }),
                        result: Ok(()),
                    }
                }
                Err(e) => StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(e.into()),
                },
            }
        }
    }
}

impl State for Operate {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            StateTransitionResult {
                state: StateStore::Operate(Operate {
                    shared_state: self.shared_state,
                }),
                result: Ok(()),
            }
        }
    }
}

impl Operate {
    pub async fn read_exact(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        port.read_exact(buf).await
    }

    pub fn set_baud_rate(&mut self, baud: SerialBaudRate) -> io::Result<()> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        *ss.baud = baud;
        {
            use tokio_serial::SerialPort;
            port.set_baud_rate(baud as u32).map_err(|e| e.into())
        }
    }

    pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        {
            use tokio_serial::SerialPort;
            port.try_read(buf)
        }
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        {
            use tokio_serial::SerialPort;
            port.read(buf).await
        }
    }

    pub fn clear(&mut self, clear_buffer: tokio_serial::ClearBuffer) -> io::Result<()> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        {
            use tokio_serial::SerialPort;
            port.clear(clear_buffer).map_err(|e| e.into())
        }
    }

    pub async fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let ss = self.shared_state.as_mut().project();
        // SAFETY: port must not None
        let port = ss.port.as_mut().unwrap();
        {
            use tokio_serial::SerialPort;
            port.write(buf).await
        }
    }
}

impl std::fmt::Debug for SerialPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialPort")
            .field("state", &self.state)
            .field("target", &self.target)
            .finish()
    }
}

#[pin_project]
pub struct SerialPort {
    state: Option<StateStore>,
    target: Option<StateTag>,
    #[pin]
    pending_fut: Option<StateFut>,
}

impl SerialPort {
    pub fn new(devpath: &str, baud: SerialBaudRate) -> Self {
        Self {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new(devpath, baud)),
            })),
            target: None,
            pending_fut: None,
        }
    }

    pub fn reset_baud(&mut self, baud: SerialBaudRate) -> std::io::Result<()> {
        // self.target = None;
        self.pending_fut = None;
        let mut shared_state = match self.state.take().expect("State must not be None") {
            StateStore::Reset(rst) => rst.shared_state,
            StateStore::Operate(oper) => oper.shared_state,
        };

        // shared_state.as_mut().project().
        // SAFETY: we can unpin as the future is finished
        let unpinned = unsafe { Pin::get_unchecked_mut(shared_state.as_mut()) };
        unpinned.baud = baud;
        {
            use tokio_serial::SerialPort;
            unpinned.port.as_mut().unwrap().set_baud_rate(baud as u32)?
        }

        self.state = Some(StateStore::Operate(Operate { shared_state }));
        Ok(())
    }

    pub fn set_target(&mut self, target: StateTag) {
        self.target = Some(target);
    }

    pub fn set_target_pinned(self: Pin<&mut Self>, target: StateTag) {
        *self.project().target = Some(target);
    }

    pub fn get_state_pinned(self: Pin<&mut Self>) -> Option<&mut StateStore> {
        self.project().state.as_mut()
    }

    pub fn get_state(&mut self) -> Option<&mut StateStore> {
        self.state.as_mut()
    }
}

impl Stream for SerialPort {
    type Item = std::io::Result<StateTag>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let mut this = self.project();

        if let Some(pending_fut) = this.pending_fut.as_mut().as_pin_mut() {
            // If the pending future is ready, we can transition to the next state
            let StateTransitionResult { state: st, result } = ready!(pending_fut.poll(cx));
            // clear the pending future
            unsafe {
                *this.pending_fut.get_unchecked_mut() = None;
            }

            let tag = st.tag();
            *this.state = Some(st); // Update the state
            if let Err(e) = result {
                error!("State transition failed: {:?}", e);
                return Poll::Ready(Some(Err(e)));
            }
            return Poll::Ready(Some(Ok(tag)));
        }

        // check if we have laready reached the specified target
        if let Some(target) = this.target {
            let current = this.state.as_ref().unwrap().tag();
            if &current == target {
                return Poll::Ready(Some(Ok(current)));
            }
        }

        // start new transition to reach the target state
        unsafe {
            *this.pending_fut.get_unchecked_mut() = Some(
                this.state
                    .take()
                    .expect("state must not be None")
                    .transition_fut(),
            );
        }

        // we return pending for this cycle
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// #[derive(Debug)]
// pub struct Localize;
// #[derive(Debug)]
// pub struct Configure;
// #[derive(Debug)]
// pub struct BringUp;
// #[derive(Debug)]
// pub struct Operate;
// #[derive(Debug)]
// pub struct Recover;
/*
pub enum SerialState<Handler: BytesHandler> {
    Localize(SerialPort<Localize, Handler>),
    Configure(SerialPort<Configure, Handler>),
    BringUp(SerialPort<BringUp, Handler>),
    Operate(SerialPort<Operate, Handler>),
    Recover(SerialPort<Recover, Handler>),
}

pub trait BytesHandler {
    fn verify_read(buf: &[u8]) -> io::Result<()>;
}

#[derive(Debug)]
pub struct SerialPort<State, Handler: BytesHandler> {
    port: SerialStream,
    handler: Handler,
    _state: PhantomData<State>,
}


impl <Handler: BytesHandler> SerialPort<Localize, Handler> {
    /// Open the port at a *default* baud.
    /// You’ll probe other rates in `detect_baud`.
    pub async fn new(path: &str, handler: Handler) -> io::Result<Self> {
        let builder = tokio_serial::new(path, 9600);
        let port = builder.open_native_async()?;
        Ok(SerialPort {
            port,
            handler,
            _state: PhantomData
        })
    }

    /// Probe a list of baud‐rates until the IMU responds.
    pub async fn detect_baud(mut self) -> io::Result<SerialPort<Configure, Handler>> {
        // A typical set of baud‐rates to try:
        let candidates = [9_600, 19_200, 115_200, 230_400, 460_800, 921_600];

        for &baud in &candidates {
            // self.port.
            {
                use tokio_serial::SerialPort;
                self.port.set_baud_rate(baud)?;
            }
            // wait until the port is readable :contentReference[oaicite:0]{index=0}
            let mut buf128 = [0u8; 128];

            // discard 128 bytes to clear buffer
            let _ = timeout(Duration::from_millis(1000), async {
                self.port.read_exact(&mut buf128).await
            }).await;

            // try read 128 new bytes for verification
            match timeout(Duration::from_millis(500), async {
                self.port.read_exact(&mut buf128).await
            }).await
            {
                // timeout
                Err(_) => {
                    warn!("baud {}: timed out waiting for 80 bytes", baud);
                    continue; // try next baud
                }
                // inner future completed with an Err
                Ok(Err(e)) => return Err(e),
                // inner future got all 80 bytes
                Ok(Ok(_n)) => {
                    // check the checksum
                    if let Err(e) = Handler::verify_read(&buf128) {
                        warn!("baud {}: {}", baud, e);
                        continue; // try next baud
                    }
                    // if we get here, we have a valid response
                    info!("baud {}: detected", baud);
                    // return the port in the next state
                    return Ok(SerialPort {
                        port: self.port,
                        handler: self.handler,
                        _state: PhantomData
                    });
                }
            }
        }
        Err(io::Error::new(io::ErrorKind::Other, "baud detection failed"))
    }
}
*/
