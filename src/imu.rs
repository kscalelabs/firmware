use pin_project::pin_project;
use std::pin::Pin;
use std::task::ready;
use tracing::{debug, error, info, warn};

use futures::{Stream, StreamExt, TryStream, TryStreamExt};
use std::task::{Context, Poll};

use crate::typestate_serial::{self};

use crate::hiwonder::HiwonderImu;

use crate::state_machine;
state_machine!(Reset, Scanning, Operate);

use crate::robot_description::ImuData;

#[derive(Debug)]
#[pin_project]
struct Store {
    bauds: Vec<typestate_serial::SerialBaudRate>,
    cur_idx: usize,
    #[pin]
    port: typestate_serial::SerialPort,
}

impl Store {
    pub fn new(devpath: &str) -> Self {
        let port =
            typestate_serial::SerialPort::new(devpath, typestate_serial::SerialBaudRate::B9600);
        Self {
            bauds: vec![
                typestate_serial::SerialBaudRate::B9600,
                typestate_serial::SerialBaudRate::B19200,
                typestate_serial::SerialBaudRate::B115200,
                typestate_serial::SerialBaudRate::B230400,
                typestate_serial::SerialBaudRate::B460800,
                typestate_serial::SerialBaudRate::B921600,
            ],
            cur_idx: 0,
            port,
        }
    }
}

#[derive(Debug)]
pub struct Reset {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Scanning {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Operate {
    shared_state: Pin<Box<Store>>,
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            *self.shared_state.as_mut().project().cur_idx = 0;
            StateTransitionResult {
                state: StateStore::Scanning(Scanning {
                    shared_state: self.shared_state,
                }),
                result: Ok(()),
            }
        }
    }
}

impl Scanning {
    async fn verify_imu(op_port: &mut typestate_serial::Operate) -> std::io::Result<()> {
        op_port.clear(tokio_serial::ClearBuffer::Input);
        let mut buf128 = [0u8; 128];
        // read data from the serial port
        let to = tokio::time::timeout(
            std::time::Duration::from_millis(500),
            op_port.read_exact(&mut buf128),
        );

        match to.await {
            Ok(Ok(_n)) => {
                // print in hex
                debug!(
                    "Read {:?} from IMU",
                    buf128
                        .iter()
                        .map(|b| format!("{b:02x}"))
                        .collect::<Vec<_>>()
                );
            }
            Ok(Err(e)) => {
                warn!("Error reading IMU data: {:?}", e);
                return Err(e);
            }
            Err(e) => {
                warn!("Timed out reading from serial port: {:?}", e);
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "Timed out reading from serial port",
                ));
            }
        }
        // verify that its correct (e.g. checksum)
        HiwonderImu::verify_read(&buf128)
    }
}

impl State for Scanning {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = &mut self.shared_state;
            let mut ss = shared_state.as_mut().project();
            let target = typestate_serial::StateTag::Operate;
            ss.port.as_mut().set_target_pinned(target);
            loop {
                match ss.port.try_next().await {
                    Ok(Some(typestate_serial::StateTag::Operate)) => break,
                    Ok(_) => continue,
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset {
                                shared_state: self.shared_state,
                            }),
                            result: Err(std::io::Error::new(
                                std::io::ErrorKind::UnexpectedEof,
                                "serial port imu stream ended unexpectedly",
                            )),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset {
                                shared_state: self.shared_state,
                            }),
                            result: Err(e),
                        };
                    }
                }
            }

            let typestate_serial::StateStore::Operate(op_port) = ss
                .port
                .as_mut()
                .get_state_pinned()
                .expect("serial port should be in Operate state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(std::io::Error::other("serial port is not in Operate state")),
                };
            };

            if let Err(e) = Self::verify_imu(op_port).await {
                warn!(
                    "Error verifying imu at baud {:?}: {:?}",
                    ss.bauds[*ss.cur_idx], e
                );
                *ss.cur_idx += 1;
                let unpinned = unsafe { Pin::get_unchecked_mut(ss.port.as_mut()) };
                unpinned.reset_baud(ss.bauds[*ss.cur_idx]);
                return StateTransitionResult {
                    state: StateStore::Scanning(Scanning {
                        shared_state: self.shared_state,
                    }),
                    result: Ok(()), // this is expected
                };
            }

            match HiwonderImu::setup(op_port).await {
                Ok(_) => {
                    info!(
                        "Successfully setup IMU at baud: {:?}",
                        ss.bauds[*ss.cur_idx]
                    );
                }
                Err(e) => {
                    warn!(
                        "Error setting up IMU: {:?} at baud: {:?}",
                        e, ss.bauds[*ss.cur_idx]
                    );
                    return StateTransitionResult {
                        state: StateStore::Reset(Reset {
                            shared_state: self.shared_state,
                        }),
                        result: Ok(()), // this is expected
                    };
                }
            }

            if let Err(e) = Self::verify_imu(op_port).await {
                warn!(
                    "Error verifying imu at baud {:?}: {:?}",
                    ss.bauds[*ss.cur_idx], e
                );
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Ok(()), // this is expected
                };
            }

            // we are ready, flush everything
            op_port.clear(tokio_serial::ClearBuffer::All);
            // print each element in hex
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
    pub async fn process_feedback(&mut self, imu_data: &mut ImuData) -> std::io::Result<()> {
        let mut shared_state = &mut self.shared_state;
        let mut ss = shared_state.as_mut().project();

        // get operational serial port

        let typestate_serial::StateStore::Operate(op_port) = ss
            .port
            .as_mut()
            .get_state_pinned()
            .expect("serial port should be in Operate state")
        else {
            return Err(std::io::Error::other("serial port is not in Operate state"));
        };

        let fdbk = HiwonderImu::parse_all(op_port).await?;
        imu_data.merge_feedback(fdbk);

        Ok(())
    }

    pub fn clear(&mut self) -> std::io::Result<()> {
        let mut shared_state = &mut self.shared_state;
        let mut ss = shared_state.as_mut().project();

        // get operational serial port
        let typestate_serial::StateStore::Operate(op_port) = ss
            .port
            .as_mut()
            .get_state_pinned()
            .expect("serial port should be in Operate state")
        else {
            return Err(std::io::Error::other("serial port is not in Operate state"));
        };

        op_port.clear(tokio_serial::ClearBuffer::All);
        Ok(())
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

#[pin_project]
pub struct ImuManager {
    state: Option<StateStore>,
    target: Option<StateTag>,
    #[pin]
    pending_fut: Option<StateFut>,
}

impl Default for ImuManager {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuManager {
    pub fn new() -> Self {
        Self {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new("/dev/ttyUSB0")),
            })),
            target: None,
            pending_fut: None,
        }
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

impl Stream for ImuManager {
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
