use crate::bytestream_fd::ByteStreamFd;
use futures_enum::Future; // derive macro for enums
use pin_project::pin_project;
use std::io;
use std::os::unix::io::AsRawFd;
use std::pin::Pin;
use std::task::ready;
use std::task::{Context, Poll};
use tracing::{debug, error, info};

use std::fmt::Debug;

use std::future::Future as StdFuture;

#[derive(Future)] // from `futures-enum`
pub enum SocketStateFut<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    Reset(ResetStateFut<C, O>),
    Configure(ConfigureStateFut<C, O>),
    Operate(OperateStateFut<C, O>),
}

impl<C, O> Debug for SocketStateFut<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SocketStateFut::Reset(_) => write!(f, "SocketStateFut::Reset"),
            SocketStateFut::Configure(_) => write!(f, "SocketStateFut::Configure"),
            SocketStateFut::Operate(_) => write!(f, "SocketStateFut::Operate"),
        }
    }
}

#[derive(Debug)]
pub enum SocketState<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    Reset(String),
    Configure(Socket<C>),
    Operate(Socket<O>),
    InFlight,
    Error,
}

pub struct StateTransitionResult<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    state: SocketState<C, O>,
    result: std::io::Result<()>,
}

macro_rules! state_fn_type {
    ($name:ident) => {
        #[allow(non_camel_case_types)]
        pub type $name<C, O>
        where
            C: SocketConfigurator + Unpin,
            O: SocketOperator + Unpin,
        = impl StdFuture<Output = StateTransitionResult<C, O>>;
    };
}

state_fn_type!(ConfigureStateFut);
state_fn_type!(OperateStateFut);
state_fn_type!(ResetStateFut);

impl<C, O> SocketState<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    pub fn transition_fut(self) -> SocketStateFut<C, O> {
        match self {
            SocketState::Reset(ifname) => SocketStateFut::Reset(Self::reset_state_fn(ifname)),
            SocketState::Configure(socket) => {
                SocketStateFut::Configure(Self::configure_state_fn(socket))
            }
            SocketState::Operate(socket) => SocketStateFut::Operate(Self::operate_state_fn(socket)),
            _ => panic!("SocketState Invalid!"),
        }
    }

    #[define_opaque(ResetStateFut)]
    pub fn reset_state_fn(ifname: String) -> ResetStateFut<C, O> {
        // duplicate an owned object to avoid moving a ref into async block
        async move {
            // Simulate some async work, e.g., configuration
            // tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            match Socket::<C>::new(&ifname) {
                Ok(socket) => StateTransitionResult {
                    state: SocketState::Configure(socket),
                    result: Ok(()),
                },
                Err(e) => StateTransitionResult {
                    state: SocketState::Reset(ifname),
                    result: Err(e),
                },
            }
        }
    }

    #[define_opaque(ConfigureStateFut)]
    pub fn configure_state_fn(socket: Socket<C>) -> ConfigureStateFut<C, O> {
        async move {
            // tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            let ifname = socket.ifname.clone();
            match socket.establish::<O>().await {
                Ok(socket) => StateTransitionResult {
                    state: SocketState::Operate(socket),
                    result: Ok(()),
                },
                Err(e) => StateTransitionResult {
                    state: SocketState::Reset(ifname),
                    result: Err(e),
                },
            }
        }
    }

    #[define_opaque(OperateStateFut)]
    pub fn operate_state_fn(socket: Socket<O>) -> OperateStateFut<C, O> {
        async move {
            StateTransitionResult {
                state: SocketState::Operate(socket),
                result: Ok(()), // In this case, we assume the operation is successful
            }
        }
    }
}

#[pin_project]
pub struct SocketGraph<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    pub state: Option<SocketState<C, O>>,
    #[pin]
    pending_fut: Option<SocketStateFut<C, O>>,
}

impl<C, O> SocketGraph<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    pub fn new(ifname: &str) -> Self {
        SocketGraph {
            state: Some(SocketState::Reset(ifname.to_string())),
            pending_fut: None,
            // SocketStateFut::Configure(
            //     SocketState::Reset.to_configure(ifname)
            // ),
        }
    }

    // pub fn tage_configure(&mut self) {
    //     // swap current state to InTransition
    //     let cur_state = std::mem::replace(&mut self.cur_state, SocketState::InTransition);
    //     self.pending_fut = SocketStateFut::Configure(cur_state.to_configure());
    // }
}

impl<C, O> StdFuture for SocketGraph<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    type Output = std::io::Result<()>;

    // just drives to operational state
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut this = self.project();

        loop {
            match this.pending_fut.as_mut().as_pin_mut() {
                Some(pending_fut) => {
                    debug!("Polling pending future: {:?}", pending_fut);
                    // If the pending future is ready, we can transition to the next state
                    // *this.state = ready!(pending_fut.poll(cx));
                    let StateTransitionResult { state: st, result } = ready!(pending_fut.poll(cx));
                    // println!("Got new state: {:?}", this.state);
                    let unpinned = unsafe { Pin::get_unchecked_mut(this.pending_fut.as_mut()) };
                    *unpinned = None; // Clear the pending future after polling
                    // println!("this.pending_fut: {:?}", this.pending_fut);

                    // return Ok if we are in Operate state, Error if there was an error, continue
                    // otherwise

                    match result {
                        Ok(_) => {
                            // println!("transition to next state: {:?}", st);
                            if let SocketState::Operate(_) = &st {
                                *this.state = Some(st); // Update the current state
                                return Poll::Ready(Ok(())); // Return ready with Ok if we are in Operate state
                            } else {
                                *this.state = Some(st); // Update the current state
                                continue; // Continue polling if we are not in Operate state
                            }
                        }
                        Err(e) => {
                            error!("Error during state transition: {:?}", e);
                            return Poll::Ready(Err(e)); // Return error
                        }
                    };
                }
                None => {
                    // println!("in none");
                    if let Some(SocketState::Operate(_)) = this.state {
                        return Poll::Ready(Ok(()));
                    }
                    // queue up pending fut
                    let unpinned = unsafe { this.pending_fut.as_mut().get_unchecked_mut() };
                    *unpinned = Some(
                        this.state
                            .take()
                            .expect("state must not be None")
                            .transition_fut(),
                    );
                    // queue up pending fut
                    // let mut state = SocketState::InFlight;
                    // std::mem::swap(this.state, &mut state);
                    // let unpinned = unsafe { Pin::get_unchecked_mut(this.pending_fut.as_mut()) };
                    // *unpinned = Some(state.transition_fut());
                    // we return pending for this cycle
                    // cx.waker().wake_by_ref();
                    // Poll::Pending
                }
            }
        }
        // loop {
        //     match this.pending_fut.as_mut().poll(cx) {
        //         Poll::Ready(state) => {

        //             // check if we are in operate state
        //             if let SocketState::Operate(_) = state {
        //                 // let ret = ready!(pinned.as_mut().poll(cx));
        //                 println!("transition to operate done");
        //                 *this.cur_state = state;
        //                 return Poll::Ready(Ok(()));  // return ready with Ok

        //                 // set pending transition change to None
        //             } else {
        //                 println!("transition to configure done");
        //             }

        //             // println!("check done, got {:?}", state);
        //             // SAFETY: since the future is done running, we can yeet it out
        //             let t = unsafe {Pin::get_unchecked_mut(this.pending_fut.as_mut())};
        //             *t = SocketStateFut::Operate(
        //                 state.to_operate()
        //             );
        //             // yield
        //             // cx.waker().wake_by_ref();  // wake the waker to poll again
        //             // Poll::Pending  // we need to poll again
        //         }
        //         Poll::Pending => {
        //             println!("transition still pending");
        //             // cx.waker().wake_by_ref();  // wake the waker to poll again
        //             return Poll::Pending;  // still running
        //         }
        //     }
        // }
    }
}

impl<C, O> Debug for SocketGraph<C, O>
where
    C: SocketConfigurator + Unpin + Debug,
    O: SocketOperator + Unpin + Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SocketGraph {{ state: {:?} }}", self.state)
    }
}

pub enum SocketStorage<C, O>
where
    C: SocketConfigurator + Unpin,
    O: SocketOperator + Unpin,
{
    Configure(Socket<C>),
    ConfigureToOperate(Pin<Box<dyn Future<Output = io::Result<Socket<O>>> + Send>>),
    Operate(Socket<O>),
}

impl<C, O> std::fmt::Debug for SocketStorage<C, O>
where
    C: SocketConfigurator + Unpin + std::fmt::Debug,
    O: SocketOperator + Unpin + std::fmt::Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SocketStorage::Configure(socket) => write!(f, "SocketStorage::Configure({socket:?})"),
            SocketStorage::ConfigureToOperate(_) => write!(f, "SocketStorage::ConfigureToOperate"),
            SocketStorage::Operate(socket) => write!(f, "SocketStorage::Operate({socket:?})"),
        }
    }
}

#[derive(Debug)]
pub struct Socket<State: Unpin> {
    // common data across all states
    socket: socket2::Socket,
    ifname: String,

    // state specific data and methods
    state: State,
}

// impl<State> Unpin for Socket<State>
// where
//     State: SocketConfigurator + Unpin,
// {
// }

pub trait SocketConfigurator: Default + std::fmt::Debug {
    fn get_domain() -> socket2::Domain;
    fn get_type() -> socket2::Type;
    fn get_protocol() -> Option<socket2::Protocol>;
    fn get_sockaddr(ifname: &str) -> std::io::Result<socket2::SockAddr>;
}

pub trait SocketOperator: std::fmt::Debug {
    type MsgType;
    fn new(bytestream_fd: ByteStreamFd) -> Self;
    // async fn write(&mut self, msg: &Self::MsgType) -> io::Result<usize>;
    // async fn write(&mut self, msg: &mut Self::MsgType) -> io::Result<usize>;
    fn write(
        &mut self,
        msg: &Self::MsgType,
    ) -> impl std::future::Future<Output = io::Result<usize>> + Send + Sync;
    fn read(
        &mut self,
        msg: &mut Self::MsgType,
    ) -> impl std::future::Future<Output = io::Result<usize>> + Send + Sync;
    fn try_read(&mut self, msg: &mut [u8]) -> std::io::Result<usize>;
}

impl<State> Socket<State>
where
    State: SocketConfigurator + Unpin,
{
    pub fn new(ifname: &str) -> io::Result<Self> {
        let socket = socket2::Socket::new(
            State::get_domain(),
            State::get_type(),
            State::get_protocol(),
        )?;

        let sockaddr = State::get_sockaddr(ifname)?;
        socket.bind(&sockaddr)?;
        socket.set_nonblocking(true)?;

        Ok(Socket {
            socket,
            ifname: ifname.to_string(),
            state: State::default(),
        })
    }

    pub async fn establish<Operator>(self) -> std::io::Result<Socket<Operator>>
    where
        Operator: SocketOperator + Unpin,
    {
        let bytestream_fd = ByteStreamFd::new(self.socket.as_raw_fd())?;

        // create the operator
        let operator = Operator::new(bytestream_fd);
        info!("establishing socket with operator: {:?}", operator);

        // Return the new socket with the operator
        Ok(Socket {
            socket: self.socket,
            ifname: self.ifname,
            state: operator,
        })
    }
}

impl<State> Socket<State>
where
    State: SocketOperator + Unpin,
{
    pub async fn write(&mut self, msg: &State::MsgType) -> io::Result<usize> {
        self.state.write(msg).await
    }

    pub async fn read(&mut self, msg: &mut State::MsgType) -> io::Result<usize> {
        self.state.read(msg).await
    }

    pub fn try_read(&mut self, msg: &mut [u8]) -> io::Result<usize> {
        self.state.try_read(msg)
    }
}

/*
use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};
use pin_project::pin_project;
use socket2::Socket;
use tokio::io::unix::AsyncFd;
use std::os::unix::io::RawFd;

/// This is your *one* Future type, carrying both common data
/// and an enum that tracks exactly which “phase” we’re in.
#[pin_project]
struct CanSocketFuture {
    // common fields (owned at construction):
    ifname: String,
    // …any config‐only fields you need…

    // the *runtime* state machine:
    #[pin]
    state: SocketState,
}

#[pin_project]
enum SocketState {
    Config {
        #[pin]
        sock: Socket,
        // other config‐only data…
    },
    Operate {
        #[pin]
        afd: AsyncFd<RawFd>,
        // other operate‐only data…
    },
    Done,
}

impl CanSocketFuture {
    pub fn new(ifname: String) -> Self {
        // construct in the Config state
        let sock = /* socket2 setup + bind */;
        Self {
            ifname,
            state: SocketState::Config { sock },
        }
    }
}

impl Future for CanSocketFuture {
    type Output = std::io::Result<()>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Project the two fields (`ifname` is Unpin so we ignore it,
        // but `state` is pinned, so we project that)
        let mut this = self.project();

        loop {
            match this.state.as_mut().project() {
                SocketStateProj::Config { sock } => {
                    // all your config‐phase logic here…
                    // when you decide to switch:
                    let afd = {
                        sock.set_nonblocking(true)?;
                        let raw = sock.into_raw_fd();
                        AsyncFd::new(raw)?
                    };
                    // overwrite the enum in place (same address, so OK even pinned)
                    *this.state = SocketState::Operate { afd };
                    // loop around and immediately enter the Operate arm
                }

                SocketStateProj::Operate { afd } => {
                    // now do your async‐fd reads/writes:
                    let mut ready = afd.readable().await?; // you can even `.await` here
                    // ... read from afd ...
                    return Poll::Ready(Ok(()));
                }

                SocketStateProj::Done => {
                    panic!("polled after completion");
                }
            }
        }
    }
}
*/
