use std::pin::Pin;
use std::task::{Context, Poll};
use communication::typestate_socket2::{
    SocketGraph,
    SocketState,
    SocketStorage,
    Socket,
};
use communication::socketcan2::{
    SocketCanConfigurator,
    SocketCanOperator,
};

use std::fmt::Debug;
use communication::socketcan::CanFrame;

use futures::{
    Future,
    Stream,
    StreamExt,
};

use std::vec::Vec;

use crate::robstride::{
    ActuatorCanClient,
    ActuatorRequestParams,
    actuator_can_id_from_response,
    mux_from_can_frame,
};

use robot_description::{
    ActuatorId,
    ActuatorFeedbackUpdate,
    ActuatorFeedback,
    ActuatorCommand,
    ActuatorState,
    BusTag,
};

use pin_project::pin_project;
use std::task::ready;

use infrastructure::state_machine;

state_machine!(Reset, Configure, Ready, Operate);

#[derive(Debug)]
pub struct Reset {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Configure {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Ready {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Operate {
    shared_state: Pin<Box<Store>>,
}

impl Ready {
    pub async fn enable(&mut self) -> std::io::Result<()> {
        send_request(self.shared_state.as_mut(), &ActuatorRequestParams::MotorEnable).await?;
        read_responses(self.shared_state.as_mut()).await
    }
}

impl Operate {
    pub async fn request_feedback(&mut self) -> std::io::Result<()> {
        // send_request(self.shared_state.as_mut(), ActuatorRequestParams::Feedback).await?;
        // read_responses(self.shared_state.as_mut()).await
        send_request(self.shared_state.as_mut(), &ActuatorRequestParams::Feedback).await
    }

    pub async fn command(&mut self, act_states: &[ActuatorState]) -> std::io::Result<()> {
        send_commands(self.shared_state.as_mut(), act_states).await
        // read_responses(self.shared_state.as_mut()).await
    }

    pub async fn process_feedback(&mut self, act_states: &mut [ActuatorState]) -> std::io::Result<()> {
        // read_responses(self.shared_state.as_mut()).await
        read_responses_update(self.shared_state.as_mut(), Some(act_states)).await
    }
}


impl State for Reset
{
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            let mut ss = shared_state.as_mut().project();

            let res = ss.socket_graph.await;
            match res {
                Ok(_) => {
                    log::debug!("Socket graph operational");
                    StateTransitionResult {
                        state: StateStore::Configure(Configure {
                            shared_state,
                        }),
                        result: Ok(()),
                    }
                }
                Err(e) => {
                    StateTransitionResult {
                        state: StateStore::Reset(Reset {
                            shared_state,
                        }), // go back to reset state on error
                        result: Err(e),
                    }
                }
            }
        }
    }
}

impl State for Ready
{
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

impl State for Configure
{
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            if let Err(e) = send_request(self.shared_state.as_mut(), &ActuatorRequestParams::ObtainId).await {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(e),
                };
            }

            tokio::task::yield_now().await;

            // wait for responses for 10ms
            let to = tokio::time::timeout(
                std::time::Duration::from_millis(10),
                read_responses(self.shared_state.as_mut())
            );

            if let Err(e) = to.await {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(e.into()),
                };
            }

            StateTransitionResult {
                state: StateStore::Ready(Ready {
                    shared_state: self.shared_state,
                }),
                result: Ok(()),
            }
        }
    }
}

impl State for Operate
{
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Operate(Operate {
                    shared_state,
                }),
                result: Ok(()),
            }
        }
    }
}

async fn send_commands(ss: Pin<&mut Store>, act_states: &[ActuatorState]) -> std::io::Result<()> {
    let mut ss = ss.project();
    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.pub_project().state else {
        // no operational socket, go back to configure state
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Socket is not in Operate state",
        ));
    };

    for (i, client) in ss.actuator_clients.iter_mut().enumerate() {
        
        let params = ActuatorRequestParams::Control(
            act_states[i].command.clone()
        );

        let req = client.stage_request(&params);
        let res = op_socket.write(&req.into()).await;
        match res {
            Ok(_) => {
                client.set_last_request(req);
            }
            Err(e) => {
                return Err(e);
            }
        }
    }
    Ok(())
}

async fn send_request(ss: Pin<&mut Store> , params: &ActuatorRequestParams) -> std::io::Result<()> {
    let mut ss = ss.project();
    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.pub_project().state else {
        // no operational socket, go back to configure state
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Socket is not in Operate state",
        ));
    };

    for client in ss.actuator_clients.iter_mut() {
        let req = client.stage_request(params);
        let res = op_socket.write(&req.into()).await;
        match res {
            Ok(_) => {
                client.set_last_request(req);
            }
            Err(e) => {
                return Err(e);
            }
        }
    }
    Ok(())
}

async fn read_responses(ss: Pin<&mut Store>) -> std::io::Result<()> {
    read_responses_update(ss, None).await
}



async fn read_responses_update(ss: Pin<&mut Store>, mut act_states: Option<&mut [ActuatorState]>) -> std::io::Result<()> {
    let mut ss = ss.project();

    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.pub_project().state else {
        // no operational socket, go back to configure state
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Socket is not in Operate state",
        ));
    };

    let mut n = ss.actuator_clients.len();

    let mut handler = |can_frame: &CanFrame| {
        let client_idx = (ss.response_to_client_idx)(&can_frame);
        if let Some(client) = ss.actuator_clients.get_mut(client_idx) {
            match client.handle_response(&can_frame) {
                Ok(Some(fdbk)) => {
                    if let Some(ref mut act_states) = act_states {
                        if let Some(state) = act_states.get_mut(client_idx) {
                            state.merge_feedback(fdbk);
                        } else {
                            return Err(std::io::Error::new(
                                std::io::ErrorKind::NotFound,
                                format!("No ActuatorFeedback found for client index {}", client_idx),
                            ));
                        }
                    }
                }
                Ok(None) => {},
                Err(e) => {
                    return Err(e);
                }
            }
        } else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("No client found for response with index {}", client_idx),
            ));
        }
        Ok(())
    };

    let mut read_data = [0u8; 16]; // read buffer
    let mut seen = vec![false; n];
    let mut rem = 5;

    // read responses until we have seen all clients
    while (rem > 0) {
        let res = op_socket.read(&mut read_data).await;
        match res {
            Ok(_) => {
                // Process the read data here
                let can_frame: CanFrame = unsafe { std::mem::transmute(read_data) };
                let client_idx = (ss.response_to_client_idx)(&can_frame);

                // decrement rem if not seen
                rem -= !seen[client_idx] as usize;
                seen[client_idx] = true;

                log::debug!("Received CanFrame: {:?}", can_frame);
                handler(&can_frame)?;
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    // drain the buffer
    while let Ok(_) = op_socket.try_read(&mut read_data) {
        // Process the read data here
        let can_frame: CanFrame = unsafe { std::mem::transmute(read_data) };
        let client_idx = (ss.response_to_client_idx)(&can_frame);
        log::debug!("Draining CanFrame: {:?}", can_frame);
        handler(&can_frame)?;
    }


    // }

    Ok(())
}

#[derive(Debug)]
#[pin_project]
struct Store {
    ifname: String,
    actuator_clients: Vec<ActuatorCanClient>,
    response_to_client_idx: fn(&CanFrame) -> usize, // map canframe to client index
    #[pin]
    socket_graph: SocketGraph<SocketCanConfigurator, SocketCanOperator>,
}

impl Store {
    pub fn new(ifname: &str, ids: Vec<ActuatorId>) -> Self {
        Self {
            ifname: ifname.to_string(),
            actuator_clients: ids.into_iter().map(|id| {
                ActuatorCanClient::new(id)
            }).collect(),
            response_to_client_idx: |frame: &CanFrame| {
                let id = actuator_can_id_from_response(frame);
                ((id % 10) - 1) as usize
            },
            socket_graph: SocketGraph::new(ifname),
        }
    }

    pub fn reset_iface(&mut self, ifname: &str) {
        self.ifname = ifname.to_owned();
        self.socket_graph = SocketGraph::new(ifname);
        // reset all clients
        for client in self.actuator_clients.iter_mut() {
            client.reset();
        }
    }
}


impl Debug for ActuatorBus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ActuatorBus")
            .field("state", &self.state)
            .field("target", &self.target)
            .finish()
    }
}

#[pin_project]
pub struct ActuatorBus {
    state: Option<StateStore>,
    target: Option<StateTag>,
    #[pin]
    pending_fut: Option<StateFut>,
}

impl ActuatorBus {
    pub fn new(ifname: &str, ids: Vec<ActuatorId>) -> Self {
        ActuatorBus {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new(ifname, ids)),
            })),
            target: None,
            pending_fut: None,
        }
    }
    
    pub fn set_target(&mut self, target: StateTag) {
        // let mut this = self.as_mut().project();
        // *this.target = Some(target);
        self.target = Some(target);
    }

    pub fn reset_iface(&mut self, ifname: &str) {
        self.target = None;
        self.pending_fut = None;
        let mut shared_state = match self.state.take().expect("State must not be None") {
            StateStore::Reset(rst) => rst.shared_state,
            StateStore::Configure(conf) => conf.shared_state,
            StateStore::Ready(rdy) => rdy.shared_state,
            StateStore::Operate(oper) => oper.shared_state,
        };

        // shared_state.as_mut().project().
        // SAFETY: we can unpin as the future is finished
        let unpinned = unsafe { Pin::get_unchecked_mut(shared_state.as_mut()) };
        unpinned.reset_iface(ifname);

        self.state = Some(StateStore::Reset(Reset {
            shared_state,
        }));
    }

    pub fn get_state_pinned(self: Pin<&mut Self>) -> Option<&mut StateStore> {
        self.project().state.as_mut()
    }

    pub fn get_state(&mut self) -> Option<&mut StateStore> {
        self.state.as_mut()
    }
}

impl Stream for ActuatorBus {
    type Item = std::io::Result<StateTag>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let mut this = self.project();

        if let Some(pending_fut) = this.pending_fut.as_mut().as_pin_mut() {
                // log::debug!("Polling pending future: {:?}", pending_fut);
                // If the pending future is ready, we can transition to the next state
                let StateTransitionResult{ state: st, result } = ready!(pending_fut.poll(cx));
                // clear the pending future
                unsafe { *this.pending_fut.get_unchecked_mut() = None; }

                let tag = st.tag();
                *this.state = Some(st); // Update the state
                if let Err(e) = result {
                    log::error!("State transition failed: {:?}", e);
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
                this.state.take().expect("state must not be None").transition_fut()
            );
        }

        // we return pending for this cycle
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}
