use crate::socketcan2::{SocketCanConfigurator, SocketCanOperator};
use crate::typestate_socket2::{Socket, SocketGraph, SocketState, SocketStorage};
use std::pin::Pin;
use std::task::{Context, Poll};
use tracing::{debug, error, warn};

use crate::socketcan::CanFrame;
use std::fmt::Debug;

use futures::{Stream, StreamExt};

use std::vec::Vec;

use crate::robstride::{
    ActuatorCanClient, ActuatorRequestParams, actuator_can_id_from_response, mux_from_can_frame,
};

use crate::robstride_utils::RobstrideActuatorParam;

use crate::robot_description::{
    ActuatorCommand, ActuatorFeedback, ActuatorFeedbackUpdate, ActuatorId, ActuatorState, BusTag,
};

use pin_project::pin_project;
use std::task::ready;

use crate::state_machine;

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

/// Primary motor fault word (indexes 0x3022 / CAN Types 02, 21, 24).
/// Use `u32` to match the 32‑bit field.
pub mod motor_fault {
    pub const MOTOR_OVERTEMP:        u32 = 1 << 0;   // >145 °C
    pub const DRIVER_FAULT:          u32 = 1 << 1;   // See drv_fault::* below
    pub const UNDERVOLTAGE:          u32 = 1 << 2;   // VBUS < 12 V
    pub const OVERVOLTAGE:           u32 = 1 << 3;   // VBUS > 60 V
    pub const ENCODER_UNCALIBRATED:  u32 = 1 << 7;   // Encoder not zeroed
    pub const STALL_I2T_OVERLOAD:    u32 = 1 << 14;  // Stall / I²t limit
}

/// DRV8353  FAULT_STATUS 1   (mirrored at RS03 index 0x3024)
/// Source: TI SLVSDY6A (Aug‑18, rev. Jun‑19) §8.6.1.1
pub mod drv_fault1 {
    pub const FAULT_OR:  u16 = 1 << 10; // Logic‑OR of *all* faults (mirrors nFAULT pin)
    pub const VDS_OCP:   u16 = 1 <<  9; // Global drain‑source over‑current monitor event
    pub const GDF:       u16 = 1 <<  8; // Gate‑driver fault (charge‑pump or VDS monitor mismatch)
    pub const UVLO:      u16 = 1 <<  7; // Device VCC undervoltage lock‑out
    pub const OTSD:      u16 = 1 <<  6; // Over‑temperature shutdown (≈ 150 °C, latched)
    pub const VDS_HA:    u16 = 1 <<  5; // Phase‑A high‑side VDS over‑current
    pub const VDS_LA:    u16 = 1 <<  4; // Phase‑A low‑side  VDS over‑current
    pub const VDS_HB:    u16 = 1 <<  3; // Phase‑B high‑side VDS over‑current
    pub const VDS_LB:    u16 = 1 <<  2; // Phase‑B low‑side  VDS over‑current
    pub const VDS_HC:    u16 = 1 <<  1; // Phase‑C high‑side VDS over‑current
    pub const VDS_LC:    u16 = 1 <<  0; // Phase‑C low‑side  VDS over‑current
}

/// DRV8353  FAULT_STATUS 2   (mirrored at RS03 index 0x3025)
/// Source: TI SLVSDY6A (Aug‑18, rev. Jun‑19) §8.6.1.2
pub mod drv_fault2 {
    pub const SA_OC:   u16 = 1 << 10; // Phase‑A sense‑amp OC  (S‑variant only)
    pub const SB_OC:   u16 = 1 <<  9; // Phase‑B sense‑amp OC  (S‑variant only)
    pub const SC_OC:   u16 = 1 <<  8; // Phase‑C sense‑amp OC  (S‑variant only)
    pub const OTW:     u16 = 1 <<  7; // Over‑temperature warning
    pub const GDUV:    u16 = 1 <<  6; // Charge‑pump / gate‑drive UV
    pub const VGS_HA:  u16 = 1 <<  5; // Gate fault: phase‑A high‑side
    pub const VGS_LA:  u16 = 1 <<  4; // Gate fault: phase‑A low‑side
    pub const VGS_HB:  u16 = 1 <<  3; // Gate fault: phase‑B high‑side
    pub const VGS_LB:  u16 = 1 <<  2; // Gate fault: phase‑B low‑side
    pub const VGS_HC:  u16 = 1 <<  1; // Gate fault: phase‑C high‑side
    pub const VGS_LC:  u16 = 1 <<  0; // Pha‑C low‑side  VDS over‑current
}

/// CAN Response packet fault flags (bits 21-16 in CAN ID)
/// According to Communication Type 2 motor feedback documentation
pub mod can_response_faults {
    pub const UNCALIBRATED:      u32 = 1 << 21; // bit21: uncalibrated
    pub const GRIDLOCK_OVERLOAD: u32 = 1 << 20; // bit20: Gridlock overload fault  
    pub const MAGNETIC_ENCODING: u32 = 1 << 19; // bit19: magnetic coding fault
    pub const OVERTEMPERATURE:   u32 = 1 << 18; // bit18: overtemperature
    pub const OVERCURRENT:       u32 = 1 << 17; // bit17: overcurrent
    pub const UNDERVOLTAGE:      u32 = 1 << 16; // bit16: undervoltage fault
}

/// Enhanced fault decoding and logging
pub struct FaultDecoder {
    pub motor_fault: u32,
    pub drv_fault1: u16,
    pub drv_fault2: u16,
    pub can_response_faults: u32, // Faults from CAN response packet
}

impl FaultDecoder {
    pub fn new() -> Self {
        Self {
            motor_fault: 0,
            drv_fault1: 0,
            drv_fault2: 0,
            can_response_faults: 0,
        }
    }

    pub fn decode_can_response_faults(&self, fault_value: u32) -> Vec<String> {
        let mut faults = Vec::new();
        
        if fault_value & can_response_faults::UNCALIBRATED != 0 {
            faults.push("CAN: Uncalibrated".to_string());
        }
        if fault_value & can_response_faults::GRIDLOCK_OVERLOAD != 0 {
            faults.push("CAN: Gridlock Overload".to_string());
        }
        if fault_value & can_response_faults::MAGNETIC_ENCODING != 0 {
            faults.push("CAN: Magnetic Encoding Fault".to_string());
        }
        if fault_value & can_response_faults::OVERTEMPERATURE != 0 {
            faults.push("CAN: Over-temperature".to_string());
        }
        if fault_value & can_response_faults::OVERCURRENT != 0 {
            faults.push("CAN: Over-current".to_string());
        }
        if fault_value & can_response_faults::UNDERVOLTAGE != 0 {
            faults.push("CAN: Undervoltage".to_string());
        }
        
        faults
    }

    pub fn decode_motor_fault(&self, fault_value: u32) -> Vec<String> {
        let mut faults = Vec::new();
        
        if fault_value & motor_fault::MOTOR_OVERTEMP != 0 {
            faults.push("Motor Over-temperature (>145°C)".to_string());
        }
        if fault_value & motor_fault::DRIVER_FAULT != 0 {
            faults.push("Driver Fault (see DRV status)".to_string());
        }
        if fault_value & motor_fault::UNDERVOLTAGE != 0 {
            faults.push("Undervoltage (VBUS < 12V)".to_string());
        }  
        if fault_value & motor_fault::OVERVOLTAGE != 0 {
            faults.push("Overvoltage (VBUS > 60V)".to_string());
        }
        if fault_value & motor_fault::ENCODER_UNCALIBRATED != 0 {
            faults.push("Encoder Uncalibrated".to_string());
        }
        if fault_value & motor_fault::STALL_I2T_OVERLOAD != 0 {
            faults.push("Stall/I²t Overload".to_string());
        }
        
        faults
    }

    pub fn decode_drv_fault1(&self, fault_value: u16) -> Vec<String> {
        let mut faults = Vec::new();
        
        if fault_value & drv_fault1::FAULT_OR != 0 {
            faults.push("DRV: General Fault (nFAULT asserted)".to_string());
        }
        if fault_value & drv_fault1::VDS_OCP != 0 {
            faults.push("DRV: Global VDS Over-current".to_string());
        }
        if fault_value & drv_fault1::GDF != 0 {
            faults.push("DRV: Gate Driver Fault".to_string());
        }
        if fault_value & drv_fault1::UVLO != 0 {
            faults.push("DRV: VCC Undervoltage Lock-out".to_string());
        }
        if fault_value & drv_fault1::OTSD != 0 {
            faults.push("DRV: Over-temperature Shutdown (~150°C)".to_string());
        }
        if fault_value & drv_fault1::VDS_HA != 0 {
            faults.push("DRV: Phase-A High VDS Over-current".to_string());
        }
        if fault_value & drv_fault1::VDS_LA != 0 {
            faults.push("DRV: Phase-A Low VDS Over-current".to_string());
        }
        if fault_value & drv_fault1::VDS_HB != 0 {
            faults.push("DRV: Phase-B High VDS Over-current".to_string());
        }
        if fault_value & drv_fault1::VDS_LB != 0 {
            faults.push("DRV: Phase-B Low VDS Over-current".to_string());  
        }
        if fault_value & drv_fault1::VDS_HC != 0 {
            faults.push("DRV: Phase-C High VDS Over-current".to_string());
        }
        if fault_value & drv_fault1::VDS_LC != 0 {
            faults.push("DRV: Phase-C Low VDS Over-current".to_string());
        }
        
        faults
    }

    pub fn decode_drv_fault2(&self, fault_value: u16) -> Vec<String> {
        let mut faults = Vec::new();
        
        if fault_value & drv_fault2::SA_OC != 0 {
            faults.push("DRV: Phase-A Sense-amp Over-current (S-variant)".to_string());
        }
        if fault_value & drv_fault2::SB_OC != 0 {
            faults.push("DRV: Phase-B Sense-amp Over-current (S-variant)".to_string());
        }
        if fault_value & drv_fault2::SC_OC != 0 {
            faults.push("DRV: Phase-C Sense-amp Over-current (S-variant)".to_string());
        }
        if fault_value & drv_fault2::OTW != 0 {
            faults.push("DRV: Over-temperature Warning (~125°C)".to_string());
        }
        if fault_value & drv_fault2::GDUV != 0 {
            faults.push("DRV: Charge-pump/Gate-drive Undervoltage".to_string());
        }
        if fault_value & drv_fault2::VGS_HA != 0 {
            faults.push("DRV: Phase-A High Gate Fault".to_string());
        }
        if fault_value & drv_fault2::VGS_LA != 0 {
            faults.push("DRV: Phase-A Low Gate Fault".to_string());
        }
        if fault_value & drv_fault2::VGS_HB != 0 {
            faults.push("DRV: Phase-B High Gate Fault".to_string());
        }
        if fault_value & drv_fault2::VGS_LB != 0 {
            faults.push("DRV: Phase-B Low Gate Fault".to_string());
        }
        if fault_value & drv_fault2::VGS_HC != 0 {
            faults.push("DRV: Phase-C High Gate Fault".to_string());
        }
        if fault_value & drv_fault2::VGS_LC != 0 {
            faults.push("DRV: Phase-C Low Gate Fault".to_string());
        }
        
        faults
    }

    pub fn log_all_faults(&self, actuator_id: usize) {
        // Log CAN response packet faults
        if self.can_response_faults != 0 {
            let can_faults = self.decode_can_response_faults(self.can_response_faults);
            if !can_faults.is_empty() {
                warn!("Actuator {} CAN Response Faults (bits 21-16): {}", 
                      actuator_id, can_faults.join(", "));
            }
        }
        
        // Log detailed register faults
        if self.motor_fault != 0 {
            let motor_faults = self.decode_motor_fault(self.motor_fault);
            if !motor_faults.is_empty() {
                warn!("Actuator {} Motor Faults (0x3022=0x{:08X}): {}", 
                      actuator_id, self.motor_fault, motor_faults.join(", "));
            }
        }
        
        if self.drv_fault1 != 0 {
            let drv1_faults = self.decode_drv_fault1(self.drv_fault1);
            if !drv1_faults.is_empty() {
                warn!("Actuator {} DRV Fault1 (0x3024=0x{:04X}): {}", 
                      actuator_id, self.drv_fault1, drv1_faults.join(", "));
            }
        }
        
        if self.drv_fault2 != 0 {
            let drv2_faults = self.decode_drv_fault2(self.drv_fault2);
            if !drv2_faults.is_empty() {
                warn!("Actuator {} DRV Fault2 (0x3025=0x{:04X}): {}", 
                      actuator_id, self.drv_fault2, drv2_faults.join(", "));
            }
        }
    }
}

impl Ready {
    pub async fn enable(&mut self) -> std::io::Result<()> {
        send_request(
            self.shared_state.as_mut(),
            &ActuatorRequestParams::MotorEnable,
        )
        .await?;
        read_responses(self.shared_state.as_mut()).await
    }
}

impl Operate {
    pub async fn request_feedback(&mut self) -> std::io::Result<()> {
        // send_request(self.shared_state.as_mut(), ActuatorRequestParams::Feedback).await?;
        // read_responses(self.shared_state.as_mut()).await
        send_request(self.shared_state.as_mut(), &ActuatorRequestParams::Feedback).await
    }

    pub async fn request_param(&mut self, param: RobstrideActuatorParam) -> std::io::Result<()> {
        send_request(
            self.shared_state.as_mut(),
            &ActuatorRequestParams::ReadParam(param),
        )
        .await
    }

    pub async fn command(&mut self, act_states: &[ActuatorState]) -> std::io::Result<()> {
        send_commands(self.shared_state.as_mut(), act_states).await
        // read_responses(self.shared_state.as_mut()).await
    }

    pub async fn process_feedback(
        &mut self,
        act_states: &mut [ActuatorState],
    ) -> std::io::Result<()> {
        // read_responses(self.shared_state.as_mut()).await
        read_responses_update(self.shared_state.as_mut(), Some(act_states)).await
    }
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            let mut ss = shared_state.as_mut().project();

            let res = ss.socket_graph.await;
            match res {
                Ok(_) => {
                    debug!("Socket graph operational");
                    StateTransitionResult {
                        state: StateStore::Configure(Configure { shared_state }),
                        result: Ok(()),
                    }
                }
                Err(e) => {
                    StateTransitionResult {
                        state: StateStore::Reset(Reset { shared_state }), // go back to reset state on error
                        result: Err(e),
                    }
                }
            }
        }
    }
}

impl State for Ready {
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

impl State for Configure {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            if let Err(e) =
                send_request(self.shared_state.as_mut(), &ActuatorRequestParams::ObtainId).await
            {
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
                read_responses(self.shared_state.as_mut()),
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

impl State for Operate {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Operate(Operate { shared_state }),
                result: Ok(()),
            }
        }
    }
}

async fn send_commands(ss: Pin<&mut Store>, act_states: &[ActuatorState]) -> std::io::Result<()> {
    let mut ss = ss.project();
    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.project().state else {
        // no operational socket, go back to configure state
        return Err(std::io::Error::other("Socket is not in Operate state"));
    };

    for (i, client) in ss.actuator_clients.iter_mut().enumerate() {
        let params = ActuatorRequestParams::Control(act_states[i].command);

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

async fn send_request(ss: Pin<&mut Store>, params: &ActuatorRequestParams) -> std::io::Result<()> {
    let mut ss = ss.project();
    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.project().state else {
        // no operational socket, go back to configure state
        return Err(std::io::Error::other("Socket is not in Operate state"));
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

async fn read_responses_update(
    ss: Pin<&mut Store>,
    mut act_states: Option<&mut [ActuatorState]>,
) -> std::io::Result<()> {
    let mut ss = ss.project();

    let Some(SocketState::Operate(op_socket)) = ss.socket_graph.project().state else {
        return Err(std::io::Error::other("Socket is not in Operate state"));
    };

    let n = ss.actuator_clients.len();
    let mut handler = |can_frame: &CanFrame| {
        let client_idx = (ss.response_to_client_idx)(can_frame);
        if let Some(client) = ss.actuator_clients.get_mut(client_idx) {
            match client.handle_response(can_frame) {
                Ok(Some(fdbk)) => {
                    if let Some(ref mut act_states) = act_states {
                        if let Some(state) = act_states.get_mut(client_idx) {
                            state.merge_feedback(fdbk);
                        } else {
                            return Err(std::io::Error::new(
                                std::io::ErrorKind::NotFound,
                                format!("No ActuatorFeedback found for client index {client_idx}"),
                            ));
                        }
                    }
                }
                Ok(None) => {}
                Err(e) => {
                    warn!("Error handling response from actuator {}: {:?}", client_idx, e);
                    // Don't propagate individual response errors - be more tolerant
                }
            }
        } else {
            warn!("No client found for response with index {client_idx}");
        }
        Ok(())
    };

    let mut read_data = [0u8; 16];
    let mut seen = vec![false; n];
    let mut rem = n; // Wait for responses from all actuators
    
    // Use overall timeout to prevent infinite hanging, but much longer than before
    const OVERALL_TIMEOUT_MS: u64 = 100; // 100ms total should be plenty
    let overall_timeout = tokio::time::timeout(
        std::time::Duration::from_millis(OVERALL_TIMEOUT_MS),
        async {
            // Restore original blocking logic but with proper error handling
            while rem > 0 {
                let res = op_socket.read(&mut read_data).await;
                match res {
                    Ok(_) => {
                        let can_frame: CanFrame = unsafe { std::mem::transmute(read_data) };
                        let client_idx = (ss.response_to_client_idx)(&can_frame);
                        
                        if client_idx < n {
                            // Only decrement rem if this is a NEW response
                            if !seen[client_idx] {
                                rem -= 1;
                                seen[client_idx] = true;
                                debug!("First response from actuator {}", client_idx);
                            } else {
                                debug!("Duplicate response from actuator {}", client_idx);
                            }
                            
                            let can_id = can_frame.can_id; // Copy to local variable first
                            debug!("Received CAN frame from actuator {}: ID=0x{:x}", client_idx, can_id);

                            if let Err(e) = handler(&can_frame) {
                                warn!("Handler error for actuator {}: {:?}", client_idx, e);
                                // Continue processing other responses
                            }
                        } else {
                            let can_id = can_frame.can_id;
                            let actual_servo_id = actuator_can_id_from_response(&can_frame);
                            warn!(
                                "Invalid client_idx {} from CAN frame (servo_id: {}, CAN ID: 0x{:08X}, expected range: 0-{})", 
                                client_idx, actual_servo_id, can_id, n - 1
                            );
                        }
                    }
                    Err(e) => {
                        warn!("Socket read error: {:?}", e);
                        break; // Exit on socket errors, but don't fail the whole operation
                    }
                }
            }
            Ok::<(), std::io::Error>(())
        }
    );
    
    match overall_timeout.await {
        Ok(Ok(_)) => {
            debug!("All {} actuators responded within timeout", n);
        }
        Ok(Err(e)) => {
            warn!("Socket error during response reading: {:?}", e);
        }
        Err(_) => {
            warn!("Overall timeout waiting for responses");
        }
    }

    // Drain any remaining responses without blocking
    let mut drain_count = 0;
    const MAX_DRAIN: usize = 10;
    while drain_count < MAX_DRAIN {
        match op_socket.try_read(&mut read_data) {
            Ok(_) => {
                let can_frame: CanFrame = unsafe { std::mem::transmute(read_data) };
                let client_idx = (ss.response_to_client_idx)(&can_frame);

                let can_id = can_frame.can_id; // Copy to local variable first
                debug!("Draining CAN frame from actuator {}: ID=0x{:x}", client_idx, can_id);

                if client_idx < n {
                    if !seen[client_idx] {
                        seen[client_idx] = true;
                        debug!("Late response from actuator {}", client_idx);
                    }
                    if let Err(e) = handler(&can_frame) {
                        warn!("Handler error during drain for actuator {}: {:?}", client_idx, e);
                    }
                }
                drain_count += 1;
            }
            Err(_) => break,
        }
    }

    // Log communication health but don't fail
    let missing_responses: Vec<usize> = seen.iter()
        .enumerate()
        .filter_map(|(idx, &responded)| if !responded { Some(idx) } else { None })
        .collect();

    if !missing_responses.is_empty() {
        warn!("Missing responses from {} actuators: {:?}", missing_responses.len(), missing_responses);
        
        if let Some(ref mut act_states) = act_states {
            for &idx in &missing_responses {
                if let Some(state) = act_states.get_mut(idx) {
                    state.feedback.faults |= 0x20; // Communication error flag
                }
            }
        }
    } else {
        debug!("All {} actuators responded", n);
    }

    Ok(()) // Always succeed - be fault tolerant
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
            actuator_clients: ids.into_iter().map(ActuatorCanClient::new).collect(),
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

        self.state = Some(StateStore::Reset(Reset { shared_state }));
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
