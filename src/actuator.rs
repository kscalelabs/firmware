use crate::socketcan2::{SocketCanConfigurator, SocketCanOperator};
use crate::typestate_socket2::{Socket, SocketGraph, SocketState, SocketStorage};
use std::pin::Pin;
use std::task::{Context, Poll};
use tracing::{debug, error, info, warn};

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

use std::fmt::{self, Display};

// Motor faults enum with bit values and descriptions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorFault {
    MotorOvertemp,      // >145 °C
    DriverFault,        // See drv_fault::* below
    Undervoltage,       // VBUS < 12 V
    Overvoltage,        // VBUS > 60 V
    EncoderUncalibrated, // Encoder not zeroed
    StallI2tOverload,   // Stall / I²t limit
}

impl MotorFault {
     pub fn critical_faults() -> &'static [Self] {
        &[Self::MotorOvertemp, Self::DriverFault, Self::StallI2tOverload]
    }

    pub fn has_critical_faults_in_mask(mask: u32) -> bool {
        Self::critical_faults()
            .iter()
            .any(|fault| mask & fault.bit_value() != 0)
    }
    pub const fn bit_value(self) -> u32 {
        match self {
            Self::MotorOvertemp => 1 << 0,
            Self::DriverFault => 1 << 1,
            Self::Undervoltage => 1 << 2,
            Self::Overvoltage => 1 << 3,
            Self::EncoderUncalibrated => 1 << 7,
            Self::StallI2tOverload => 1 << 14,
        }
    }

    pub fn all_variants() -> &'static [Self] {
        &[
            Self::MotorOvertemp,
            Self::DriverFault,
            Self::Undervoltage,
            Self::Overvoltage,
            Self::EncoderUncalibrated,
            Self::StallI2tOverload,
        ]
    }

    pub fn from_bitmask(mask: u32) -> Vec<Self> {
        Self::all_variants()
            .iter()
            .filter(|fault| mask & fault.bit_value() != 0)
            .copied()
            .collect()
    }

    pub fn to_bitmask(faults: &[Self]) -> u32 {
        faults.iter().fold(0, |acc, fault| acc | fault.bit_value())
    }
}

impl Display for MotorFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MotorOvertemp => write!(f, "Motor Over-temperature (>145°C)"),
            Self::DriverFault => write!(f, "Driver Fault (see DRV status)"),
            Self::Undervoltage => write!(f, "Undervoltage (VBUS < 12V)"),
            Self::Overvoltage => write!(f, "Overvoltage (VBUS > 60V)"),
            Self::EncoderUncalibrated => write!(f, "Encoder Uncalibrated"),
            Self::StallI2tOverload => write!(f, "Stall/I²t Overload"),
        }
    }
}

impl TryFrom<u32> for MotorFault {
    type Error = &'static str;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            v if v == Self::MotorOvertemp.bit_value() => Ok(Self::MotorOvertemp),
            v if v == Self::DriverFault.bit_value() => Ok(Self::DriverFault),
            v if v == Self::Undervoltage.bit_value() => Ok(Self::Undervoltage),
            v if v == Self::Overvoltage.bit_value() => Ok(Self::Overvoltage),
            v if v == Self::EncoderUncalibrated.bit_value() => Ok(Self::EncoderUncalibrated),
            v if v == Self::StallI2tOverload.bit_value() => Ok(Self::StallI2tOverload),
            _ => Err("Invalid motor fault bit value"),
        }
    }
}

// DRV Fault1 enum
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DrvFault1 {
    FaultOr,    // Logic‑OR of *all* faults (mirrors nFAULT pin)
    VdsOcp,     // Global drain‑source over‑current monitor event
    Gdf,        // Gate‑driver fault (charge‑pump or VDS monitor mismatch)
    Uvlo,       // Device VCC undervoltage lock‑out
    Otsd,       // Over‑temperature shutdown (≈ 150 °C, latched)
    VdsHa,      // Phase‑A high‑side VDS over‑current
    VdsLa,      // Phase‑A low‑side  VDS over‑current
    VdsHb,      // Phase‑B high‑side VDS over‑current
    VdsLb,      // Phase‑B low‑side  VDS over‑current
    VdsHc,      // Phase‑C high‑side VDS over‑current
    VdsLc,      // Phase‑C low‑side  VDS over‑current
}

impl DrvFault1 {
    pub const fn bit_value(self) -> u16 {
        match self {
            Self::FaultOr => 1 << 10,
            Self::VdsOcp => 1 << 9,
            Self::Gdf => 1 << 8,
            Self::Uvlo => 1 << 7,
            Self::Otsd => 1 << 6,
            Self::VdsHa => 1 << 5,
            Self::VdsLa => 1 << 4,
            Self::VdsHb => 1 << 3,
            Self::VdsLb => 1 << 2,
            Self::VdsHc => 1 << 1,
            Self::VdsLc => 1 << 0,
        }
    }

    pub fn all_variants() -> &'static [Self] {
        &[
            Self::FaultOr, Self::VdsOcp, Self::Gdf, Self::Uvlo, Self::Otsd,
            Self::VdsHa, Self::VdsLa, Self::VdsHb, Self::VdsLb, Self::VdsHc, Self::VdsLc,
        ]
    }

    pub fn from_bitmask(mask: u16) -> Vec<Self> {
        Self::all_variants()
            .iter()
            .filter(|fault| mask & fault.bit_value() != 0)
            .copied()
            .collect()
    }

    pub fn to_bitmask(faults: &[Self]) -> u16 {
        faults.iter().fold(0, |acc, fault| acc | fault.bit_value())
    }
}

impl Display for DrvFault1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FaultOr => write!(f, "DRV: General Fault (nFAULT asserted)"),
            Self::VdsOcp => write!(f, "DRV: Global VDS Over-current"),
            Self::Gdf => write!(f, "DRV: Gate Driver Fault"),
            Self::Uvlo => write!(f, "DRV: VCC Undervoltage Lock-out"),
            Self::Otsd => write!(f, "DRV: Over-temperature Shutdown (~150°C)"),
            Self::VdsHa => write!(f, "DRV: Phase-A High VDS Over-current"),
            Self::VdsLa => write!(f, "DRV: Phase-A Low VDS Over-current"),
            Self::VdsHb => write!(f, "DRV: Phase-B High VDS Over-current"),
            Self::VdsLb => write!(f, "DRV: Phase-B Low VDS Over-current"),
            Self::VdsHc => write!(f, "DRV: Phase-C High VDS Over-current"),
            Self::VdsLc => write!(f, "DRV: Phase-C Low VDS Over-current"),
        }
    }
}

// DRV Fault2 enum  
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DrvFault2 {
    SaOc,    // Phase‑A sense‑amp OC  (S‑variant only)
    SbOc,    // Phase‑B sense‑amp OC  (S‑variant only)
    ScOc,    // Phase‑C sense‑amp OC  (S‑variant only)
    Otw,     // Over‑temperature warning
    Gduv,    // Charge‑pump / gate‑drive UV
    VgsHa,   // Gate fault: phase‑A high‑side
    VgsLa,   // Gate fault: phase‑A low‑side
    VgsHb,   // Gate fault: phase‑B high‑side
    VgsLb,   // Gate fault: phase‑B low‑side
    VgsHc,   // Gate fault: phase‑C high‑side
    VgsLc,   // Gate fault: phase‑C low‑side
}

impl DrvFault2 {
    pub const fn bit_value(self) -> u16 {
        match self {
            Self::SaOc => 1 << 10,
            Self::SbOc => 1 << 9,
            Self::ScOc => 1 << 8,
            Self::Otw => 1 << 7,
            Self::Gduv => 1 << 6,
            Self::VgsHa => 1 << 5,
            Self::VgsLa => 1 << 4,
            Self::VgsHb => 1 << 3,
            Self::VgsLb => 1 << 2,
            Self::VgsHc => 1 << 1,
            Self::VgsLc => 1 << 0,
        }
    }

    pub fn all_variants() -> &'static [Self] {
        &[
            Self::SaOc, Self::SbOc, Self::ScOc, Self::Otw, Self::Gduv,
            Self::VgsHa, Self::VgsLa, Self::VgsHb, Self::VgsLb, Self::VgsHc, Self::VgsLc,
        ]
    }

    pub fn from_bitmask(mask: u16) -> Vec<Self> {
        Self::all_variants()
            .iter()
            .filter(|fault| mask & fault.bit_value() != 0)
            .copied()
            .collect()
    }

    pub fn to_bitmask(faults: &[Self]) -> u16 {
        faults.iter().fold(0, |acc, fault| acc | fault.bit_value())
    }
}

impl Display for DrvFault2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SaOc => write!(f, "DRV: Phase-A Sense-amp Over-current (S-variant)"),
            Self::SbOc => write!(f, "DRV: Phase-B Sense-amp Over-current (S-variant)"),
            Self::ScOc => write!(f, "DRV: Phase-C Sense-amp Over-current (S-variant)"),
            Self::Otw => write!(f, "DRV: Over-temperature Warning (~125°C)"),
            Self::Gduv => write!(f, "DRV: Charge-pump/Gate-drive Undervoltage"),
            Self::VgsHa => write!(f, "DRV: Phase-A High Gate Fault"),
            Self::VgsLa => write!(f, "DRV: Phase-A Low Gate Fault"),
            Self::VgsHb => write!(f, "DRV: Phase-B High Gate Fault"),
            Self::VgsLb => write!(f, "DRV: Phase-B Low Gate Fault"),
            Self::VgsHc => write!(f, "DRV: Phase-C High Gate Fault"),
            Self::VgsLc => write!(f, "DRV: Phase-C Low Gate Fault"),
        }
    }
}

// CAN Response faults enum
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanResponseFault {
    Uncalibrated,      // bit21: uncalibrated
    GridlockOverload,  // bit20: Gridlock overload fault  
    MagneticEncoding,  // bit19: magnetic coding fault
    Overtemperature,   // bit18: overtemperature
    Overcurrent,       // bit17: overcurrent
    Undervoltage,      // bit16: undervoltage fault
}

impl CanResponseFault {
    pub fn has_critical_faults_in_mask(mask: u32) -> bool {
        Self::critical_faults()
            .iter()
            .any(|fault| mask & fault.bit_value() != 0)
    }

    pub const fn bit_value(self) -> u32 {
        match self {
            Self::Uncalibrated => 1 << 21,
            Self::GridlockOverload => 1 << 20,
            Self::MagneticEncoding => 1 << 19,
            Self::Overtemperature => 1 << 18,
            Self::Overcurrent => 1 << 17,
            Self::Undervoltage => 1 << 16,
        }
    }

    pub fn all_variants() -> &'static [Self] {
        &[
            Self::Uncalibrated,
            Self::GridlockOverload,
            Self::MagneticEncoding,
            Self::Overtemperature,
            Self::Overcurrent,
            Self::Undervoltage,
        ]
    }

    pub fn from_bitmask(mask: u32) -> Vec<Self> {
        Self::all_variants()
            .iter()
            .filter(|fault| mask & fault.bit_value() != 0)
            .copied()
            .collect()
    }

    pub fn to_bitmask(faults: &[Self]) -> u32 {
        faults.iter().fold(0, |acc, fault| acc | fault.bit_value())
    }

    /// Get critical faults that make operation unsafe
    pub fn critical_faults() -> &'static [Self] {
        &[Self::Overtemperature, Self::Overcurrent]
    }
}

impl Display for CanResponseFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Uncalibrated => write!(f, "CAN: Uncalibrated"),
            Self::GridlockOverload => write!(f, "CAN: Gridlock Overload"),
            Self::MagneticEncoding => write!(f, "CAN: Magnetic Encoding Fault"),
            Self::Overtemperature => write!(f, "CAN: Over-temperature"),
            Self::Overcurrent => write!(f, "CAN: Over-current"),
            Self::Undervoltage => write!(f, "CAN: Undervoltage"),
        }
    }
}

// Additional fault for communication errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SystemFault {
    CommunicationError, // Communication timeout/error
}

impl SystemFault {
    pub const fn bit_value(self) -> u32 {
        match self {
            Self::CommunicationError => 0x20,
        }
    }
}

impl Display for SystemFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CommunicationError => write!(f, "Communication Error"),
        }
    }
}

// Unified fault collection for easier handling
#[derive(Debug, Clone)]
pub struct FaultCollection {
    pub motor_faults: Vec<MotorFault>,
    pub drv_fault1: Vec<DrvFault1>,
    pub drv_fault2: Vec<DrvFault2>,
    pub can_response_faults: Vec<CanResponseFault>,
    pub system_faults: Vec<SystemFault>,
}

impl FaultCollection {
    pub fn new() -> Self {
        Self {
            motor_faults: Vec::new(),
            drv_fault1: Vec::new(),
            drv_fault2: Vec::new(),
            can_response_faults: Vec::new(),
            system_faults: Vec::new(),
        }
    }

    pub fn from_raw_values(
        motor_faults: Vec<MotorFault>,
        drv_fault1: Vec<DrvFault1>,
        drv_fault2: Vec<DrvFault2>,
        can_response_faults: Vec<CanResponseFault>,
        system_faults: Vec<SystemFault>,
    ) -> Self {
        Self {
            motor_faults,
            drv_fault1,
            drv_fault2,
            can_response_faults,
            system_faults,
        }
    }

    pub fn has_faults(&self) -> bool {
        !self.motor_faults.is_empty()
            || !self.drv_fault1.is_empty()
            || !self.drv_fault2.is_empty()
            || !self.can_response_faults.is_empty()
            || !self.system_faults.is_empty()
    }

    pub fn is_safe_to_operate(&self) -> bool {
        let critical_motor_faults = [
            MotorFault::MotorOvertemp,
            MotorFault::DriverFault,
            MotorFault::StallI2tOverload,
        ];

        let has_critical_motor = self.motor_faults.iter()
            .any(|fault| critical_motor_faults.contains(fault));

        let has_critical_can = self.can_response_faults.iter()
            .any(|fault| CanResponseFault::critical_faults().contains(fault));

        !(has_critical_motor || has_critical_can)
    }

    pub fn description(&self) -> String {
        if !self.has_faults() {
            return "No faults".to_string();
        }

        let mut descriptions = Vec::new();

        descriptions.extend(self.can_response_faults.iter().map(|f| f.to_string()));
        descriptions.extend(self.motor_faults.iter().map(|f| f.to_string()));
        descriptions.extend(self.drv_fault1.iter().map(|f| f.to_string()));
        descriptions.extend(self.drv_fault2.iter().map(|f| f.to_string()));
        descriptions.extend(self.system_faults.iter().map(|f| f.to_string()));

        descriptions.join(", ")
    }

    pub fn log_all_faults(&self, actuator_id: usize) {
        if !self.can_response_faults.is_empty() {
            let descriptions: Vec<String> = self.can_response_faults.iter().map(|f| f.to_string()).collect();
            let bitmask = CanResponseFault::to_bitmask(&self.can_response_faults);
            warn!("Actuator {} CAN Response Faults (bits 21-16=0x{:06X}): {}", 
                  actuator_id, bitmask >> 16, descriptions.join(", "));
        }

        if !self.motor_faults.is_empty() {
            let descriptions: Vec<String> = self.motor_faults.iter().map(|f| f.to_string()).collect();
            let bitmask = MotorFault::to_bitmask(&self.motor_faults);
            warn!("Actuator {} Motor Faults (0x3022=0x{:08X}): {}", 
                  actuator_id, bitmask, descriptions.join(", "));
        }

        if !self.drv_fault1.is_empty() {
            let descriptions: Vec<String> = self.drv_fault1.iter().map(|f| f.to_string()).collect();
            let bitmask = DrvFault1::to_bitmask(&self.drv_fault1);
            warn!("Actuator {} DRV Fault1 (0x3024=0x{:04X}): {}", 
                  actuator_id, bitmask, descriptions.join(", "));
        }

        if !self.drv_fault2.is_empty() {
            let descriptions: Vec<String> = self.drv_fault2.iter().map(|f| f.to_string()).collect();
            let bitmask = DrvFault2::to_bitmask(&self.drv_fault2);
            warn!("Actuator {} DRV Fault2 (0x3025=0x{:04X}): {}", 
                  actuator_id, bitmask, descriptions.join(", "));
        }
    }
}

#[derive(Debug)]
pub struct FaultDecoder {
    pub motor_fault: Vec<MotorFault>,
    pub drv_fault1: Vec<DrvFault1>,
    pub drv_fault2: Vec<DrvFault2>,
    pub can_response_faults: Vec<CanResponseFault>,
}

impl FaultDecoder {
    pub fn new() -> Self {
        Self {
            motor_fault: Vec::new(),
            drv_fault1: Vec::new(),
            drv_fault2: Vec::new(),
            can_response_faults: Vec::new(),
        }
    }

    pub fn from_raw_values(
        motor_fault: u32,
        drv_fault1: u16,
        drv_fault2: u16,
        can_response_faults: u32,
    ) -> Self {
        Self {
            motor_fault: MotorFault::from_bitmask(motor_fault),
            drv_fault1: DrvFault1::from_bitmask(drv_fault1),
            drv_fault2: DrvFault2::from_bitmask(drv_fault2),
            can_response_faults: CanResponseFault::from_bitmask(can_response_faults),
        }
    }

    pub fn get_fault_collection(&self) -> FaultCollection {
        FaultCollection::from_raw_values(
            self.motor_fault.clone(),
            self.drv_fault1.clone(),
            self.drv_fault2.clone(),
            self.can_response_faults.clone(),
            Vec::new(), // system faults...
        )
    }

    pub fn log_all_faults(&self, actuator_id: usize) {
        self.get_fault_collection().log_all_faults(actuator_id);
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
    }

    pub async fn process_feedback(
        &mut self,
        act_states: &mut [ActuatorState],
    ) -> std::io::Result<()> {
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

            // wait for responses
            let to = tokio::time::timeout(
                std::time::Duration::from_millis(150),
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
    
    // Pre-compute servo ID mapping to avoid borrow conflicts
    let actuator_ids: Vec<u8> = ss.actuator_clients.iter()
        .map(|client| client.actuator_can_id)
        .collect();

    let mut handler = |can_frame: &CanFrame| {
        let client_idx = (ss.response_to_client_idx)(can_frame);
        if let Some(client) = ss.actuator_clients.get_mut(client_idx) {
            match client.handle_response(can_frame) {
                Ok(Some(fdbk)) => {
                    if let Some(ref mut act_states) = act_states {
                        if let Some(state) = act_states.get_mut(client_idx) {
                            // Rising-edge CAN fault logging
                            if let Some(new_faults) = fdbk.faults {
                                let prev_faults = state.feedback.faults;
                                let rising = new_faults & !prev_faults;
                                if rising != 0 {
                                    let descriptions: Vec<String> = CanResponseFault::from_bitmask(rising)
                                        .iter()
                                        .map(|f| f.to_string())
                                        .collect();
                                    let actuator_id = actuator_ids.get(client_idx).copied().unwrap_or(0xFF);
                                    warn!(
                                        "Actuator {} CAN Response Faults (0x{:06X}): {}",
                                        actuator_id,
                                        rising >> 16,
                                        descriptions.join(", ")
                                    );
                                }
                            }
            
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
                    //warn!("Error handling response from actuator {}: {:?}", client_idx, e);
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
    const OVERALL_TIMEOUT_MS: u64 = 100; // This is not good, please do better
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
                                // Extract actuator_id directly from the CAN frame - no borrow conflicts
                                let actuator_id = actuator_can_id_from_response(&can_frame);
                                debug!("First response from actuator {} (actuator_id: {}) on {}", client_idx, actuator_id, ss.ifname);
                            } else {
                                let actuator_id = actuator_ids.get(client_idx).copied().unwrap_or(0xFF);
                                debug!("Duplicate response from actuator {} (actuator_id: {}) on {}", client_idx, actuator_id, ss.ifname);
                            }
                            
                            let can_id = can_frame.can_id; // Copy to local variable first
                            let actuator_id = actuator_ids.get(client_idx).copied().unwrap_or(0xFF);
                            debug!("Received CAN frame from actuator {} (actuator_id: {}) on {}: ID=0x{:x}", client_idx, actuator_id, ss.ifname, can_id);

                            if let Err(e) = handler(&can_frame) {
                                let actuator_id = actuator_ids.get(client_idx).copied().unwrap_or(0xFF);
                                warn!("Handler error for actuator {} (actuator_id: {}) on {}: {:?}", client_idx, actuator_id, ss.ifname, e);
                                // Continue processing other responses
                            }
                        } else {
                            let can_id = can_frame.can_id;
                            let actual_actuator_id = actuator_can_id_from_response(&can_frame);
                            warn!(
                                "Invalid client_idx {} from CAN frame on {} (actuator_id: {}, CAN ID: 0x{:08X}, expected range: 0-{})", 
                                client_idx, ss.ifname, actual_actuator_id, can_id, n - 1
                            );
                        }
                    }
                    Err(e) => {
                        warn!("Socket read error on {}: {:?}", ss.ifname, e);
                        break; // Exit on socket errors, but don't fail the whole operation
                    }
                }
            }
            Ok::<(), std::io::Error>(())
        }
    );
    
    match overall_timeout.await {
        Ok(Ok(_)) => {
            debug!("All {} actuators responded within timeout on {}", n, ss.ifname);
        }
        Ok(Err(e)) => {
            warn!("Socket error during response reading on {}: {:?}", ss.ifname, e);
        }
        Err(_) => {
            // Count without collecting
            let missing_count = seen.iter()
                .enumerate()
                .filter(|&(_, responded)| !responded)
                .count();

            // Edge-Trigger / Rate-Limit the missing count
            let now = std::time::Instant::now();
            let prev_missing = *ss.last_missing_count;
            let prev_time = *ss.last_missing_log_at;
            let changed = prev_missing.map_or(true, |prev| prev != missing_count);
            let due = prev_time
                .map_or(true, |t| now.duration_since(t) >= std::time::Duration::from_millis(2000));

            if missing_count > 0 && (changed || due) {
                warn!(
                    "Overall timeout waiting for responses on {}: Missing responses from {} actuators",
                    ss.ifname, missing_count
                );
                *ss.last_missing_count = Some(missing_count);
                *ss.last_missing_log_at = Some(now);
            } else if missing_count == 0 && prev_missing.unwrap_or(0) != 0 {
                info!("Recovered from missing responses on {}", ss.ifname);
                *ss.last_missing_count = Some(0);
                *ss.last_missing_log_at = Some(now);
            }
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
                let actuator_id = if client_idx < n {
                    actuator_ids.get(client_idx).copied().unwrap_or(0xFF)
                } else {
                    actuator_can_id_from_response(&can_frame)
                };
                debug!("Draining CAN frame from actuator {} (actuator_id: {}) on {}: ID=0x{:x}", client_idx, actuator_id, ss.ifname, can_id);

                if client_idx < n {
                    if !seen[client_idx] {
                        seen[client_idx] = true;
                        debug!("Late response from actuator {} (actuator_id: {}) on {}", client_idx, actuator_id, ss.ifname);
                    }
                    if let Err(e) = handler(&can_frame) {
                        warn!("Handler error during drain for actuator {} (actuator_id: {}) on {}: {:?}", client_idx, actuator_id, ss.ifname, e);
                    }
                }
                drain_count += 1;
            }
            Err(_) => break,
        }
    }

    // Log communication health but don't fail
    // Instead of collecting every cycle, iterate directly
    let mut missing_count = 0;
    let mut any_missing = false;

    // Set fault flags and count in one pass - no allocation
    if let Some(ref mut act_states) = act_states {
        for (idx, &responded) in seen.iter().enumerate() {
            if !responded {
                any_missing = true;
                missing_count += 1;
                if let Some(state) = act_states.get_mut(idx) {
                    state.feedback.faults |= 0x20; // Communication error flag
                }
            }
        }
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

    last_missing_count: Option<usize>,
    last_missing_log_at: Option<std::time::Instant>,
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
            last_missing_count: None,
            last_missing_log_at: None,
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
