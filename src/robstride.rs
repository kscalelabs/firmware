use crate::socketcan::CAN_MAX_DLEN;
use tracing::{debug, warn};

use tracing::info;

use crate::socketcan::CanFrame;

use crate::robot_description::{ActuatorCommand, ActuatorFeedbackUpdate, ActuatorId};
use crate::actuator::{CanResponseFault, MotorFault, SystemFault, FaultDecoder, DrvFault1, DrvFault2};

use crate::robstride_utils::*;

impl<T> From<T> for crate::socketcan::CanFrame
where
    T: RobstrideActuatorFrame + bytemuck::Pod + bytemuck::Zeroable,
{
    fn from(req: T) -> Self {
        let mut ret = bytemuck::must_cast::<T, Self>(req);
        ret.can_id |= 0x8000_0000; // EFF FLAG
        ret
    }
}

impl From<ActuatorRequest> for crate::socketcan::CanFrame {
    fn from(req: ActuatorRequest) -> Self {
        match req {
            ActuatorRequest::ObtainId(req) => req.into(),
            ActuatorRequest::Control(req) => req.into(),
            ActuatorRequest::ReadParam(req) => req.into(),
            ActuatorRequest::MotorEnable(req) => req.into(),
            ActuatorRequest::Feedback(req) => req.into(),
        }
    }
}

impl From<crate::socketcan::CanFrame> for ActuatorResponse {
    fn from(mut val: crate::socketcan::CanFrame) -> Self {
        val.can_id ^= 0x8000_0000; // remove EFF FLAG
        let mux = mux_from_can_frame(&val);
        match mux {
            0x00 => ActuatorResponse::ObtainId(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                ObtainIdResponse,
            >(val)),
            0x02 => ActuatorResponse::Feedback(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                FeedbackResponse,
            >(val)),
            0x11 => ActuatorResponse::ReadParam(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                ReadParamResponse,
            >(val)),
            0x15 => ActuatorResponse::Fault(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                FaultResponse,
            >(val)),
            _ => panic!("Unknown mux value: {mux}"),
        }
    }
}

impl From<crate::socketcan::CanFrame> for ActuatorRequest {
    fn from(mut val: crate::socketcan::CanFrame) -> Self {
        val.can_id &= !0x80; // clear EFF FLAG
        let mux = mux_from_can_frame(&val);
        // TODO: change mux values from u8 to an enum
        match mux {
            0x00 => ActuatorRequest::ObtainId(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                ObtainIdRequest,
            >(val)),
            0x01 => ActuatorRequest::Control(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                ControlCommandRequest,
            >(val)),
            0x02 => ActuatorRequest::Feedback(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                FeedbackRequest,
            >(val)),
            0x03 => ActuatorRequest::MotorEnable(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                MotorEnableRequest,
            >(val)),
            0x11 => ActuatorRequest::ReadParam(bytemuck::must_cast::<
                crate::socketcan::CanFrame,
                ReadParamRequest,
            >(val)),
           
        
            _ => panic!("Unknown mux value: {mux}"),
        }
    }
}

pub trait RobstrideActuatorFrame {}
impl RobstrideActuatorFrame for ObtainIdRequest {}
impl RobstrideActuatorFrame for ObtainIdResponse {}
impl RobstrideActuatorFrame for ControlCommandRequest {}
impl RobstrideActuatorFrame for FeedbackRequest {}
impl RobstrideActuatorFrame for FeedbackResponse {}
impl RobstrideActuatorFrame for ReadParamRequest {}
impl RobstrideActuatorFrame for ReadParamResponse {}
impl RobstrideActuatorFrame for MotorEnableRequest {}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct ObtainIdRequest {
    pub actuator_can_id: u8,
    pub host_id: u16,
    mux: u8, /* 0x00 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    can_data: [u8; CAN_MAX_DLEN],
}

impl ObtainIdRequest {
    pub fn new(host_id: u16, actuator_can_id: u8) -> Self {
        Self {
            mux: 0x00,
            host_id,
            actuator_can_id,
            len: 8,
            ..Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct ObtainIdResponse {
    fe: u8,
    pub actuator_can_id: u16,
    mux: u8, /* 0x00 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    mcu_uid: u64,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct ControlCommandRequest {
    pub actuator_can_id: u8,
    pub torque_scale: u16,
    mux: u8, /* 0x01 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    pub angle_scale: u16,
    pub angular_vel_scale: u16,
    pub kp_scale: u16,
    pub kd_scale: u16,
}

impl ControlCommandRequest {
    pub fn new(
        actuator_can_id: u8,
        torque_scale: u16,
        angle_scale: u16,
        angular_vel_scale: u16,
        kd_scale: u16,
        kp_scale: u16,
    ) -> Self {
        Self {
            mux: 0x01,
            torque_scale,
            actuator_can_id,
            len: 8,
            angle_scale: angle_scale.to_be(),
            angular_vel_scale: angular_vel_scale.to_be(),
            kp_scale: kp_scale.to_be(),
            kd_scale: kd_scale.to_be(),
            ..Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct MotorEnableRequest {
    pub actuator_can_id: u8,
    pub host_id: u16,
    mux: u8, /* 0x03 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    can_data: [u8; CAN_MAX_DLEN],
}

impl MotorEnableRequest {
    pub fn new(host_id: u16, actuator_can_id: u8) -> Self {
        Self {
            mux: 0x03,
            host_id,
            actuator_can_id,
            len: 8,
            ..Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct FeedbackResponse {
    host_id: u8,
    pub actuator_can_id: u8,
    fault_flags: u8,
    mux: u8, /* 0x2 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    // can_data: [u8; CAN_MAX_DLEN],
    pub angle_scale_be: u16,
    pub angular_vel_scale_be: u16,
    pub torque_be: u16,
    pub temp_be: u16,
}

impl FeedbackResponse {
    /// Get the fault flags as a u32
    pub fn get_fault_flags(&self) -> u32 {
        self.fault_flags as u32
    }
    
    /// Check if a specific fault flag is set
    pub fn has_fault(&self, fault_flag: u8) -> bool {
        (self.fault_flags & fault_flag) != 0
    }
    
    /// Get temperature in Celsius
    pub fn get_temperature(&self) -> f64 {
        self.temp_be.swap_bytes() as f64 / 10.0 // Protocol says Temp(Celsius) * 10
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct FeedbackRequest {
    /**
     * NOTE: this is not in Robstride Docs! I don't know if this is safe
     */
    pub actuator_can_id: u8,
    pub host_id: u16,
    mux: u8, /* 0x02 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    can_data: [u8; CAN_MAX_DLEN],
}

impl FeedbackRequest {
    pub fn new(host_id: u16, actuator_can_id: u8) -> Self {
        Self {
            mux: 0x02,
            host_id,
            actuator_can_id,
            len: 8,
            ..Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct FaultResponse {
    host_id: u8,
    pub actuator_can_id: u8,
    reserved1: u8,  // Split the u16 into two u8 fields to match the pattern
    mux: u8, /* 0x15 */
    
    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    
    // Data field (8 bytes total) - matches CAN_MAX_DLEN
    // Based on the fault table: Byte0~3 fault value, Byte4~7 warning value
    pub fault_value: u32,    // 4 bytes
    pub warning_value: u32,  // 4 bytes
}

impl FaultResponse {
    /// Check if any faults are present
    pub fn has_faults(&self) -> bool {
        self.fault_value != 0
    }
    
    /// Check if any warnings are present
    pub fn has_warnings(&self) -> bool {
        self.warning_value != 0
    }
    
    // Get fault descriptions
    pub fn get_fault_descriptions(&self) -> Vec<String> {
        use crate::actuator::MotorFault;
        MotorFault::from_bitmask(self.fault_value)
        .iter()
        .map(|fault| fault.to_string())
        .collect()
    }
    
    pub fn get_warning_descriptions(&self) -> Vec<String> {
        let mut warnings = Vec::new();
        let warning = self.warning_value;

        if warning & (1 << 0) != 0 {
            warnings.push("Motor overtemperature warning (default 135°C)".to_string());
        }

        warnings
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct ReadParamRequest {
    pub actuator_can_id: u8,
    pub host_id: u16,
    mux: u8, /* 0x11 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    index: u16,
    res1: u16,
    res2: u32,
}

impl ReadParamRequest {
    pub fn new(host_id: u16, actuator_can_id: u8, index: u16) -> Self {
        Self {
            mux: 0x11,
            index: index.to_le(),
            actuator_can_id,
            host_id,
            len: 8,
            ..Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct ReadParamResponse {
    pub host_id: u8,
    pub actuator_can_id: u8,
    res_id: u8,
    mux: u8, /* 0x11 */

    len: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    index: u16,
    res1: u16,
    value: u32,
}

#[derive(Debug, Clone)]
pub enum ActuatorRequest {
    ObtainId(ObtainIdRequest),
    Control(ControlCommandRequest),
    ReadParam(ReadParamRequest),
    MotorEnable(MotorEnableRequest),
    Feedback(FeedbackRequest),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActuatorRequestParams {
    ObtainId,
    ReadParam(RobstrideActuatorParam),
    MotorEnable,
    Control(ActuatorCommand),
    Feedback,
}

impl ActuatorRequest {
    pub fn response_mux(&self) -> u8 {
        match self {
            Self::ObtainId(_) => 0x0,     // Obtain Id mux
            Self::Control(_) => 0x2,      // feedback mux
            Self::ReadParam(_) => 0x11,   // read param mux
            Self::MotorEnable(_) => 0x02, // feedback mux
            Self::Feedback(_) => 0x02,    // feedback mux
        }
    }
}

#[derive(Debug, Clone)]
pub enum ActuatorResponse {
    ObtainId(ObtainIdResponse),
    Feedback(FeedbackResponse),
    ReadParam(ReadParamResponse),
    Fault(FaultResponse),
}

#[allow(dead_code)]
impl ActuatorResponse {
    pub fn mux(&self) -> u8 {
        match self {
            ActuatorResponse::ObtainId(req) => req.mux,
            ActuatorResponse::Feedback(req) => req.mux,
            ActuatorResponse::ReadParam(req) => req.mux,
            ActuatorResponse::Fault(req) => req.mux,
        }
    }
}

pub fn mux_from_can_frame(frame: &crate::socketcan::CanFrame) -> u8 {
    // SAFETY: CanFrame is POD and has the same size as [u8; std::mem::size_of::<CanFrame>()].
    let frame: &[u8; std::mem::size_of::<crate::socketcan::CanFrame>()] = bytemuck::cast_ref(frame);
    frame[3] & 0x1F // Mask to get the mux (5 bits)
}

pub fn actuator_can_id_from_response(frame: &crate::socketcan::CanFrame) -> u8 {
    let mux = mux_from_can_frame(frame);
    match mux {
        0x00 => {
            bytemuck::must_cast::<crate::socketcan::CanFrame, ObtainIdResponse>(*frame)
                .actuator_can_id as u8
        }
        0x02 => {
            bytemuck::must_cast::<crate::socketcan::CanFrame, FeedbackResponse>(*frame)
                .actuator_can_id as u8
        }
        0x11 => {
            bytemuck::must_cast::<crate::socketcan::CanFrame, ReadParamResponse>(*frame)
                .actuator_can_id as u8
        }
        0x15 => {
            // For fault frames, extract servo ID directly from CAN ID 
            // since the fault table shows servo ID is in bits 23-8 of CAN ID
            actuator_id_from_can_id(frame.can_id)
        }
        _ => {
            let can_id = frame.can_id;
            warn!(
                "Unknown mux value: 0x{:02X} in actuator_can_id_from_response, CAN ID: 0x{:08X}, returning 0x7F",
                mux, can_id
            );
            0x7F // Return a default value if the mux is unknown
        }
    }
}

/// Extract fault flags from CAN ID (bits 21-16) according to protocol documentation
pub fn fault_flags_from_can_id(can_id: u32) -> u8 {
    ((can_id >> 16) & 0x3F) as u8  // Extract bits 21-16 (6 bits)
}

/// Extract mode status from CAN ID (bits 23-22) according to protocol documentation  
pub fn mode_status_from_can_id(can_id: u32) -> u8 {
    ((can_id >> 22) & 0x03) as u8  // Extract bits 23-22 (2 bits)
}

/// Extract actuator CAN ID from CAN ID field (bits 15-8) according to protocol documentation
pub fn actuator_id_from_can_id(can_id: u32) -> u8 {
    ((can_id >> 8) & 0xFF) as u8  // Extract bits 15-8 (8 bits)
}

/// Mode status constants according to protocol documentation
pub mod mode_status {
    pub const RESET: u8 = 0;
    pub const CALIBRATION: u8 = 1; 
    pub const RUN: u8 = 2;
}

/// Fault flag constants according to protocol documentation (bits 21-16)
pub mod protocol_fault_flags {
    pub const UNCALIBRATED: u8 = 0x20;      // bit21
    pub const GRIDLOCK_OVERLOAD: u8 = 0x10; // bit20  
    pub const MAGNETIC_ENCODING: u8 = 0x08;  // bit19
    pub const OVERTEMPERATURE: u8 = 0x04;    // bit18
    pub const OVERCURRENT: u8 = 0x02;        // bit17
    pub const UNDERVOLTAGE: u8 = 0x01;       // bit16
}

// should basically be part of robsstride crate, but for now we keep it here
#[derive(Debug)]
enum ActuatorClientState {
    Reset,
    AwaitingIdRequest,
    AwaitingIdResponse,

    AwaitingMotorEnableRequest,
    AwaitingMotorEnableResponse,

    Ready,
    AwaitingDataRequest,
    AwaitingDataResponse,

    AwaitingFeedbackRequest,
    AwaitingFeedbackResponse,

    AwaitingReadParamRequest,
    AwaitingReadParamResponse,
}

// should basically be part of robsstride crate, but for now we keep it here
#[derive(Debug)]
pub struct ActuatorCanClient {
    host_id: u16, // Host ID for the actuator
    pub actuator_can_id: u8,
    last_request: Option<ActuatorRequest>, // (expected response mux, request)
    state: ActuatorClientState,
    actuator_ranges: RangeSet<f64>,
    can_range: RangeSet<f64>,
}

impl ActuatorCanClient {
    pub fn new(actuator_id: ActuatorId) -> Self {
        let actuator_can_id = Self::actuator_id_to_can_id(actuator_id);
        ActuatorCanClient {
            host_id: 0xFD,
            actuator_can_id,
            state: ActuatorClientState::Reset,
            last_request: None,
            actuator_ranges: RobstrideActuatorType::from(actuator_can_id).actuator_ranges(),
            can_range: RobstrideActuatorType::from(actuator_can_id).can_ranges(),
        }
    }

    pub fn reset(&mut self) {
        self.state = ActuatorClientState::Reset;
        self.last_request = None;
    }

    fn build_request(&self, params: &ActuatorRequestParams) -> ActuatorRequest {
        match params {
            ActuatorRequestParams::ObtainId => {
                ActuatorRequest::ObtainId(ObtainIdRequest::new(self.host_id, self.actuator_can_id))
            }
            ActuatorRequestParams::ReadParam(param) => ActuatorRequest::ReadParam(
                ReadParamRequest::new(self.host_id, self.actuator_can_id, *param as u16),
            ),
            ActuatorRequestParams::MotorEnable => ActuatorRequest::MotorEnable(
                MotorEnableRequest::new(self.host_id, self.actuator_can_id),
            ),
            ActuatorRequestParams::Feedback => {
                ActuatorRequest::Feedback(FeedbackRequest::new(self.host_id, self.actuator_can_id))
            }
            ActuatorRequestParams::Control(cmd) => {
                ActuatorRequest::Control(ControlCommandRequest::new(
                    self.actuator_can_id,
                    self.actuator_ranges
                        .torque
                        .scale_value(cmd.qfrc, &self.can_range.torque) as u16,
                    self.actuator_ranges
                        .angle
                        .scale_value(cmd.qpos, &self.can_range.angle) as u16,
                    self.actuator_ranges
                        .velocity
                        .scale_value(cmd.qvel, &self.can_range.velocity) as u16,
                    self.actuator_ranges
                        .kd
                        .scale_value(cmd.kd, &self.can_range.kd) as u16,
                    self.actuator_ranges
                        .kp
                        .scale_value(cmd.kp, &self.can_range.kp) as u16,
                ))
            }
        }
    }

    pub fn stage_request(&mut self, params: &ActuatorRequestParams) -> CanFrame {
        // Stage the request based on the provided parameters
        self.state = match params {
            ActuatorRequestParams::ObtainId => ActuatorClientState::AwaitingIdRequest,
            ActuatorRequestParams::ReadParam(_) => ActuatorClientState::AwaitingReadParamRequest,
            ActuatorRequestParams::MotorEnable => ActuatorClientState::AwaitingMotorEnableRequest,
            ActuatorRequestParams::Control(_) => ActuatorClientState::AwaitingDataRequest,
            ActuatorRequestParams::Feedback => ActuatorClientState::AwaitingFeedbackRequest,
        };
        let request = self.build_request(params);
        let expected_response_mux = request.response_mux();
        debug!("Staging request for actuator {}: {:?}, expecting response mux 0x{:02X}", 
               self.actuator_can_id, params, expected_response_mux);
        request.into()
    }

    pub fn set_last_request(&mut self, transaction: CanFrame) {
        // map can frame back to request, update state and store it
        let req = transaction.into();

        self.state = match req {
            ActuatorRequest::ObtainId(_) => ActuatorClientState::AwaitingIdRequest,
            ActuatorRequest::Control(_) => ActuatorClientState::AwaitingDataRequest,
            ActuatorRequest::ReadParam(_) => ActuatorClientState::AwaitingReadParamRequest,
            ActuatorRequest::MotorEnable(_) => ActuatorClientState::AwaitingMotorEnableRequest,
            ActuatorRequest::Feedback(_) => ActuatorClientState::AwaitingFeedbackRequest,
        };

        self.last_request = Some(req);
    }

    pub fn handle_response(
        &mut self,
        response: &CanFrame,
    ) -> std::io::Result<Option<ActuatorFeedbackUpdate>> {
        let response_mux = mux_from_can_frame(response);
        
        // Handle unsolicited responses first (no transaction validation needed)
        if response_mux == 0x15 { // Fault response - unsolicited
            match (*response).into() {
                ActuatorResponse::Fault(resp) => {
                    // Copy packed struct fields to local variables to avoid alignment issues
                    let actuator_id = resp.actuator_can_id;
                    let fault_value = resp.fault_value;
                    let warning_value = resp.warning_value;
                    
                    // Fault frames don't affect the client state - they're just informational
                    return Ok(None);
                }
                _ => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Expected fault response for mux 0x15",
                    ));
                }
            }
        }

        // Treat feedback (0x02) as unsolicited unless we’re currently expecting it.
        if response_mux == 0x02 {
            let expecting_feedback = self
                .last_request
                .as_ref()
                .map(|r| r.response_mux())
                == Some(0x02);

            if !expecting_feedback {
                match (*response).into() {
                    ActuatorResponse::Feedback(resp) => {
                        // Ignore frames for other actuators
                        if resp.actuator_can_id != self.actuator_can_id {
                            return Ok(None);
                        }
                        // Do NOT change state/last_request; just surface the update
                        return Ok(Some(self.update_from_feedback(&resp, response)));
                    }
                    _ => {
                        return Ok(None);
                    }
                }
            }
        }
        
        // For solicited responses, validate transaction state
        if self.last_request.is_none() {
            return Err(std::io::Error::other(
                "No current transaction to handle response for",
            ));
        }
    
        // Check if the response matches the current transaction
        if let Some(ref cur_req) = self.last_request {
            if response_mux != cur_req.response_mux() {
                warn!("Transaction mismatch: received response mux 0x{:02X} but expected 0x{:02X} for actuator {}", 
                      response_mux, cur_req.response_mux(), self.actuator_can_id);
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!(
                        "Response ID {} does not match current transaction {}",
                        response_mux,
                        cur_req.response_mux()
                    ),
                ));
            }
        } else {
            return Err(std::io::Error::other(
                "No current transaction to handle response for",
            ));
        }
    
        // convert can frame into ActuatorResponse for solicited responses
        match (*response).into() {
            ActuatorResponse::ObtainId(resp) => {
                debug!("Received ObtainId response: {:?}", resp);
                if resp.actuator_can_id as u8 != self.actuator_can_id {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Response does not match expected host ID or actuator CAN ID",
                    ));
                }
                self.state = ActuatorClientState::Ready;
                Ok(None)
            }
            ActuatorResponse::Feedback(resp) => {
                debug!("Received Feedback response: {:?}", resp);
                if resp.actuator_can_id != self.actuator_can_id {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Feedback response does not match expected actuator CAN ID",
                    ));
                }
                self.state = ActuatorClientState::Ready;
                self.last_request = None; // Clear the last request to avoid re-using it
                Ok(Some(self.update_from_feedback(&resp, response)))
            }
            ActuatorResponse::ReadParam(resp) => {
                debug!("Received Feedback response: {:?}", resp);
                if resp.actuator_can_id != self.actuator_can_id {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Feedback response does not match expected actuator CAN ID",
                    ));
                }
                self.state = ActuatorClientState::Ready;
                self.last_request = None; // Clear the last request to avoid re-using it
                let param = RobstrideActuatorParam::from_address(resp.index).unwrap();
                let value = f32::from_bits(resp.value);
                match param {
                    RobstrideActuatorParam::Iqf => Ok(Some(self.update_from_current(&value))),
                    _ => Ok(None),
                }
            }
            ActuatorResponse::Fault(_) => {
                // This should not happen since fault responses are handled above
                Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Fault response should have been handled as unsolicited",
                ))
            }
        }
    }

    pub fn update_from_feedback(&self, resp: &FeedbackResponse, can_frame: &CanFrame) -> ActuatorFeedbackUpdate {
        // Extract fault flags from CAN ID according to protocol documentation (bits 21-16)
        let fault_flags = fault_flags_from_can_id(can_frame.can_id);
        let mode_status = mode_status_from_can_id(can_frame.can_id);
        
        // Log mode status for debugging
        debug!("Actuator {} mode status: {}", self.actuator_can_id, mode_status);
        
        // Convert protocol fault flags (from CAN packet) to standardized fault format
        let can_faults = self.convert_protocol_faults_to_standard(fault_flags);
        
        ActuatorFeedbackUpdate {
            qpos: Some(self.can_range.angle.scale_value(
                resp.angle_scale_be.swap_bytes() as f64,
                &self.actuator_ranges.angle,
            )),
            qvel: Some(self.can_range.velocity.scale_value(
                resp.angular_vel_scale_be.swap_bytes() as f64,
                &self.actuator_ranges.velocity,
            )),
            qfrc: Some(self.can_range.torque.scale_value(
                resp.torque_be.swap_bytes() as f64,
                &self.actuator_ranges.torque,
            )),
            kp: None,
            kd: None,
            temp: Some(resp.get_temperature()),
            faults: Some(can_faults),
            amps: None,
        }
    }

    fn convert_protocol_faults_to_standard(&self, protocol_faults: u8) -> u32 {
        let mut standard_faults = 0u32;
        
        if protocol_faults & protocol_fault_flags::UNCALIBRATED != 0 {
            standard_faults |= CanResponseFault::Uncalibrated.bit_value();
        }
        if protocol_faults & protocol_fault_flags::GRIDLOCK_OVERLOAD != 0 {
            standard_faults |= CanResponseFault::GridlockOverload.bit_value();
        }
        if protocol_faults & protocol_fault_flags::MAGNETIC_ENCODING != 0 {
            standard_faults |= CanResponseFault::MagneticEncoding.bit_value();
        }
        if protocol_faults & protocol_fault_flags::OVERTEMPERATURE != 0 {
            standard_faults |= CanResponseFault::Overtemperature.bit_value();
        }
        if protocol_faults & protocol_fault_flags::OVERCURRENT != 0 {
            standard_faults |= CanResponseFault::Overcurrent.bit_value();
        }
        if protocol_faults & protocol_fault_flags::UNDERVOLTAGE != 0 {
            standard_faults |= CanResponseFault::Undervoltage.bit_value();
        }
        
        standard_faults
    }

    pub fn update_from_current(&self, amps: &f32) -> ActuatorFeedbackUpdate {
        ActuatorFeedbackUpdate {
            qpos: None,
            qvel: None,
            qfrc: None,
            kp: None,
            kd: None,
            temp: None,
            faults: None,
            amps: Some(*amps as f64),
        }
    }

    fn actuator_id_to_can_id(actuator_id: ActuatorId) -> u8 {
        match actuator_id {
            ActuatorId::Lsp => 11,
            ActuatorId::Lsr => 12,
            ActuatorId::Lsy => 13,
            ActuatorId::Lep => 14,
            ActuatorId::Lwr => 15,
            ActuatorId::Rsp => 21,
            ActuatorId::Rsr => 22,
            ActuatorId::Rsy => 23,
            ActuatorId::Rep => 24,
            ActuatorId::Rwr => 25,
            ActuatorId::Lhp => 31,
            ActuatorId::Lhr => 32,
            ActuatorId::Lhy => 33,
            ActuatorId::Lkp => 34,
            ActuatorId::Lap => 35,
            ActuatorId::Rhp => 41,
            ActuatorId::Rhr => 42,
            ActuatorId::Rhy => 43,
            ActuatorId::Rkp => 44,
            ActuatorId::Rap => 45,
        }
    }

    pub fn handle_fault_param_response(&mut self, resp: &ReadParamResponse) -> std::io::Result<Option<FaultDecoder>> {
        let param = RobstrideActuatorParam::from_address(resp.index);
        if let Some(param) = param {
            match param {
                RobstrideActuatorParam::MotorFault => {
                    let motor_fault = MotorFault::from_bitmask(resp.value);
                    let mut decoder = FaultDecoder::new();
                    decoder.motor_fault = motor_fault;
                    decoder.log_all_faults(self.actuator_can_id as usize);
                    Ok(Some(decoder))
                }
                RobstrideActuatorParam::DrvFault1 => {
                    let drv_fault1 = DrvFault1::from_bitmask(resp.value as u16);
                    let mut decoder = FaultDecoder::new();
                    decoder.drv_fault1 = drv_fault1;
                    decoder.log_all_faults(self.actuator_can_id as usize);
                    Ok(Some(decoder))
                }
                RobstrideActuatorParam::DrvFault2 => {
                    let drv_fault2 = DrvFault2::from_bitmask(resp.value as u16);
                    let mut decoder = FaultDecoder::new();
                    decoder.drv_fault2 = drv_fault2;
                    decoder.log_all_faults(self.actuator_can_id as usize);
                    Ok(Some(decoder))
                }
                _ => Ok(None),
            }
        } else {
            Ok(None)
        }
    }
}

impl From<u8> for RobstrideActuatorType {
    fn from(id: u8) -> RobstrideActuatorType {
        match id {
            // Left arm
            11 => RobstrideActuatorType::Robstride03, // left_shoulder_pitch_03
            12 => RobstrideActuatorType::Robstride03, // left_shoulder_roll_03
            13 => RobstrideActuatorType::Robstride02, // left_shoulder_yaw_02
            14 => RobstrideActuatorType::Robstride02, // left_elbow_02
            15 => RobstrideActuatorType::Robstride00, // left_wrist_00

            // Right arm
            21 => RobstrideActuatorType::Robstride03, // right_shoulder_pitch_03
            22 => RobstrideActuatorType::Robstride03, // right_shoulder_roll_03
            23 => RobstrideActuatorType::Robstride02, // right_shoulder_yaw_02
            24 => RobstrideActuatorType::Robstride02, // right_elbow_02
            25 => RobstrideActuatorType::Robstride00, // right_wrist_00

            // Left leg
            31 => RobstrideActuatorType::Robstride04, // left_hip_pitch_04
            32 => RobstrideActuatorType::Robstride03, // left_hip_roll_03
            33 => RobstrideActuatorType::Robstride03, // left_hip_yaw_03
            34 => RobstrideActuatorType::Robstride04, // left_knee_04
            35 => RobstrideActuatorType::Robstride02, // left_ankle_02

            // Right leg
            41 => RobstrideActuatorType::Robstride04, // right_hip_pitch_04
            42 => RobstrideActuatorType::Robstride03, // right_hip_roll_03
            43 => RobstrideActuatorType::Robstride03, // right_hip_yaw_03
            44 => RobstrideActuatorType::Robstride04, // right_knee_04
            45 => RobstrideActuatorType::Robstride02, // right_ankle_02

            _ => panic!("Invalid Robstride actuator ID: {id}"),
        }
    }
}