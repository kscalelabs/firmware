use crate::socketcan::CAN_MAX_DLEN;

use crate::socketcan::CanFrame;

use crate::robot_description::{
    ActuatorFeedbackUpdate,
    ActuatorCommand,
    ActuatorId,
};

use crate::robstride_utils::*;
use crate::{warn, debug};

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

impl From<ActuatorRequest> for crate::socketcan::CanFrame 
{
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

impl Into<ActuatorResponse> for crate::socketcan::CanFrame 
{
    fn into(mut self) -> ActuatorResponse {
        self.can_id ^= 0x8000_0000; // remove EFF FLAG
        let mux = mux_from_can_frame(&self);
        match mux {
            0x00 => ActuatorResponse::ObtainId(bytemuck::must_cast::<Self, ObtainIdResponse>(self)),
            0x02 => ActuatorResponse::Feedback(bytemuck::must_cast::<Self, FeedbackResponse>(self)),
            _ => panic!("Unknown mux value: {}", mux),
        }
    }
}

impl Into<ActuatorRequest> for crate::socketcan::CanFrame 
{
    fn into(mut self) -> ActuatorRequest {
        self.can_id &= !0x80; // clear EFF FLAG
        let mux = mux_from_can_frame(&self);
        // TODO: change mux values from u8 to an enum
        match mux {
            0x00 => ActuatorRequest::ObtainId(bytemuck::must_cast::<Self, ObtainIdRequest>(self)),
            0x01 => ActuatorRequest::Control(bytemuck::must_cast::<Self, ControlCommandRequest>(self)),
            0x11 => ActuatorRequest::ReadParam(bytemuck::must_cast::<Self, ReadParamRequest>(self)),
            0x03 => ActuatorRequest::MotorEnable(bytemuck::must_cast::<Self, MotorEnableRequest>(self)),
            0x02 => ActuatorRequest::Feedback(bytemuck::must_cast::<Self, FeedbackRequest>(self)),
            _ => panic!("Unknown mux value: {}", mux),
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
impl RobstrideActuatorFrame for MotorEnableRequest {}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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
            .. Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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
            .. Default::default()
        }
    }
}


#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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
            .. Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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
            .. Default::default()
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
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
    pub fn new(
        host_id: u16,
        actuator_can_id: u8,
        index: u16,
    ) -> Self {
        Self {
            mux: 0x11,
            index: index.to_le(),
            actuator_can_id,
            host_id: host_id as u16,
            len: 8,
            .. Default::default()
        }
    }
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
    ReadParam,
    MotorEnable,
    Control(ActuatorCommand),
    Feedback,
}

impl ActuatorRequest {
    
    pub fn response_mux(&self) -> u8 {
        match self {
            Self::ObtainId(_) => 0x0, // Obtain Id mux
            Self::Control(_) => 0x2, // feedback mux
            Self::ReadParam(_) => 0x11, // read param mux
            Self::MotorEnable(_) => 0x02, // feedback mux
            Self::Feedback(_) => 0x02, // feedback mux
            
        }
    }
}

#[derive(Debug, Clone)]
pub enum ActuatorResponse {
    ObtainId(ObtainIdResponse),
    Feedback(FeedbackResponse),
}

#[allow(dead_code)]
impl ActuatorResponse {
    pub fn mux(&self) -> u8 {
        match self {
            ActuatorResponse::ObtainId(req) => req.mux,
            ActuatorResponse::Feedback(req) => req.mux,
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
        0x00 => bytemuck::must_cast::<crate::socketcan::CanFrame, ObtainIdResponse>(*frame).actuator_can_id as u8,
        0x02 => bytemuck::must_cast::<crate::socketcan::CanFrame, FeedbackResponse>(*frame).actuator_can_id as u8,
        _ => {
            warn!("Unknown mux value: {} in actuator_can_id_from_response, returning  0x7F", mux);
            0x7F // Return a default value if the mux is unknown
        }
    }
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
            ActuatorRequestParams::ObtainId => ActuatorRequest::ObtainId(ObtainIdRequest::new(self.host_id, self.actuator_can_id)),
            ActuatorRequestParams::ReadParam => ActuatorRequest::ReadParam(ReadParamRequest::new(self.host_id, self.actuator_can_id, 0x7005)),
            ActuatorRequestParams::MotorEnable => ActuatorRequest::MotorEnable(MotorEnableRequest::new(self.host_id, self.actuator_can_id)),
            ActuatorRequestParams::Feedback => ActuatorRequest::Feedback(FeedbackRequest::new(self.host_id, self.actuator_can_id)),
            ActuatorRequestParams::Control(cmd) => ActuatorRequest::Control(ControlCommandRequest::new(
                self.actuator_can_id,
                self.actuator_ranges.torque.scale_value(cmd.qfrc, &self.can_range.torque) as u16,
                self.actuator_ranges.angle.scale_value(cmd.qpos, &self.can_range.angle) as u16,
                self.actuator_ranges.velocity.scale_value(cmd.qvel, &self.can_range.velocity) as u16,
                self.actuator_ranges.kd.scale_value(cmd.kd, &self.can_range.kd) as u16,
                self.actuator_ranges.kp.scale_value(cmd.kp, &self.can_range.kp) as u16,
            )),
        }
    }

    pub fn stage_request(&mut self, params: &ActuatorRequestParams) -> CanFrame {
        // Stage the request based on the provided parameters
        self.state = match params {
            ActuatorRequestParams::ObtainId => ActuatorClientState::AwaitingIdRequest,
            ActuatorRequestParams::ReadParam => ActuatorClientState::AwaitingReadParamRequest,
            ActuatorRequestParams::MotorEnable => ActuatorClientState::AwaitingMotorEnableRequest,
            ActuatorRequestParams::Control(_) => ActuatorClientState::AwaitingDataRequest,
            ActuatorRequestParams::Feedback => ActuatorClientState::AwaitingFeedbackRequest,
        };
        self.build_request(params).into()
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


    pub fn handle_response(&mut self, response: &CanFrame) -> std::io::Result<Option<ActuatorFeedbackUpdate>> {

        if self.last_request.is_none() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "No current transaction to handle response for",
            ));
        }

        // Check if the response matches the current transaction
        if let Some(ref cur_req) = self.last_request {
            if mux_from_can_frame(&response) != cur_req.response_mux() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("Response ID {} does not match current transaction {}",
                        mux_from_can_frame(&response),
                        cur_req.response_mux()),
                ));
            }
        } else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "No current transaction to handle response for",
            ));
        }

        // convert can frame into ActuatorResponse
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
                Ok(Some(self.update_from_feedback(&resp)))
            }
        }
    }

    pub fn update_from_feedback(&self, resp: &FeedbackResponse) -> ActuatorFeedbackUpdate {
        ActuatorFeedbackUpdate {
            qpos: Some(self.can_range.angle.scale_value(resp.angle_scale_be.swap_bytes() as f64, &self.actuator_ranges.angle)),
            qvel: Some(self.can_range.velocity.scale_value(resp.angular_vel_scale_be.swap_bytes() as f64, &self.actuator_ranges.velocity)),
            qfrc: Some(self.can_range.torque.scale_value(resp.torque_be.swap_bytes() as f64, &self.actuator_ranges.torque)),
            kp: None,
            kd: None,
            temp: None,
            faults: None,
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

            _ => panic!("Invalid Robstride actuator ID: {}", id),
        }
    }
}
