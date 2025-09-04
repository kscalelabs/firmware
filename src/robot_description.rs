use crossterm::event::KeyEvent;
use enum_map::{Enum, EnumMap, enum_map};
use heapless::Deque;
use nalgebra as na;
use tracing::info;
use crate::actuator::{CanResponseFault, MotorFault, SystemFault};

pub fn normalize_actuator_qpos(mut qpos: f64) -> f64 {
    const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
    // rem_euclid gives a value in [0, 2π)
    qpos = qpos.rem_euclid(TWO_PI);
    // shift to (−π, π]
    if qpos > std::f64::consts::PI {
        qpos -= TWO_PI;
    }
    qpos
}

#[derive(Debug, Default)]
pub struct ActuatorFeedback {
    pub qpos: f64,   // Position
    pub qvel: f64,   // Velocity
    pub qfrc: f64,   // Force
    pub kp: f64,     // Position gain
    pub kd: f64,     // Velocity gain
    pub temp: f64,   // Temperature
    pub faults: u32, // Faults
    pub amps: f64,   // Current in Amperes
}

impl ActuatorFeedback {
    pub fn merge(&mut self, update: ActuatorFeedbackUpdate) {
        if let Some(qpos) = update.qpos {
            self.qpos = qpos;
        }
        if let Some(qvel) = update.qvel {
            self.qvel = qvel;
        }
        if let Some(qfrc) = update.qfrc {
            self.qfrc = qfrc;
        }
        if let Some(kp) = update.kp {
            self.kp = kp;
        }
        if let Some(kd) = update.kd {
            self.kd = kd;
        }
        if let Some(temp) = update.temp {
            self.temp = temp;
        }
        if let Some(faults) = update.faults {
            self.faults = faults;
        }
        if let Some(amps) = update.amps {
            self.amps = amps;
        }
    }

    pub fn has_faults(&self) -> bool {
        self.faults != 0
    }

    pub fn has_fault(&self, fault_flag: u32) -> bool {
        (self.faults & fault_flag) != 0
    }


    pub fn fault_description(&self) -> String {
        if !self.has_faults() {
            return "No faults".to_string();
        }
        
        let mut descriptions = Vec::new();
        
        descriptions.extend(
            CanResponseFault::from_bitmask(self.faults)
                .iter()
                .map(|f| f.to_string())
        );
        
        descriptions.extend(
            MotorFault::from_bitmask(self.faults)
                .iter()
                .map(|f| f.to_string())
        );
        
        // Check system fault
        if self.faults & SystemFault::CommunicationError.bit_value() != 0 {
            descriptions.push(SystemFault::CommunicationError.to_string());
        }
        
        descriptions.join(", ")
    }
    
    /// Check if the actuator is safe to operate (no critical faults)
    pub fn is_safe_to_operate(&self) -> bool {
        !MotorFault::has_critical_faults_in_mask(self.faults) 
            && !CanResponseFault::has_critical_faults_in_mask(self.faults)
    }
    
    /// Check if temperature is within safe operating range
    pub fn is_temperature_safe(&self) -> bool {
        const MAX_SAFE_TEMP: f64 = 80.0; // Celsius
        const MIN_SAFE_TEMP: f64 = -20.0; // Celsius
        
        self.temp >= MIN_SAFE_TEMP && self.temp <= MAX_SAFE_TEMP
    }
}


pub struct ActuatorFeedbackUpdate {
    pub qpos: Option<f64>,
    pub qvel: Option<f64>,
    pub qfrc: Option<f64>,
    pub kp: Option<f64>,
    pub kd: Option<f64>,
    pub temp: Option<f64>,
    pub faults: Option<u32>,
    pub amps: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActuatorCommand {
    pub qpos: f64,
    pub qvel: f64,
    pub qfrc: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Debug, Default)]
pub struct ActuatorState {
    pub feedback: ActuatorFeedback,
    pub command: ActuatorCommand,
}

impl ActuatorState {
    pub fn merge_feedback(&mut self, fdbk: ActuatorFeedbackUpdate) {
        self.feedback.merge(fdbk);
    }

    pub fn set_command(&mut self, command: ActuatorCommand) {
        self.command = command;
    }
}

#[repr(usize)]
#[derive(Enum, Debug, Clone, Copy)]
pub enum BusTag {
    LeftArm = 0,
    RightArm = 1,
    LeftLeg = 2,
    RightLeg = 3,
}

impl From<usize> for BusTag {
    fn from(value: usize) -> Self {
        match value {
            0 => BusTag::LeftArm,
            1 => BusTag::RightArm,
            2 => BusTag::LeftLeg,
            3 => BusTag::RightLeg,
            _ => panic!("Invalid BusTag value"),
        }
    }
}

impl BusTag {
    pub fn id_vec(&self) -> Vec<ActuatorId> {
        match self {
            BusTag::LeftArm => vec![
                ActuatorId::Lsp,
                ActuatorId::Lsr,
                ActuatorId::Lsy,
                ActuatorId::Lep,
                ActuatorId::Lwr,
            ],
            BusTag::RightArm => vec![
                ActuatorId::Rsp,
                ActuatorId::Rsr,
                ActuatorId::Rsy,
                ActuatorId::Rep,
                ActuatorId::Rwr,
            ],
            BusTag::LeftLeg => vec![
                ActuatorId::Lhp,
                ActuatorId::Lhr,
                ActuatorId::Lhy,
                ActuatorId::Lkp,
                ActuatorId::Lap,
            ],
            BusTag::RightLeg => vec![
                ActuatorId::Rhp,
                ActuatorId::Rhr,
                ActuatorId::Rhy,
                ActuatorId::Rkp,
                ActuatorId::Rap,
            ],
        }
    }
}

#[repr(usize)]
#[derive(Enum, Debug, Clone, Copy, PartialEq)]
pub enum ActuatorId {
    Lsp,
    Lsr,
    Lsy,
    Lep,
    Lwr,

    Rsp,
    Rsr,
    Rsy,
    Rep,
    Rwr,

    Lhp,
    Lhr,
    Lhy,
    Lkp,
    Lap,

    Rhp,
    Rhr,
    Rhy,
    Rkp,
    Rap,
}

impl ActuatorId {
    pub fn flattened_idx(&self) -> usize {
        *self as usize
    }
}

pub struct ActuatorStateStore {
    pub actuator_states: EnumMap<ActuatorId, ActuatorState>, // 20 actuators
}

impl Default for ActuatorStateStore {
    fn default() -> Self {
        Self::new()
    }
}

impl ActuatorStateStore {
    pub fn new() -> Self {
        Self {
            actuator_states: enum_map::EnumMap::from_fn(|_| ActuatorState {
                feedback: ActuatorFeedback::default(),
                command: ActuatorCommand::default(),
            }),
        }
    }
    pub fn slice_mut(&mut self, bus_tag: BusTag) -> &mut [ActuatorState] {
        let ids = bus_tag.id_vec();
        let start = ids[0].flattened_idx();
        let end = start + ids.len();
        &mut self.actuator_states.as_mut_slice()[start..end]
    }

    pub fn slice(&self, bus_tag: BusTag) -> &[ActuatorState] {
        let ids = bus_tag.id_vec();
        let start = ids[0].flattened_idx();
        let end = start + ids.len();
        &self.actuator_states.as_slice()[start..end]
    }

    pub fn slice_all(&self) -> &[ActuatorState] {
        self.actuator_states.as_slice()
    }

    pub fn len(&self) -> usize {
        self.actuator_states.len()
    }

    pub fn is_empty(&self) -> bool {
        self.actuator_states.len() == 0
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Acceleration including gravity (m/s²)
    pub accelerometer: na::Vector3<f64>,
    /// Angular velocity (deg/s)
    pub gyroscope: na::Vector3<f64>,
    /// Magnetic field vector (micro Tesla, µT)
    pub magnetometer: na::Vector3<f64>,
    /// Orientation as a unit quaternion (WXYZ order)
    pub quaternion: na::Quaternion<f64>,
    /// Orientation as Euler angles (deg)
    pub euler: na::Vector3<f64>,
    /// Linear acceleration (acceleration without gravity) (m/s²)
    pub linear_acceleration: na::Vector3<f64>,
    /// Estimated gravity vector (m/s²)
    pub gravity: na::Vector3<f64>,
    /// Temperature (°C)
    pub temperature: f32,
    /// Calibration status
    pub calibration_status: u8, // TODO change to enum
}

impl ImuData {
    pub fn merge_feedback(&mut self, feedback: ImuFeedback) {
        if let Some(ref accel) = feedback.accelerometer {
            self.accelerometer.as_mut_slice().copy_from_slice(accel);
        }
        if let Some(ref gyro) = feedback.gyroscope {
            self.gyroscope.as_mut_slice().copy_from_slice(gyro);
        }
        if let Some(ref mag) = feedback.magnetometer {
            self.magnetometer.as_mut_slice().copy_from_slice(mag);
        }
        if let Some(ref quat) = feedback.quaternion {
            self.quaternion.coords.as_mut_slice().copy_from_slice(quat);
        }
        if let Some(ref euler) = feedback.euler {
            self.euler.as_mut_slice().copy_from_slice(euler);
        }
        if let Some(ref lin_acc) = feedback.linear_acceleration {
            self.linear_acceleration
                .as_mut_slice()
                .copy_from_slice(lin_acc);
        }
        if let Some(ref grav) = feedback.gravity {
            self.gravity.as_mut_slice().copy_from_slice(grav);
        }
        if let Some(temp) = feedback.temperature {
            self.temperature = temp;
        }
        if let Some(calib_status) = feedback.calibration_status {
            self.calibration_status = calib_status;
        }
    }
}

#[derive(Debug, Default)]
pub struct ImuFeedback {
    pub accelerometer: Option<[f64; 3]>,
    pub gyroscope: Option<[f64; 3]>,
    pub magnetometer: Option<[f64; 3]>,
    pub quaternion: Option<[f64; 4]>,
    pub euler: Option<[f64; 3]>,
    pub linear_acceleration: Option<[f64; 3]>,
    pub gravity: Option<[f64; 3]>,
    pub temperature: Option<f32>,
    pub calibration_status: Option<u8>,
}

#[repr(usize)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataType {
    JointAngles,
    JointAngularVelocities,
    InitialHeading,
    Quaternion,
    ProjectedGravity,
    Accelerometer,
    Gyroscope,
    Time,
}

use clap::Parser;
#[derive(Debug, Parser)]
#[command(name = "faux-rtos", about = "Parse three floats")]
pub struct Args {
    /// scale factor for the policy
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    policy_scale: f64,

    /// proportional gain scale
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    kp_scale: f64,

    /// derivative gain scale
    #[arg(long, value_name = "FLOAT", default_value_t = 1.0)]
    kd_scale: f64,

    /// low-pass filter cutoff in Hz for policy outputs (0 disables filtering)
    #[arg(long, value_name = "FLOAT", default_value_t = 6.0)]
    lpf_cutoff_hz: f64,

    /// minimum-jerk blend duration in milliseconds (0 disables re-timing)
    #[arg(long, value_name = "FLOAT", default_value_t = 0.0)]
    min_jerk_blend_ms: f64,
}

pub struct RobotDescription {
    pub actuators: ActuatorStateStore,
    pub imu: ImuData,
    pub initial_imu: ImuData,
    pub kb_pending_events: Deque<KeyEvent, 16>,
    pub udp_command_state: crate::udp_command::UdpExtendedCommand,
    pub home_position: EnumMap<ActuatorId, ActuatorCommand>,
    pub policy_position: EnumMap<ActuatorId, ActuatorCommand>,
    pub policy_scale: f64,
    pub kp_scale: f64,
    pub kd_scale: f64,
    pub lpf_cutoff_hz: f64,
    pub min_jerk_blend_ms: f64,
}

impl Default for RobotDescription {
    fn default() -> Self {
        Self::new()
    }
}

impl RobotDescription {
    pub fn new() -> Self {
        let args = Args::parse();
        info!("Args; {:?}", args);
        Self {
            actuators: ActuatorStateStore::new(),
            imu: ImuData::default(),
            initial_imu: ImuData::default(),
            kb_pending_events: Deque::new(),
            udp_command_state: crate::udp_command::UdpExtendedCommand::default(),
            kp_scale: args.kp_scale,
            kd_scale: args.kd_scale,
            policy_scale: args.policy_scale,
            lpf_cutoff_hz: args.lpf_cutoff_hz,
            min_jerk_blend_ms: args.min_jerk_blend_ms,
            home_position: enum_map! {
                ActuatorId::Lsp => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
                ActuatorId::Lsr => ActuatorCommand { qpos: (10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
                ActuatorId::Lsy => ActuatorCommand { qpos: 0.0, kp: 40.0, kd: 0.945, ..Default::default() },
                ActuatorId::Lep => ActuatorCommand { qpos: (-90.0_f64).to_radians(), kp: 40.0, kd: 1.266, ..Default::default() },
                ActuatorId::Lwr => ActuatorCommand { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

                ActuatorId::Rsp => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
                ActuatorId::Rsr => ActuatorCommand { qpos: (-10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
                ActuatorId::Rsy => ActuatorCommand { qpos: 0.0, kp: 40.0, kd: 0.945, ..Default::default() },
                ActuatorId::Rep => ActuatorCommand { qpos: (90.0_f64).to_radians(), kp: 40.0, kd: 1.266, ..Default::default() },
                ActuatorId::Rwr => ActuatorCommand { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

                ActuatorId::Lhp => ActuatorCommand { qpos: (20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
                ActuatorId::Lhr => ActuatorCommand { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
                ActuatorId::Lhy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
                ActuatorId::Lkp => ActuatorCommand { qpos: (50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
                ActuatorId::Lap => ActuatorCommand { qpos: (-30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },

                ActuatorId::Rhp => ActuatorCommand { qpos: (-20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
                ActuatorId::Rhr => ActuatorCommand { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
                ActuatorId::Rhy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
                ActuatorId::Rkp => ActuatorCommand { qpos: (-50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
                ActuatorId::Rap => ActuatorCommand { qpos: (30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },
            },

            policy_position: enum_map! {
                ActuatorId::Lsp => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
                ActuatorId::Lsr => ActuatorCommand { qpos: (10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
                ActuatorId::Lsy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 2.945, ..Default::default() },
                ActuatorId::Lep => ActuatorCommand { qpos: (-90.0_f64).to_radians(), kp: 80.0, kd: 2.266, ..Default::default() },
                ActuatorId::Lwr => ActuatorCommand { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

                ActuatorId::Rsp => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
                ActuatorId::Rsr => ActuatorCommand { qpos: (-10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
                ActuatorId::Rsy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 2.945, ..Default::default() },
                ActuatorId::Rep => ActuatorCommand { qpos: (90.0_f64).to_radians(), kp: 100.0, kd: 2.266, ..Default::default() },
                ActuatorId::Rwr => ActuatorCommand { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

                ActuatorId::Lhp => ActuatorCommand { qpos: (20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
                ActuatorId::Lhr => ActuatorCommand { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
                ActuatorId::Lhy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
                ActuatorId::Lkp => ActuatorCommand { qpos: (50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
                ActuatorId::Lap => ActuatorCommand { qpos: (-30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },

                ActuatorId::Rhp => ActuatorCommand { qpos: (-20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
                ActuatorId::Rhr => ActuatorCommand { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
                ActuatorId::Rhy => ActuatorCommand { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
                ActuatorId::Rkp => ActuatorCommand { qpos: (-50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
                ActuatorId::Rap => ActuatorCommand { qpos: (30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },
            },
        }
    }
    pub fn actuator_states_mut(&mut self) -> &mut ActuatorStateStore {
        &mut self.actuators
    }

    pub fn dimensions(&self, dtype: DataType) -> Vec<usize> {
        let ret = match dtype {
            // Actuators
            DataType::JointAngles | DataType::JointAngularVelocities => self.actuators.len(),

            // Imu
            DataType::Quaternion => self.imu.quaternion.coords.len(),
            DataType::ProjectedGravity => self.imu.gravity.len(),
            DataType::Accelerometer => self.imu.accelerometer.len(),
            DataType::Gyroscope => self.imu.gyroscope.len(),

            // constants
            DataType::InitialHeading => 1,
            DataType::Time => 1,
        };
        vec![ret]
    }
}
