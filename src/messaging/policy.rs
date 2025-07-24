use crate::inference::ModelInputType;
use crate::robot_description::{ActuatorId, DataType, RobotDescription};
use enum_map::EnumMap;
use ort;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Default)]
pub struct PolicyStepDescriptorPod {
    pub step_id: Option<u64>,
    pub t_us: Option<u64>,
    pub joint_angles: Option<EnumMap<ActuatorId, f32>>,
    pub joint_vels: Option<EnumMap<ActuatorId, f32>>,
    pub initial_heading: Option<f32>,
    pub joint_amps: Option<EnumMap<ActuatorId, f32>>,
    pub quaternion: Option<[f32; 4]>,
    pub projected_g: Option<[f32; 3]>,
    pub accel: Option<[f32; 3]>,
    pub gyro: Option<[f32; 3]>,
    pub command: [Option<f32>; 7],
    pub output: Option<EnumMap<ActuatorId, f32>>,
}

impl PolicyStepDescriptorPod {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_session(step_session: &ort::session::Session) -> std::io::Result<Self> {
        let mut ret = Self::default();
        for input in step_session.inputs.iter() {
            let length = *input.input_type.tensor_shape().unwrap().first().unwrap() as usize;
            let model_input_type = ModelInputType::try_from(input)?;

            match model_input_type {
                ModelInputType::DataType(data_type) => match data_type {
                    DataType::JointAngles => {
                        ret.joint_angles = Some(EnumMap::default());
                        ret.output = Some(EnumMap::default());
                        ret.joint_amps = Some(EnumMap::default());
                    }
                    DataType::JointAngularVelocities => {
                        ret.joint_vels = Some(EnumMap::default());
                    }
                    DataType::InitialHeading => {
                        ret.initial_heading = Some(0.0);
                    }
                    DataType::Quaternion => {
                        ret.quaternion = Some([0.0; 4]);
                    }
                    DataType::ProjectedGravity => {
                        ret.projected_g = Some([0.0; 3]);
                    }
                    DataType::Accelerometer => {
                        ret.accel = Some([0.0; 3]);
                    }
                    DataType::Gyroscope => {
                        ret.gyro = Some([0.0; 3]);
                    }
                    DataType::Time => {
                        ret.t_us = Some(0);
                    }
                },
                ModelInputType::Command(_) => {
                    // command array is already initialized to [None; 7] by default
                }
                ModelInputType::Carry => {
                    // carry is not a data type, so we skip it
                }
            }
        }

        Ok(ret)
    }

    pub fn fill_outputs(
        &mut self,
        outputs: &ort::session::SessionOutputs,
        cmd_idx_to_actuator_id: &[ActuatorId],
    ) -> std::io::Result<()> {
        let commands = outputs[0]
            .try_extract_array::<f32>()
            .map_err(std::io::Error::other)?
            .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
            .map_err(std::io::Error::other)?;

        let dst = self.output.as_mut().unwrap();
        for (i, &actuator_id) in cmd_idx_to_actuator_id.iter().enumerate() {
            if i < commands.len() {
                dst[actuator_id] = commands[i];
            }
        }

        Ok(())
    }

    pub fn fill_input(
        &mut self,
        input_type: &ModelInputType,
        input_val: &ort::session::SessionInputValue,
        cmd_idx_to_actuator_id: &[ActuatorId],
    ) -> std::io::Result<()> {
        let ort::session::SessionInputValue::Owned(src) = input_val else {
            return Err(std::io::Error::other("Expected Owned tensor"));
        };
        let src = src
            .try_extract_array::<f32>()
            .map_err(std::io::Error::other)?;

        match input_type {
            ModelInputType::DataType(data_type) => match data_type {
                DataType::JointAngles => {
                    let dst = self.joint_angles.as_mut().unwrap();
                    for (i, &actuator_id) in cmd_idx_to_actuator_id.iter().enumerate() {
                        if i < src.len() {
                            dst[actuator_id] = src[i];
                        }
                    }
                }
                DataType::JointAngularVelocities => {
                    let dst = self.joint_vels.as_mut().unwrap();
                    for (i, &actuator_id) in cmd_idx_to_actuator_id.iter().enumerate() {
                        if i < src.len() {
                            dst[actuator_id] = src[i];
                        }
                    }
                }
                DataType::InitialHeading => {
                    self.initial_heading = Some(src[0]);
                }
                DataType::Quaternion => {
                    if let Some(ref mut quaternion) = self.quaternion {
                        for (i, value) in quaternion.iter_mut().enumerate() {
                            if i < src.len() {
                                *value = src[i];
                            }
                        }
                    }
                }
                DataType::ProjectedGravity => {
                    if let Some(ref mut projected_g) = self.projected_g {
                        for (i, value) in projected_g.iter_mut().enumerate() {
                            if i < src.len() {
                                *value = src[i];
                            }
                        }
                    }
                }
                DataType::Accelerometer => {
                    if let Some(ref mut accel) = self.accel {
                        for (i, value) in accel.iter_mut().enumerate() {
                            if i < src.len() {
                                *value = src[i];
                            }
                        }
                    }
                }
                DataType::Gyroscope => {
                    if let Some(ref mut gyro) = self.gyro {
                        for (i, value) in gyro.iter_mut().enumerate() {
                            if i < src.len() {
                                *value = src[i];
                            }
                        }
                    }
                }
                DataType::Time => {
                    self.t_us = Some(src[0] as u64);
                }
            },
            ModelInputType::Command(_) => {
                for (i, &value) in src.iter().enumerate() {
                    if i < 7 {
                        self.command[i] = Some(value);
                    }
                }
            }
            ModelInputType::Carry => {
                return Ok(());
            }
        };

        Ok(())
    }

    pub fn fill_from_description(
        &mut self,
        robot_description: &RobotDescription,
        cmd_idx_to_actuator_id: &[ActuatorId],
    ) {
        let Some(ref mut dst) = self.joint_amps else {
            return;
        };

        for &act_id in cmd_idx_to_actuator_id {
            dst[act_id] = robot_description.actuators.actuator_states[act_id]
                .feedback
                .amps as f32;
        }
    }

    pub fn finalize(&mut self) {
        if let Some(ref mut step_id) = self.step_id {
            *step_id += 1;
        } else {
            self.step_id = Some(0);
        }
        self.timestamp_now();
    }

    fn timestamp_now(&mut self) {
        self.t_us = Some(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_micros() as u64,
        );
    }
}
