use crate::robot_description::{self, ActuatorId, RobotDescription};
use serde::{Deserialize, Serialize};
use tracing::{debug, error, info, trace, warn};

#[derive(Debug, Clone, Serialize, Default)]
pub struct PolicyStepDescriptor {
    pub step_id: Option<u64>,
    pub t_us: Option<u64>,
    pub joint_angles: Option<Vec<f32>>,
    pub joint_vels: Option<Vec<f32>>,
    pub initial_heading: Option<f32>,
    pub joint_amps: Option<Vec<f32>>,
    pub joint_torques: Option<Vec<f32>>,
    pub joint_temps: Option<Vec<f32>>,
    pub quaternion: Option<Vec<f32>>,
    pub projected_g: Option<Vec<f32>>,
    pub accel: Option<Vec<f32>>,
    pub gyro: Option<Vec<f32>>,
    pub command: Option<Vec<f32>>,
    pub output: Option<Vec<f32>>,
}

impl PolicyStepDescriptor {
    pub fn from_session(step_session: &ort::session::Session) -> std::io::Result<Self> {
        let mut ret = Self::default();
        for input in step_session.inputs.iter() {
            let name = input.name.clone();
            let length = *input.input_type.tensor_shape().unwrap().first().unwrap() as usize;

            let model_input_type = ModelInputType::try_from(input)?;

            match model_input_type {
                ModelInputType::DataType(data_type) => match data_type {
                    DataType::JointAngles => {
                        ret.joint_angles = Some(vec![0.0; length]);
                        ret.output = Some(vec![0.0; length]);
                        // always add joint_amps
                        ret.joint_amps = Some(vec![0.0; length]);
                        ret.joint_torques = Some(vec![0.0; length]);
                        ret.joint_temps = Some(vec![0.0; length]);
                    }
                    DataType::JointAngularVelocities => {
                        ret.joint_vels = Some(vec![0.0; length]);
                    }
                    DataType::InitialHeading => {
                        ret.initial_heading = Some(0.0);
                    }
                    DataType::Quaternion => {
                        ret.quaternion = Some(vec![0.0; 4]);
                    }
                    DataType::ProjectedGravity => {
                        ret.projected_g = Some(vec![0.0; 3]);
                    }
                    DataType::Accelerometer => {
                        ret.accel = Some(vec![0.0; 3]);
                    }
                    DataType::Gyroscope => {
                        ret.gyro = Some(vec![0.0; 3]);
                    }
                    DataType::Time => {
                        ret.t_us = Some(0);
                    }
                },
                ModelInputType::Command(_) => {
                    ret.command = Some(vec![0.0; length]);
                }
                ModelInputType::Carry => {
                    // carry is not a data type, so we skip it
                }
            }
        }

        Ok(ret)
    }

    pub fn fill_outputs(&mut self, outputs: &ort::session::SessionOutputs) -> std::io::Result<()> {
        let commands = outputs[0]
            .try_extract_array::<f32>()
            .map_err(std::io::Error::other)?
            .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
            .map_err(std::io::Error::other)?;

        let dst = self.output.as_mut().unwrap();
        dst.iter_mut().zip(commands.iter()).for_each(|(d, s)| {
            *d = *s;
        });

        Ok(())
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

    pub fn fill_input(
        &mut self,
        input_type: &ModelInputType,
        input_val: &ort::session::SessionInputValue,
    ) -> std::io::Result<()> {
        let ort::session::SessionInputValue::Owned(src) = input_val else {
            return Err(std::io::Error::other("Expected Owned tensor"));
        };
        let mut src = src
            .try_extract_array::<f32>()
            .map_err(std::io::Error::other)?;

        // copy into the appropriate field
        match input_type {
            ModelInputType::DataType(data_type) => match data_type {
                DataType::JointAngles => {
                    let dst = self.joint_angles.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::JointAngularVelocities => {
                    let dst = self.joint_vels.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::InitialHeading => {
                    self.initial_heading = Some(src[0]);
                }
                DataType::Quaternion => {
                    let dst = self.quaternion.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::ProjectedGravity => {
                    let dst = self.projected_g.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::Accelerometer => {
                    let dst = self.accel.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::Gyroscope => {
                    let dst = self.gyro.as_mut().unwrap();
                    dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                        *d = *s;
                    });
                }
                DataType::Time => {
                    self.t_us = Some(src[0] as u64);
                }
            },
            ModelInputType::Command(_) => {
                let dst = self.command.as_mut().unwrap();
                dst.iter_mut().zip(src.iter()).for_each(|(d, s)| {
                    *d = *s;
                });
            }
            ModelInputType::Carry => {
                // carry is not a data type, so we skip it
                return Ok(());
            }
        };

        Ok(())
    }

    /// fills additional fields from robot description
    pub fn fill_from_description(
        &mut self,
        robot_description: &RobotDescription,
        cmd_idx_to_actuator_id: &[ActuatorId],
    ) {
        // create the output to actuator id map
        // TODO: read from metadata to create this
        let Some(ref mut dst) = self.joint_amps else {
            // if joint_amps is not set, we just return
            return;
        };

        for (i, act_id) in cmd_idx_to_actuator_id.iter().enumerate() {
            dst[i] = robot_description.actuators.actuator_states[*act_id]
                .feedback
                .amps as f32;
        }

        if let Some(ref mut torque_dst) = self.joint_torques {
            for (i, act_id) in cmd_idx_to_actuator_id.iter().enumerate() {
                torque_dst[i] = robot_description.actuators.actuator_states[*act_id]
                    .feedback
                    .qfrc as f32;
            }
        }

        if let Some(ref mut temp_dst) = self.joint_temps {
            for (i, act_id) in cmd_idx_to_actuator_id.iter().enumerate() {
                temp_dst[i] = robot_description.actuators.actuator_states[*act_id]
                    .feedback
                    .temp as f32;
            }
        }
    }
}

use futures::{Stream, StreamExt, TryStream, TryStreamExt};
use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll, ready},
};

use std::io::Read;

use ort::session::Session;
use pin_project::pin_project;

use crate::state_machine;
state_machine!(Reset, Operate);

use crate::policy_control::CommandType;
use crate::robot_description::DataType;

impl TryFrom<&ort::session::Input> for DataType {
    type Error = std::io::Error;
    fn try_from(input: &ort::session::Input) -> Result<Self, Self::Error> {
        match input.name.as_str() {
            "joint_angles" => Ok(DataType::JointAngles),
            "joint_angular_velocities" => Ok(DataType::JointAngularVelocities),
            "initial_heading" => Ok(DataType::InitialHeading),
            "quaternion" => Ok(DataType::Quaternion),
            "projected_gravity" => Ok(DataType::ProjectedGravity),
            "accelerometer" => Ok(DataType::Accelerometer),
            "gyroscope" => Ok(DataType::Gyroscope),
            "time" => Ok(DataType::Time),
            _ => Err(std::io::Error::other(format!(
                "Unknown data type: {input:?}"
            ))),
        }
    }
}

#[derive(Debug)]
pub enum ModelInputType {
    DataType(DataType),
    Command(CommandType),
    Carry,
}

impl TryFrom<&ort::session::Input> for ModelInputType {
    type Error = std::io::Error;
    fn try_from(input: &ort::session::Input) -> Result<Self, Self::Error> {
        match input.name.as_str() {
            "carry" => Ok(ModelInputType::Carry),
            "command" => {
                if input.input_type.tensor_shape().is_none() {
                    return Err(std::io::Error::other(
                        "Command input must have a tensor shape",
                    ));
                }

                let dims = input.input_type.tensor_shape().unwrap();
                if dims.len() != 1 {
                    return Err(std::io::Error::other(
                        "Command input must have a 1D tensor shape",
                    ));
                }

                let dims = dims[0];
                Ok(ModelInputType::Command(CommandType::from_dims(
                    dims as usize,
                )?))
            }
            _ => {
                let data_type = DataType::try_from(input)?;
                Ok(ModelInputType::DataType(data_type))
            }
        }
    }
}

impl std::fmt::Debug for Store {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Store")
            .field("init_session", &self.init_session)
            .field("step_session", &self.step_session)
            .finish()
    }
}

#[pin_project(project = StoreProj)]
pub struct Store {
    init_session: ort::session::Session,
    step_session: ort::session::Session,
    step_input_vec: Vec<ort::session::SessionInputValue<'static>>,
    step_input_types: Vec<ModelInputType>,
    cmd_idx_to_actuator_id: Vec<ActuatorId>,
    actuator_id_to_cmd_idx: enum_map::EnumMap<ActuatorId, usize>,
    step_description: PolicyStepDescriptor,
    kb_manager: crate::keyboard::KeyboardManager,
}

impl Store {
    pub fn validate_sessions(
        init_fn: &ort::session::Session,
        step_fn: &ort::session::Session,
        robot_description: &RobotDescription,
    ) -> std::io::Result<()> {
        use std::io::{Error, ErrorKind};
        if !init_fn.inputs.is_empty() {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "init_fn should not have any inputs",
            ));
        }
        if init_fn.outputs.len() != 1 {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "init_fn should have exactly one output",
            ));
        }

        let init_carry_shape = init_fn.outputs[0]
            .output_type
            .tensor_shape()
            .expect("init_fn output should have a tensor shape");

        let mut step_input_carry_shape: Option<ort::tensor::Shape> = None;

        for input in step_fn.inputs.iter() {
            let name = input.name.clone();
            let dims = input
                .input_type
                .tensor_shape()
                .expect("step_fn input should have a tensor shape");
            info!("input :{:?}", input);

            if let Ok(model_input_type) = ModelInputType::try_from(input) {
                match model_input_type {
                    ModelInputType::DataType(data_type) => {
                        let target =
                            ort::tensor::Shape::from(robot_description.dimensions(data_type));

                        if *dims != target {
                            return Err(Error::new(
                                std::io::ErrorKind::InvalidInput,
                                format!(
                                    "step_fn input {name} shape ({dims:?}) does not match robot description ({target:?})"
                                ),
                            ));
                        }
                    }
                    ModelInputType::Command(_) => {
                        // input shape is implictly validated during the conversion from input_type
                        // to ModelInputType::Command
                        // here we can cross check against the metadata if needed
                    }
                    ModelInputType::Carry => {
                        step_input_carry_shape = Some(dims.clone());
                    }
                }
            } else {
                return Err(Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("step_fn input {name} is not a valid data type"),
                ));
            }
        }

        // validate outputs
        if step_fn.outputs.len() != 2 {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "step_fn should have exactly two outputs",
            ));
        }

        // validate controller output shape
        let name = step_fn.outputs[0].name.clone();
        let dims = step_fn.outputs[0]
            .output_type
            .tensor_shape()
            .expect("step_fn input should have a tensor shape");
        let target = ort::tensor::Shape::from(
            robot_description.dimensions(robot_description::DataType::JointAngles),
        );

        if *dims != target {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "step_fn output[0] ({name}) shape ({dims:?}) does not match robot description ({target:?})"
                ),
            ));
        }

        let step_output_carry_shape = step_fn.outputs[1]
            .output_type
            .tensor_shape()
            .expect("step_fn output should have a tensor shape");

        // check all carry shapes
        if step_input_carry_shape.is_none() {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "step_fn should have a carry output",
            ));
        }

        let step_input_carry_shape = step_input_carry_shape.unwrap();

        if *init_carry_shape != step_input_carry_shape {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "init_fn output shape does not match step_fn carry input shape",
            ));
        }

        if init_carry_shape != step_output_carry_shape {
            return Err(Error::new(
                std::io::ErrorKind::InvalidInput,
                "init_fn output shape does not match step_fn carry output shape",
            ));
        }

        Ok(())
    }

    /// Synchronously load the model archive, initialize ONNX sessions, and return a Store
    pub fn new<P: AsRef<std::path::Path>>(
        model_path: P,
        robot_description: &RobotDescription,
    ) -> std::io::Result<Self> {
        // Open the file synchronously
        let mut file = std::fs::File::open(model_path.as_ref())?;

        // Read entire file into memory
        let mut buffer = Vec::new();
        file.read_to_end(&mut buffer)?;

        // Decompress and read the tar archive from memory
        let gz = flate2::read::GzDecoder::new(&buffer[..]);
        let mut archive = tar::Archive::new(gz);

        // Extract and validate entries
        let mut init_fn: Option<Vec<u8>> = None;
        let mut step_fn: Option<Vec<u8>> = None;
        let mut metadata: Option<PolicyMetadata> = None;

        for entry in archive.entries()? {
            let mut entry = entry?;
            let path = entry.path()?;
            match path.to_string_lossy().as_ref() {
                "metadata.json" => {
                    let mut contents = String::new();
                    entry.read_to_string(&mut contents)?;
                    metadata = Some(serde_json::from_str(&contents)?);
                }
                "init_fn.onnx" => {
                    let mut contents = Vec::new();
                    entry.read_to_end(&mut contents)?;
                    init_fn = Some(contents);
                }
                "step_fn.onnx" => {
                    let mut contents = Vec::new();
                    entry.read_to_end(&mut contents)?;
                    step_fn = Some(contents);
                }
                _ => {
                    // Ignore other files
                    continue;
                }
            }
        }

        if metadata.is_none() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "metadata.json not found in archive",
            ));
        }

        let metadata = metadata.unwrap();

        use std::io::{Error, ErrorKind};
        let mut init_session = Session::builder()
            .map_err(Error::other)?
            .commit_from_memory(&init_fn.ok_or_else(|| {
                Error::new(ErrorKind::NotFound, "init_fn.onnx not found in archive")
            })?)
            .map_err(Error::other)?;

        let step_session = Session::builder()
            .map_err(Error::other)?
            .commit_from_memory(&step_fn.ok_or_else(|| {
                Error::new(ErrorKind::NotFound, "step_fn.onnx not found in archive")
            })?)
            .map_err(Error::other)?;

        Self::validate_sessions(&init_session, &step_session, robot_description)?;

        // create the session inputs
        let mut step_input_vec = vec![];
        let mut step_input_types = vec![];

        let mut step_description = PolicyStepDescriptor::from_session(&step_session)?;

        for i in 0..step_session.inputs.len() {
            let input = &step_session.inputs[i];
            debug!("input name: {}", input.name);
            debug!("input type: {:?}", input.input_type);
            let name = input.name.clone();
            let model_input_type = ModelInputType::try_from(input)?;
            step_input_types.push(model_input_type);

            // allocate and push tensors
            step_input_vec.push(ort::session::SessionInputValue::Owned(
                ort::value::DynTensor::new(
                    step_session.allocator(),
                    input.input_type.tensor_type().unwrap(),
                    input.input_type.tensor_shape().unwrap().clone(),
                )
                .expect("Failed to create tensor")
                .into(),
            ));

            if let ModelInputType::Carry = step_input_types.last().unwrap() {
                // seed the carry state
                let Some(ort::session::SessionInputValue::Owned(dst)) = step_input_vec.last_mut()
                else {
                    panic!("Expected a mutable reference to a DynTensor");
                };
                let mut dst = dst
                    .try_extract_array_mut::<f32>()
                    .map_err(std::io::Error::other)?;

                // get the carry state
                let input_values: Vec<(String, ort::value::Value)> = Vec::new();
                let outputs = init_session
                    .run(input_values)
                    .map_err(std::io::Error::other)?;

                let src = outputs[0]
                    .try_extract_array::<f32>()
                    .map_err(std::io::Error::other)?;

                ndarray::Zip::from(src.view())
                    .and(dst.view_mut())
                    .for_each(|srcp, dstp| {
                        *dstp = *srcp;
                    });
            }
        }

        let mut cmd_idx_to_actuator_id =
            Vec::<ActuatorId>::with_capacity(metadata.joint_names.len());
        for dof in metadata.joint_names.iter() {
            cmd_idx_to_actuator_id.push(try_dof_to_actuator_id(dof)?);
        }
        let actuator_id_to_cmd_idx = enum_map::EnumMap::from_fn(|actuator_id| {
            cmd_idx_to_actuator_id
                .iter()
                .position(|id| *id == actuator_id)
                .unwrap()
        });

        Ok(Self {
            init_session,
            step_session,
            step_input_vec,
            step_input_types,
            cmd_idx_to_actuator_id,
            actuator_id_to_cmd_idx,
            step_description,
            kb_manager: crate::keyboard::KeyboardManager::new(),
        })
    }

    /// Check if this model has a 16D command input (for extended UDP)
    pub fn has_16d_command(&self) -> bool {
        self.step_input_types.iter().any(|input_type| {
            matches!(input_type, ModelInputType::Command(CommandType::Udp16ControlVectorInputState(_)))
        })
    }
}

pub struct Reset {
    shared_state: Pin<Box<Store>>,
    session_input_vec: Vec<ort::session::SessionInputValue<'static>>,
}

impl std::fmt::Debug for Reset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Reset")
            .field("shared_state", &self.shared_state)
            .finish()
    }
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Operate(Operate {
                    shared_state,
                    creation_time: std::time::Instant::now(),
                    session_input_vec: self.session_input_vec,
                    lpf_prev_qpos: enum_map::EnumMap::default(),
                    lpf_last_update: std::time::Instant::now(),
                    mj_prev_out: enum_map::EnumMap::default(),
                    mj_blend_t_elapsed: enum_map::EnumMap::default(),
                    mj_blend_t_total: enum_map::EnumMap::default(),
                }),
                result: Ok(()),
            }
        }
    }
}

pub struct Operate {
    shared_state: Pin<Box<Store>>,
    creation_time: std::time::Instant,
    session_input_vec: Vec<ort::session::SessionInputValue<'static>>,
    // Per-actuator low-pass filter state for commanded qpos
    lpf_prev_qpos: enum_map::EnumMap<crate::robot_description::ActuatorId, f64>,
    lpf_last_update: std::time::Instant,
    // Minimum-jerk blend state
    mj_prev_out: enum_map::EnumMap<crate::robot_description::ActuatorId, f64>,
    mj_blend_t_elapsed: enum_map::EnumMap<crate::robot_description::ActuatorId, f64>,
    mj_blend_t_total: enum_map::EnumMap<crate::robot_description::ActuatorId, f64>,
}

impl std::fmt::Debug for Operate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Operate")
            .field("shared_state", &self.shared_state)
            .finish()
    }
}

impl Operate {
    /// Check if this model has a 16D command input (for extended UDP)
    pub fn has_16d_command(&self) -> bool {
        self.shared_state.step_input_types.iter().any(|input_type| {
            matches!(input_type, ModelInputType::Command(CommandType::Udp16ControlVectorInputState(_)))
        })
    }

    pub fn step_controller(
        &mut self,
        robot_description: &mut RobotDescription,
    ) -> std::io::Result<()> {
        // policy forward function
        let StoreProj {
            step_input_vec,
            step_input_types,
            step_session,
            cmd_idx_to_actuator_id,
            actuator_id_to_cmd_idx,
            step_description,
            kb_manager,
            ..
        } = self.shared_state.as_mut().project();

        for (input_type, input_val) in step_input_types.iter_mut().zip(step_input_vec.iter_mut()) {
            match input_type {
                ModelInputType::Carry => {
                    // carry input is the output of the previous step
                }
                ModelInputType::Command(cmd) => {
                    // get feedback from the keyboard manager
                    while let Some(key_event) = robot_description.kb_pending_events.pop_front() {
                        cmd.update(key_event);
                    }

                    // set the command input to zero
                    let ort::session::SessionInputValue::Owned(arr) = input_val else {
                        return Err(std::io::Error::other(
                            "Expected a mutable reference to a DynTensor",
                        ));
                    };

                    let arr = arr
                        .try_extract_array_mut::<f32>()
                        .map_err(std::io::Error::other)?;
                    let arr = arr
                        .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
                        .expect("Failed to convert to 1D array");
                    use crate::policy_control::InputState;
                    cmd.extract_with_robot(arr, robot_description);
                }
                ModelInputType::DataType(data_type) => {
                    // extract data from robot description
                    if let Err(e) = Self::extract_data(
                        &self.creation_time,
                        input_val,
                        data_type,
                        robot_description,
                        actuator_id_to_cmd_idx,
                    ) {
                        warn!(
                            "Failed to extract data for input type {:?}: {}",
                            data_type, e
                        );
                    }
                }
            }
        }

        let inputs = step_input_vec.as_slice();
        let outputs = step_session.run(inputs).map_err(std::io::Error::other)?;

        step_description
            .fill_outputs(&outputs)
            .map_err(std::io::Error::other)?;

        // put the carry output back into the input
        for (input_type, input_val) in step_input_types.iter().zip(step_input_vec.iter_mut()) {
            step_description
                .fill_input(input_type, input_val)
                .map_err(std::io::Error::other)?;

            if let ModelInputType::Carry = input_type {
                let ort::session::SessionInputValue::Owned(dst) = input_val else {
                    panic!("Expected a mutable reference to a DynTensor");
                };
                let mut dst = dst
                    .try_extract_array_mut::<f32>()
                    .map_err(std::io::Error::other)?;

                let src = outputs[1]
                    .try_extract_array::<f32>()
                    .map_err(std::io::Error::other)?;

                ndarray::Zip::from(src.view())
                    .and(dst.view_mut())
                    .for_each(|srcp, dstp| {
                        *dstp = *srcp;
                    });
            }
        }

        // fill the step description with the current state
        step_description.fill_from_description(robot_description, cmd_idx_to_actuator_id);

        step_description.finalize();
        let json_str = serde_json::to_string(&step_description).map_err(std::io::Error::other)?;
        trace!(policy_step = json_str);

        // extract the outputs
        let commands = outputs[0]
            .try_extract_array::<f32>()
            .map_err(std::io::Error::other)?
            .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
            .map_err(std::io::Error::other)?;

        let actuator_states = &mut robot_description.actuators.actuator_states;

        let cutoff_hz = robot_description.lpf_cutoff_hz;
        let prev_map = &mut self.lpf_prev_qpos;
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.lpf_last_update).as_secs_f64();
        self.lpf_last_update = now;

        for (i, command) in commands.iter().enumerate() {
            let actuator_id = cmd_idx_to_actuator_id[i];
            let act_state = &mut actuator_states[actuator_id];
            // get the normalized qpos
            let normalized_qpos =
                robot_description::normalize_actuator_qpos(act_state.feedback.qpos);
            let err = *command as f64 - normalized_qpos;
            let unfiltered = act_state.feedback.qpos + err * robot_description.policy_scale;

            // One-pole LPF: y = y_prev + alpha * (x - y_prev); alpha = 1 - exp(-2*pi*fc*dt)
            let filtered = if cutoff_hz <= 0.0 || dt <= 0.0 {
                unfiltered
            } else {
                let alpha = 1.0 - (-2.0 * std::f64::consts::PI * cutoff_hz * dt).exp();
                let y_prev = prev_map[actuator_id];
                let y = y_prev + alpha * (unfiltered - y_prev);
                prev_map[actuator_id] = y;
                y
            };

            // Minimum-jerk retiming blend (optional)
            let mut final_command = filtered;
            let blend_ms = robot_description.min_jerk_blend_ms;
            if blend_ms > 0.0 && dt > 0.0 {
                let t_total = (blend_ms / 1000.0).max(1e-6);
                // Initialize previous output from current command on first use
                let mut prev_out = if self.mj_blend_t_total[actuator_id] == 0.0 {
                    let start = act_state.command.qpos;
                    self.mj_prev_out[actuator_id] = start;
                    start
                } else {
                    self.mj_prev_out[actuator_id]
                };
                let target = filtered;

                // If the target changed meaningfully, restart blend
                if (target - prev_out).abs() > 1e-9 {
                    self.mj_blend_t_elapsed[actuator_id] = 0.0;
                    self.mj_blend_t_total[actuator_id] = t_total;
                }

                // Advance blending time
                let t = (self.mj_blend_t_elapsed[actuator_id] + dt)
                    .min(self.mj_blend_t_total[actuator_id]);
                self.mj_blend_t_elapsed[actuator_id] = t;
                let denom = self.mj_blend_t_total[actuator_id];
                let s = if denom > 0.0 { (t / denom).clamp(0.0, 1.0) } else { 1.0 };
                // Minimum-jerk polynomial: 10 s^3 - 15 s^4 + 6 s^5
                let s2 = s * s;
                let s3 = s2 * s;
                let s4 = s3 * s;
                let s5 = s4 * s;
                let mj = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
                final_command = prev_out + mj * (target - prev_out);

                // Snap when done
                if (self.mj_blend_t_elapsed[actuator_id]
                    >= self.mj_blend_t_total[actuator_id] - 1e-12)
                {
                    final_command = target;
                }

                // Persist
                self.mj_prev_out[actuator_id] = final_command;
            } else {
                // Persist last even without blending
                self.mj_prev_out[actuator_id] = final_command;
            }
            act_state.command.qpos = final_command;
            act_state.command.qvel = 0.0; // no velocity command
            act_state.command.qfrc = 0.0; // no torque command
            act_state.command.kp =
                robot_description.policy_position[actuator_id].kp * robot_description.kp_scale;
            act_state.command.kd =
                robot_description.policy_position[actuator_id].kd * robot_description.kd_scale;
        }

        Ok(())
    }

    fn extract_data(
        start_time: &std::time::Instant,
        tensor: &mut ort::session::SessionInputValue<'_>,
        data_type: &DataType,
        robot_description: &RobotDescription,
        actuator_id_to_cmd_idx: &enum_map::EnumMap<ActuatorId, usize>,
    ) -> ort::Result<()> {
        // we have validated all sizes, no need to do size checks
        let ort::session::SessionInputValue::Owned(arr) = tensor else {
            return Err(ort::Error::new("Expected Owned tensor"));
        };

        let mut arr = arr
            .try_extract_array_mut::<f32>()?
            .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
            .expect("Failed to convert to 1D array");

        match data_type {
            DataType::JointAngles => {
                robot_description.actuators.actuator_states.iter().for_each(
                    |(act_id, act_state)| {
                        let i = actuator_id_to_cmd_idx[act_id];
                        arr[i] = robot_description::normalize_actuator_qpos(act_state.feedback.qpos)
                            as f32;
                    },
                );
            }
            DataType::JointAngularVelocities => {
                robot_description.actuators.actuator_states.iter().for_each(
                    |(act_id, act_state)| {
                        let i = actuator_id_to_cmd_idx[act_id];
                        arr[i] = act_state.feedback.qvel as f32;
                    },
                );
            }
            DataType::Quaternion => {
                // store in w, x, y, z order
                let quat = robot_description.imu.quaternion;
                arr[0] = quat.scalar() as f32; // this is w 

                // the vector part is x, y, z
                arr[1] = quat.vector()[0] as f32;
                arr[2] = quat.vector()[1] as f32;
                arr[3] = quat.vector()[2] as f32;
                let quat = nalgebra::UnitQuaternion::from_quaternion(quat);
                // scalar part is w
                let (_, _, yaw) = quat.euler_angles();
                log::warn!("Current heading yaw: {yaw}");
            }
            DataType::ProjectedGravity => {
                let unit_quat =
                    nalgebra::UnitQuaternion::from_quaternion(robot_description.imu.quaternion);

                let projected = unit_quat.conjugate() * nalgebra::Vector3::new(0.0, 0.0, -9.81);
                projected.iter().enumerate().for_each(|(i, g)| {
                    arr[i] = *g as f32;
                });
            }
            DataType::Accelerometer => {
                robot_description
                    .imu
                    .accelerometer
                    .iter()
                    .enumerate()
                    .for_each(|(i, a)| {
                        arr[i] = *a as f32;
                    });
            }
            DataType::Gyroscope => {
                robot_description
                    .imu
                    .gyroscope
                    .iter()
                    .enumerate()
                    .for_each(|(i, g)| {
                        arr[i] = *g as f32;
                    });
            }
            DataType::Time => {
                start_time.elapsed().as_secs_f32();
            }
            DataType::InitialHeading => {
                let unit_quat = nalgebra::UnitQuaternion::from_quaternion(
                    robot_description.initial_imu.quaternion,
                );
                let (_, _, yaw) = unit_quat.euler_angles();
                log::warn!("Initial heading yaw: {yaw}");
                arr[0] = yaw as f32;
            }
            _ => {
                // unsupported data type, we just continue
                // commented as this will spam:
                // log::warn!("Skipping unsupported data type: {:?}", data_type);
            }
        }

        Ok(())
    }
}

impl State for Operate {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Operate(Operate {
                    shared_state,
                    creation_time: self.creation_time,
                    session_input_vec: self.session_input_vec,
                    lpf_prev_qpos: self.lpf_prev_qpos,
                    lpf_last_update: self.lpf_last_update,
                    mj_prev_out: self.mj_prev_out,
                    mj_blend_t_elapsed: self.mj_blend_t_elapsed,
                    mj_blend_t_total: self.mj_blend_t_total,
                }),
                result: Ok(()),
            }
        }
    }
}

#[pin_project]
pub struct ModelManager {
    state: Option<StateStore>,
    target: Option<StateTag>,

    #[pin]
    pending_fut: Option<StateFut>,
}

impl ModelManager {
    pub fn new<P: AsRef<std::path::Path>>(
        model_path: P,
        robot_description: &RobotDescription,
    ) -> std::io::Result<Self> {
        info!("LPF Cutoff freq: {} hz", robot_description.lpf_cutoff_hz);
        Ok(Self {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new(model_path, robot_description)?),
                session_input_vec: vec![],
            })),
            target: None,
            pending_fut: None,
        })
    }

    pub fn set_target_pinned(self: Pin<&mut Self>, target: StateTag) {
        *self.project().target = Some(target);
    }

    pub fn get_state_pinned(self: Pin<&mut Self>) -> Option<&mut StateStore> {
        self.project().state.as_mut()
    }
}

impl std::fmt::Debug for ModelManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BehaviorManager")
            .field("state", &self.state)
            .field("target", &self.target)
            .finish()
    }
}

impl Stream for ModelManager {
    type Item = std::io::Result<StateTag>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let mut this = self.project();

        if let Some(pending_fut) = this.pending_fut.as_mut().as_pin_mut() {
            debug!("Polling pending future");
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyMetadata {
    pub joint_names: Vec<String>,
    pub num_commands: u32,
    pub carry_size: Vec<u32>,
}

// map these strings to ActuatorId
fn try_dof_to_actuator_id(dof: &str) -> std::io::Result<ActuatorId> {
    let res = match dof {
        "dof_right_shoulder_pitch_03" => ActuatorId::Rsp,
        "dof_right_shoulder_roll_03" => ActuatorId::Rsr,
        "dof_right_shoulder_yaw_02" => ActuatorId::Rsy,
        "dof_right_elbow_02" => ActuatorId::Rep,
        "dof_right_wrist_00" => ActuatorId::Rwr,
        "dof_left_shoulder_pitch_03" => ActuatorId::Lsp,
        "dof_left_shoulder_roll_03" => ActuatorId::Lsr,
        "dof_left_shoulder_yaw_02" => ActuatorId::Lsy,
        "dof_left_elbow_02" => ActuatorId::Lep,
        "dof_left_wrist_00" => ActuatorId::Lwr,
        "dof_right_hip_pitch_04" => ActuatorId::Rhp,
        "dof_right_hip_roll_03" => ActuatorId::Rhr,
        "dof_right_hip_yaw_03" => ActuatorId::Rhy,
        "dof_right_knee_04" => ActuatorId::Rkp,
        "dof_right_ankle_02" => ActuatorId::Rap,
        "dof_left_hip_pitch_04" => ActuatorId::Lhp,
        "dof_left_hip_roll_03" => ActuatorId::Lhr,
        "dof_left_hip_yaw_03" => ActuatorId::Lhy,
        "dof_left_knee_04" => ActuatorId::Lkp,
        "dof_left_ankle_02" => ActuatorId::Lap,
        _ => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Unknown dof: {dof}"),
            ));
        }
    };
    Ok(res)
}
