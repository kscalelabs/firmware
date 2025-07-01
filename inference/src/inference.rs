use robot_description::{
    RobotDescription,
    ActuatorId,
    DataType,
};

use std::{
    pin::Pin,
    future::Future,
    task::{Context, Poll, ready},
};
use futures::Stream;

use std::io::Read;

use pin_project::pin_project;
use ort::session::Session;

use infrastructure::state_machine;
state_machine!(Reset, Operate);


#[derive(Debug, Clone)]
enum ModelInputType {
    DataType(DataType),
    Carry,
}

impl TryFrom<String> for ModelInputType {
    type Error = std::io::Error;
    fn try_from(value: String) -> Result<Self, Self::Error> {
        match value.as_str() {
            "carry" => Ok(ModelInputType::Carry),
            _ => {
                let data_type = DataType::try_from(value)?;
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
}

impl Store {

    pub fn validate_sessions(
        init_fn: &ort::session::Session,
        step_fn: &ort::session::Session,
        robot_description: &RobotDescription,
    ) -> std::io::Result<()> {

        use std::io::{Error, ErrorKind};
        if !init_fn.inputs.is_empty() {
            return Err(Error::new(ErrorKind::InvalidInput, "init_fn should not have any inputs"));
        }
        if init_fn.outputs.len() != 1 {
            return Err(Error::new(ErrorKind::InvalidInput, "init_fn should have exactly one output"));
        }

        let init_carry_shape = init_fn.outputs[0]
            .output_type
            .tensor_shape()
            .expect("init_fn output should have a tensor shape");

        let mut step_input_carry_shape: Option<ort::tensor::Shape> = None;

        for input in step_fn.inputs.iter() {
            
            let name = input.name.clone();
            let dims = input.input_type.tensor_shape().expect("step_fn input should have a tensor shape");
            log::info!("input :{:?}", input);

            if let Ok(model_input_type) = ModelInputType::try_from(name.clone()) {
                match model_input_type {
                    ModelInputType::DataType(data_type) => {
                        let target = ort::tensor::Shape::try_from(robot_description.dimensions(data_type))
                            .expect("step_fn input should have a tensor shape");

                        if *dims != target {
                            return Err(Error::new(std::io::ErrorKind::InvalidInput, format!("step_fn input {} shape ({:?}) does not match robot description ({:?})",
                                name, dims, target)));
                        }
                    }
                    ModelInputType::Carry => {
                        step_input_carry_shape = Some(dims.clone());
                    }
                }
            } else {
                return Err(Error::new(std::io::ErrorKind::InvalidInput, format!("step_fn input {} is not a valid data type", name)));
            }
        }

        // validate outputs
        if step_fn.outputs.len() != 2 {
            return Err(Error::new(std::io::ErrorKind::InvalidInput, "step_fn should have exactly two outputs"));
        }

        // validate controller output shape
        let name = step_fn.outputs[0].name.clone();
        let dims = step_fn.outputs[0].output_type.tensor_shape().expect("step_fn input should have a tensor shape");
        let target = ort::tensor::Shape::try_from(
            robot_description.dimensions(robot_description::DataType::JointAngles)
        ).expect("step_fn output should have a tensor shape");

        if *dims != target {
            return Err(Error::new(std::io::ErrorKind::InvalidInput, format!("step_fn output[0] ({}) shape ({:?}) does not match robot description ({:?})",
                name, dims, target)));
        }



        let step_output_carry_shape = step_fn.outputs[1]
            .output_type
            .tensor_shape()
            .expect("step_fn output should have a tensor shape");

        // check all carry shapes
        if step_input_carry_shape.is_none() {
            return Err(Error::new(std::io::ErrorKind::InvalidInput, "step_fn should have a carry output"));
        }

        let step_input_carry_shape = step_input_carry_shape.unwrap();

        if *init_carry_shape != step_input_carry_shape {
            return Err(Error::new(std::io::ErrorKind::InvalidInput, "init_fn output shape does not match step_fn carry input shape"));
        }

        if init_carry_shape != step_output_carry_shape {
            return Err(Error::new(std::io::ErrorKind::InvalidInput, "init_fn output shape does not match step_fn carry output shape"));
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
        let mut _metadata: Option<String> = None;

        for entry in archive.entries()? {
            let mut entry = entry?;
            let path = entry.path()?;
            match path.to_string_lossy().as_ref() {
                "metadata.json" => {
                    let mut contents = String::new();
                    entry.read_to_string(&mut contents)?;
                    _metadata = Some(contents);
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

        use std::io::{Error, ErrorKind};
        let mut init_session = Session::builder()
            .map_err(|e| Error::new(ErrorKind::Other, e))?
            .commit_from_memory(
                &init_fn
                    .ok_or_else(|| Error::new(ErrorKind::NotFound, "init_fn.onnx not found in archive"))?
            )
            .map_err(|e| Error::new(ErrorKind::Other, e))?;

        let step_session = Session::builder()
            .map_err(|e| Error::new(ErrorKind::Other, e))?
            .commit_from_memory(
                &step_fn
                    .ok_or_else(|| Error::new(ErrorKind::NotFound, "step_fn.onnx not found in archive"))?
            )
            .map_err(|e| Error::new(ErrorKind::Other, e))?;

        Self::validate_sessions(
            &init_session,
            &step_session,
            robot_description,
        )?;

        // crate the session inputs
        let mut step_input_vec = vec![];
        let mut step_input_types = vec![];

        for i in 0..step_session.inputs.len() {
            let input = &step_session.inputs[i];
            println!("input name: {}", input.name);
            println!("input type: {:?}", input.input_type);
            let name = input.name.clone();
            let model_input_type = ModelInputType::try_from(name)?;
            step_input_types.push(model_input_type.clone());

            // allocate and push tensors
            step_input_vec.push(
                ort::session::SessionInputValue::Owned(
                    ort::value::DynTensor::new(
                        step_session.allocator(),
                        input.input_type.tensor_type().unwrap(),
                        input.input_type.tensor_shape().unwrap().clone(),
                    ).expect("Failed to create tensor").into()
                )
            );

            if let ModelInputType::Carry = model_input_type {
                // seed the carry state
                let Some(ort::session::SessionInputValue::Owned(dst)) = step_input_vec.last_mut() else {
                    panic!("Expected a mutable reference to a DynTensor");
                };
                let mut dst = dst.try_extract_array_mut::<f32>().map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::Other, e)
                })?;

                // get the carry state
                let input_values: Vec<(String, ort::value::Value)> = Vec::new();
                let outputs = init_session.run(input_values).map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::Other, e)
                })?;

                let src = outputs[0].try_extract_array::<f32>().map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::Other, e)
                })?;

                ndarray::Zip::from(src.view()).and(dst.view_mut()).for_each(|srcp, dstp| {
                    *dstp = *srcp;
                });
            }
        }

        // create the output to actuator id map
        let cmd_idx_to_actuator_id = vec![

            ActuatorId::Rsp,
            ActuatorId::Rsr,
            ActuatorId::Rsy,
            ActuatorId::Rep,
            ActuatorId::Rwr,

            ActuatorId::Lsp,
            ActuatorId::Lsr,
            ActuatorId::Lsy,
            ActuatorId::Lep,
            ActuatorId::Lwr,

            ActuatorId::Rhp,
            ActuatorId::Rhr,
            ActuatorId::Rhy,
            ActuatorId::Rkp,
            ActuatorId::Rap,

            ActuatorId::Lhp,
            ActuatorId::Lhr,
            ActuatorId::Lhy,
            ActuatorId::Lkp,
            ActuatorId::Lap,
        ];

        let actuator_id_to_cmd_idx = enum_map::EnumMap::from_fn(|actuator_id| {
            let idx = cmd_idx_to_actuator_id.iter().position(|id| *id == actuator_id).unwrap();
            idx
        });

        Ok(Self {
            init_session,
            step_session,
            step_input_vec,
            step_input_types,
            cmd_idx_to_actuator_id,
            actuator_id_to_cmd_idx,
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

impl State for Reset
{
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Operate(Operate {
                    shared_state,
                    creation_time: std::time::Instant::now(),
                    session_input_vec: self.session_input_vec,
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
}

impl std::fmt::Debug for Operate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Operate")
            .field("shared_state", &self.shared_state)
            .finish()
    }
}

impl Operate {
    pub fn step_controller(&mut self, robot_description: &mut RobotDescription) -> std::io::Result<()> {
        // policy forward function
        let StoreProj {
            step_input_vec,
            step_input_types,
            step_session,
            cmd_idx_to_actuator_id,
            actuator_id_to_cmd_idx,
            ..
        } = self.shared_state.as_mut().project(); 

        for (input_type, input_val) in step_input_types.iter().zip(step_input_vec.iter_mut()) {
            match input_type {
                ModelInputType::Carry => {
                    // carry input is the output of the previous step
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
                        log::warn!("Failed to extract data for input type {:?}: {}", data_type, e);
                    }
                }
            }
        }

        let inputs = step_input_vec.as_slice();
        let outputs = step_session.run(inputs).map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::Other, e)
        })?;

        // put the carry output back into the input
        for (input_type, input_val) in step_input_types.iter().zip(step_input_vec.iter_mut()) {
            match input_type {
                ModelInputType::Carry => {
                    let ort::session::SessionInputValue::Owned(dst) = input_val else {
                        panic!("Expected a mutable reference to a DynTensor");
                    };
                    let mut dst = dst.try_extract_array_mut::<f32>().map_err(|e| {
                        std::io::Error::new(std::io::ErrorKind::Other, e)
                    })?;

                    let src = outputs[1].try_extract_array::<f32>().map_err(|e| {
                        std::io::Error::new(std::io::ErrorKind::Other, e)
                    })?;

                    ndarray::Zip::from(src.view()).and(dst.view_mut()).for_each(|srcp, dstp| {
                        *dstp = *srcp;
                    });
                }
                _ => {}
            }
        }

        // extract the outputs
        let commands = outputs[0].try_extract_array::<f32>().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::Other, e)
        })?
        .into_dimensionality::<ndarray::Dim<[usize; 1]>>().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::Other, e)
        })?;

        let actuator_states = &mut robot_description.actuators.actuator_states;
        for (i, command) in commands.iter().enumerate() {
            let actuator_id = cmd_idx_to_actuator_id[i];
            let act_state = &mut actuator_states[actuator_id];
            // get the normalized qpso
            let normalized_qpos = robot_description::normalize_actuator_qpos(act_state.feedback.qpos);
            let err = *command as f64 - normalized_qpos;
            let final_command = act_state.feedback.qpos + err * robot_description.policy_scale;
            // TODO: action scale
            // act_state.command.qpos = *command as f64 * robot_description.policy_scale;
            act_state.command.qpos = final_command;
            act_state.command.qvel = 0.0; // no velocity
            act_state.command.qfrc = 0.0; // no force
            act_state.command.kp = robot_description.policy_position[actuator_id].kp * robot_description.kp_scale;
            act_state.command.kd = robot_description.policy_position[actuator_id].kd * robot_description.kd_scale;
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
            return Err(ort::Error::new(
                "Expected Owned tensor",
            ))
        };

        let mut arr = arr.try_extract_array_mut::<f32>()?
            .into_dimensionality::<ndarray::Dim<[usize; 1]>>()
            .expect("Failed to convert to 1D array");

        match data_type {
            DataType::JointAngles => {

                robot_description.actuators.actuator_states.iter().for_each(|(act_id, act_state)| {
                    let i = actuator_id_to_cmd_idx[act_id];
                    arr[i] = robot_description::normalize_actuator_qpos(act_state.feedback.qpos) as f32;
                });
            }
            DataType::JointAngularVelocities => {
                robot_description.actuators.actuator_states.iter().for_each(|(act_id, act_state)| {
                    let i = actuator_id_to_cmd_idx[act_id];
                    arr[i] = act_state.feedback.qvel as f32;
                });
            }
            DataType::Quaternion => {
                robot_description.imu.quaternion.coords.iter().enumerate().for_each(|(i, q)| {
                    arr[i] = *q as f32;
                });
            }
            DataType::ProjectedGravity => {

                let unit_quat = nalgebra::UnitQuaternion::from_quaternion(
                    robot_description.imu.quaternion
                );

                let projected = unit_quat.conjugate() * nalgebra::Vector3::new(0.0, 0.0, -9.81);
                projected.iter().enumerate().for_each(|(i, g)| {
                    arr[i] = *g as f32;
                });
            }
            DataType::Accelerometer => {
                robot_description.imu.accelerometer.iter().enumerate().for_each(|(i, a)| {
                    arr[i] = *a as f32;
                });
            }
            DataType::Gyroscope => {
                robot_description.imu.gyroscope.iter().enumerate().for_each(|(i, g)| {
                    arr[i] = *g as f32;
                });
            }
            DataType::Time => {
                arr[0] = start_time.elapsed().as_secs_f32();
            }
            _ => {
                return Err(ort::Error::new(
                    "Unsupported data type",
                ))
            }
        }

        Ok(())
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
                    creation_time: self.creation_time,
                    session_input_vec: self.session_input_vec,
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
                log::debug!("Polling pending future");
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
