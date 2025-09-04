use crate::actuator_manager::{self, ActuatorManager};
use futures::{Stream, StreamExt, TryStream, TryStreamExt};
use pin_project::pin_project;
use std::{
    future::Future,
    io,
    pin::Pin,
    task::{Context, Poll, ready},
    time::Duration,
};
use tracing::{debug, error, info, trace, warn};

use crate::imu::{self, ImuManager};

use crate::robot_description::{self, ActuatorId, RobotDescription};

use crate::inference::{self, ModelManager};

use crate::robstride_utils::RobstrideActuatorParam;
use crate::udp_command::UnifiedUdpManager;

crate::state_machine!(Reset, Ready, Home, Policy);

impl std::fmt::Debug for Store {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Store")
            .field("actuator_manager", &self.actuator_manager)
            .finish()
    }
}

#[pin_project]
pub struct Store {
    robot_description: RobotDescription,
    #[pin]
    actuator_manager: ActuatorManager,
    #[pin]
    imu_manager: ImuManager,
    #[pin]
    model_manager: ModelManager,
    kb_manager: crate::keyboard::KeyboardManager,
    udp_manager: Option<UnifiedUdpManager>,
}

impl Default for Store {
    fn default() -> Self {
        Self::new()
    }
}

impl Store {
    pub fn new() -> Self {
        let robot_description = RobotDescription::new();

        let model_manager = ModelManager::new("model.kinfer", &robot_description)
            .expect("Failed to create model manager");

        // UDP manager will be initialized later based on policy requirements
        let udp_manager = None;

        Self {
            robot_description,
            actuator_manager: ActuatorManager::new(),
            imu_manager: ImuManager::new(),
            model_manager,
            kb_manager: crate::keyboard::KeyboardManager::new(),
            udp_manager,
        }
    }

    /// Initialize the UDP manager based on policy requirements
    pub async fn initialize_udp_manager(&mut self, use_extended: bool) -> std::io::Result<()> {
        self.udp_manager = Some(UnifiedUdpManager::new(10000, use_extended).await?);
        Ok(())
    }
}

#[derive(Debug)]
pub struct Reset {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Ready {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Home {
    shared_state: Pin<Box<Store>>,
    start: std::time::Instant,
}

#[derive(Debug)]
pub struct Policy {
    shared_state: Pin<Box<Store>>,
}

impl Home {
    pub fn new(shared_state: Pin<Box<Store>>) -> Self {
        Self {
            shared_state,
            start: std::time::Instant::now(),
        }
    }

    // returns max error
    pub fn step_controller(robot_desc: &mut RobotDescription, op_act_manager: &actuator_manager::Operate) -> f64 {
        let (actuators, home_position) = (&mut robot_desc.actuators, &mut robot_desc.home_position);

        // Check if we have only 2 buses (upper body only)
        let is_upper_body_only = op_act_manager.get_bus_count() == 2;

        let mut ret = 0.0f64;
        for (act_id, act_state) in actuators.actuator_states.iter_mut() {
            // Skip leg actuators if we only have 2 buses (upper body only)
            if is_upper_body_only {
                match act_id {
                    ActuatorId::Lhp | ActuatorId::Lhr | ActuatorId::Lhy | ActuatorId::Lkp | ActuatorId::Lap |
                    ActuatorId::Rhp | ActuatorId::Rhr | ActuatorId::Rhy | ActuatorId::Rkp | ActuatorId::Rap => {
                        debug!("Skipping leg actuator {:?} in upper body only mode", act_id);
                        continue;
                    }
                    _ => {}
                }
            }

            // feedback could be anywhere between [-4PI, 4PI]
            // home position is in [-PI, PI]
            // first we must normalize the feedback into [-PI, PI]
            let mut normalized_feedback =
                robot_description::normalize_actuator_qpos(act_state.feedback.qpos);

            let err = home_position[act_id].qpos - normalized_feedback;

            debug!(
                "Actuator {:?} feedback: {}, home position: {}, error: {}",
                act_id, normalized_feedback, home_position[act_id].qpos, err
            );
            // NOTE: these are currently not homed
            if act_id != ActuatorId::Rwr && act_id != ActuatorId::Lwr {
                ret = ret.max(err.abs());
            }
            // proportional control with clamping
            let step = (err).clamp(-4.0f64.to_radians(), 4.0f64.to_radians());
            // log::warn!("Actuator {:?} error: {}, step: {}", act_id, err, step);
            act_state.command.qpos = act_state.feedback.qpos + step;
            act_state.command.qvel = 0.0; // no velocity
            act_state.command.qfrc = 0.0; // no force
            act_state.command.kp = home_position[act_id].kp; // proportional gain
            act_state.command.kd = home_position[act_id].kd; // derivative gain
        }
        ret
    }
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Ready(Ready { shared_state }),
                result: Ok(()),
            }
        }
    }
}

impl State for Ready {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            let mut ss = shared_state.as_mut().project();

            // drive imu manager to the operate state
            let target = imu::StateTag::Operate;
            ss.imu_manager.as_mut().set_target_pinned(target);
            loop {
                match ss.imu_manager.try_next().await {
                    Ok(Some(imu::StateTag::Operate)) => break,
                    Ok(Some(tag)) => {
                        debug!("IMU manager state: {:?}", tag);
                        continue;
                    }
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(io::Error::new(
                                io::ErrorKind::UnexpectedEof,
                                "IMU manager stream ended unexpectedly",
                            )),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(e),
                        };
                    }
                }
            }
            let imu::StateStore::Operate(op_imu_manager) = ss
                .imu_manager
                .as_mut()
                .get_state_pinned()
                .expect("IMU manager should be in Operate state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset { shared_state }),
                    result: Err(io::Error::other("IMU manager is not in Operate state")),
                };
            };
            op_imu_manager
                .process_feedback(&mut ss.robot_description.imu)
                .await;
            debug!("IMU state: {:#?}", ss.robot_description.imu);
            info!("Press Enter to continue..");
            ss.kb_manager.wait_for_enter().await;

            // drive actuator manager to the operate state
            let target = actuator_manager::StateTag::Ready;
            ss.actuator_manager.as_mut().set_target_pinned(target);
            loop {
                match ss.actuator_manager.try_next().await {
                    Ok(Some(actuator_manager::StateTag::Ready)) => break,
                    Ok(Some(tag)) => {
                        debug!("Actuator manager state: {:?}", tag);
                        continue;
                    }
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(io::Error::new(
                                io::ErrorKind::UnexpectedEof,
                                "Actuator manager stream ended unexpectedly",
                            )),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(e),
                        };
                    }
                }
            }

            // reached target state
            let actuator_manager::StateStore::Ready(rdy_act_manager) = ss
                .actuator_manager
                .as_mut()
                .get_state_pinned()
                .expect("Actuator manager should be in Ready state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset { shared_state }),
                    result: Err(io::Error::other("Actuator manager is not in Ready state")),
                };
            };

            // drive model manager to operate state
            let target = inference::StateTag::Operate;
            ss.model_manager.as_mut().set_target_pinned(target);
            loop {
                match ss.model_manager.try_next().await {
                    Ok(Some(inference::StateTag::Operate)) => break,
                    Ok(Some(tag)) => {
                        debug!("model manager state: {:?}", tag);
                        continue;
                    }
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(io::Error::new(
                                io::ErrorKind::UnexpectedEof,
                                "model manager stream ended unexpectedly",
                            )),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(e),
                        };
                    }
                }
            }

            // reached target state
            let inference::StateStore::Operate(op_model) = ss
                .model_manager
                .as_mut()
                .get_state_pinned()
                .expect("Model Manager should be in operate state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset { shared_state }),
                    result: Err(io::Error::other("Model manager is not in Operate state")),
                };
            };

            // Initialize UDP manager based on policy requirements
            let use_extended = op_model.has_16d_command();
            match UnifiedUdpManager::new(10000, use_extended).await {
                Ok(udp_mgr) => {
                    *ss.udp_manager = Some(udp_mgr);
                }
                Err(e) => {
                    return StateTransitionResult {
                        state: StateStore::Reset(Reset { shared_state }),
                        result: Err(io::Error::new(io::ErrorKind::Other, format!("Failed to initialize UDP manager: {}", e))),
                    };
                }
            }

            warn!("Press Enter to drive the buses...");
            ss.kb_manager.wait_for_enter().await;

            rdy_act_manager.enable().await;
            StateTransitionResult {
                state: StateStore::Home(Home::new(shared_state)),
                result: Ok(()),
            }
        }
    }
}

impl State for Home {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = &mut self.shared_state;
            let mut ss = shared_state.as_mut().project();

            // move the actuator manager to the operate state

            let target = actuator_manager::StateTag::Operate;
            ss.actuator_manager.as_mut().set_target_pinned(target);
            loop {
                match ss.actuator_manager.try_next().await {
                    Ok(Some(actuator_manager::StateTag::Operate)) => break,
                    Ok(_) => continue,
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset {
                                shared_state: self.shared_state,
                            }),
                            result: Err(io::Error::new(
                                io::ErrorKind::UnexpectedEof,
                                "Actuator manager stream ended unexpectedly",
                            )),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset {
                                shared_state: self.shared_state,
                            }),
                            result: Err(e),
                        };
                    }
                }
            }

            let actuator_manager::StateStore::Operate(op_act_manager) = ss
                .actuator_manager
                .as_mut()
                .get_state_pinned()
                .expect("Actuator manager should be in Operate state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(io::Error::other("Actuator manager is not in Operate state")),
                };
            };

            let imu::StateStore::Operate(op_imu_manager) = ss
                .imu_manager
                .as_mut()
                .get_state_pinned()
                .expect("IMU manager should be in Operate state")
            else {
                return StateTransitionResult {
                    state: StateStore::Reset(Reset {
                        shared_state: self.shared_state,
                    }),
                    result: Err(io::Error::other("IMU manager is not in Operate state")),
                };
            };

            // request initial feedback to seed the state
            if let Err(e) = op_act_manager.request_feedback().await {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            // process initial feedback
            let act_states = ss.robot_description.actuator_states_mut();
            if let Err(e) = op_act_manager.process_feedback(act_states).await {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            // request motor current feedback
            if let Err(e) = op_act_manager
                .request_param(RobstrideActuatorParam::Iqf)
                .await
            {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            // and wait for responses
            let act_states = ss.robot_description.actuator_states_mut();
            if let Err(e) = op_act_manager.process_feedback(act_states).await {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            op_imu_manager
                .process_feedback(&mut ss.robot_description.imu)
                .await;
            // run controller

            let err = Self::step_controller(ss.robot_description, op_act_manager);

            let act_states = ss.robot_description.actuator_states_mut();
            // TODO
            // get time since start

            // let elapsed = self.start.elapsed();
            // // sine wave with a period of 2 seconds and ampliture of 0.5 radians
            // let period = std::time::Duration::from_secs(2);
            // let amplitude = 0.5; // rad
            //
            // let angle = amplitude * (2.0 * std::f64::consts::PI * elapsed.as_secs_f64() / period.as_secs_f64()).sin();

            // for act_state in act_states.actuator_states.values_mut() {
            //     act_state.command.qpos = angle;
            //     act_state.command.qvel = 0.0; // no velocity
            //     act_state.command.qfrc = 0.0; // no force
            //     act_state.command.kp = 1.0; // proportional gain
            //     act_state.command.kd = 1.0; // derivative gain
            // }

            // send commadn to buses and wait for responses
            let act_states = &mut ss.robot_description.actuators;
            if let Err(e) = op_act_manager.send_command(act_states).await {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            if let Err(e) = op_act_manager.process_feedback(act_states).await {
                return StateTransitionResult {
                    state: StateStore::Home(Home::new(self.shared_state)),
                    result: Err(e),
                };
            }

            if err < 0.1 {
                // can go to next state
                info!("error to home: {}", err);
                info!("Home position reached, press enter to run policy");
                ss.kb_manager.wait_for_enter().await;
                op_imu_manager.clear();
                std::thread::sleep(std::time::Duration::from_millis(30));
                // we need to arm the imu with the initial value. Again, this needs to be done once
                // when we transition to Policy state.
                op_imu_manager
                    .process_feedback(&mut ss.robot_description.imu)
                    .await;
                ss.robot_description.initial_imu = ss.robot_description.imu;

                return StateTransitionResult {
                    state: StateStore::Policy(Policy {
                        shared_state: self.shared_state,
                    }),
                    result: Ok(()),
                };
            }

            StateTransitionResult {
                state: StateStore::Home(Home {
                    shared_state: self.shared_state,
                    start: self.start, // keep the start time to continue the sine wave
                }),
                result: Ok(()),
            }
        }
    }
}

impl State for Policy {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(mut self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = &mut self.shared_state;
            let mut ss = shared_state.as_mut().project();

            // Note we need to do this only once when we enter raw mode. However, our current
            // transition function is simply a return. This can be fixed by refactoring this
            // function to be a loop instead of returning to Policy State
            // ss.kb_manager.enable_raw_mode().expect("Failed to enable raw mode");
            // ss.kb_manager.process_feedback(&mut ss.robot_description.kb_pending_events);

            let start_time = std::time::Instant::now();

            let mut interval = tokio::time::interval(Duration::from_millis(20));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut iteration_start = std::time::Instant::now();

            loop {
                interval.tick().await; // Wait for next 20ms deadline

                let total_iteration_time = iteration_start.elapsed();
                iteration_start = std::time::Instant::now();

                // Drain UDP buffer
                let udp_update_start = std::time::Instant::now();
                if let Some(udp_manager) = ss.udp_manager {
                    if let Err(e) = udp_manager.try_update_command().await {
                        warn!("UDP command update failed: {:?}", e);
                    }

                    // Update robot description with current command
                    if udp_manager.has_recent_command() {
                        udp_manager.update_robot_description(ss.robot_description);
                    } else {
                        // Clear command state when no recent command (timeout)
                        udp_manager.clear_robot_description(ss.robot_description);
                    }
                } else {
                    warn!("UDP manager not initialized");
                }
                let udp_update_time = udp_update_start.elapsed();


                let actuator_manager::StateStore::Operate(op_act_manager) = ss
                    .actuator_manager
                    .as_mut()
                    .get_state_pinned()
                    .expect("Actuator manager should be in Operate state")
                else {
                    return StateTransitionResult {
                        state: StateStore::Reset(Reset {
                            shared_state: self.shared_state,
                        }),
                        result: Err(io::Error::other("Actuator manager is not in Operate state")),
                    };
                };

                let imu::StateStore::Operate(op_imu_manager) = ss
                    .imu_manager
                    .as_mut()
                    .get_state_pinned()
                    .expect("IMU manager should be in Operate state")
                else {
                    return StateTransitionResult {
                        state: StateStore::Reset(Reset {
                            shared_state: self.shared_state,
                        }),
                        result: Err(io::Error::other("IMU manager is not in Operate state")),
                    };
                };

                // request motor current feedback
                if let Err(e) = op_act_manager
                    .request_param(RobstrideActuatorParam::Iqf)
                    .await
                {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }

                // and wait for responses
                let act_states = ss.robot_description.actuator_states_mut();
                if let Err(e) = op_act_manager.process_feedback(act_states).await {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }
                // request initial feedback to seed the state
                if let Err(e) = op_act_manager.request_feedback().await {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }
                // and wait for responses
                let act_states = ss.robot_description.actuator_states_mut();
                if let Err(e) = op_act_manager.process_feedback(act_states).await {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }

                op_imu_manager
                    .process_feedback(&mut ss.robot_description.imu)
                    .await;
                // log::info!("IMU state: {:#?}", ss.robot_description.imu);
                // let unit_quat = nalgebra::UnitQuaternion::from_quaternion(
                //     ss.robot_description.imu.quaternion
                // );
                // let projected = unit_quat.conjugate() * nalgebra::Vector3::new(0.0, 0.0, -9.81);
                // log::info!("Project gravity: {}", projected.transpose());

                let inference::StateStore::Operate(op_model) = ss
                    .model_manager
                    .as_mut()
                    .get_state_pinned()
                    .expect("Model Manager should be in operate state")
                else {
                    return StateTransitionResult {
                        state: StateStore::Reset(Reset {
                            shared_state: self.shared_state,
                        }),
                        result: Err(io::Error::other("Model manager is not in Operate state")),
                    };
                };

                trace!("Reading took {:?}", start_time.elapsed());
                let policy_stamp = std::time::Instant::now();
                op_model.step_controller(ss.robot_description);
                trace!("Policy step took {:?}", policy_stamp.elapsed());

                // print out the commands
                // for (id, act_state) in ss.robot_description.actuators.actuator_states.iter() {
                //     if (id == ActuatorId::Rkp) {
                //         log::info!("Actuator {:?} command: {:?}", id, act_state.command);
                //
                //         log::info!("Actuator {:?} feedback: {:?}", id, act_state.feedback);
                //         // calculate next torque
                //         let ep = act_state.command.qpos - act_state.feedback.qpos;
                //         let ev = act_state.command.qvel - act_state.feedback.qvel;
                //         let kd = act_state.command.kd;
                //         let kp = act_state.command.kp;

                //         log::info!("Actuator {:?} ep: {:?} ev: {:?} expected qfrc {:?}", id, ep, ev, ep * kp + ev * kd);
                //     }
                // }

                // send command to buses and wait for responses
                let send_stamp = std::time::Instant::now();
                let act_states = &mut ss.robot_description.actuators;
                if let Err(e) = op_act_manager.send_command(act_states).await {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }
                if let Err(e) = op_act_manager.process_feedback(act_states).await {
                    return StateTransitionResult {
                        state: StateStore::Home(Home::new(self.shared_state)),
                        result: Err(e),
                    };
                }
                trace!("Send command took {:?}", send_stamp.elapsed());
                trace!("UDP update took {:?}", udp_update_time);
                trace!("iteration took {:?}", total_iteration_time);
            }
        }
    }
}

#[pin_project]
pub struct BehaviorManager {
    state: Option<StateStore>,
    target: Option<StateTag>,

    #[pin]
    pending_fut: Option<StateFut>,
}

impl Default for BehaviorManager {
    fn default() -> Self {
        Self::new()
    }
}

impl BehaviorManager {
    pub fn new() -> Self {
        Self {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new()),
            })),
            target: None,
            pending_fut: None,
        }
    }
}

impl std::fmt::Debug for BehaviorManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BehaviorManager")
            .field("state", &self.state)
            .field("target", &self.target)
            .finish()
    }
}

impl Stream for BehaviorManager {
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
