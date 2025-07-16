use crate::robot_description::{
    ActuatorState,
    ActuatorId,
    ActuatorCommand,
    ActuatorFeedback,
    ActuatorFeedbackEpsilon,
    ActuatorStateStore,
    normalize_actuator_qpos,
};

use tracing::{info, warn, error};

use enum_map::EnumMap;

use approx::{
    abs_diff_eq,
    AbsDiffEq,
};

use std::task::Poll;

#[derive(Debug, Clone, PartialEq)]
pub struct Waypoint {
    pub feedbacks: EnumMap<ActuatorId, ActuatorFeedback>,
}

impl AbsDiffEq for Waypoint {
    type Epsilon = ActuatorFeedbackEpsilon;

    fn default_epsilon() -> Self::Epsilon {
        ActuatorFeedbackEpsilon::default()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.feedbacks.iter().all(|(id, val)| {
            abs_diff_eq!(val, &other.feedbacks[id], epsilon = epsilon.clone())
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum WaypointTraversal {
    Position,
    Velocity,
}

pub struct BoundedSegment {
    pub target: Waypoint,
    pub epsilon: ActuatorFeedbackEpsilon,
    pub traversal: WaypointTraversal,
}

impl BoundedSegment {
    pub fn new(target: Waypoint, traversal: WaypointTraversal) -> Self {
        BoundedSegment {
            target,
            traversal,
            epsilon: ActuatorFeedbackEpsilon::default(),
        }
    }
}

// we drive this segment blindly, to infinity
pub type UnboundedSegment = EnumMap<ActuatorId, ActuatorCommand>;

/**
* A trajectory is a sequence of segments that the robot will follow.
*
* Each segment can either be:
* 1. A hard stop segment, which commands all actuators to drive blind
* 2. A waypoint segment, which commands each actuator to a specific target position or velocity.
*
*/
pub enum TrajectorySegment {
    Unbounded(UnboundedSegment),
    Bounded(BoundedSegment),
}

pub struct Trajectory {
    segments: Vec<TrajectorySegment>,
    segidx: Option<usize>, // index of the current segment being driven, None if no more remaining segments
}

impl Trajectory {

    // must have at least one segment
    pub fn new() -> Self {
        Trajectory {
            segments: Vec::new(),
            segidx: None,
        }
    }

    pub fn push(&mut self, segment: TrajectorySegment) {
        self.segments.push(segment);
        if self.segidx.is_none() {
            self.segidx = Some(0); // start with the first segment
        }
    }

    pub fn segments(&self) -> &Vec<TrajectorySegment> {
        &self.segments
    }

    pub fn segments_mut(&mut self) -> &mut Vec<TrajectorySegment> {
        &mut self.segments
    }

    pub fn drive(&mut self, states: &mut ActuatorStateStore) -> Poll<()> {
        if self.segidx.is_none() {
            return Poll::Ready(());
        }

        let idx = self.segidx.unwrap();

        match self.segments[idx] {
            TrajectorySegment::Unbounded(ref segment) => {
                for (dst, src) in states.actuator_states.values_mut().zip(segment.values()) {
                    // drive all actuators blindly to the target
                    dst.command = src.clone();
                    // we don't care about feedback here, we just drive blindly
                    dst.feedback = ActuatorFeedback::default();
                }
                Poll::Pending
            }
            TrajectorySegment::Bounded(ref segment) => {
                // check if we have reached the target
                let reached = segment.target.feedbacks.iter().all(|(id, val)| {
                    abs_diff_eq!(val, &states.actuator_states[id].feedback, epsilon = segment.epsilon.clone())
                });

                if reached {
                    info!("Reached target for segment {}", idx);
                    // we have reached the target, move to the next segment
                    self.segidx = if idx + 1 < self.segments.len() {
                        Some(idx + 1)
                    } else {
                        None // no more segments
                    };
                    if self.segidx.is_none() {
                        return Poll::Ready(()); // no more segments to drive
                    }
                } else {
                    for (act_id, act_state) in states.actuator_states.iter_mut() {

                        if act_id == ActuatorId::Rwr || act_id == ActuatorId::Lwr {
                            // these are not homed, so we skip them
                            continue;
                        }

                        let target = &segment.target.feedbacks[act_id];
                        let mut normalized_qpos = normalize_actuator_qpos(act_state.feedback.qpos);
                        let err_qpos = target.qpos - normalized_qpos;
                        // This can be refactored
                        // Let WaypointTraversal be decoupled from the segment (i.e remove the traversal field)
                        // Let it be standalond struct / enum that implements enum dispatch to a
                        // set of WaypointTraveral structs. Each struct provides the below
                        // implmeentation, where it takes the current feedback and target feedback,
                        // and produdces an actuator command.
                        //
                        // i.e make a WaypointTraversal trait with a method called
                        // traverse(Waypoint, Waypoint) -> EnumMap<ActuatorId, ActuatorCommand>
                        //
                        // Then we can do 
                        // WaypointTraversal::Position.traverse(target, act_state.feedback)
                        // WaypointTraversal::Velocity.traverse(target, act_state.feedback)
                        // this is more modular, and decouples traversal from segment and
                        // trajectory logic
                        match segment.traversal {
                            WaypointTraversal::Position => {
                                let step = (err_qpos).clamp(-4.0f64.to_radians(), 4.0f64.to_radians());
                                // drive to the target position
                                act_state.command.qpos = act_state.feedback.qpos + step;
                                act_state.command.qvel = 0.0; // no velocity
                                act_state.command.qfrc = 0.0; // no force
                                act_state.command.kp = target.kp / 2.0; // proportional gain
                                act_state.command.kd = target.kd; // derivative gain
                            }
                            WaypointTraversal::Velocity => {
                                // drive to the target using velocity control
                                // 10 degrees per second is the maximum velocity
                                let step = (err_qpos).signum() * 10f64.to_radians();
                                act_state.command.qpos = 0.0; // no position command
                                act_state.command.qvel = step; // fixed velocity
                                act_state.command.qfrc = 0.0; // no force
                                act_state.command.kp = 0.0; // proportional gain
                                act_state.command.kd = target.kd; // derivative gain
                            }
                        }
                        warn!("Actuator {:?} command: {}, feedback: {}, error: {}", act_id, act_state.command.qpos, normalized_qpos, err_qpos);
                    }
                }
                Poll::Pending // not yet reached the target, continue driving
            }
        }
    }
}

