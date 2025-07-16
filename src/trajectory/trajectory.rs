use crate::robot_description::{
    ActuatorState,
    ActuatorId,
    ActuatorCommand,
    ActuatorFeedback,
    ActuatorFeedbackEpsilon,
    ActuatorStateStore,
    normalize_actuator_qpos,
};

use crate::trajectory::traversal::WaypointTraversal;

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

    /// used to skip the current segment and move to the next one
    /// Useful with Unbounded segments, where we don't care about the feedback
    pub fn skip(&mut self) {
        if let Some(idx) = self.segidx {
            if idx + 1 < self.segments.len() {
                self.segidx = Some(idx + 1);
            } else {
                self.segidx = None; // no more segments to drive
            }
        }
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
                    if id == ActuatorId::Rwr || id == ActuatorId::Lwr {
                        // these are not homed, so we skip them
                        return true;
                    }
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
                    let cur_waypoint = Waypoint {
                        feedbacks: states.actuator_states.iter().map(|(id, state)| (id, state.feedback.clone())).collect(),
                    };

                    segment.traversal.traverse(
                        &cur_waypoint,
                        &segment.target,
                    ).into_iter().for_each(|(act_id, cmd)| {
                        states.actuator_states[act_id].command = cmd;
                    });
                }
                Poll::Pending // not yet reached the target, continue driving
            }
        }
    }
}

