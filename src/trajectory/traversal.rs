use crate::trajectory::trajectory::{
    Waypoint,
};

use crate::robot_description::{
    ActuatorId,
    ActuatorCommand,
    normalize_actuator_qpos,
};

use tracing::{info, warn, error};

use enum_map::EnumMap;
use strum::IntoEnumIterator;


#[derive(Debug, Clone, PartialEq)]
pub enum WaypointTraversal {
    Position,
    Velocity,
}

impl WaypointTraversal {

    pub fn traverse(&self, cur: &Waypoint, dst: &Waypoint) -> EnumMap<ActuatorId, ActuatorCommand> {
        let mut ret = EnumMap::<ActuatorId, ActuatorCommand>::default();

        for (((act_id, cur), dst), cmd) in cur.feedbacks.iter().zip(dst.feedbacks.values()).zip(ret.values_mut()) {
            if act_id == ActuatorId::Rwr || act_id == ActuatorId::Lwr {
                // these are not homed, so we skip them
                continue;
            }
            let mut normalized_qpos = normalize_actuator_qpos(cur.qpos);
            let err_qpos = dst.qpos - normalized_qpos;

            match self {
                Self::Position => {
                    let step = (err_qpos).clamp(-4.0f64.to_radians(), 4.0f64.to_radians());
                    // drive to the target position
                    cmd.qpos = cur.qpos + step;
                    cmd.qvel = 0.0; // no velocity
                    cmd.qfrc = 0.0; // no force
                    cmd.kp = dst.kp / 2.0; // proportional gain
                    // cmd.kp = 0.0; // proportional gain
                    cmd.kd = dst.kd; // derivative gain
                }
                Self::Velocity => {
                    // drive to the target using velocity control
                    // 10 degrees per second is the maximum velocity
                    let step = (err_qpos).signum() * 10f64.to_radians();
                    cmd.qpos = 0.0; // no position command
                    cmd.qvel = step; // fixed velocity
                    cmd.qfrc = 0.0; // no force
                    cmd.kp = 0.0; // proportional gain
                    cmd.kd = dst.kd; // derivative gain
                }
            }
            warn!("Actuator {:?} command: {}, feedback: {}, error: {}", act_id, cmd.qpos, normalized_qpos, err_qpos);
        }
        ret
    }
}
