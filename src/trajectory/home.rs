use crate::trajectory::trajectory::Waypoint;
use crate::robot_description::{
    ActuatorId,
    ActuatorFeedback,
};
use enum_map::enum_map;
use std::sync::LazyLock;

pub static HOME_WAYPOINT: LazyLock<Waypoint> = LazyLock::new(|| Waypoint {
    feedbacks: enum_map! {
        ActuatorId::Lsp => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
        ActuatorId::Lsr => ActuatorFeedback { qpos: (10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
        ActuatorId::Lsy => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 2.945, ..Default::default() },
        ActuatorId::Lep => ActuatorFeedback { qpos: (-90.0_f64).to_radians(), kp: 80.0, kd: 2.266, ..Default::default() },
        ActuatorId::Lwr => ActuatorFeedback { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

        ActuatorId::Rsp => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 8.284, ..Default::default() },
        ActuatorId::Rsr => ActuatorFeedback { qpos: (-10.0_f64).to_radians(), kp: 100.0, kd: 8.257, ..Default::default() },
        ActuatorId::Rsy => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 2.945, ..Default::default() },
        ActuatorId::Rep => ActuatorFeedback { qpos: (90.0_f64).to_radians(), kp: 100.0, kd: 2.266, ..Default::default() },
        ActuatorId::Rwr => ActuatorFeedback { qpos: 0.0, kp: 20.0, kd: 0.295, ..Default::default() },

        ActuatorId::Lhp => ActuatorFeedback { qpos: (20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
        ActuatorId::Lhr => ActuatorFeedback { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
        ActuatorId::Lhy => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
        ActuatorId::Lkp => ActuatorFeedback { qpos: (50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
        ActuatorId::Lap => ActuatorFeedback { qpos: (-30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },

        ActuatorId::Rhp => ActuatorFeedback { qpos: (-20.0_f64).to_radians(), kp: 150.0, kd: 24.722, ..Default::default() },
        ActuatorId::Rhr => ActuatorFeedback { qpos: 0.0, kp: 200.0, kd: 26.387, ..Default::default() },
        ActuatorId::Rhy => ActuatorFeedback { qpos: 0.0, kp: 100.0, kd: 3.419, ..Default::default() },
        ActuatorId::Rkp => ActuatorFeedback { qpos: (-50.0_f64).to_radians(), kp: 150.0, kd: 8.654, ..Default::default() },
        ActuatorId::Rap => ActuatorFeedback { qpos: (30.0_f64).to_radians(), kp: 40.0, kd: 0.99, ..Default::default() },
    },
});
