use crossterm::event::{KeyCode, KeyEvent};
use enum_dispatch::enum_dispatch;
use enum_map::{Enum, EnumMap};
use ndarray::ArrayViewMut1;

use tracing::info;

#[enum_dispatch(InputState)]
#[derive(Debug)]
pub enum CommandType {
    JoystickInputState,
    SimpleJoystickInputState,
    ControlVectorInputState,
    ExpandedControlVectorInputState,
    BartControlVectorInputState,
    UdpControlVectorInputState(crate::udp_command::UdpControlVectorInputState),
    Udp16ControlVectorInputState(crate::udp_command::Udp16ControlVectorInputState),
}

#[enum_dispatch]
pub trait InputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()>;
    fn extract(&mut self, arr: ArrayViewMut1<f32>) -> std::io::Result<()>;
    fn extract_with_robot(&mut self, arr: ArrayViewMut1<f32>, _robot_description: &crate::robot_description::RobotDescription) -> std::io::Result<()> {
        // Default implementation calls the regular extract method
        self.extract(arr)
    }
}

impl CommandType {
    pub fn from_dims(dims: usize) -> std::io::Result<Self> {
        match dims {
            4 => Ok(CommandType::SimpleJoystickInputState(
                SimpleJoystickInputState::new(),
            )),
            3 => {
                log::info!("Joystick: UdpBasic");
                Ok(CommandType::UdpControlVectorInputState(
                    crate::udp_command::UdpControlVectorInputState::new(),
                ))
            },
            /*3 => Ok(CommandType::ControlVectorInputState(
                ControlVectorInputState::new(),
            )),*/
            6 => Ok(CommandType::ExpandedControlVectorInputState(
                ExpandedControlVectorInputState::new(),
            )),
            7 => Ok(CommandType::BartControlVectorInputState(
                BartControlVectorInputState::new(),
            )),
            16 => {
                log::info!("Joystick: UdpExtended (Bart!)");
                Ok(CommandType::Udp16ControlVectorInputState(
                    crate::udp_command::Udp16ControlVectorInputState::new(),
                ))
            },
            _ => Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Unsupported input state dimensions: {dims}"),
            )),
        }
    }
}

#[derive(Debug)]
pub struct JoystickInputState {
    one_hot_index: usize,
}

#[derive(Debug)]
pub struct SimpleJoystickInputState {
    one_hot_index: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Enum)]
enum ControlVectorDType {
    XVel,
    YVel,
    YawRate,
    Yaw,
    Height,
    Pitch,
    Roll,
    KeyframeIndex,
}

#[derive(Debug)]
pub struct ControlVectorInputState {
    cmds: EnumMap<ControlVectorDType, f32>,
    step_size: f32,
}

#[derive(Debug)]
pub struct ExpandedControlVectorInputState {
    cmds: EnumMap<ControlVectorDType, f32>,
    step_size: f32,
}

#[derive(Debug)]
pub struct BartControlVectorInputState {
    cmds: EnumMap<ControlVectorDType, f32>,
    step_size: f32,
    last_update_time: Option<std::time::Instant>,
}

impl JoystickInputState {
    fn new() -> Self {
        Self { one_hot_index: 0 }
    }
}

impl InputState for JoystickInputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()> {
        match key.code {
            KeyCode::Char('w') => self.one_hot_index = 1,
            KeyCode::Char('s') => self.one_hot_index = 2,
            KeyCode::Char('a') => self.one_hot_index = 5,
            KeyCode::Char('d') => self.one_hot_index = 6,
            KeyCode::Char('q') => self.one_hot_index = 3,
            KeyCode::Char('e') => self.one_hot_index = 4,
            _ => {} // Ignore other keys
        }
        Ok(())
    }

    fn extract(&mut self, mut arr: ArrayViewMut1<f32>) -> std::io::Result<()> {
        // Clear array and set one-hot encoding
        for val in arr.iter_mut() {
            *val = 0.0;
        }
        if self.one_hot_index < arr.len() {
            arr[self.one_hot_index] = 1.0;
        }
        Ok(())
    }
}

impl SimpleJoystickInputState {
    fn new() -> Self {
        Self { one_hot_index: 0 }
    }
}

impl InputState for SimpleJoystickInputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()> {
        match key.code {
            KeyCode::Char('w') => self.one_hot_index = 1,
            KeyCode::Char('s') => self.one_hot_index = 2,
            KeyCode::Char('a') => self.one_hot_index = 3,
            KeyCode::Char('d') => self.one_hot_index = 0,
            _ => {} // Ignore other keys
        }
        Ok(())
    }

    fn extract(&mut self, mut arr: ArrayViewMut1<f32>) -> std::io::Result<()> {
        // Clear array and set one-hot encoding
        for val in arr.iter_mut() {
            *val = 0.0;
        }
        if self.one_hot_index < arr.len() {
            arr[self.one_hot_index] = 1.0;
        }
        Ok(())
    }
}

impl ControlVectorInputState {
    fn new() -> Self {
        Self {
            cmds: EnumMap::default(),
            step_size: 0.1,
        }
    }
}

impl InputState for ControlVectorInputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()> {
        match key.code {
            KeyCode::Char('w') => self.cmds[ControlVectorDType::XVel] += self.step_size,
            KeyCode::Char('s') => self.cmds[ControlVectorDType::XVel] -= self.step_size,
            KeyCode::Char('a') => self.cmds[ControlVectorDType::YVel] -= self.step_size,
            KeyCode::Char('d') => self.cmds[ControlVectorDType::YVel] += self.step_size,
            KeyCode::Char('q') => self.cmds[ControlVectorDType::YawRate] -= self.step_size,
            KeyCode::Char('e') => self.cmds[ControlVectorDType::YawRate] += self.step_size,
            _ => {} // Ignore other keys
        }
        Ok(())
    }

    fn extract(&mut self, mut arr: ArrayViewMut1<f32>) -> std::io::Result<()> {
        arr[0] = self.cmds[ControlVectorDType::XVel];
        arr[1] = self.cmds[ControlVectorDType::YVel];
        arr[2] = self.cmds[ControlVectorDType::YawRate];
        Ok(())
    }
}

impl ExpandedControlVectorInputState {
    fn new() -> Self {
        Self {
            cmds: EnumMap::default(),
            step_size: 0.1,
        }
    }
}

impl InputState for ExpandedControlVectorInputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()> {
        match key.code {
            KeyCode::Char('w') => self.cmds[ControlVectorDType::XVel] += self.step_size,
            KeyCode::Char('s') => self.cmds[ControlVectorDType::XVel] -= self.step_size,
            KeyCode::Char('a') => self.cmds[ControlVectorDType::YVel] -= self.step_size,
            KeyCode::Char('d') => self.cmds[ControlVectorDType::YVel] += self.step_size,
            KeyCode::Char('q') => self.cmds[ControlVectorDType::Yaw] -= self.step_size,
            KeyCode::Char('e') => self.cmds[ControlVectorDType::Yaw] += self.step_size,
            KeyCode::Char('r') => self.cmds[ControlVectorDType::Roll] += self.step_size,
            KeyCode::Char('f') => self.cmds[ControlVectorDType::Roll] -= self.step_size,
            KeyCode::Char('t') => self.cmds[ControlVectorDType::Pitch] += self.step_size,
            KeyCode::Char('g') => self.cmds[ControlVectorDType::Pitch] -= self.step_size,
            _ => {} // Ignore other keys
        }
        Ok(())
    }

    fn extract(&mut self, mut arr: ArrayViewMut1<f32>) -> std::io::Result<()> {
        arr[0] = self.cmds[ControlVectorDType::XVel];
        arr[1] = self.cmds[ControlVectorDType::YVel];
        arr[2] = self.cmds[ControlVectorDType::Yaw];
        arr[3] = self.cmds[ControlVectorDType::Height];
        arr[4] = self.cmds[ControlVectorDType::Pitch];
        arr[5] = self.cmds[ControlVectorDType::Roll];
        Ok(())
    }
}

impl BartControlVectorInputState {
    fn new() -> Self {
        Self {
            cmds: EnumMap::default(),
            step_size: 0.1,
            last_update_time: None,
        }
    }
}

impl InputState for BartControlVectorInputState {
    fn update(&mut self, key: KeyEvent) -> std::io::Result<()> {
        log::info!("Updating BartControlVectorInputState with key: {key:?}");
        match key.code {
            KeyCode::Char('w') => self.cmds[ControlVectorDType::XVel] += self.step_size,
            KeyCode::Char('s') => self.cmds[ControlVectorDType::XVel] -= self.step_size,
            KeyCode::Char('a') => self.cmds[ControlVectorDType::YVel] -= self.step_size,
            KeyCode::Char('d') => self.cmds[ControlVectorDType::YVel] += self.step_size,
            KeyCode::Char('q') => self.cmds[ControlVectorDType::YawRate] -= self.step_size,
            KeyCode::Char('e') => self.cmds[ControlVectorDType::YawRate] += self.step_size,
            KeyCode::Char('r') => self.cmds[ControlVectorDType::Roll] += self.step_size,
            KeyCode::Char('f') => self.cmds[ControlVectorDType::Roll] -= self.step_size,
            KeyCode::Char('t') => self.cmds[ControlVectorDType::Pitch] += self.step_size,
            KeyCode::Char('g') => self.cmds[ControlVectorDType::Pitch] -= self.step_size,
            _ => {} // Ignore other keys
        }
        Ok(())
    }

    fn extract(&mut self, mut arr: ArrayViewMut1<f32>) -> std::io::Result<()> {
        // get time since last update as seconds in f32
        if let Some(last_time) = self.last_update_time {
            let now = std::time::Instant::now();
            let elapsed = now.duration_since(last_time).as_secs_f32();
            self.cmds[ControlVectorDType::Yaw] += self.cmds[ControlVectorDType::YawRate] * elapsed;
        } else {
            self.last_update_time = Some(std::time::Instant::now());
            self.cmds[ControlVectorDType::Yaw] = 0.0; // Initialize yaw if no previous time
        }
        arr[0] = self.cmds[ControlVectorDType::XVel];
        arr[1] = self.cmds[ControlVectorDType::YVel];
        arr[2] = self.cmds[ControlVectorDType::YawRate];
        arr[3] = self.cmds[ControlVectorDType::Yaw];
        arr[4] = self.cmds[ControlVectorDType::Height];
        arr[5] = self.cmds[ControlVectorDType::Pitch];
        arr[6] = self.cmds[ControlVectorDType::Roll];
        Ok(())
    }
}
