
use crate::robot_description::{
    RobotDescription,
    JoystickCommand,
    JoystickCommandType,
};
use crossterm::{
    terminal,
    event,
};

use std::io;
use std::process;
use std::time::Duration;
pub struct KeyboardManager {
}

impl KeyboardManager {
    pub fn new() -> io::Result<Self> {
        terminal::enable_raw_mode()?;
        Ok(KeyboardManager {})
    }

    pub fn process_feedback(
        &self,
        joystick_command: &mut JoystickCommand,
    ) -> std::io::Result<()> {
        self.drain_keyboard_events(joystick_command)
    }

    fn drain_keyboard_events(&self, cur_cmd: &mut JoystickCommand) -> io::Result<()> {
        // drain the buffered events
        let mut update = &mut cur_cmd.cmd_map;
        while event::poll(Duration::from_millis(0))? {
            if let event::Event::Key(key) = event::read()? {
                if key.kind == event::KeyEventKind::Press {
                    match key.code {
                        event::KeyCode::Char('c') if key.modifiers.contains(event::KeyModifiers::CONTROL) => {
                            terminal::disable_raw_mode().unwrap_or(());
                            process::exit(130);
                        }
                        event::KeyCode::Esc | event::KeyCode::Char('x') => {
                            terminal::disable_raw_mode().unwrap_or(());
                            process::exit(0);
                        }
                        event::KeyCode::Char('w') => {
                            update[JoystickCommandType::XVel] = 0.2;
                        }
                        event::KeyCode::Char('s') => {
                            update[JoystickCommandType::XVel] = -0.2;
                        }
                        event::KeyCode::Char('a') => {
                            update[JoystickCommandType::YVel] = 0.2;
                        }
                        event::KeyCode::Char('d') => {
                            update[JoystickCommandType::YVel] = -0.2;
                        }
                        // yaw rate is added to the current yaw rate
                        event::KeyCode::Char('q') => {
                            update[JoystickCommandType::Yaw] += 0.1;
                            update[JoystickCommandType::YawRate] = 0.1;
                        }
                        event::KeyCode::Char('e') => {
                            update[JoystickCommandType::Yaw] += -0.1;
                            update[JoystickCommandType::YawRate] = -0.1;
                        }
                        event::KeyCode::Char('r') => {
                            update[JoystickCommandType::Roll] += 0.1;
                        }
                        event::KeyCode::Char('f') => {
                            update[JoystickCommandType::Roll] += -0.1;
                        }
                        event::KeyCode::Char('t') => {
                            update[JoystickCommandType::Pitch] += 0.1;
                        }
                        event::KeyCode::Char('g') => {
                            update[JoystickCommandType::Pitch] += -0.1;
                        }
                        event::KeyCode::Char('2') => {
                            // reset everything to 0
                            for cmd in update.values_mut() {
                                *cmd = 0.0;
                            }
                        }
                        event::KeyCode::Char('6') => {
                            update[JoystickCommandType::KeyframeIndex] = 6.0;
                        }
                        event::KeyCode::Char('7') => {
                            update[JoystickCommandType::KeyframeIndex] = 7.0;
                        }
                        event::KeyCode::Char('8') => {
                            update[JoystickCommandType::KeyframeIndex] = 8.0;
                        }
                        event::KeyCode::Char('9') => {
                            update[JoystickCommandType::KeyframeIndex] = 9.0;
                        }
                        _ => {}
                    }
                } else if key.kind == event::KeyEventKind::Release {
                    // on release, we just set the velocities to 0
                    match key.code {
                        event::KeyCode::Char('w') | event::KeyCode::Char('s') => {
                            update[JoystickCommandType::XVel] = 0.0;
                        }
                        event::KeyCode::Char('a') | event::KeyCode::Char('d') => {
                            update[JoystickCommandType::YVel] = 0.0;
                        }
                        event::KeyCode::Char('q') | event::KeyCode::Char('e') => {
                            update[JoystickCommandType::YawRate] = 0.0;
                        }
                        _ => {}
                    }
                }
            }
        }
        Ok(())
    }
}















