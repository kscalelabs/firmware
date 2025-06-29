use crossterm::{
    terminal,
    event,
};

use std::io;
use std::process;
use std::time::Duration;
pub struct KeyboardManager {
}

use crate::policy_control::{
    CommandType,
    InputState,
};

impl KeyboardManager {
    pub fn new() -> io::Result<Self> {
        terminal::enable_raw_mode()?;
        Ok(KeyboardManager {})
    }

    pub fn process_feedback(
        &self,
        cmd: &mut impl InputState,
    ) -> std::io::Result<()> {
        // drain the buffered events
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
                        _ => {
                            cmd.update(key)?;
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
