use crossterm::{event, terminal};

use heapless::Deque;

use std::io;
use std::process;
use std::time::Duration;
pub struct KeyboardManager {
    raw_mode_enabled: bool,
}

use crate::policy_control::{CommandType, InputState};

impl Default for KeyboardManager {
    fn default() -> Self {
        Self::new()
    }
}

impl KeyboardManager {
    pub fn new() -> Self {
        KeyboardManager {
            raw_mode_enabled: false,
        }
    }

    pub fn process_feedback<const N: usize>(
        &self,
        pending_events: &mut Deque<event::KeyEvent, N>,
    ) -> std::io::Result<()> {
        // drain the buffered events
        while event::poll(Duration::from_millis(0))? {
            if let event::Event::Key(key) = event::read()?
                && key.kind == event::KeyEventKind::Press
            {
                match key.code {
                    event::KeyCode::Char('c')
                        if key.modifiers.contains(event::KeyModifiers::CONTROL) =>
                    {
                        terminal::disable_raw_mode().unwrap_or(());
                        process::exit(130);
                    }
                    event::KeyCode::Esc | event::KeyCode::Char('x') => {
                        terminal::disable_raw_mode().unwrap_or(());
                        process::exit(0);
                    }
                    _ => {
                        pending_events
                            .push_back(key)
                            .map_err(|_| io::Error::other("Event queue is full"))?;
                    }
                }
            }
        }
        Ok(())
    }

    pub fn enable_raw_mode(&mut self) -> io::Result<()> {
        if !self.raw_mode_enabled {
            terminal::enable_raw_mode()?;
            self.raw_mode_enabled = true;
        }
        Ok(())
    }

    pub fn disable_raw_mode(&mut self) -> io::Result<()> {
        if self.raw_mode_enabled {
            terminal::disable_raw_mode()?;
            self.raw_mode_enabled = false;
        }
        Ok(())
    }

    // For simple line-based input (like "press enter to continue")
    pub async fn wait_for_enter(&mut self) -> io::Result<()> {
        let was_raw_mode = self.raw_mode_enabled;

        // Disable raw mode if it was enabled
        if was_raw_mode {
            self.disable_raw_mode()?;
        }

        // Wait for enter using line-buffered input
        use tokio::io::{self, AsyncBufReadExt};
        let mut lines = io::BufReader::new(io::stdin()).lines();
        lines.next_line().await?;

        // Re-enable raw mode if it was previously enabled
        if was_raw_mode {
            self.enable_raw_mode()?;
        }

        Ok(())
    }
}
