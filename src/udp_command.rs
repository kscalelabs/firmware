use std::{
    io,
    net::SocketAddr,
    time::Duration,
};
use tokio::net::UdpSocket;
use serde::{Deserialize, Serialize};
use tracing::{debug, error, warn, info};

/// Unified UDP manager that handles both 3D and 16D command formats
#[derive(Debug)]
pub struct UnifiedUdpManager {
    socket: UdpSocket,
    current_command: UdpExtendedCommand,
    last_command_time: Option<std::time::Instant>,
    command_timeout: Duration,
}

impl UnifiedUdpManager {
    /// Create a UDP manager that can handle both 3D and 16D commands
    pub async fn new(port: u16, _use_extended: bool) -> io::Result<Self> {
        let addr = SocketAddr::from(([0, 0, 0, 0], port));
        let std_socket = socket2::Socket::new(
            socket2::Domain::IPV4, socket2::Type::DGRAM, Some(socket2::Protocol::UDP)
        )?;
        std_socket.set_recv_buffer_size(1024)?;
        std_socket.set_nonblocking(true)?;
        std_socket.bind(&addr.into())?;
        let std_socket: std::net::UdpSocket = std_socket.into();
        let socket = UdpSocket::from_std(std_socket)?;
        debug!("UDP extended command manager listening on port {}", port);
        Ok(Self {
            socket,
            current_command: UdpExtendedCommand::default(),
            last_command_time: None,
            command_timeout: Duration::from_millis(500),
        })
    }

    /// Update commands from UDP buffer
    pub async fn try_update_command(&mut self) -> io::Result<bool> {
        let mut buf = [0u8; 1024];
        let mut latest: Option<UdpExtendedCommand> = None;
        let mut packets_read = 0;
        loop {
            match self.socket.try_recv(&mut buf) {
                Ok(len) => {
                    packets_read += 1;
                    
                    // Try to parse as extended command first
                    if let Ok(cmd) = serde_json::from_slice::<UdpExtendedCommand>(&buf[..len]) {
                        latest = Some(cmd);
                        debug!("Parsed extended UDP command #{} (x={}, y={}, yaw_rate={})",
                               packets_read, cmd.x, cmd.y, cmd.yaw_rate);
                    }
                    // Fall back to basic command format
                    else if let Ok(basic_cmd) = serde_json::from_slice::<UdpCommand>(&buf[..len]) {
                        // Convert basic command to extended format
                        let extended_cmd = UdpExtendedCommand {
                            x: basic_cmd.x,
                            y: basic_cmd.y,
                            yaw_rate: basic_cmd.yaw,  // Convert yaw to yaw_rate
                            ..Default::default()  // All other fields remain zero
                        };
                        latest = Some(extended_cmd);
                        debug!("Parsed basic UDP command #{} (converted to extended): x={}, y={}, yaw={}",
                               packets_read, basic_cmd.x, basic_cmd.y, basic_cmd.yaw);
                    }
                    else {
                        warn!("Failed to parse UDP command JSON (packet #{}) - not valid UdpExtendedCommand or UdpCommand format", packets_read);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => { error!("UDP socket error: {}", e); return Err(e); }
            }
        }
        if let Some(cmd) = latest {
            if packets_read > 1 { debug!("Drained {} UDP packets", packets_read); }
            self.current_command = cmd;
            self.last_command_time = Some(std::time::Instant::now());
            Ok(true)
        } else { Ok(false) }
    }

    /// Get the current command, applying timeout logic
    pub fn get_current_command(&self) -> UdpExtendedCommand {
        if let Some(last_time) = self.last_command_time {
            if last_time.elapsed() > self.command_timeout {
                // Command has timed out, return zero command
                debug!("UDP command timed out, returning zero command");
                UdpExtendedCommand::default()
            } else {
                self.current_command
            }
        } else {
            // No command received yet
            UdpExtendedCommand::default()
        }
    }

    /// Check if we have recent commands
    pub fn has_recent_command(&self) -> bool {
        if let Some(last_time) = self.last_command_time {
            last_time.elapsed() <= self.command_timeout
        } else {
            false
        }
    }

    /// Update robot description with current command
    pub fn update_robot_description(&self, robot_description: &mut crate::robot_description::RobotDescription) {
        let cmd = self.get_current_command();
        robot_description.udp_command_state = cmd;
    }

    /// Clear robot description command state (on timeout)
    pub fn clear_robot_description(&self, robot_description: &mut crate::robot_description::RobotDescription) {
        robot_description.udp_command_state = Default::default();
    }
}



#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct UdpCommand {
    #[serde(rename = "XVel")]
    pub x: f32,
    #[serde(rename = "YVel")]
    pub y: f32,
    #[serde(rename = "YawRate")]
    pub yaw: f32,
}

impl Default for UdpCommand {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        }
    }
}



/// UDP Command State for policy control integration
#[derive(Debug)]
pub struct UdpControlVectorInputState {
    udp_manager: Option<UnifiedUdpManager>,
    last_command: UdpCommand,
}

impl UdpControlVectorInputState {
    pub fn new() -> Self {
        Self {
            udp_manager: None,
            last_command: UdpCommand::default(),
        }
    }

    pub async fn initialize(&mut self, port: u16) -> io::Result<()> {
        self.udp_manager = Some(UnifiedUdpManager::new(port, false).await?);
        Ok(())
    }

    pub async fn update_from_udp(&mut self) -> io::Result<()> {
        if let Some(ref mut manager) = self.udp_manager {
            manager.try_update_command().await?;
            // Convert extended command back to basic format for compatibility
            let extended_cmd = manager.get_current_command();
            self.last_command = UdpCommand {
                x: extended_cmd.x,
                y: extended_cmd.y,
                yaw: extended_cmd.yaw_rate,
            };
        }
        Ok(())
    }
}

impl crate::policy_control::InputState for UdpControlVectorInputState {
    fn update(&mut self, _key: crossterm::event::KeyEvent) -> std::io::Result<()> {
        // For UDP control, we ignore keyboard events
        Ok(())
    }

    fn extract(&mut self, mut arr: ndarray::ArrayViewMut1<f32>) -> std::io::Result<()> {
        // Extract UDP command into the policy array
        // Assuming 3D control vector: [x_vel, y_vel, yaw_rate]
        if arr.len() >= 3 {
            arr[0] = self.last_command.x;
            arr[1] = self.last_command.y;
            arr[2] = self.last_command.yaw;
        }
        debug!("UDP command: x={}, y={}, yaw={}", self.last_command.x, self.last_command.y, self.last_command.yaw);
        Ok(())
    }

    fn extract_with_robot(&mut self, mut arr: ndarray::ArrayViewMut1<f32>, robot_description: &crate::robot_description::RobotDescription) -> std::io::Result<()> {
        // Extract UDP command from robot description
        let udp_state = &robot_description.udp_command_state;
        if arr.len() >= 3 {
            arr[0] = udp_state.x;
            arr[1] = udp_state.y;
            arr[2] = udp_state.yaw_rate;  // Now using yaw_rate from unified structure
        }
        debug!("UDP command from robot_description: x={}, y={}, yaw_rate={}", udp_state.x, udp_state.y, udp_state.yaw_rate);
        Ok(())
    }
}

impl UdpControlVectorInputState {
    /// Update the command directly (called from behavior loop)
    pub fn set_command(&mut self, command: UdpCommand) {
        self.last_command = command;
    }
}


#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct UdpExtendedCommand {
    // 0..2
    #[serde(rename = "XVel")]
    pub x: f32,
    #[serde(rename = "YVel")]
    pub y: f32,
    #[serde(rename = "YawRate")]
    pub yaw_rate: f32,

    // 3..5
    #[serde(rename = "BaseHeight")]
    pub base_height: f32,
    #[serde(rename = "BaseRoll")]
    pub base_roll: f32,
    #[serde(rename = "BasePitch")]
    pub base_pitch: f32,

    // 6..10 right arm
    #[serde(rename = "RShoulderPitch")]
    pub r_shoulder_pitch: f32,
    #[serde(rename = "RShoulderRoll")]
    pub r_shoulder_roll: f32,
    #[serde(rename = "RElbowPitch")]
    pub r_elbow_pitch: f32,
    #[serde(rename = "RElbowRoll")]
    pub r_elbow_roll: f32,
    #[serde(rename = "RWristPitch")]
    pub r_wrist_pitch: f32,

    // 11..15 left arm
    #[serde(rename = "LShoulderPitch")]
    pub l_shoulder_pitch: f32,
    #[serde(rename = "LShoulderRoll")]
    pub l_shoulder_roll: f32,
    #[serde(rename = "LElbowPitch")]
    pub l_elbow_pitch: f32,
    #[serde(rename = "LElbowRoll")]
    pub l_elbow_roll: f32,
    #[serde(rename = "LWristPitch")]
    pub l_wrist_pitch: f32,
}

impl Default for UdpExtendedCommand {
    fn default() -> Self {
        Self {
            x: 0.0, y: 0.0, yaw_rate: 0.0,
            base_height: 0.0, base_roll: 0.0, base_pitch: 0.0,
            r_shoulder_pitch: 0.0, r_shoulder_roll: 0.0, r_elbow_pitch: 0.0, r_elbow_roll: 0.0, r_wrist_pitch: 0.0,
            l_shoulder_pitch: 0.0, l_shoulder_roll: 0.0, l_elbow_pitch: 0.0, l_elbow_roll: 0.0, l_wrist_pitch: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct Udp16ControlVectorInputState {
    udp_manager: Option<UnifiedUdpManager>,
    last_command: UdpExtendedCommand,
}

impl Udp16ControlVectorInputState {
    pub fn new() -> Self {
        Self { udp_manager: None, last_command: UdpExtendedCommand::default() }
    }
    pub async fn initialize(&mut self, port: u16) -> io::Result<()> {
        self.udp_manager = Some(UnifiedUdpManager::new(port, true).await?);
        Ok(())
    }
    pub async fn update_from_udp(&mut self) -> io::Result<()> {
        if let Some(ref mut m) = self.udp_manager {
            m.try_update_command().await?;
            self.last_command = m.get_current_command();
        }
        Ok(())
    }
}

impl crate::policy_control::InputState for Udp16ControlVectorInputState {
    fn update(&mut self, _key: crossterm::event::KeyEvent) -> std::io::Result<()> {
        // No keyboard; UDP-only.
        Ok(())
    }
    fn extract(&mut self, mut arr: ndarray::ArrayViewMut1<f32>) -> std::io::Result<()> {
        if arr.len() < 16 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "expected arr.len() >= 16"));
        }
        let c = self.last_command;
        arr[0] = c.x;               // x linear velocity [m/s]
        arr[1] = c.y;               // y linear velocity [m/s]
        arr[2] = c.yaw_rate;        // z angular velocity [rad/s]
        arr[3] = c.base_height;     // base height offset [m]
        arr[4] = c.base_roll;       // base roll [rad]
        arr[5] = c.base_pitch;      // base pitch [rad]
        arr[6] = c.r_shoulder_pitch;
        arr[7] = c.r_shoulder_roll;
        arr[8] = c.r_elbow_pitch;
        arr[9] = c.r_elbow_roll;
        arr[10] = c.r_wrist_pitch;
        arr[11] = c.l_shoulder_pitch;
        arr[12] = c.l_shoulder_roll;
        arr[13] = c.l_elbow_pitch;
        arr[14] = c.l_elbow_roll;
        arr[15] = c.l_wrist_pitch;
        info!("16D UDP command: x={}, y={}, yaw={}, base_height={}, r_shoulder_pitch={}", 
              c.x, c.y, c.yaw_rate, c.base_height, c.r_shoulder_pitch);
        Ok(())
    }
    fn extract_with_robot(&mut self, mut arr: ndarray::ArrayViewMut1<f32>, robot_description: &crate::robot_description::RobotDescription) -> std::io::Result<()> {
        // Extract UDP command from unified robot description state
        let cmd_state = &robot_description.udp_command_state;
        if arr.len() >= 16 {
            arr[0] = cmd_state.x;
            arr[1] = cmd_state.y;
            arr[2] = cmd_state.yaw_rate;
            arr[3] = cmd_state.base_height;
            arr[4] = cmd_state.base_roll;
            arr[5] = cmd_state.base_pitch;
            arr[6] = cmd_state.r_shoulder_pitch;
            arr[7] = cmd_state.r_shoulder_roll;
            arr[8] = cmd_state.r_elbow_pitch;
            arr[9] = cmd_state.r_elbow_roll;
            arr[10] = cmd_state.r_wrist_pitch;
            arr[11] = cmd_state.l_shoulder_pitch;
            arr[12] = cmd_state.l_shoulder_roll;
            arr[13] = cmd_state.l_elbow_pitch;
            arr[14] = cmd_state.l_elbow_roll;
            arr[15] = cmd_state.l_wrist_pitch;
        }
        info!("16D UDP command from robot_description: x={}, y={}, yaw_rate={}, base_height={}", 
              cmd_state.x, cmd_state.y, cmd_state.yaw_rate, cmd_state.base_height);
        Ok(())
    }
}