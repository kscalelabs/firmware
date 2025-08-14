use std::{
    io,
    net::SocketAddr,
    time::Duration,
};
use tokio::net::UdpSocket;
use serde::{Deserialize, Serialize};
use tracing::{debug, error, warn, info};



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

pub struct UdpCommandManager {
    socket: UdpSocket,
    current_command: UdpCommand,
    last_command_time: Option<std::time::Instant>,
    command_timeout: Duration,
}

impl std::fmt::Debug for UdpCommandManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("UdpCommandManager")
            .field("current_command", &self.current_command)
            .field("last_command_time", &self.last_command_time)
            .finish()
    }
}

impl UdpCommandManager {
    pub async fn new(port: u16) -> io::Result<Self> {
        let addr = SocketAddr::from(([0, 0, 0, 0], port));
        
        // Create socket with socket2 for better control, then convert to tokio
        let std_socket = socket2::Socket::new(
            socket2::Domain::IPV4, 
            socket2::Type::DGRAM, 
            Some(socket2::Protocol::UDP)
        )?;
        
        // Set small receive buffer to minimize latency and avoid buffering old packets
        std_socket.set_recv_buffer_size(1024)?;
        std_socket.set_nonblocking(true)?;
        std_socket.bind(&addr.into())?;
        
        // Convert to tokio UdpSocket
        let std_socket: std::net::UdpSocket = std_socket.into();
        let socket = UdpSocket::from_std(std_socket)?;
        
        debug!("UDP command manager listening on port {}", port);
        
        Ok(Self {
            socket,
            current_command: UdpCommand::default(),
            last_command_time: None,
            command_timeout: Duration::from_millis(500), // 500ms timeout
        })
    }

    /// Non-blocking method to drain UDP buffer and get the LATEST command
    /// Returns true if a new command was received
    pub async fn try_update_command(&mut self) -> io::Result<bool> {
        let mut buf = [0u8; 1024];
        let mut latest_command: Option<UdpCommand> = None;
        let mut packets_read = 0;
        
        // Drain ALL packets from the UDP buffer, keeping only the latest valid one
        loop {
            match self.socket.try_recv(&mut buf) {
                Ok(len) => {
                    packets_read += 1;
                    
                    // Try to parse this packet
                    match serde_json::from_slice::<UdpCommand>(&buf[..len]) {
                        Ok(command) => {
                            // This is a valid command - keep it as the latest
                            latest_command = Some(command);
                            debug!("Parsed UDP command #{}: x={}, y={}, yaw={}", 
                                   packets_read, command.x, command.y, command.yaw);
                        }
                        Err(e) => {
                            warn!("Failed to parse UDP command JSON (packet #{}): {}", packets_read, e);
                            // Continue reading more packets
                        }
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    // No more data available - this is expected for non-blocking
                    break;
                }
                Err(e) => {
                    error!("UDP socket error: {}", e);
                    return Err(e);
                }
            }
        }
        
        // If we got at least one valid command, use the latest one
        if let Some(command) = latest_command {
            if packets_read > 1 {
                debug!("Drained {} UDP packets, using latest command", packets_read);
            }
            
            self.current_command = command;
            self.last_command_time = Some(std::time::Instant::now());
            Ok(true)
        } else {
            // No new valid commands received
            Ok(false)
        }
    }

    /// Get the current command, applying timeout logic
    pub fn get_current_command(&self) -> UdpCommand {
        if let Some(last_time) = self.last_command_time {
            if last_time.elapsed() > self.command_timeout {
                // Command has timed out, return zero command
                debug!("UDP command timed out, returning zero command");
                UdpCommand::default()
            } else {
                self.current_command
            }
        } else {
            // No command received yet
            UdpCommand::default()
        }
    }

    /// Check if we have received any commands recently
    pub fn has_recent_command(&self) -> bool {
        if let Some(last_time) = self.last_command_time {
            last_time.elapsed() <= self.command_timeout
        } else {
            false
        }
    }

    /// For backwards compatibility - wait for any UDP packet (like wait_for_enter)
    /// This also drains the buffer to get the latest packet
    pub async fn wait_for_any_command(&mut self) -> io::Result<UdpCommand> {
        loop {
            // First, try to drain any existing packets
            if self.try_update_command().await? {
                return Ok(self.current_command);
            }
            
            // If no packets were available, wait for the next one
            let mut buf = [0u8; 1024];
            let len = self.socket.recv(&mut buf).await?;
            
            match serde_json::from_slice::<UdpCommand>(&buf[..len]) {
                Ok(command) => {
                    self.current_command = command;
                    self.last_command_time = Some(std::time::Instant::now());
                    
                    // After receiving one packet, drain any additional packets to get the latest
                    self.try_update_command().await?;
                    
                    return Ok(self.current_command);
                }
                Err(e) => {
                    warn!("Failed to parse UDP command JSON: {}", e);
                    // Continue waiting for a valid command
                }
            }
        }
    }

    /// Get timing information for debugging
    pub fn get_command_age(&self) -> Option<Duration> {
        self.last_command_time.map(|t| t.elapsed())
    }
}



/// UDP Command State for policy control integration
#[derive(Debug)]
pub struct UdpControlVectorInputState {
    udp_manager: Option<UdpCommandManager>,
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
        self.udp_manager = Some(UdpCommandManager::new(port).await?);
        Ok(())
    }

    pub async fn update_from_udp(&mut self) -> io::Result<()> {
        if let Some(ref mut manager) = self.udp_manager {
            manager.try_update_command().await?;
            self.last_command = manager.get_current_command();
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
            arr[2] = udp_state.yaw;
        }
        debug!("UDP command from robot_description: x={}, y={}, yaw={}", udp_state.x, udp_state.y, udp_state.yaw);
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


pub struct UdpExtendedCommandManager {
    socket: UdpSocket,
    current_command: UdpExtendedCommand,
    last_command_time: Option<std::time::Instant>,
    command_timeout: Duration,
}


impl std::fmt::Debug for UdpExtendedCommandManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("UdpExtendedCommandManager")
            .field("current_command", &self.current_command)
            .field("last_command_time", &self.last_command_time)
            .finish()
    }
}


impl UdpExtendedCommandManager {
    pub async fn new(port: u16) -> io::Result<Self> {
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

    pub async fn try_update_command(&mut self) -> io::Result<bool> {
        let mut buf = [0u8; 1024];
        let mut latest: Option<UdpExtendedCommand> = None;
        let mut packets_read = 0;
        loop {
            match self.socket.try_recv(&mut buf) {
                Ok(len) => {
                    packets_read += 1;
                    match serde_json::from_slice::<UdpExtendedCommand>(&buf[..len]) {
                        Ok(cmd) => {
                            latest = Some(cmd);
                            debug!("Parsed extended UDP command #{} (x={}, y={}, yaw={})",
                                   packets_read, cmd.x, cmd.y, cmd.yaw_rate);
                        }
                        Err(e) => warn!("Failed to parse extended UDP command JSON (#{}) {}", packets_read, e),
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => { error!("UDP socket error: {}", e); return Err(e); }
            }
        }
        if let Some(cmd) = latest {
            if packets_read > 1 { debug!("Drained {} UDP packets (extended)", packets_read); }
            self.current_command = cmd;
            self.last_command_time = Some(std::time::Instant::now());
            Ok(true)
        } else { Ok(false) }
    }

    pub fn get_current_command(&self) -> UdpExtendedCommand {
        if let Some(t) = self.last_command_time {
            if t.elapsed() > self.command_timeout { UdpExtendedCommand::default() } else { self.current_command }
        } else { UdpExtendedCommand::default() }
    }

    pub fn has_recent_command(&self) -> bool {
        self.last_command_time.map(|t| t.elapsed() <= self.command_timeout).unwrap_or(false)
    }
}

#[derive(Debug)]
pub struct Udp16ControlVectorInputState {
    udp_manager: Option<UdpExtendedCommandManager>,
    last_command: UdpExtendedCommand,
}

impl Udp16ControlVectorInputState {
    pub fn new() -> Self {
        Self { udp_manager: None, last_command: UdpExtendedCommand::default() }
    }
    pub async fn initialize(&mut self, port: u16) -> io::Result<()> {
        self.udp_manager = Some(UdpExtendedCommandManager::new(port).await?);
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
        Ok(())
    }
    fn extract_with_robot(&mut self, arr: ndarray::ArrayViewMut1<f32>, _robot_description: &crate::robot_description::RobotDescription) -> std::io::Result<()> {
        self.extract(arr)
    }
}