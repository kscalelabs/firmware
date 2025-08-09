use std::io;
use tracing::{debug, info, warn};

/**

00000000  55 51 03 00 e9 ff fc 07  87 0b 26 55 52 00 00 00  |UQ........&UR...|
00000010  00 00 00 87 0b 39 55 53  92 ff f6 ff 63 e7 f8 46  |.....9US....c..F|
00000020  b6 55 59 ce 85 aa 00 dc  ff 14 26 c0 55 51 02 00  |.UY.......&.UQ..|
00000030  ea ff fa 07 82 0b 1f 55  52 00 00 00 00 00 00 82  |.......UR.......|
00000040  26 c0 55 51 01 00 e9 ff  fa 07 9e 0b 39 55 52 00  |&.UQ........9UR.|
*/
const START_BYTE: u8 = 0x55;
const PACKET_SIZE: usize = 11; // data packets at 11 bytes
const PAYLOAD_SIZE: usize = 8; // payload size in bytes

use crate::robot_description::ImuFeedback;
use crate::typestate_serial::Operate as OperationalPort;
use crate::typestate_serial::SerialBaudRate;

#[derive(Debug)]
pub struct HiwonderImu {}

impl HiwonderImu {
    fn checksum(buffer: &[u8]) -> u8 {
        // iterate over the first 10 bytes, reduce them to sum, and get the lowest byte
        (buffer.iter().take(10).map(|&x| x as u16).sum::<u16>() & 0xff) as u8
    }

    pub fn verify_read(buf: &[u8]) -> io::Result<()> {
        // find the 0x55
        let idx = buf
            .iter()
            .position(|&b| b == 0x55)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "0x55 not found"))?;
        // check the length
        if buf.len() < idx + 11 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "buffer too short",
            ));
        }
        let checksum = Self::checksum(&buf[idx..idx + 11]);

        (checksum == buf[idx + 10])
            .then_some(())
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "checksum mismatch"))
    }

    pub async fn setup(port: &mut OperationalPort) -> std::io::Result<()> {
        // send the unlock command
        let mut cmd: [u8; 5] = CommandType::unlock().into();
        port.write(&cmd).await?;
        // must wait for unlock to take effect
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        cmd = CommandType::enable_output(
            OutputType::Acc | OutputType::Gyro | OutputType::Angle | OutputType::Quaternion,
        )
        .into();
        port.write(&cmd).await?;

        cmd = CommandType::set_frequency(ImuFrequency::Hz100).into();
        port.write(&cmd).await?;

        cmd = CommandType::set_baud_rate(SerialBaudRate::B230400)?.into();
        port.write(&cmd).await?;

        // must save to take effect
        cmd = CommandType::save().into();
        port.write(&cmd).await?;
        // precautionary wait for save
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        Ok(())
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C, packed)]
pub struct HiwonderRawFrame {
    pub magic: u8, // 0x55
    pub frame_type: u8,
    pub data: [u8; PAYLOAD_SIZE], // 9 bytes of data
    pub checksum: u8,             // 1 byte checksum
}

impl HiwonderRawFrame {
    pub fn checksum(&self) -> u8 {
        // Calculate the checksum for the frame
        let mut sum = self.magic as u16 + self.frame_type as u16;
        for &byte in &self.data {
            sum += byte as u16;
        }
        (sum & 0xFF) as u8
    }
}

impl HiwonderImu {
    pub async fn parse_all(port: &mut OperationalPort) -> io::Result<ImuFeedback> {
        // read upto 1024
        let mut buf = [0u8; 1024];

        match port.read(&mut buf).await {
            Err(e) => return Err(e),
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "No data read from port",
                ));
            }
            Ok(n) => {
                if n < PACKET_SIZE {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "Not enough data read",
                    ));
                }
            }
        }

        // find the start byte 0x55
        let start_idx = buf.iter().position(|&b| b == START_BYTE).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "Start byte 0x55 not found")
        })?;
        let buf = &buf[start_idx..];

        let mut fdbk: ImuFeedback = ImuFeedback::default();
        // now we parse this data
        buf.chunks_exact(PACKET_SIZE).for_each(|chunk| {
            let frame: &HiwonderRawFrame = bytemuck::from_bytes(chunk);
            if frame.checksum() != frame.checksum {
                debug!(
                    "Checksum mismatch for frame: expected {}, got {}",
                    frame.checksum(),
                    frame.checksum,
                );
                return;
            }
            Self::merge_frame(frame, &mut fdbk);
        });
        Ok(fdbk)
    }

    fn merge_frame(frame: &HiwonderRawFrame, fdbk: &mut ImuFeedback) -> io::Result<()> {
        let frame = (*frame).try_into()?;
        debug!("Parsed frame: {:?}", frame);
        match frame {
            ReadFrame::Acceleration { x, y, z, temp } => {
                fdbk.accelerometer = Some([x.into(), y.into(), z.into()]);
                fdbk.temperature = Some(temp);
            }
            ReadFrame::Gyro { x, y, z, voltage } => {
                fdbk.gyroscope = Some([x.into(), y.into(), z.into()]);
            }
            ReadFrame::Angle {
                roll,
                pitch,
                yaw,
                version,
            } => {
                fdbk.euler = Some([roll.into(), pitch.into(), yaw.into()]);
            }
            ReadFrame::Quaternion { w, x, y, z } => {
                fdbk.quaternion = Some([x.into(), y.into(), z.into(), w.into()]);
            }
            ReadFrame::Magnetometer { x, y, z, temp } => {
                fdbk.magnetometer = Some([x.into(), y.into(), z.into()]);
                fdbk.temperature = Some(temp);
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Unsupported frame type",
                ));
            }
        }
        Ok(())
    }
}

pub enum CommandType {
    Generic(Command),
    Unlock(Command),
    SetFusionAlgorithm(Command),
    EnableOutput(Command),
    Save(Command),
    Reboot(Command),
    FactoryReset(Command),
    SetFrequency(Command),
    SetBaudRate(Command),
}

impl CommandType {
    pub fn generic() -> Self {
        CommandType::Generic(Command::new(Register::ReadAddr, [0, 0]))
    }

    pub fn unlock() -> Self {
        CommandType::Unlock(Command::new(Register::Key, [0x88, 0xB5]))
    }

    pub fn fusion(algorithm: FusionAlgorithm) -> Self {
        let data: [u8; 2] = match algorithm {
            FusionAlgorithm::NineAxis => [0x00, 0x00],
            FusionAlgorithm::SixAxis => [0x01, 0x00],
        };
        CommandType::SetFusionAlgorithm(Command::new(Register::Axis6, data))
    }

    pub fn enable_output(output: enumflags2::BitFlags<OutputType, u16>) -> Self {
        let bits = output.bits();
        let data = [bits as u8, (bits >> 8) as u8];
        // let data = [ 0, 0xff];
        CommandType::EnableOutput(Command::new(Register::Rsw, data))
    }

    pub fn read_register(reg: Register) -> Self {
        CommandType::Generic(Command::new(Register::ReadAddr, [reg as u8, 0x00]))
    }

    pub fn save() -> Self {
        CommandType::Save(Command::new(Register::Save, [0x00, 0x00]))
    }

    pub fn reboot() -> Self {
        CommandType::Reboot(Command::new(Register::Save, [0xFF, 0x00]))
    }

    pub fn factory_reset() -> Self {
        CommandType::FactoryReset(Command::new(Register::Save, [0x01, 0x00]))
    }

    pub fn set_frequency(frequency: ImuFrequency) -> Self {
        CommandType::SetFrequency(Command::new(Register::Rrate, frequency.into()))
    }

    pub fn set_baud_rate(baud_rate: SerialBaudRate) -> std::io::Result<Self> {
        Ok(CommandType::SetBaudRate(Command::new(
            Register::Baud,
            baud_rate.try_into()?,
        )))
    }
}

impl From<CommandType> for Command {
    fn from(command_type: CommandType) -> Self {
        match command_type {
            CommandType::Generic(cmd) => cmd,
            CommandType::Unlock(cmd) => cmd,
            CommandType::SetFusionAlgorithm(cmd) => cmd,
            CommandType::EnableOutput(cmd) => cmd,
            CommandType::Save(cmd) => cmd,
            CommandType::Reboot(cmd) => cmd,
            CommandType::FactoryReset(cmd) => cmd,
            CommandType::SetFrequency(cmd) => cmd,
            CommandType::SetBaudRate(cmd) => cmd,
        }
    }
}

#[repr(C, packed)]
pub struct Command {
    pub magic: u16, // 0xaaff
    pub register: Register,
    pub data: [u8; 2],
}

impl Command {
    pub fn new(register: Register, data: [u8; 2]) -> Self {
        Self {
            magic: 0xaaff,
            register,
            data,
        }
    }
}

impl From<CommandType> for [u8; 5] {
    fn from(command: CommandType) -> Self {
        let cmd: Command = command.into();
        [
            (cmd.magic & 0xFF) as u8, // low byte
            (cmd.magic >> 8) as u8,   // high byte
            cmd.register as u8,
            cmd.data[0],
            cmd.data[1],
        ]
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Time = 0x50,
    Acceleration = 0x51,
    Gyro = 0x52,
    Angle = 0x53,
    Magnetometer = 0x54,
    PortStatus = 0x55,
    BaroAltitude = 0x56,
    LatLon = 0x57,
    Gps = 0x58,
    Quaternion = 0x59,
    GpsAccuracy = 0x5A,
    GenericRead = 0x5F,
}

impl FrameType {
    pub fn get_constant_value(self) -> f32 {
        match self {
            FrameType::Gyro => 2000.0 * std::f32::consts::PI / 180.0,
            FrameType::Angle => std::f32::consts::PI,
            FrameType::Acceleration => 16.0 * 9.80665,
            _ => 1.0,
        }
    }
}

impl TryFrom<u8> for FrameType {
    type Error = io::Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x50 => Ok(FrameType::Time),
            0x51 => Ok(FrameType::Acceleration),
            0x52 => Ok(FrameType::Gyro),
            0x53 => Ok(FrameType::Angle),
            0x54 => Ok(FrameType::Magnetometer),
            0x55 => Ok(FrameType::PortStatus),
            0x56 => Ok(FrameType::BaroAltitude),
            0x57 => Ok(FrameType::LatLon),
            0x58 => Ok(FrameType::Gps),
            0x59 => Ok(FrameType::Quaternion),
            0x5A => Ok(FrameType::GpsAccuracy),
            0x5F => Ok(FrameType::GenericRead),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Unknown frame type byte: {value:#02x}"),
            )),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ReadFrame {
    Time {
        year: u8,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
        ms: u16,
    },
    Acceleration {
        x: f32,
        y: f32,
        z: f32,
        temp: f32,
    },
    Gyro {
        x: f32,
        y: f32,
        z: f32,
        voltage: f32,
    },
    Angle {
        roll: f32,
        pitch: f32,
        yaw: f32,
        version: f32,
    },
    Magnetometer {
        x: f32,
        y: f32,
        z: f32,
        temp: f32,
    },
    PortStatus {
        d0: u16,
        d1: u16,
        d2: u16,
        d3: u16,
    },
    BaroAltitude {
        pressure: u32,
        height_cm: u32,
    },
    LatLon {
        lon: f64,
        lat: f64,
    },
    Gps {
        altitude: f32,
        heading: f32,
        ground_speed: f32,
    },
    Quaternion {
        w: f32,
        x: f32,
        y: f32,
        z: f32,
    },
    GpsAccuracy {
        sv: u16,
        pdop: f32,
        hdop: f32,
        vdop: f32,
    },
    GenericRead {
        data: [u8; 8],
    },
}

impl TryFrom<HiwonderRawFrame> for ReadFrame {
    type Error = io::Error;

    fn try_from(frame: HiwonderRawFrame) -> Result<Self, Self::Error> {
        let frame_type = FrameType::try_from(frame.frame_type)?;
        let k = frame_type.get_constant_value();
        match frame_type {
            FrameType::Time => {
                let year = frame.data[0];
                let month = frame.data[1];
                let day = frame.data[2];
                let hour = frame.data[3];
                let minute = frame.data[4];
                let second = frame.data[5];
                let ms = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::Time {
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                    ms,
                })
            }
            FrameType::Acceleration => {
                let acc_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let acc_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let acc_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let temp = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Acceleration {
                    x: acc_x as f32 / 32768.0 * k,
                    y: acc_y as f32 / 32768.0 * k,
                    z: acc_z as f32 / 32768.0 * k,
                    temp: temp as f32 / 100.0,
                })
            }
            // Returns radians per second
            FrameType::Gyro => {
                let gyro_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let gyro_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let gyro_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let voltage = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Gyro {
                    x: gyro_x as f32 / 32768.0 * k,
                    y: gyro_y as f32 / 32768.0 * k,
                    z: gyro_z as f32 / 32768.0 * k,
                    voltage: voltage as f32 / 100.0, // "Non-Bluetooth Products，the data is invalid" - https://github.com/YahboomTechnology/10-axis_IMU_Module
                })
            }
            FrameType::Angle => {
                let angle_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let angle_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let angle_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let version = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Angle {
                    roll: angle_x as f32 / 32768.0 * k,
                    pitch: angle_y as f32 / 32768.0 * k,
                    yaw: angle_z as f32 / 32768.0 * k,
                    version: version as f32,
                })
            }
            FrameType::Magnetometer => {
                let mag_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let mag_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let mag_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let temp = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Magnetometer {
                    x: mag_x as f32 / 32768.0 * k,
                    y: mag_y as f32 / 32768.0 * k,
                    z: mag_z as f32 / 32768.0 * k,
                    temp: temp as f32 / 100.0,
                })
            }
            FrameType::PortStatus => {
                let d0 = u16::from(frame.data[1]) << 8 | u16::from(frame.data[0]);
                let d1 = u16::from(frame.data[3]) << 8 | u16::from(frame.data[2]);
                let d2 = u16::from(frame.data[5]) << 8 | u16::from(frame.data[4]);
                let d3 = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::PortStatus { d0, d1, d2, d3 })
            }
            FrameType::BaroAltitude => {
                let pressure = u32::from(frame.data[3]) << 24
                    | u32::from(frame.data[2]) << 16
                    | u32::from(frame.data[1]) << 8
                    | u32::from(frame.data[0]);
                let height_cm = u32::from(frame.data[7]) << 24
                    | u32::from(frame.data[6]) << 16
                    | u32::from(frame.data[5]) << 8
                    | u32::from(frame.data[4]);
                Ok(ReadFrame::BaroAltitude {
                    pressure,
                    height_cm,
                })
            }
            FrameType::LatLon => {
                let longitude = i32::from(frame.data[3]) << 24
                    | i32::from(frame.data[2]) << 16
                    | i32::from(frame.data[1]) << 8
                    | i32::from(frame.data[0]);
                let latitude = i32::from(frame.data[7]) << 24
                    | i32::from(frame.data[6]) << 16
                    | i32::from(frame.data[5]) << 8
                    | i32::from(frame.data[4]);
                Ok(ReadFrame::LatLon {
                    lon: longitude as f64,
                    lat: latitude as f64,
                })
            }
            FrameType::Gps => {
                let altitude = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let heading = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let ground_speed = i32::from(frame.data[7]) << 24
                    | i32::from(frame.data[6]) << 16
                    | i32::from(frame.data[5]) << 8
                    | i32::from(frame.data[4]);
                Ok(ReadFrame::Gps {
                    altitude: altitude as f32 / 10.0,
                    heading: heading as f32 / 100.0,
                    ground_speed: ground_speed as f32 / 1000.0,
                })
            }
            FrameType::Quaternion => {
                let quat_w = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let quat_x = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let quat_y = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let quat_z = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Quaternion {
                    w: quat_w as f32 / 32768.0 * k,
                    x: quat_x as f32 / 32768.0 * k,
                    y: quat_y as f32 / 32768.0 * k,
                    z: quat_z as f32 / 32768.0 * k,
                })
            }
            FrameType::GpsAccuracy => {
                let num_satellites = u16::from(frame.data[1]) << 8 | u16::from(frame.data[0]);
                let pdop = u16::from(frame.data[3]) << 8 | u16::from(frame.data[2]);
                let hdop = u16::from(frame.data[5]) << 8 | u16::from(frame.data[4]);
                let vdop = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::GpsAccuracy {
                    sv: num_satellites,
                    pdop: pdop as f32 / 100.0,
                    hdop: hdop as f32 / 100.0,
                    vdop: vdop as f32 / 100.0,
                })
            }
            FrameType::GenericRead => {
                let data = [
                    frame.data[0],
                    frame.data[1],
                    frame.data[2],
                    frame.data[3],
                    frame.data[4],
                    frame.data[5],
                    frame.data[6],
                    frame.data[7],
                ];
                Ok(ReadFrame::GenericRead { data })
            }
        }
    }
}

pub enum FusionAlgorithm {
    NineAxis,
    SixAxis,
}

#[enumflags2::bitflags]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum OutputType {
    Time = 1 << 0,
    Acc = 1 << 1,
    Gyro = 1 << 2,
    Angle = 1 << 3,
    Mag = 1 << 4,
    Port = 1 << 5,
    Press = 1 << 6,
    Gps = 1 << 7,
    Velocity = 1 << 8,
    Quaternion = 1 << 9,
    GpsAccuracy = 1 << 10,
}

#[derive(Debug, Clone, Copy)]
pub enum ImuFrequency {
    Hz0_2,  // 0.2 Hz
    Hz0_5,  // 0.5 Hz
    Hz1,    // 1 Hz
    Hz2,    // 2 Hz
    Hz5,    // 5 Hz
    Hz10,   // 10 Hz
    Hz20,   // 20 Hz
    Hz50,   // 50 Hz
    Hz100,  // 100 Hz
    Hz200,  // 200 Hz
    Single, // Single reading
    None,   // No readings
}

impl From<ImuFrequency> for [u8; 2] {
    fn from(value: ImuFrequency) -> Self {
        match value {
            ImuFrequency::Hz0_2 => [0x01, 0x00],
            ImuFrequency::Hz0_5 => [0x02, 0x00],
            ImuFrequency::Hz1 => [0x03, 0x00],
            ImuFrequency::Hz2 => [0x04, 0x00],
            ImuFrequency::Hz5 => [0x05, 0x00],
            ImuFrequency::Hz10 => [0x06, 0x00],
            ImuFrequency::Hz20 => [0x07, 0x00],
            ImuFrequency::Hz50 => [0x08, 0x00],
            ImuFrequency::Hz100 => [0x09, 0x00],
            ImuFrequency::Hz200 => [0x0B, 0x00],
            ImuFrequency::Single => [0x0C, 0x00],
            ImuFrequency::None => [0x0D, 0x00],
        }
    }
}

impl TryFrom<SerialBaudRate> for [u8; 2] {
    type Error = io::Error;

    fn try_from(value: SerialBaudRate) -> Result<Self, Self::Error> {
        Ok(match value {
            SerialBaudRate::B4800 => [0x01, 0x00],
            SerialBaudRate::B9600 => [0x02, 0x00],
            SerialBaudRate::B19200 => [0x03, 0x00],
            SerialBaudRate::B38400 => [0x04, 0x00],
            SerialBaudRate::B57600 => [0x05, 0x00],
            SerialBaudRate::B115200 => [0x06, 0x00],
            SerialBaudRate::B230400 => [0x07, 0x00],
            SerialBaudRate::B460800 => [0x08, 0x00],
            SerialBaudRate::B921600 => [0x09, 0x00],
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Invalid baud rate",
                ));
            }
        })
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Register {
    Save = 0x00,
    CalSw = 0x01,        // Calibration mode
    Rsw = 0x02,          // output content
    Rrate = 0x03,        // output rate
    Baud = 0x04,         // Serial port baud rate
    AxOffset = 0x05,     // Acceleration X Bias
    AyOffset = 0x06,     // Acceleration Y Bias
    AzOffset = 0x07,     // Acceleration Z Bias
    GxOffset = 0x08,     // Angular velocity X Bias
    GyOffset = 0x09,     // Angular velocity Y Bias
    GzOffset = 0x0A,     // Angular velocity Z Bias
    HxOffset = 0x0B,     // Magnetic Field X Bias
    HyOffset = 0x0C,     // Magnetic Field Y Bias
    HzOffset = 0x0D,     // Magnetic Field Z Bias
    D0Mode = 0x0E,       // D0 Pin mode
    D1Mode = 0x0F,       // D1 Pin mode
    D2Mode = 0x10,       // D2 Pin mode
    D3Mode = 0x11,       // D3 Pin mode
    IicAddr = 0x1A,      // Device address
    LedOff = 0x1B,       // Turn off the LED lights
    MagRangX = 0x1C,     // Magnetic Field X Calibration Range
    MagRangY = 0x1D,     // Magnetic Field Y Calibration Range
    MagRangZ = 0x1E,     // Magnetic Field Z Calibration Range
    Bandwidth = 0x1F,    // Bandwidth
    GyroRange = 0x20,    // Gyroscope range
    AccRange = 0x21,     // Acceleration range
    Sleep = 0x22,        // Hibernate
    Orient = 0x23,       // Installation direction
    Axis6 = 0x24,        // algorithm
    FilTk = 0x25,        // Dynamic filtering
    GpsBaud = 0x26,      // GPS baud rate
    ReadAddr = 0x27,     // read register
    AccFilt = 0x2A,      // acceleration filter
    PowOnSend = 0x2D,    // command start
    Version = 0x2E,      // version number (Read-only)
    YYMM = 0x30,         // Year Month
    DDHH = 0x31,         // Day Hour
    MMSS = 0x32,         // Minute Second
    Ms = 0x33,           // Millisecond
    Ax = 0x34,           // Acceleration X (Read-only)
    Ay = 0x35,           // Acceleration Y (Read-only)
    Az = 0x36,           // Acceleration Z (Read-only)
    Gx = 0x37,           // Angular velocity X (Read-only)
    Gy = 0x38,           // Angular velocity Y (Read-only)
    Gz = 0x39,           // Angular velocity Z (Read-only)
    Hx = 0x3A,           // Magnetic Field X (Read-only)
    Hy = 0x3B,           // Magnetic Field Y (Read-only)
    Hz = 0x3C,           // Magnetic Field Z (Read-only)
    Roll = 0x3D,         // roll angle (Read-only)
    Pitch = 0x3E,        // Pitch angle (Read-only)
    Yaw = 0x3F,          // Heading (Read-only)
    Temp = 0x40,         // temperature (Read-only)
    D0Status = 0x41,     // D0 pin state (Read-only)
    D1Status = 0x42,     // D1 pin state (Read-only)
    D2Status = 0x43,     // D2 pin state (Read-only)
    D3Status = 0x44,     // D3 pin state (Read-only)
    PressureL = 0x45,    // Air pressure low 16 bits (Read-only)
    PressureH = 0x46,    // Air pressure high 16 bits (Read-only)
    HeightL = 0x47,      // Height lower 16 bits (Read-only)
    HeightH = 0x48,      // Height high 16 bits (Read-only)
    LonL = 0x49,         // Longitude lower 16 bits (Read-only)
    LonH = 0x4A,         // Longitude high 16 bits (Read-only)
    LatL = 0x4B,         // Latitude lower 16 bits (Read-only)
    LatH = 0x4C,         // Latitude high 16 bits (Read-only)
    GpsHeight = 0x4D,    // GPS Altitude (Read-only)
    GpsYaw = 0x4E,       // GPS heading angle (Read-only)
    GpsVL = 0x4F,        // GPS ground speed low 16 bits (Read-only)
    GpsVH = 0x50,        // GPS ground speed high 16 bits (Read-only)
    Q0 = 0x51,           // Quaternion 0 (Read-only)
    Q1 = 0x52,           // Quaternion 1 (Read-only)
    Q2 = 0x53,           // Quaternion 2 (Read-only)
    Q3 = 0x54,           // Quaternion 3 (Read-only)
    SvNum = 0x55,        // number of satellites (Read-only)
    Pdop = 0x56,         // Position accuracy (Read-only)
    Hdop = 0x57,         // Horizontal accuracy (Read-only)
    Vdop = 0x58,         // vertical accuracy (Read-only)
    DelayT = 0x59,       // Alarm signal delay
    XMin = 0x5A,         // X-axis angle alarm minimum value
    XMax = 0x5B,         // X-axis angle alarm maximum value
    BatVal = 0x5C,       // Supply voltage (Read-only)
    AlarmPin = 0x5D,     // Alarm Pin Mapping
    YMin = 0x5E,         // Y-axis angle alarm minimum value
    YMax = 0x5F,         // Y-axis angle alarm maximum value
    GyroCaliThr = 0x61,  // Gyro Still Threshold
    AlarmLevel = 0x62,   // Angle alarm level
    GyroCaliTime = 0x63, // Gyro auto calibration time
    TrigTime = 0x68,     // Alarm continuous trigger time
    Key = 0x69,          // unlock
    WError = 0x6A,       // Gyroscope change value (Read-only)
    TimeZone = 0x6B,     // GPS time zone
    WzTime = 0x6E,       // Angular velocity continuous rest time
    WzStatic = 0x6F,     // Angular velocity integral threshold
    ModDelay = 0x74,     // 485 data response delay
    XRefRoll = 0x79,     // Roll angle zero reference value (Read-only)
    YRefPitch = 0x7A,    // Pitch angle zero reference value (Read-only)
    NumberId1 = 0x7F,    // Device ID 1-2 (Read-only)
    NumberId2 = 0x80,    // Device ID 3-4 (Read-only)
    NumberId3 = 0x81,    // Device ID 5-6 (Read-only)
    NumberId4 = 0x82,    // Device ID 7-8 (Read-only)
    NumberId5 = 0x83,    // Device ID 9-10 (Read-only)
    NumberId6 = 0x84,    // Device ID 11-12 (Read-only)
}
