
// TODO: make a struct that mirrors the structure of this enum (i.e name is a string with 16
// charecters, Barcode is a String, etc.)

// Then we need to per function code parsing logic

#[derive(Debug, Clone, PartialEq)]
pub enum FunctionCode {
    // 0x0000 - 0x0FFF range
    Name,
    BarCode,
    BootCodeVersion,
    BootBuildDate,
    BootBuildTime,
    AppCodeVersion,
    AppGitVersion,
    AppBuildDate,
    AppBuildTime,
    AppCodeName,

    // 0x2000 - 0x2FFF range
    EchoPara1,
    EchoPara2,
    EchoPara3,
    EchoPara4,
    EchoFreHz,
    MechOffset,
    MechPosInit,
    LimitTorque,
    LfwMax,
    MotorBaud,
    CanId,
    CanMaster,
    CanTimeout,
    Status2,
    Status3,
    Status1,
    Status6,
    CurFiltGain,
    CurKp,
    CurKi,
    SpdKp,
    SpdKi,
    LocKp,
    SpdFiltGain,
    LimitSpd,
    LimitCur,
    LocRefFiltGain,
    LimitLoc,
    PositionOffset,
    ChasuAngleOffset,
    SpdStepValue,
    VelMax,
    AccSet,
    ZeroSta,

    // 0x3000 - 0x3FFF range
    TimeUse0,
    TimeUse1,
    TimeUse2,
    TimeUse3,
    EncoderRaw,
    McuTemp,
    MotorTemp,
    VBusMv,
    Adc1Offset,
    Adc2Offset,
    Adc1Raw,
    Adc2Raw,
    VBus,
    CmdId,
    CmdIq,
    CmdLoccRef,
    CmdSpdRef,
    CmdTorque,
    CmdPos,
    CmdVel,
    Rotation,
    ModPos,
    MechPos,
    MechVel,
    ElecPos,
    Ia,
    Ib,
    Ic,
    Timeout,
    PhaseOrder,
    Iqf,
    BoardTemp,
    Iq,
    Id,
    FaultSta,
    WarnSta,
    DrvFault,
    DrvTemp,
    Uq,
    Ud,
    DtcU,
    DtcV,
    DtcW,
    VBusFloat,
    TorqueFdb,
    RatedI,
    LimitI,
    SpdRef,
    SpdReff,
    ZeroFault,
    ChasuCoderRaw,
    ChasuAngle,
    AsAngle,
    VelMaxFloat,
    Judge,
    Fault1,
    Fault2,
    Fault3,
    Fault4,
    Fault5,
    Fault6,
    Fault7,
    Fault8,
    ElecOffset,
    McOverTemp,
    KtNmAmp,
    TqcaliType,
    LowPosition,
    ThetaMech1,
    Instep,
}

impl TryFrom<u16> for FunctionCode {
    type Error = ();
    fn try_from(code: u16) -> Result<Self, Self::Error> {
        let ret = match code {
            // 0x0000 - 0x0FFF range
            0x0000 => FunctionCode::Name,
            0x0001 => FunctionCode::BarCode,
            0x1000 => FunctionCode::BootCodeVersion,
            0x1001 => FunctionCode::BootBuildDate,
            0x1002 => FunctionCode::BootBuildTime,
            0x1003 => FunctionCode::AppCodeVersion,
            0x1004 => FunctionCode::AppGitVersion,
            0x1005 => FunctionCode::AppBuildDate,
            0x1006 => FunctionCode::AppBuildTime,
            0x1007 => FunctionCode::AppCodeName,

            // 0x2000 - 0x2FFF range
            0x2000 => FunctionCode::EchoPara1,
            0x2001 => FunctionCode::EchoPara2,
            0x2002 => FunctionCode::EchoPara3,
            0x2003 => FunctionCode::EchoPara4,
            0x2004 => FunctionCode::EchoFreHz,
            0x2005 => FunctionCode::MechOffset,
            0x2006 => FunctionCode::MechPosInit,
            0x2007 => FunctionCode::LimitTorque,
            0x2008 => FunctionCode::LfwMax,
            0x2009 => FunctionCode::MotorBaud,
            0x200a => FunctionCode::CanId,
            0x200b => FunctionCode::CanMaster,
            0x200c => FunctionCode::CanTimeout,
            0x200d => FunctionCode::Status2,
            0x200e => FunctionCode::Status3,
            0x200f => FunctionCode::Status1,
            0x2010 => FunctionCode::Status6,
            0x2011 => FunctionCode::CurFiltGain,
            0x2012 => FunctionCode::CurKp,
            0x2013 => FunctionCode::CurKi,
            0x2014 => FunctionCode::SpdKp,
            0x2015 => FunctionCode::SpdKi,
            0x2016 => FunctionCode::LocKp,
            0x2017 => FunctionCode::SpdFiltGain,
            0x2018 => FunctionCode::LimitSpd,
            0x2019 => FunctionCode::LimitCur,
            0x201a => FunctionCode::LocRefFiltGain,
            0x201b => FunctionCode::LimitLoc,
            0x201c => FunctionCode::PositionOffset,
            0x201d => FunctionCode::ChasuAngleOffset,
            0x201e => FunctionCode::SpdStepValue,
            0x201f => FunctionCode::VelMax,
            0x2020 => FunctionCode::AccSet,
            0x2021 => FunctionCode::ZeroSta,

            // 0x3000 - 0x3FFF range
            0x3000 => FunctionCode::TimeUse0,
            0x3001 => FunctionCode::TimeUse1,
            0x3002 => FunctionCode::TimeUse2,
            0x3003 => FunctionCode::TimeUse3,
            0x3004 => FunctionCode::EncoderRaw,
            0x3005 => FunctionCode::McuTemp,
            0x3006 => FunctionCode::MotorTemp,
            0x3007 => FunctionCode::VBusMv,
            0x3008 => FunctionCode::Adc1Offset,
            0x3009 => FunctionCode::Adc2Offset,
            0x300a => FunctionCode::Adc1Raw,
            0x300b => FunctionCode::Adc2Raw,
            0x300c => FunctionCode::VBus,
            0x300d => FunctionCode::CmdId,
            0x300e => FunctionCode::CmdIq,
            0x300f => FunctionCode::CmdLoccRef,
            0x3010 => FunctionCode::CmdSpdRef,
            0x3011 => FunctionCode::CmdTorque,
            0x3012 => FunctionCode::CmdPos,
            0x3013 => FunctionCode::CmdVel,
            0x3014 => FunctionCode::Rotation,
            0x3015 => FunctionCode::ModPos,
            0x3016 => FunctionCode::MechPos,
            0x3017 => FunctionCode::MechVel,
            0x3018 => FunctionCode::ElecPos,
            0x3019 => FunctionCode::Ia,
            0x301a => FunctionCode::Ib,
            0x301b => FunctionCode::Ic,
            0x301c => FunctionCode::Timeout,
            0x301d => FunctionCode::PhaseOrder,
            0x301e => FunctionCode::Iqf,
            0x301f => FunctionCode::BoardTemp,
            0x3020 => FunctionCode::Iq,
            0x3021 => FunctionCode::Id,
            0x3022 => FunctionCode::FaultSta,
            0x3023 => FunctionCode::WarnSta,
            0x3024 => FunctionCode::DrvFault,
            0x3025 => FunctionCode::DrvTemp,
            0x3026 => FunctionCode::Uq,
            0x3027 => FunctionCode::Ud,
            0x3028 => FunctionCode::DtcU,
            0x3029 => FunctionCode::DtcV,
            0x302a => FunctionCode::DtcW,
            0x302b => FunctionCode::VBusFloat,
            0x302c => FunctionCode::TorqueFdb,
            0x302d => FunctionCode::RatedI,
            0x302e => FunctionCode::LimitI,
            0x302f => FunctionCode::SpdRef,
            0x3030 => FunctionCode::SpdReff,
            0x3031 => FunctionCode::ZeroFault,
            0x3032 => FunctionCode::ChasuCoderRaw,
            0x3033 => FunctionCode::ChasuAngle,
            0x3034 => FunctionCode::AsAngle,
            0x3035 => FunctionCode::VelMaxFloat,
            0x3036 => FunctionCode::Judge,
            0x3037 => FunctionCode::Fault1,
            0x3038 => FunctionCode::Fault2,
            0x3039 => FunctionCode::Fault3,
            0x303a => FunctionCode::Fault4,
            0x303b => FunctionCode::Fault5,
            0x303c => FunctionCode::Fault6,
            0x303d => FunctionCode::Fault7,
            0x303e => FunctionCode::Fault8,
            0x303f => FunctionCode::ElecOffset,
            0x3040 => FunctionCode::McOverTemp,
            0x3041 => FunctionCode::KtNmAmp,
            0x3042 => FunctionCode::TqcaliType,
            0x3043 => FunctionCode::LowPosition,
            0x3044 => FunctionCode::ThetaMech1,
            0x3045 => FunctionCode::Instep,

            // Unknown code - return error variant
            _ => return Err(()),
        };
        Ok(ret)
    }
}

impl FunctionCode {
    pub fn to_hex(&self) -> u16 {
        match self {
            FunctionCode::Name                 => 0x0000,
            FunctionCode::BarCode              => 0x0001,
            FunctionCode::BootCodeVersion      => 0x1000,
            FunctionCode::BootBuildDate        => 0x1001,
            FunctionCode::BootBuildTime        => 0x1002,
            FunctionCode::AppCodeVersion       => 0x1003,
            FunctionCode::AppGitVersion        => 0x1004,
            FunctionCode::AppBuildDate         => 0x1005,
            FunctionCode::AppBuildTime         => 0x1006,
            FunctionCode::AppCodeName          => 0x1007,
            FunctionCode::EchoPara1            => 0x2000,
            FunctionCode::EchoPara2            => 0x2001,
            FunctionCode::EchoPara3            => 0x2002,
            FunctionCode::EchoPara4            => 0x2003,
            FunctionCode::EchoFreHz            => 0x2004,
            FunctionCode::MechOffset           => 0x2005,
            FunctionCode::MechPosInit          => 0x2006,
            FunctionCode::LimitTorque          => 0x2007,
            FunctionCode::LfwMax               => 0x2008,
            FunctionCode::MotorBaud            => 0x2009,
            FunctionCode::CanId                => 0x200a,
            FunctionCode::CanMaster            => 0x200b,
            FunctionCode::CanTimeout           => 0x200c,
            FunctionCode::Status2              => 0x200d,
            FunctionCode::Status3              => 0x200e,
            FunctionCode::Status1              => 0x200f,
            FunctionCode::Status6              => 0x2010,
            FunctionCode::CurFiltGain          => 0x2011,
            FunctionCode::CurKp                => 0x2012,
            FunctionCode::CurKi                => 0x2013,
            FunctionCode::SpdKp                => 0x2014,
            FunctionCode::SpdKi                => 0x2015,
            FunctionCode::LocKp                => 0x2016,
            FunctionCode::SpdFiltGain          => 0x2017,
            FunctionCode::LimitSpd             => 0x2018,
            FunctionCode::LimitCur             => 0x2019,
            FunctionCode::LocRefFiltGain       => 0x201a,
            FunctionCode::LimitLoc             => 0x201b,
            FunctionCode::PositionOffset       => 0x201c,
            FunctionCode::ChasuAngleOffset     => 0x201d,
            FunctionCode::SpdStepValue         => 0x201e,
            FunctionCode::VelMax               => 0x201f,
            FunctionCode::AccSet               => 0x2020,
            FunctionCode::ZeroSta              => 0x2021,
            FunctionCode::TimeUse0             => 0x3000,
            FunctionCode::TimeUse1             => 0x3001,
            FunctionCode::TimeUse2             => 0x3002,
            FunctionCode::TimeUse3             => 0x3003,
            FunctionCode::EncoderRaw           => 0x3004,
            FunctionCode::McuTemp              => 0x3005,
            FunctionCode::MotorTemp            => 0x3006,
            FunctionCode::VBusMv               => 0x3007,
            FunctionCode::Adc1Offset           => 0x3008,
            FunctionCode::Adc2Offset           => 0x3009,
            FunctionCode::Adc1Raw              => 0x300a,
            FunctionCode::Adc2Raw              => 0x300b,
            FunctionCode::VBus                 => 0x300c,
            FunctionCode::CmdId                => 0x300d,
            FunctionCode::CmdIq                => 0x300e,
            FunctionCode::CmdLoccRef           => 0x300f,
            FunctionCode::CmdSpdRef            => 0x3010,
            FunctionCode::CmdTorque            => 0x3011,
            FunctionCode::CmdPos               => 0x3012,
            FunctionCode::CmdVel               => 0x3013,
            FunctionCode::Rotation             => 0x3014,
            FunctionCode::ModPos               => 0x3015,
            FunctionCode::MechPos              => 0x3016,
            FunctionCode::MechVel              => 0x3017,
            FunctionCode::ElecPos              => 0x3018,
            FunctionCode::Ia                   => 0x3019,
            FunctionCode::Ib                   => 0x301a,
            FunctionCode::Ic                   => 0x301b,
            FunctionCode::Timeout              => 0x301c,
            FunctionCode::PhaseOrder           => 0x301d,
            FunctionCode::Iqf                  => 0x301e,
            FunctionCode::BoardTemp            => 0x301f,
            FunctionCode::Iq                   => 0x3020,
            FunctionCode::Id                   => 0x3021,
            FunctionCode::FaultSta             => 0x3022,
            FunctionCode::WarnSta              => 0x3023,
            FunctionCode::DrvFault             => 0x3024,
            FunctionCode::DrvTemp              => 0x3025,
            FunctionCode::Uq                   => 0x3026,
            FunctionCode::Ud                   => 0x3027,
            FunctionCode::DtcU                 => 0x3028,
            FunctionCode::DtcV                 => 0x3029,
            FunctionCode::DtcW                 => 0x302a,
            FunctionCode::VBusFloat            => 0x302b,
            FunctionCode::TorqueFdb            => 0x302c,
            FunctionCode::RatedI               => 0x302d,
            FunctionCode::LimitI               => 0x302e,
            FunctionCode::SpdRef               => 0x302f,
            FunctionCode::SpdReff              => 0x3030,
            FunctionCode::ZeroFault            => 0x3031,
            FunctionCode::ChasuCoderRaw        => 0x3032,
            FunctionCode::ChasuAngle           => 0x3033,
            FunctionCode::AsAngle              => 0x3034,
            FunctionCode::VelMaxFloat          => 0x3035,
            FunctionCode::Judge                => 0x3036,
            FunctionCode::Fault1               => 0x3037,
            FunctionCode::Fault2               => 0x3038,
            FunctionCode::Fault3               => 0x3039,
            FunctionCode::Fault4               => 0x303a,
            FunctionCode::Fault5               => 0x303b,
            FunctionCode::Fault6               => 0x303c,
            FunctionCode::Fault7               => 0x303d,
            FunctionCode::Fault8               => 0x303e,
            FunctionCode::ElecOffset           => 0x303f,
            FunctionCode::McOverTemp           => 0x3040,
            FunctionCode::KtNmAmp              => 0x3041,
            FunctionCode::TqcaliType           => 0x3042,
            FunctionCode::LowPosition          => 0x3043,
            FunctionCode::ThetaMech1           => 0x3044,
            FunctionCode::Instep               => 0x3045,
        }
    }
}

use heapless::String;

#[derive(Debug, Clone, PartialEq)]
pub struct RobstrideActuatorParams {
    pub params_00: [String<32>; 2],
    pub params_10: [String<32>; 8],
    pub params_20: [String<32>; 34],
    pub params_30: [String<32>; 70],
}


pub struct RobstrideActuatorParamFragment<'a> {
    pub function_code: FunctionCode,
    pub bytemarker: u8,
    pub data: &'a [u8],
}

impl<'a> RobstrideActuatorParamFragment<'a> {
    pub fn new(function_code: FunctionCode, bytemarker: u8, data: &'a [u8]) -> Self {
        Self {
            function_code,
            bytemarker,
            data,
        }
    }
}

impl Default for RobstrideActuatorParams {
    fn default() -> Self {
        RobstrideActuatorParams {
            params_00: [const { String::new() }; 2],
            params_10: [const { String::new() }; 8],
            params_20: [const { String::new() }; 34],
            params_30: [const { String::new() }; 70],
        }
    }
}

impl RobstrideActuatorParams {

    fn get_param_mut(&mut self, code: FunctionCode) -> &mut heapless::String<32> {
        let idx = code.to_hex() as usize;

        let arr_id = (idx >> 8) & 0xff;
        let idx = idx & 0xff;

        match arr_id {
            0 => &mut self.params_00[idx],
            1 => &mut self.params_10[idx],
            2 => &mut self.params_20[idx],
            3 => &mut self.params_30[idx],
            _ => panic!("Invalid function code: {}", code.to_hex()),
        }
    }

    pub fn merge_fragment(&mut self, fragment: RobstrideActuatorParamFragment) -> std::io::Result<()> {

        let dst: &mut String<32> = self.get_param_mut(fragment.function_code);

        let len = fragment.data.len();

        let offset = match fragment.bytemarker {
            0x0 => 0x0,
            0x1 => 0x1,
            0x2 => 0x2,
            0x6 | 0x3 => 0x3,
            0x7 | 0x4 => 0x4,
            0x8 => 0x5,
            _ => return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Unknown byte marker: {}", fragment.bytemarker),
            )),
        };

        if offset + len > dst.len() {
            panic!("Fragment exceeds destination string length");
        }

        // TODO: improve (this is not very idiomatic)
        unsafe {
            // SAFETY: We ensure that the offset and length are within bounds
            // of the destination string.
            let dst_bytes = dst.as_bytes_mut();
            dst_bytes[offset..(offset + len)]
                .copy_from_slice(fragment.data);
        }

        log::warn!("after merge: {}", dst);
        Ok(())
    }

    fn get_param(&self, code: FunctionCode) -> &heapless::String<32> {
        let idx = code.to_hex() as usize;

        let arr_id = (idx >> 8) & 0xff;
        let idx = idx & 0xff;

        match arr_id {
            0 => &self.params_00[idx],
            1 => &self.params_10[idx],
            2 => &self.params_20[idx],
            3 => &self.params_30[idx],
            _ => panic!("Invalid function code: {}", code.to_hex()),
        }
    }

}














