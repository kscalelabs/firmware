#[derive(Debug, Clone, Copy)]
pub enum TelemetryStreamType {
    Log,
    Data(DataType),
}

impl std::fmt::Display for TelemetryStreamType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TelemetryStreamType::Log => write!(f, "Log"),
            TelemetryStreamType::Data(data_type) => write!(f, "Data({})", data_type),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum DataType {
    Policy,
    Actuator,
    Imu,
}

impl std::fmt::Display for DataType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DataType::Policy => write!(f, "Policy"),
            DataType::Actuator => write!(f, "Actuator"),
            DataType::Imu => write!(f, "Imu"),
        }
    }
}

// Override standard logging macros to automatically add TelemetryStreamType::Log
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        tracing::info!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Log, $($arg)*)
    };
}

#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        tracing::debug!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Log, $($arg)*)
    };
}

#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        tracing::warn!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Log, $($arg)*)
    };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        tracing::error!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Log, $($arg)*)
    };
}

#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        tracing::trace!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Log, $($arg)*)
    };
}

// Data logging macro that takes data type as first argument
#[macro_export]
macro_rules! data {
    (Policy, $level:ident, $($arg:tt)*) => {
        tracing::$level!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Data($crate::telemetry::telemetry_macros::DataType::Policy), $($arg)*)
    };
    (Actuator, $level:ident, $($arg:tt)*) => {
        tracing::$level!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Data($crate::telemetry::telemetry_macros::DataType::Actuator), $($arg)*)
    };
    (Imu, $level:ident, $($arg:tt)*) => {
        tracing::$level!(stream_type = %$crate::telemetry::telemetry_macros::TelemetryStreamType::Data($crate::telemetry::telemetry_macros::DataType::Imu), $($arg)*)
    };
}
