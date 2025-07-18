/*
*
* Robstride Utils
*
*
* Convertsion
*
* Robstride's language is somewhat ambiguous. We attempt to specify it:
*
* - when they mention 'front' they mean lower order bytes
* - high byte means higher order byte
*
* - after the conversion, the high byte is in front and the low byte is in ...
*
*  conversion 'means' nothing, i.e on reception the byte order is big endian
*
*
*
*/

use std::f64::consts::PI;
use std::ops::{Add, Div, Mul, Sub};

#[derive(Debug, Clone, Copy)]
pub struct Range<T> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
where
    T: Copy + PartialOrd,
    T: Sub<Output = T> + Add<Output = T> + Mul<Output = T> + Div<Output = T>,
    T: From<f64>,
{
    pub fn scale_value(&self, value: T, to: &Range<T>) -> T {
        let proportion = (value - self.min) / (self.max - self.min);
        to.min + proportion * (to.max - to.min)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RangeSet<T> {
    pub angle: Range<T>,
    pub velocity: Range<T>,
    pub torque: Range<T>,
    pub kp: Range<T>,
    pub kd: Range<T>,
}

#[derive(Debug, Clone, Copy)]
pub enum RobstrideActuatorType {
    Robstride00,
    Robstride01,
    Robstride02,
    Robstride03,
    Robstride04,
}

impl RobstrideActuatorType {
    pub fn can_ranges(&self) -> RangeSet<f64> {
        RangeSet {
            // angle: Range { min: u16::MIN, max: u16::MAX },
            // velocity: Range { min: u16::MIN, max: u16::MAX },
            // torque: Range { min: u16::MIN, max: u16::MAX },
            // kp: Range { min: u16::MIN, max: u16::MAX },
            // kd: Range { min: u16::MIN, max: u16::MAX },
            angle: Range {
                min: u16::MIN as f64,
                max: u16::MAX as f64,
            },
            velocity: Range {
                min: u16::MIN as f64,
                max: u16::MAX as f64,
            },
            torque: Range {
                min: u16::MIN as f64,
                max: u16::MAX as f64,
            },
            kp: Range {
                min: u16::MIN as f64,
                max: u16::MAX as f64,
            },
            kd: Range {
                min: u16::MIN as f64,
                max: u16::MAX as f64,
            },
        }
    }

    pub fn actuator_ranges(&self) -> RangeSet<f64> {
        match self {
            RobstrideActuatorType::Robstride00 => RangeSet {
                angle: Range {
                    min: -4.0 * PI,
                    max: 4.0 * PI,
                },
                velocity: Range {
                    min: -33.0,
                    max: 33.0,
                },
                torque: Range {
                    min: -14.0,
                    max: 14.0,
                },
                kp: Range {
                    min: 0.0,
                    max: 500.0,
                },
                kd: Range { min: 0.0, max: 5.0 },
            },
            RobstrideActuatorType::Robstride01 => RangeSet {
                angle: Range {
                    min: -4.0 * PI,
                    max: 4.0 * PI,
                },
                velocity: Range {
                    min: -44.0,
                    max: 44.0,
                },
                torque: Range {
                    min: -17.0,
                    max: 17.0,
                },
                kp: Range {
                    min: 0.0,
                    max: 500.0,
                },
                kd: Range { min: 0.0, max: 5.0 },
            },
            RobstrideActuatorType::Robstride02 => RangeSet {
                angle: Range {
                    min: -4.0 * PI,
                    max: 4.0 * PI,
                },
                velocity: Range {
                    min: -44.0,
                    max: 44.0,
                },
                torque: Range {
                    min: -17.0,
                    max: 17.0,
                },
                kp: Range {
                    min: 0.0,
                    max: 500.0,
                },
                kd: Range { min: 0.0, max: 5.0 },
            },
            RobstrideActuatorType::Robstride03 => RangeSet {
                angle: Range {
                    min: -4.0 * PI,
                    max: 4.0 * PI,
                },
                velocity: Range {
                    min: -20.0,
                    max: 20.0,
                },
                torque: Range {
                    min: -60.0,
                    max: 60.0,
                },
                kp: Range {
                    min: 0.0,
                    max: 5000.0,
                },
                kd: Range {
                    min: 0.0,
                    max: 100.0,
                },
            },
            RobstrideActuatorType::Robstride04 => RangeSet {
                angle: Range {
                    min: -4.0 * PI,
                    max: 4.0 * PI,
                },
                velocity: Range {
                    min: -15.0,
                    max: 15.0,
                },
                torque: Range {
                    min: -120.0,
                    max: 120.0,
                },
                kp: Range {
                    min: 0.0,
                    max: 5000.0,
                },
                kd: Range {
                    min: 0.0,
                    max: 100.0,
                },
            },
        }
    }
}
