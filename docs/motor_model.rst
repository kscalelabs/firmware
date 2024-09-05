


Motors
==========================

This section provides an overview of the motor interfaces used in the robot. Each motor is an extension of the MotorInterface class, allowing for easy integration of new motor types.

Motor Interface
---------------

The core idea is that the robot uses a generic motor interface defined in the :py:mod:`firmware.motor_utils.motor_utils` module. This interface provides a set of methods that all motor classes must implement, ensuring a consistent API for controlling motors.

The :py:class:`MotorInterface` class defines the following abstract methods that must be implemented by any motor class:

- `set_position(position: float, **kwargs: Any) -> None`: Sets the position of the motor.
- `set_current(current: float) -> None`: Sets the current of the motor.
- `set_zero_position() -> None`: Sets the zero position of the motor.
- `get_position() -> float`: Gets the current position of the motor.
- `get_speed() -> float`: Gets the current speed of the motor.
- `calibrate(current_limit: float) -> None`: Calibrates the motor assuming the existence of hard stops.

By implementing these methods, different motor classes can provide their own specific logic while adhering to a common interface.

Motor Implementations
---------------------

The robot currently supports two types of motors: Bionic Motors and Robstride Motors. Each motor type has its own implementation of the `MotorInterface`.

- :py:mod:`firmware.bionic_motors.motors` module: Defines the `BionicMotor` class, which implements the `MotorInterface` for Bionic Motors.
- :py:mod:`firmware.robstride_motors.motors` module: Defines the `RobstrideMotor` class, which implements the `MotorInterface` for Robstride Motors.

The `MotorFactory` class in the :py:mod:`firmware.motor_utils.motor_factory` module is used to create motor objects based on the configuration. It abstracts the creation logic and ensures that the correct motor class is instantiated based on the specified motor type.

Key Components
--------------

1. **Motor Interface**: Defines the common interface for all motor types.
2. **Motor Implementations**: Provides specific implementations for different motor types.
3. **Motor Factory**: Creates motor objects based on the configuration.

For detailed API documentation, refer to the following modules:

- :py:mod:`firmware.motor_utils.motor_utils` module: Provides the generic motor interface.
- :py:mod:`firmware.bionic_motors.motors` module: Contains the implementation for Bionic Motors.
- :py:mod:`firmware.robstride_motors.motors` module: Contains the implementation for Robstride Motors.
- :py:mod:`firmware.motor_utils.motor_factory` module: Contains the factory class for creating motor objects.
