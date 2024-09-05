Robot Controller
==========================

This document provides an overview of how the robot controller works, including its architecture and key components. The robot controller is designed to be motor agnostic, allowing it to interface with various types of motors seamlessly.

The core idea is that the robot is collection of limbs as defined in the :py:mod:`firmware.robot.model` module.

The limbs themselves are simply sequences of motors with increasing CAN IDs. For example, a 5 DOF arm with starting CAN ID of 11 would have motors with CAN IDs 11, 12, 13, 14, 15.

The robot controller class itself contains a number of useful functions for controlling multiple limbs in a coordinated way. For example, it can automatically find the correct CAN bus for each limb.
It also contains logic for calibrating, zeroing, and disabling all limbs. Finally, it packages the actual joint positions into a dictionary that can be pulled or sent to the robot.

Robot Config
-----------

The robot config is a YAML file that defines the robot's limbs and motors. It is used to configure the robot's limbs and motors.

Reference the config.yaml in the robot package for the most up to date configuration format. 

However, the yaml should contain a `robots` key that contains a list of robot configurations. Each configuration includes:

- `setup`: The name of the setup.
- `motor_type`: The type of motors used (e.g., "bionic" or "robstride").
- `delta_change`: The maximum allowed change in motor position.
- `body_parts`: A dictionary defining the body parts, their starting CAN ID, the number of degrees of freedom (DOF), and the CAN bus ID.
- `motor_config`: Configuration for the motors, including signs, increments, maximum values, and offsets.
- `params`: Control parameters for the motors, with default parameters and specific overrides for individual motors.

The `Robot` class in `robot.py` uses this configuration to initialize and control the robot's limbs and motors. It includes methods for setting motor positions, zeroing out motors, disabling motors, updating motor data, and more.

Key Components
--------------

1. **Body Model**: Defines the robot's body parts and their configurations.
2. **Robot Model**: Manages the control logic for the robot, including movement and sensor integration.

For detailed API documentation, refer to the following modules:

- :py:mod:`firmware.robot.model` module: Provides the model definitions for the robot's body parts.
- :py:mod:`firmware.robot.robot` module: Contains the control logic for the robot.
