Creating and Running a New Robot
==========================================

This guide will walk you through the steps to create a new configuration for a robot with Robstride motors, calibrate it, and run it using the provided codebase.

Step 1: Create a New Configuration
----------------------------------

First, create a new configuration file for your Robstride robot. Save the following YAML content into a file named `robstride_config.yaml`:

.. code-block:: yaml

  - setup: "legs"
    motor_type: "robstride"
    delta_change: 10
    body_parts:
      right_leg:
        start_id: 1
        canbus_id: 0
        dof: 5
      left_leg:
        start_id: 6
        canbus_id: 1
        dof: 5
    motor_config:
      leg:
        signs: [-1, 1, -1, 1, 1, 1]
        increments: [4, 4, 4, 4, 4, 4]
        maximum_values: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
        offsets: [0, 0, 0, 0, 0, 0]
    params:
      - motor_id: "default"
        limit_torque: 10
        cur_kp: 0.17
        cur_ki: 0.012
        cur_fit_gain: 0.1
        limit_spd: 100
        limit_cur: 10
        loc_kp: 3
        spd_kp: 0.5
        spd_ki: 0.4
        spd_filt_gain: 0.1

Here, we have two legs, each with 5 DOF. The start_id is the first motor ID for the leg, and the canbus_id is the CAN bus ID. The dof is the number of degrees of freedom for the leg.

Step 2: Calibrate the Motors
----------------------------

Now that we have our configuration file, it's easy to instantiate a new robot. 

.. code-block:: python

    from firmware.robot.robot import Robot

    robot = Robot(config_path="robstride_config.yaml", setup="legs")

With our robot instantiated, we can now calibrate the motors and run the robot.

.. code-block:: python
    
    import time

    robot.calibrate_motors()
    positions = {
        "right_leg": [0, 0, 0, 0, 0],
        "left_leg": [0, 0, 0, 0, 0]
    }

    # Move all the joints in a sine wave

    start_time = time.time()
    while True:
        robot.set_position(positions)

        for leg in ["right_leg", "left_leg"]:
            for joint in range(5):
                positions[leg][joint] = 1.5 * math.sin(time.time() - start_time)
        
        time.sleep(0.01)

