import time

import can

from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH

# Torque profile
# Trapezoidal ramp parameters
I_max = 20   # Maximum current
t_ramp = 1  # Ramp time in seconds

# Control parameters (as an example)
KP = 30
KD = 6
KT = 2.5

# Initial conditions
def trapezoidal_profile(t, I_max, t_ramp):
    if t < t_ramp:
        # Ramp up
        return I_max * (t / t_ramp)
    elif t_ramp <= t < 2 * t_ramp:
        # Constant max current
        return I_max
    elif 2 * t_ramp <= t < 3 * t_ramp:
        # Ramp down
        return I_max * (1 - (t - 2 * t_ramp) / t_ramp)
    else:
        # Beyond the ramp down phase
        return 0

def calculate_motor_current(pos_desired, pos_current, 
                            spd_desired, spd_current, 
                            torque_ff, start_time):
    # Time elapsed since start
    t = time.time() - start_time

    # PID-like control effort
    control_effort = (KP * (pos_desired - pos_current) +
                      KD * (spd_desired - spd_current) +
                      torque_ff) / KT

    # Apply the trapezoidal profile
    modulated_current = control_effort * trapezoidal_profile(t, I_max, t_ramp) / 100
    sign = -1 if modulated_current < 0 else 1

    return min(abs(modulated_current), I_max) * sign

# Initialize the CAN bus
write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
buffer_reader = can.BufferedReader()
notifier = can.Notifier(write_bus, [buffer_reader])

CAN_BUS = CANInterface(write_bus, buffer_reader, notifier)

# Initialize a motor with ID 1
motor = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS)

# Initialize the motor and set zero position
motor.set_zero_position()
motor.update_position(0.05)

# Define target positions
pos = 60
runtime = 5  # time in seconds to hold each position

while True:
    end_time = time.time() + runtime  # end time of loop
    
    while time.time() < end_time:
        # Update motor state
        motor.update_position(0.01)
        motor.update_speed(0.01)
        current_position = motor.position
        current_speed = motor.speed
        
        # Bang-bang control logic
        position_error = pos - current_position
        torque = 0
        
        if position_error > 5:
            torque = 20
        elif position_error < -5:
            torque = -20
        else:
            torque = 0
        
        print("Position: ", current_position)
        print("Torque: ", torque)
        
        # Set the torque
        motor.set_position_torque_control(torque)
        time.sleep(0.01)
    
    # Toggle target position between 0 and 90 degrees
    pos = 0 if pos == 90 else 90
    