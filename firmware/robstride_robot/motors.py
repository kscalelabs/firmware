"""Defines a class that dictates how to communicate with the motors."""

import can

from firmware.bionic_motors.motors import *

SPECIAL_IDENTIFIER = 0x7FF

@dataclass
class RobstrideParams:
    torque_limit: float
    cur_kp: float
    cur_ki: float
    cur_fit_gain: float
    speed_limit: float
    current_limit: float
    loc_kp: float
    spd_kp: float
    spd_ki: float
    spd_fit_gain: float

class RobstrideMotor(BionicMotor):
    """A class to interface with a motor over a CAN bus."""

    def __init__(self, motor_id: int, control_params: RobstrideParams, can_bus: CANInterface) -> None:
        """Initializes the motor.

        Args:
        motor_id: The ID of the motor.
        control_params: The control parameters for the motor.
        can_bus: The CAN bus interface.
        """
        super().__init__(motor_id, control_params, can_bus)
        self.control_params = control_params
        self.update_position()

    def send(self, can_id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            can_id: The motor ID.
            data: The data to send.
            length: The length of the data.
        """
        assert len(data) == length, f"Data length must be {length} bytes"
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.can_bus.bus.send(message)

    def read(self, timeout: float = 0.25, read_data_only: bool = True) -> None:
        """Generic read can bus method that reads messages from the can bus.

        Args:
            timeout: how long to read messages for in seconds
            read_data_only: whether to read only data that has been queried. If true, only type 5 messages are read.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.can_bus.channel.get_message(timeout=timeout)
            if message is not None:
                if valid_message(message.data):
                    message_id = message.arbitration_id
                    message_data = read_result(message.data)
                    if read_data_only:
                        if message_data and message_data["Message Type"] == 5:
                            BionicMotor.can_messages.append(CanMessage(id=message_id, data=message_data))
                    else:
                        BionicMotor.can_messages.append(CanMessage(id=message_id, data=str(message_data)))
                else:
                    pass

    def _send(self, id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            id: The motor ID.
            data: The data to send.
            length: The length of the data.
        """
        can_id = id
        assert len(data) == length, f"Data length must be {length} bytes"
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.can_bus.bus.send(message)

    def set_position(self, position: float, speed: float, torque: int) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
            position: The position to set the motor to (in degrees)
            speed: The speed to set the motor to (in rpm)
            torque: The torque to set the motor to (in Nm)
        """
        command = force_position_hybrid_control(self.control_params.kp, self.control_params.kd, position, speed, torque)
        self._send(self.motor_id, bytes(command))

    def set_position_torque_control(self, torque: int) -> None:
        """Sets the position of the motor using torque control.

        Args:
            torque: The torque to set the motor to (in Nm)
        """
        command = set_current_torque_control(motor_id=self.motor_id, value=torque, control_status=1)
        self._send(self.motor_id, bytes(command), 3)

    def set_position_current_control(self, current: int) -> None:
        """Sets the position of the motor using current control.

        Args:
            current: The current to set the motor to (in A)
        """
        command = set_current_torque_control(motor_id=self.motor_id, value=current, control_status=0)
        self._send(self.motor_id, bytes(command), 3)

    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        command = set_zero_position(self.motor_id)
        self._send(SPECIAL_IDENTIFIER, bytes(command), 4)

    def update_position(self, wait_time: float = 0.1, read_only: bool = False) -> str:
        """Updates the value of the motor's position attribute.

        NOTE: Do NOT use this to access the motor's position value.

        Just use <motor>.position instead.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the position value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        command = get_motor_pos()
        self._send(self.motor_id, bytes(command), 2)
        self.read(wait_time)
        for message in BionicMotor.can_messages:
            if message.id == self.motor_id and message.data["Message Type"] == 5:
                BionicMotor.can_messages.remove(message)
                if read_only:
                    return message.data["Data"]
                else:
                    self.position = message.data["Data"]
                # Flushes out any previous messages and ensures that the next message is fresh
                return "Valid"
            else:
                continue
        return "Valid"

    def update_speed(self, wait_time: float = 0.1, read_only: bool = False) -> str:
        """Updates the value of the motor's speed attribute.

        NOTE: Do NOT use this to access the motor's speed value.

        Just use <motor>.speed instead.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the speed value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        command = get_motor_speed(self.motor_id)
        self._send(self.motor_id, bytes(command), 2)
        self.read(wait_time)
        for message in BionicMotor.can_messages:
            if message.id == self.motor_id and message.data["Message Type"] == 5:
                BionicMotor.can_messages.remove(message)
                if read_only:
                    return message.data["Data"]
                else:
                    self.speed = message.data["Data"]
                # Flushes out any previous messages and ensures that the next message is fresh
                return "Valid"
            else:
                continue
                # return "Invalid"
        return "Valid"

    def __str__(self) -> str:
        return f"BionicMotor ({self.motor_id})"
