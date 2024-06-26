"""Simple async script to tick all motors back and forth by n degrees."""

import time
from typing import List

import can

from ..motors.bionic_motors import force_position_hybrid_control, set_position_control, set_zero_position

DEFAULT_MAX_DPS = 360.0


class InvalidMotorIDError(Exception):
    pass


def send_id(id: int) -> int:
    """ [ DEPRECATED ]
    Returns the CAN send ID for the given motor ID.

    Args:
        id: The motor ID (from 1 to 32, inclusive).

    Raises:
        InvalidMotorIDError: If the motor ID is out of range.

    Returns:
        The CAN send ID.
    """
    # if not (1 <= id <= 32):
    #    raise InvalidMotorIDError(f"Motor ID {hex(id)} out of range")
    return id


def recv_id(id: int) -> int:
    """ [ DEPRECATED ]
    Returns the CAN receive ID for the given motor ID.

    Args:
        id: The motor ID (from 1 to 32, inclusive).

    Raises:
        InvalidMotorIDError: If the motor ID is out of range.

    Returns:
        The CAN receive ID.
    """
    # if not (1 <= id <= 32):
    #    raise InvalidMotorIDError(f"Motor ID {hex(id)} out of range")
    return id


class TestCanBus:
    """A class to interface with motors over a CAN bus."""

    def __init__(
        self,
        channel: str = "can0",
        bustype: str = "socketcan",
        motor_idxs: list = [1],
        timeout: float = 2, # SET LOWER to avoid stuttering
        delta: float = 2.0,
        seq_timeout: float = 0.005, # SET set lower to test faster can messages
        hold_time: float = 2, # SET
    ) -> None:
        """Initializes the TestCanBus class.

        Args:
            channel: The CAN channel to use.
            bustype: The type of CAN bus.
            motor_idxs: List of motor IDs to control.
            timeout: Timeout for receiving messages.
            delta: The position delta to move the motors.
            seq_timeout: Delay between sending commands to different motors.
        """
        self.write_bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.motor_idxs = motor_idxs
        self.buffer_reader = can.BufferedReader()
        self.notifier = can.Notifier(self.write_bus, [self.buffer_reader])
        self.timeout = timeout
        self.delta = delta
        self.seq_timeout = seq_timeout
        self.hold_time = hold_time

    def _send(self, id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            id: The motor ID.
            data: The data to send.
        """
        can_id = send_id(id)
        assert len(data) == length, "Data length must be 8 bytes"
        print(hex(can_id), hex(int.from_bytes(data, "big")))
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.write_bus.send(message)

    def set_relative_position(self, id: int, location: float, max_dps: float = DEFAULT_MAX_DPS) -> None:
        """Sets the target location of the motor in degrees.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            location: The target location in degrees.
            max_dps: The maximum velocity in degrees per second.
        """
        # data = set_position_control(id, location, max_speed=max_dps/6) # takes max RPM instead
        data = force_position_hybrid_control(15, 0.5, location, 0, 5)
        self._send(id, bytes(data))
    
    def hold_positions(self, positions: List[float]) -> None:
        """Holds the given position for the specified hold time."""
        assert len(positions) == len(self.motor_idxs), "Number of positions must match number of motors"
        end_time = time.time() + self.hold_time
        # while time.time() < end_time:
        for idx, pos in zip(self.motor_idxs, positions):
            time.sleep(self.seq_timeout)
            self.set_relative_position(idx, pos)    
    
    def send_positions(self) -> None:
        """Sends the target positions to all motors and waits for the timeout period."""
        self.delta = -self.delta
        start_time = time.time()
        for idx in self.motor_idxs:
            time.sleep(self.seq_timeout)
            self.set_relative_position(idx, self.delta)

        while (time.time() - start_time) < self.timeout:
            time.sleep(time.time() - start_time)
        print("Send all positions")

    def receive_messages(self) -> list:
        """ TODO: Needs to be implemented
        Reads messages from the buffer within a given timeout.

        Returns:
            A list of received  messages.
        """
        received_count = 0
        messages = []

        start_time = time.time()
        while (time.time() - start_time) < self.timeout:
            message = self.buffer_reader.get_message(0.01)  # Check every 10ms
            if message:
                messages.append(message)
                print(f"Received message: {message}")
                received_count += 1

        print(f"Received {len(messages)} messages in {self.timeout} seconds")
        return messages
    
    def zero_motors(self) -> None:
        """Zeros all motors."""
        for idx in self.motor_idxs:
            data = set_zero_position(idx)
            self._send(0x7FF, bytes(data), 4)

    def policy_loop(self) -> None:
        """Continuously sends positions to motors and processes received messages."""
        with self.write_bus:
            self.zero_motors() # zero all motors
            while True:
                # self.hold_positions([90, 0, 0, 0, 0, 0])
                # self.hold_positions([45, 0, 0, 0, 0, 0])
                self.hold_positions([90])
                # self.send_positions()
                # self.receive_messages()

                # TODO:
                # self._post_process_messages()

    def _post_process_messages(self) -> None:
        """Processes received messages (placeholder for actual logic)."""
        # add logic for non responsive motors
        pass


if __name__ == "__main__":
    motor_idxs = [6]
    delta = 90
    test = TestCanBus(motor_idxs=motor_idxs, delta=delta)
    test.policy_loop()
