"""Takes in responses from the Q&A return type and interprets them."""

import struct
from typing import Dict, Union

# Defintions

ERROR_MAP = {
    0: "No Errors",
    1: "Motor Overheating",
    2: "Motor Overcurrent",
    3: "Motor Voltage too low",
    4: "Motor Encoder Error",
    6: "Motor brake voltage too high",
    7: "DRV Driver Error",
}


QUERY_MAP = {0: "Reserved", 1: "Angle", 2: "Speed", 3: "Current", 4: "Power", 5: "Accel", 6: "Current Flux"}


CONFIG_MAP = {
    0: "Reserved",
    1: "Configuring System Acceleration",
    2: "Configuring Flux Observation Gain",
    3: "Configuring Damping Coefficient",
}

CONFIG_STATUS_MAP = {0: "Failure", 1: "Success"}


def get_message_type(msg: bytes) -> int:
    """Returns the message type of the message."""
    for i in range(1, 6):
        if msg[0] >> 5 == i:
            return i
    return -1


def position_speed_message(msg: bytes) -> dict:
    """Message Type 1.

    Interprets the message type 1 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    """

    def get_position(data: int) -> float:
        return data * (25.0 / 65536.0) - 12.5

    def get_speed(data: int) -> float:
        return data * (36.0 / 4095.0) - 18.0

    def get_current(data: int) -> float:
        return data * (140.0 / 4095) - 70.0

    def get_temp(data: int) -> float:
        return (data - 50.0) / 2.0

    def get_mos_temp(data: int) -> float:
        return (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_pos = int.from_bytes(msg[1:3], "big")
    motor_speed = int.from_bytes(msg[3:5], "big") >> 4
    motor_current = int.from_bytes(msg[4:6], "big") & 0xFFF
    motor_temp: int = int(((msg[6]) - 50) / 2)
    motor_mos_temp: int = int(((msg[7]) - 50) / 2)

    return {
        "Message Type": 1,
        "Error": ERROR_MAP[error],
        "Position": get_position(motor_pos),
        "Speed": get_speed(motor_speed),
        "Current": get_current(motor_current),
        "Temperature": get_temp(motor_temp),
        "MOS": get_mos_temp(motor_mos_temp),
    }


def position_message(msg: bytes) -> dict:
    """Message Type 2.

    Interprets the message type 2 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results

    """

    def get_position(data: int) -> float:
        return data

    def get_current(data: int) -> float:
        return data / 10.0

    def get_temp(data: int) -> float:
        return (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_pos = struct.unpack("!f", msg[1:5])[0]
    motor_current = int.from_bytes(msg[5:7])
    motor_temp = msg[7]

    return {
        "Message Type": 2,
        "Error": ERROR_MAP[error],
        "Position": get_position(motor_pos),
        "Current": get_current(motor_current),
        "Temperature": get_temp(motor_temp),
    }


def speed_message(msg: bytes) -> dict:
    """Message Type 3.

    Interprets the message type 3 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    """

    def get_speed(data: int) -> float:
        return data

    def get_current(data: int) -> float:
        return data / 10.0

    def get_temp(data: int) -> float:
        return (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_speed = struct.unpack("!f", msg[1:5])[0]
    motor_current = int.from_bytes(msg[5:7])
    motor_temp = msg[7]

    return {
        "Message Type": 3,
        "Error": ERROR_MAP[error],
        "Speed": get_speed(motor_speed),
        "Current": get_current(motor_current),
        "Temperature": get_temp(motor_temp),
    }


def configuration_message(msg: bytes) -> dict:
    """Message Type 4.

    Interprets the message type 4 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results

    """
    error = msg[0] & 0x1F
    configuration_code = msg[1]
    configuration_status = msg[2]

    return {
        "Message Type": 4,
        "Error": ERROR_MAP[error],
        "Configuration Code": CONFIG_MAP[configuration_code],
        "Configuration Status": CONFIG_STATUS_MAP[configuration_status],
    }


def custom_message(msg: bytes) -> dict:
    """Message Type 5.

    Interprets the message type 5 and returns a list of the results
    Args:
        msg: bytes: The message to interpret.

    Returns:
        dictionary of the message results

    """
    error = msg[0] & 0x1F
    query_code = msg[1]
    data = bytes(msg[2:])
    data = struct.unpack("!f", data)[0]

    return {"Message Type": 5, "Error": ERROR_MAP[error], "Query Code": QUERY_MAP[query_code], "Data": data}


MESSAGE_MAP = {
    1: position_speed_message,
    2: position_message,
    3: speed_message,
    4: configuration_message,
    5: custom_message,
}


def valid_message(msg: bytes) -> bool:
    """Checks if the message is valid by checking the message type.

    Args:
        msg: bytes: The message to check

    Returns:
        bool: True if the message is valid, False otherwise

    """
    return get_message_type(msg) in MESSAGE_MAP


def read_result(msg: bytes) -> Union[Dict, None]:
    """Reads the result of the message and returns a list of the results regardless of Message Type.

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results

    """
    message_type = get_message_type(msg)
    if valid_message(msg):
        return MESSAGE_MAP[message_type](msg)
    return None


if __name__ == "__main__":
    vector = bytes([0xA0, 0x01, 0x39, 0xF7, 0x24, 0x7D])
    print(read_result(vector))
