"""Implements the interface for talking to the motor controller."""

import asyncio
from dataclasses import dataclass
from types import TracebackType
from typing import Literal, NotRequired, ParamSpec, Self, TypedDict, TypeVar

from firmware.motors.can.base import CanBase
from firmware.motors.can.ip import CanIP

T = TypeVar("T")
P = ParamSpec("P")

DEFAULT_MAX_DPS = 360.0

MotorError = Literal[
    "motor_stall",
    "low_pressure",
    "overvoltage",
    "overcurrent",
    "power_overrun",
    "calibration_parameter_write_error",
    "speeding",
    "motor_temperature_over_temperature",
    "encoder_calibration_error",
]

OperatingMode = Literal["current_loop", "speed_loop", "position_loop"]


@dataclass
class MotorStatus:
    temperature: int
    mos_temperature: int
    brake_is_locked: bool
    voltage: float
    errors: list[MotorError]


@dataclass
class Status:
    motor_id: int
    temperature: int
    torque_current: float
    shaft_velocity: int
    shaft_angle: int


@dataclass
class PID:
    current_kp: int
    current_ki: int
    speed_kp: int
    speed_ki: int
    position_kp: int
    position_ki: int


@dataclass
class EncoderPositions:
    position: int
    original_position: int
    zero_bias: int


class MotionModeArgs(TypedDict):
    desired_position: NotRequired[float]
    desired_velocity: NotRequired[float]
    feedforward_torque: NotRequired[float]
    kp: NotRequired[int]
    kd: NotRequired[int]


@dataclass
class MotionModeResponse:
    position: float
    velocity: float
    torque: float


class InvalidMotorIDError(Exception):
    pass


def send_id(id: int) -> int:
    if not (1 <= id <= 32):
        raise InvalidMotorIDError(f"Motor ID {hex(id)} out of range")
    return 0x140 + id


def recv_id(id: int) -> int:
    if not (1 <= id <= 32):
        raise InvalidMotorIDError(f"Motor ID {hex(id)} out of range")
    return 0x240 + id


class Motors:
    def __init__(self, can: CanBase, dry_run: bool = False) -> None:
        super().__init__()

        self.can = can

    async def __aenter__(self) -> Self:
        self.can = await self.can.__aenter__()
        return self

    async def __aexit__(self, t: type[BaseException] | None, e: BaseException | None, tr: TracebackType | None) -> None:
        await self.can.__aexit__(t, e, tr)

    async def read_status(self, id: int) -> Status:
        """Reads the status of the motor.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The status of the motor.
        """
        data = [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0x9C)

    async def read_single_turn_encoder(self, id: int) -> EncoderPositions:
        """Reads the single-turn encoder positions.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The single-turn encoder positions.
        """
        data = [0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return EncoderPositions(
            position=int.from_bytes(response_data[2:4], "little"),
            original_position=int.from_bytes(response_data[4:6], "little"),
            zero_bias=int.from_bytes(response_data[6:8], "little"),
        )

    async def read_multi_turn_angle(self, id: int) -> float:
        """Reads the multi-turn encoder angle in degrees.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The multi-turn encoder angle in degrees.
        """
        data = [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little", signed=True) * 0.01

    async def read_multi_turn_encoder_position(self, id: int) -> int:
        """Reads the raw multi-turn encoder position.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The raw multi-turn encoder position.
        """
        data = [0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little", signed=True)

    async def read_multi_turn_encoder_original_position(self, id: int) -> int:
        """Reads the raw multi-turn encoder position.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The raw multi-turn encoder position.
        """
        data = [0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little", signed=True)

    async def read_motor_status_and_errors(self, id: int) -> MotorStatus:
        """Reads the motor status and errors.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The motor status and errors.
        """
        data = [0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        errors: list[MotorError] = []
        error_status = int.from_bytes(response_data[6:8], "little")
        if error_status & 0x0002:
            errors.append("motor_stall")
        if error_status & 0x0004:
            errors.append("low_pressure")
        if error_status & 0x0008:
            errors.append("overvoltage")
        if error_status & 0x0010:
            errors.append("overcurrent")
        if error_status & 0x0020:
            errors.append("power_overrun")
        if error_status & 0x0040:
            errors.append("calibration_parameter_write_error")
        if error_status & 0x0080:
            errors.append("speeding")
        if error_status & 0x0100:
            errors.append("motor_temperature_over_temperature")
        if error_status & 0x0200:
            errors.append("encoder_calibration_error")
        return MotorStatus(
            temperature=int.from_bytes(response_data[1:2], "little"),
            mos_temperature=int.from_bytes(response_data[2:3], "little"),
            brake_is_locked=bool(response_data[3]),
            voltage=int.from_bytes(response_data[4:6], "little") * 0.1,
            errors=errors,
        )

    async def system_operating_mode(self, id: int) -> OperatingMode:
        data = [0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        mode = response_data[7]
        match mode:
            case 0x01:
                return "current_loop"
            case 0x02:
                return "speed_loop"
            case 0x03:
                return "position_loop"
            case _:
                raise ValueError(f"Invalid operating mode {mode}")

    async def reset(self, id: int) -> None:
        """Resets the motor controller.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
        """
        data = [0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))

    async def reset_all(self, wait_time: float = 0.005) -> None:
        for motor_id in range(1, 33):
            await self.reset(motor_id)
            await asyncio.sleep(wait_time)

    async def get_system_runtime(self, id: int) -> float:
        """Gets the system runtime in seconds.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The system runtime in seconds.
        """
        data = [0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little") * 0.001

    async def set_interrupt_protection(self, id: int, ms: float) -> None:
        """Sets the interrupt protection.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            ms: The interrupt protection time in milliseconds.
        """
        data = [0xB3, 0x00, 0x00, 0x00, *round(ms).to_bytes(4, "little")]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def get_power(self, id: int) -> float:
        """Gets the current power to the motor, in Watts.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The power to the motor in Watts.
        """
        data = [0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[6:8], "little", signed=True) * 0.1  # Not sure about this...

    async def read_pid(self, id: int) -> PID:
        """Reads the motor PID values.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The PID values.
        """
        data = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return PID(
            current_kp=int.from_bytes(response_data[2:3], "little"),
            current_ki=int.from_bytes(response_data[3:4], "little"),
            speed_kp=int.from_bytes(response_data[4:5], "little"),
            speed_ki=int.from_bytes(response_data[5:6], "little"),
            position_kp=int.from_bytes(response_data[6:7], "little"),
            position_ki=int.from_bytes(response_data[7:8], "little"),
        )

    async def write_pid_to_ram(self, id: int, pid: PID) -> None:
        """Writes the motor PID values to RAM.

        The PID values will not be saved after a power cycle.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            pid: The PID values.
        """
        data = [
            0x31,
            0x00,
            *pid.current_kp.to_bytes(1, "little"),
            *pid.current_ki.to_bytes(1, "little"),
            *pid.speed_kp.to_bytes(1, "little"),
            *pid.speed_ki.to_bytes(1, "little"),
            *pid.position_kp.to_bytes(1, "little"),
            *pid.position_ki.to_bytes(1, "little"),
        ]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def write_pid_to_rom(self, id: int, pid: PID) -> None:
        """Writes the motor PID values to ROM.

        The PID values will be saved after a power cycle.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            pid: The PID values.
        """
        data = [
            0x32,
            0x00,
            *pid.current_kp.to_bytes(1, "little"),
            *pid.current_ki.to_bytes(1, "little"),
            *pid.speed_kp.to_bytes(1, "little"),
            *pid.speed_ki.to_bytes(1, "little"),
            *pid.position_kp.to_bytes(1, "little"),
            *pid.position_ki.to_bytes(1, "little"),
        ]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def read_acceleration(self, id: int) -> float:
        """Reads the acceleration of the motor in degrees per second squared.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The acceleration of the motor in degrees per second squared.
        """
        data = [0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return float(int.from_bytes(response_data[4:8], "little", signed=True))

    async def write_acceleration(
        self,
        id: int,
        function: Literal["pos_acc", "pos_dec", "speed_acc", "speed_dec"],
        dpss: float,
    ) -> None:
        """Writes the acceleration of the motor in degrees per second squared.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            function: The acceleration function to write.
            dpss: The acceleration in degrees per second squared.
        """
        match function:
            case "pos_acc":
                index = 0x00
            case "pos_dec":
                index = 0x01
            case "speed_acc":
                index = 0x02
            case "speed_dec":
                index = 0x03
            case _:
                raise ValueError(f"Invalid function {function}")
        data = [0x43, index, 0x00, 0x00, *round(dpss).to_bytes(4, "little", signed=True)]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def read_multi_turn_encoder_zero_offset(self, id: int) -> int:
        """Reads the raw multi-turn encoder position.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The raw multi-turn encoder position.
        """
        data = [0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little", signed=True)

    async def write_multi_turn_encoder_zero_offset(self, id: int, offset: int | None = None) -> int:
        """Writes the raw multi-turn encoder position.

        Restart the motor after this command is executed to take effect.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            offset: The raw multi-turn encoder position. If None, the current
                position is used.
        """
        if offset is None:
            data = [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            data = [0x63, 0x00, 0x00, 0x00, *offset.to_bytes(4, "little", signed=True)]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return int.from_bytes(response_data[4:8], "little", signed=True)

    async def shutdown(self, id: int) -> None:
        """Shuts down the motor.

        Turns off the motor output and clears the motor running state.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
        """
        data = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def stop(self, id: int) -> None:
        """Stops down the motor.

        Stops the motor from spinning, but doesn't clear the running state.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
        """
        data = [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def set_brake(self, id: int, on: bool) -> None:
        """Sets the brake on or off.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            on: Whether to turn the brake on or off.
        """
        data = [0x78 if on else 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

    async def set_torque(self, id: int, torque: float) -> Status:
        """Sets the target torque of the motor, as current, in amps.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            torque: The target motor current, in amps.

        Returns:
            The status of the motor after the command is executed.
        """
        data = [0xA1, 0x00, 0x00, 0x00, *round(torque * 100).to_bytes(2, "little", signed=True), 0x00, 0x00]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA1)

    async def set_velocity(self, id: int, dps: float) -> Status:
        """Sets the target velocity of the motor in degrees per second.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            dps: The target velocity in degrees per second.

        Returns:
            The status of the motor after the command is executed.
        """
        data = [0xA2, 0x00, 0x00, 0x00, *round(dps * 100).to_bytes(4, "little", signed=True)]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA2)

    async def get_system_version(self, id: int) -> str:
        """Gets the system version.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The system version.
        """
        data = [0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        return str(int.from_bytes(response_data[4:8], "little"))

    async def set_position(self, id: int, position: float, max_dps: float = DEFAULT_MAX_DPS) -> Status:
        return await self.set_absolute_location(id, position, max_dps)

    async def set_absolute_location(self, id: int, location: float, max_dps: float = DEFAULT_MAX_DPS) -> Status:
        """Sets the target location of the motor in degrees.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            location: The target location in degrees.
            max_dps: The maximum velocity in degrees per second.

        Returns:
            The status of the motor after the command is executed.
        """
        data = [
            0xA4,
            0x00,
            *round(max_dps).to_bytes(2, "little"),
            *round(location * 100).to_bytes(4, "little", signed=True),
        ]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA4)

    async def set_tracking_position(self, id: int, location: float) -> Status:
        """Sets the target location of the motor in degrees.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            location: The target location in degrees.

        Returns:
            The status of the motor after the command is executed.
        """
        data = [0xA3, 0x00, 0x00, 0x00, *round(location * 100).to_bytes(4, "little", signed=True)]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA3)

    async def set_relative_position(self, id: int, location: float, max_dps: float = DEFAULT_MAX_DPS) -> Status:
        data = [
            0xA8,
            0x00,
            *round(max_dps).to_bytes(2, "little"),
            *round(location * 100).to_bytes(4, "little", signed=True),
        ]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA8)

    async def _send(self, id: int, data: bytes) -> None:
        can_id = send_id(id)
        assert len(data) == 8, "Data length must be 8 bytes"
        await self.can.send(can_id, data)

    async def _read(self, id: int, cmd_byte: int | None) -> bytes:
        response_data = await self.can.recv_id(recv_id(id))
        if cmd_byte is not None and response_data[0] != cmd_byte:
            raise ValueError(f"Unexpected response command byte {response_data[0]:02x} != {cmd_byte:02x}")
        return response_data

    async def _read_status(self, id: int, cmd_byte: int | None) -> Status:
        response_data = await self.can.recv_id(recv_id(id))
        if cmd_byte is not None and response_data[0] != cmd_byte:
            raise ValueError(f"Unexpected response command byte {response_data[0]:02x} != {cmd_byte:02x}")
        temperature = int.from_bytes(response_data[1:2], "little")
        torque_current = int.from_bytes(response_data[2:4], "little") / 100
        shaft_velocity = int.from_bytes(response_data[4:6], "little", signed=True)
        shaft_angle = int.from_bytes(response_data[6:8], "little", signed=True)
        return Status(id, temperature, torque_current, shaft_velocity, shaft_angle)

    async def get_ids(self, first_motor_timeout: float = 5.0, other_motor_timeout: float = 0.05) -> list[int]:
        """Gets all the motor IDs which return a status response.

        This function works by sending out read status commands to each motor.
        Any motors which return a status response are considered to be attached.

        Args:
            first_motor_timeout: The amount of time to wait for the first
                motor to respond.
            other_motor_timeout: The amount fo tiem to wait for any additional
                motors to respond, after getting a response from the first
                motor.

        Returns:
            The motor IDs which are attached.
        """
        await self.can.send(0x280, bytes([0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        motor_ids: list[int] = []
        while True:
            try:
                async with asyncio.timeout(first_motor_timeout if len(motor_ids) == 0 else other_motor_timeout):
                    response_id, response_data = await self.can.recv()
                    if response_data[0] != 0x9C:
                        raise ValueError(f"Unexpected response command byte {response_data[0]:02x} != 0x9C")
                    motor_ids.append(response_id - 0x240)
            except asyncio.TimeoutError:
                break
        return motor_ids

    async def get_single_motor_id(self, first_motor_timeout: float = 5.0, other_motor_timeout: float = 0.05) -> int:
        """Gets a single motor ID.

        This function works by sending out read status commands to each motor.
        If there are no returned values or multiple returned values, it will
        throw an error, ensuring that only a single motor is attached.

        Args:
            first_motor_timeout: The amount of time to wait for the first
                motor to respond.
            other_motor_timeout: The amount fo tiem to wait for any additional
                motors to respond, after getting a response from the first
                motor.

        Returns:
            The motor ID.
        """
        motor_ids = await self.get_ids(first_motor_timeout, other_motor_timeout)
        if len(motor_ids) == 0:
            raise RuntimeError("Didn't find any attached motors")
        if len(motor_ids) > 1:
            raise RuntimeError(f"Found multiple attached motors: {motor_ids}")
        return motor_ids[0]


async def test_motor_adhoc() -> None:
    async with Motors(CanIP("can0")) as motor:
        motor_ids = await motor.get_ids(False)

        # for motor_id in motor_ids:
        #     await motor.set_absolute_location(motor_id, 0)

        print(motor_ids)


if __name__ == "__main__":
    # python -m firmware.motors.motor
    asyncio.run(test_motor_adhoc())
