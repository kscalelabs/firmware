"""Implements the interface for talking to the motor controller."""

from typing import Callable
import functools
import asyncio
from dataclasses import dataclass
from typing import Literal
from types import TracebackType
import datetime
from typing import Self, TypeVar

from typing import ParamSpec
from firmware.motors.can.base import CanBase
from firmware.motors.can.ip import CanIP

T = TypeVar("T")
P = ParamSpec("P")

DEFAULT_MAX_DPS = 360.0


@dataclass
class Status:
    motor_id: int
    temperature: int
    torque_current: int
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


class InvalidMotorID(Exception):
    pass


def send_id(id: int) -> int:
    if not (1 <= id <= 32):
        raise InvalidMotorID(f"Motor ID {hex(id)} out of range")
    return 0x140 + id


def recv_id(id: int) -> int:
    if not (1 <= id <= 32):
        raise InvalidMotorID(f"Motor ID {hex(id)} out of range")
    return 0x240 + id


class Motors:
    def __init__(self, can: CanBase) -> None:
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

    async def reset(self, id: int) -> None:
        """Resets the motor controller.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
        """
        data = [0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))

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

    async def set_brake(self, id: int, on: bool) -> None:
        """Sets the brake on or off.

        Args:
            id: The motor ID (from 1 to 32, inclusive).
            on: Whether to turn the brake on or off.
        """
        if on:
            data = [0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            data = [0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        await self._read(id, data[0])

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

    async def get_system_version(self, id: int) -> datetime.datetime:
        """Gets the system version.

        Args:
            id: The motor ID (from 1 to 32, inclusive).

        Returns:
            The system version.
        """
        data = [0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        await self._send(id, bytes(data))
        response_data = await self._read(id, data[0])
        date_str = str(int.from_bytes(response_data[4:8], "little"))[:8]
        return datetime.datetime.strptime(date_str, "%Y%m%d")

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

    async def set_tracking_location(self, id: int, location: float) -> Status:
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

    async def set_relative_location(self, id: int, location: float, max_dps: float = DEFAULT_MAX_DPS) -> Status:
        data = [
            0xA8,
            0x00,
            *round(max_dps).to_bytes(2, "little"),
            *round(location * 100).to_bytes(4, "little", signed=True),
        ]
        await self._send(id, bytes(data))
        return await self._read_status(id, 0xA8)

    async def set_can_id(self, new_id: int, sleep_time: float = 0.1) -> None:
        """Sets the motor ID to a new value.

        The CAN bus should only have a single motor controller connected when
        running this command. In order to update the motor ID, we first
        disable the CAN filter, then send the command to update the motor ID,
        and finally re-enable the CAN filter.

        Args:
            id: The current motor ID (from 1 to 32, inclusive).
            new_id: The new motor ID (from 1 to 32, inclusive).
            sleep_time: The time to sleep between each command.
        """

        # Gets the only attached ID or raises an error if there are multiple
        # or none.
        attached_ids = await self.get_ids(sleep_time)
        if len(attached_ids) == 0:
            raise ValueError("No motor controllers found")
        if len(attached_ids) > 1:
            raise ValueError(f"Multiple motor controllers found: {attached_ids}")
        id = attached_ids[0]

        async def _set_can_filter(id: int, enabled: bool) -> None:
            data = [0x20, 0x02, 0x00, 0x00, 0x00 if enabled else 0x01, 0x00, 0x00, 0x00]
            await self._send(id, bytes(data))
            await self._read(id, data[0])

        async def _set_motor_id(new_id: int) -> None:
            data = [0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, *new_id.to_bytes(1, "little")]
            await self.can.send(0x300, bytes(data))

        await _set_can_filter(id, True)
        await asyncio.sleep(sleep_time)
        await _set_motor_id(new_id)
        await asyncio.sleep(sleep_time)
        await _set_can_filter(new_id, False)
        await asyncio.sleep(sleep_time)

    async def _send(self, id: int, data: bytes) -> None:
        can_id = send_id(id)
        assert len(data) == 8, "Data length must be 8 bytes"
        await self.can.send(can_id, data)

    async def _read(self, id: int, cmd_byte: int | None) -> bytes:
        response_data = await self.can.recv_id(recv_id(id))
        if cmd_byte is not None and response_data[0] != cmd_byte:
            raise ValueError(f"Unexpected response command byte {response_data[0]}")
        return response_data

    async def _read_status(self, id: int, cmd_byte: int) -> Status:
        response_data = await self.can.recv_id(recv_id(id))
        if response_data[0] != cmd_byte:
            raise ValueError(f"Unexpected response command byte {response_data[0]}")
        temperature = int.from_bytes(response_data[1:2], "little")
        torque_current = int.from_bytes(response_data[2:4], "little")
        shaft_velocity = int.from_bytes(response_data[4:6], "little", signed=True)
        shaft_angle = int.from_bytes(response_data[6:8], "little", signed=True)
        return Status(id, temperature, torque_current, shaft_velocity, shaft_angle)

    async def get_ids(self, timeout: float = 0.1) -> list[int]:
        """Gets all the motor IDs which return a status response."""

        async def _has_motor_id(i: int) -> bool:
            try:
                async with asyncio.timeout(timeout):
                    await self.read_status(i)
                    return True
            except asyncio.TimeoutError:
                return False

        has_id = await asyncio.gather(*[_has_motor_id(i) for i in range(1, 33)])
        return [i for i, has in enumerate(has_id, 1) if has]


async def test_motor_adhoc() -> None:
    async with Motors(CanIP("can0")) as motor:
        # await asyncio.gather(*(motor.reset(i) for i in range(1, 33)))
        print("ids:", await motor.get_ids())

        # await asyncio.gather(
        #     motor.set_absolute_location(1, 0),
        #     motor.set_absolute_location(2, 0),
        #     motor.set_absolute_location(3, 0),
        # )
        # await asyncio.sleep(0.5)

        # for _ in range(3):
        #     await asyncio.gather(
        #         motor.set_absolute_location(1, 30),
        #         motor.set_absolute_location(2, 10),
        #         motor.set_absolute_location(3, 45),
        #     )
        #     await asyncio.sleep(0.5)
        #     await asyncio.gather(
        #         motor.set_absolute_location(1, -30),
        #         motor.set_absolute_location(2, -10),
        #         motor.set_absolute_location(3, -45),
        #     )
        #     await asyncio.sleep(0.5)

        # await asyncio.gather(
        #     motor.set_absolute_location(1, 0),
        #     motor.set_absolute_location(2, 0),
        #     motor.set_absolute_location(3, 0),
        # )


if __name__ == "__main__":
    # python -m firmware.motors.motor
    asyncio.run(test_motor_adhoc())
