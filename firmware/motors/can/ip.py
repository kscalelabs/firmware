"""Implements the CAN bus interface over the CAN0 or CAN1 interfaces."""

import asyncio
from pathlib import Path
from types import TracebackType
from typing import Self

import can

from firmware.motors.can.base import CanBase


class CanIP(CanBase):
    def __init__(
        self,
        channel: str,
        interface: str = "socketcan",
        log_file: str | Path | None = None,
        read_channel: str | None = None,
    ) -> None:
        super().__init__()

        self.write_bus = can.interface.Bus(channel, interface)
        self.read_bus = self.write_bus if read_channel is None else can.interface.Bus(read_channel, interface)

        self.reader = can.AsyncBufferedReader()
        self.logger = None if log_file is None else can.Logger(log_file)
        self.notifier = can.Notifier(self.read_bus, self.listeners, loop=asyncio.get_event_loop())

    @property
    def listeners(self) -> list[can.Listener]:
        return [listener for listener in (self.reader, self.logger) if listener is not None]

    async def __aenter__(self) -> Self:
        if self.read_bus == self.write_bus:
            self.write_bus = self.write_bus.__enter__()
            self.read_bus = self.write_bus
        else:
            self.write_bus = self.write_bus.__enter__()
            self.read_bus = self.read_bus.__enter__()
        return await super().__aenter__()

    async def __aexit__(self, t: type[BaseException] | None, e: BaseException | None, tr: TracebackType | None) -> None:
        if self.read_bus == self.write_bus:
            self.write_bus.__exit__(t, e, tr)
        else:
            self.write_bus.__exit__(t, e, tr)
            self.read_bus.__exit__(t, e, tr)
        await super().__aexit__(t, e, tr)

    async def send(self, id: int, data: bytes) -> None:
        message = can.Message(
            arbitration_id=id,
            data=data,
            is_extended_id=False,
        )
        self.write_bus.send(message)

    async def recv(self) -> tuple[int, bytes]:
        message = await self.reader.get_message()
        return message.arbitration_id, message.data


async def test_can_adhoc() -> None:
    async with CanIP("can0") as can:
        await can.send(0x123, b"hello")


if __name__ == "__main__":
    # python -m firmware.motors.can.ip
    asyncio.run(test_can_adhoc())
