"""Implements a dummy CAN bus interface for testing."""

import asyncio
import logging

from firmware.motors.can.base import CanBase

logger = logging.getLogger(__name__)


class CanDryRun(CanBase):
    def __init__(self) -> None:
        super().__init__()

        self._last_command_queue: asyncio.Queue[tuple[int, bytes]] = asyncio.Queue()

    async def send(self, id: int, data: bytes) -> None:
        logger.info("CAN: %s %s", hex(id), data.hex())
        await self._last_command_queue.put((id, data))

    async def recv(self) -> tuple[int, bytes]:
        return await self._last_command_queue.get()


async def test_can_adhoc() -> None:
    async with CanDryRun() as can:
        await can.send(0x123, b"hello")


if __name__ == "__main__":
    # python -m firmware.motors.can.ip
    asyncio.run(test_can_adhoc())
