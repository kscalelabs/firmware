"""Implements a dummy CAN bus interface for testing."""

import asyncio
import logging

from firmware.motors.can.base import CanBase

logger = logging.getLogger(__name__)


def send_to_recv_id(id: int) -> int:
    if 0x140 + 1 <= id <= 0x140 + 32:
        return id - 0x140 + 0x240
    return id


class CanDryRun(CanBase):
    def __init__(self) -> None:
        super().__init__()

        self._last_command_queue: asyncio.Queue[tuple[int, bytes]] = asyncio.Queue()

    async def send(self, id: int, data: bytes) -> None:
        logger.info("CAN: %s %s", hex(id), data.hex())
        await self._last_command_queue.put((id, data))

    async def recv(self) -> tuple[int, bytes]:
        id, data = await self._last_command_queue.get()
        return send_to_recv_id(id), data


async def test_can_adhoc() -> None:
    async with CanDryRun() as can:
        await can.send(0x123, b"hello")


if __name__ == "__main__":
    # python -m firmware.motors.can.ip
    asyncio.run(test_can_adhoc())
