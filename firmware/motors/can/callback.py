"""Defines a wrapper CAN interface which supports callback."""

from typing import Awaitable, Callable

from firmware.motors.can.base import CanBase


class CanWithCallback(CanBase):
    def __init__(
        self,
        can: CanBase,
        send_callback: Callable[[int, bytes], Awaitable[None]],
        recv_callback: Callable[[int, bytes], Awaitable[None]],
    ) -> None:
        super().__init__()

        self.can = can
        self.send_callback = send_callback
        self.recv_callback = recv_callback

    async def send(self, id: int, data: bytes) -> None:
        await self.can.send(id, data)
        await self.send_callback(id, data)

    async def recv(self) -> tuple[int, bytes]:
        id, data = await self.can.recv()
        await self.recv_callback(id, data)
        return id, data
