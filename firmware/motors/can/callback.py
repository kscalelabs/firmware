"""Defines a wrapper CAN interface which supports callback."""

from types import TracebackType
from typing import Awaitable, Callable, Self

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

    async def __aenter__(self) -> Self:
        await self.can.__aenter__()
        return await super().__aenter__()

    async def __aexit__(self, t: type[BaseException] | None, e: BaseException | None, tr: TracebackType | None) -> None:
        await self.can.__aexit__(t, e, tr)
        await super().__aexit__(t, e, tr)
