"""Defines the base interface for controlling the CAN bus."""

import asyncio
from abc import ABC, abstractmethod
from collections import defaultdict, deque
from types import TracebackType
from typing import Deque


class CanBase(ABC):
    """Defines the base interface for controlling the CAN bus.

    Each CAN bus interface should implement a ``send`` and ``recv`` method.
    Both methods should be blocking.
    """

    def __init__(self) -> None:
        super().__init__()

        self._recv_queues: dict[int, Deque[bytes]] = defaultdict(deque)
        self._recv_lock = asyncio.Lock()

    @abstractmethod
    async def send(self, id: int, data: bytes) -> None: ...

    @abstractmethod
    async def recv(self) -> tuple[int, bytes]: ...

    async def __aenter__(self) -> "CanBase":
        return self

    async def __aexit__(self, t: type[BaseException] | None, e: BaseException | None, tr: TracebackType | None) -> None:
        pass

    async def recv_id(self, id: int) -> bytes:
        """Receive a message with a specific ID.

        This function is used when multiple messages were sent with different
        IDs.

        Args:
            id: The ID of the message to receive.

        Returns:
            The data of the message.
        """
        if self._recv_queues[id]:
            return self._recv_queues[id].popleft()
        while True:
            async with self._recv_lock:
                if self._recv_queues[id]:
                    return self._recv_queues[id].popleft()
                msg_id, data = await self.recv()
                if msg_id == id:
                    return data
                self._recv_queues[msg_id].append(data)
