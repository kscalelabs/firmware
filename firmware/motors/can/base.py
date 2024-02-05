""""Defines the base interface for controlling the CAN bus."""

from abc import ABC, abstractmethod


class CanBase(ABC):
    """Defines the base interface for controlling the CAN bus."""

    @abstractmethod
    def send(self, id: int, data: bytes) -> None: ...

    @abstractmethod
    def recv(self) -> tuple[int, bytes] | None: ...
