"""Implements the CAN bus interface for the Raspberry Pi."""

from adafruit_mcp2515.canio import Message, RemoteTransmissionRequest
import busio
from firmware.motors.can.base import CanBase
from adafruit_mcp2515 import MCP2515 as CAN


class CanRaspi(CanBase):
    def __init__(self, spi: busio.SPI, bus: CAN) -> None:
        super().__init__()

        self.spi = spi
        self.bus = bus

    def send(self, id: int, data: bytes) -> None:
        message = Message(id=id, data=data, extended=False)
        self.bus.send(message)

    def recv(self) -> tuple[int, bytes] | None:
        message = self.bus.read_message()
        if message is None:
            return None
        if isinstance(message, Message):
            return message.id, bytes(message.data)
        if isinstance(message, RemoteTransmissionRequest):
            return message.id, bytes([0] * message.length)
        raise ValueError("Unexpected message type")


def test_can_adhoc() -> None:
    can = CanRaspi()


if __name__ == "__main__":
    test_can_adhoc()
