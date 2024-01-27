"""Defines utility functions for interfacing with the CAN bus."""

import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Message
from adafruit_mcp2515 import MCP2515 as CAN

STEP_VALUE = 140
GEN_VEL = 200
ID = 0x141


def to_pos(buf: bytearray) -> int:
    buf6 = buf[6]
    buf7 = buf[7]
    pos = (buf7 << 8) | buf6
    pos = 100 * pos
    print(f"gen pose: {pos}")
    return pos

def request_pos(can_bus: CAN):
    buf = bytes([0 for _ in range(8)])
    buf[0] = 0x9C
    message = Message(id=ID, data=bytes(buf), extended=False)
    can_bus.send(message)

def send_cmd(pos: int, vel: int, can_bus: CAN):
    buf = bytes([0 for _ in range(8)])
    buf[0] = 0xA4
    buf[2] = vel
    buf[3] = vel >> 8
    buf[4] = pos
    buf[5] = pos >> 8
    buf[6] = pos >> 16
    buf[7] = pos >> 24
    message = Message(id=ID, data=bytes(buf), extended=False)
    can_bus.send(message)

def listen_for_pos(can_bus: CAN, timeout_sec:float =1.0) -> int:
    with can_bus.listen(timeout_sec) as listener:
        raw_pos = listener.receive()
        if raw_pos is not None:
            print("raw pos from ", hex(raw_pos.id))
            print("raw pos data: ", raw_pos.data)
            return raw_pos

def main() -> None:
    cs = DigitalInOut(board.D5)
    cs.switch_to_output()
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

    can_bus = CAN(spi, cs, loopback=True, silent=True)

    gen_pos = 0

    request_pos(can_bus)
    time.sleep(0.05)

    raw_pose = listen_for_pos(can_bus)
    if raw_pose is not None:
        gen_pos = to_pos(raw_pose)

    time.sleep(5)

    while True:
        send_cmd(gen_pos + STEP_VALUE, GEN_VEL)
        time.sleep(0.01)

        request_pos(can_bus)
        time.sleep(0.01)

        raw_pose = listen_for_pos(can_bus)
        if raw_pose is not None:
            gen_pos = to_pos(raw_pose)

        time.sleep(1)

    # while True:
    #     with can_bus.listen(timeout=1.0) as listener:

    #         message = Message(id=0x1234ABCD, data=b"adafruit", extended=True)
    #         send_success = can_bus.send(message)
    #         print("Send success:", send_success)
    #         message_count = listener.in_waiting()
    #         print(message_count, "messages available")
    #         for _i in range(message_count):
    #             msg = listener.receive()
    #             print("Message from ", hex(msg.id))
    #             if isinstance(msg, Message):
    #                 print("message data:", msg.data)
    #             if isinstance(msg, RemoteTransmissionRequest):
    #                 print("RTR length:", msg.length)
    #     sleep(1)

if __name__ == "__main__":
    # python -m firmware.can
    main()
