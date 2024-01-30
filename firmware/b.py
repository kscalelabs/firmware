"""Test script using the CAN library."""

import can

bus = can.interface.Bus(interface="socketcan", channel="can0")
for _ in range(0, 1000):
    bus.send(can.Message(arbitration_id=140, data=[0x9c, 0, 0, 0, 0, 0, 0, 0]))
