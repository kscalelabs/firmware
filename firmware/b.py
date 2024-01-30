"""Test script using the CAN library."""

import can

with can.interface.Bus(interface="socketcan", channel="can0") as bus:
    for i in range(0, 1000):
        bus.send(
            can.Message(
                channel=i,
                is_extended_id=False,
                is_rx=False,
                data=[0x9C, 0, 0, 0, 0, 0, 0, 0],
            )
        )
        bus.flush_tx_buffer()
