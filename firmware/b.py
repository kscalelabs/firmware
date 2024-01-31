"""Test script using the CAN library."""

import can

with can.interface.Bus(interface="socketcan", channel="can0") as bus:
    bus.send(
        can.Message(
            arbitration_id=141,
            is_extended_id=False,
            data=[0x9C, 0, 0, 0, 0, 0, 0, 0],
        )
    )
    bus.flush_tx_buffer()
