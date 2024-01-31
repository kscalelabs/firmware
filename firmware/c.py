"""Receives data from the CAN bus."""

import can

print("Starting...")
with can.interface.Bus(interface="socketcan", channel="can0") as bus:
    for message in bus:
        print(message)
print("Done")
