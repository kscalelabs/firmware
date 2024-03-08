#!/usr/bin/env python
# Tick all motors back and forth by 2 degrees.

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors

async def main() -> None:
    flag = False
    delta = 3

    print("Starting")
    async with Motors(CanIP("can0")) as motor:
        await motor.reset_all()

        while True:
            motor_ids = await motor.get_ids(False, 0.005)
            print("Tick", flag, motor_ids)

            for motor_id in motor_ids:
                # position = await motor.read_multi_turn_encoder_position(motor_id)
                # await motor.set_absolute_location(motor_id, position + (delta if flag else -delta))
                await motor.set_relative_location(motor_id, delta if flag else -delta)

            flag = not flag
            await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())

