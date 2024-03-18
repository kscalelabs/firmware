#!/usr/bin/env python
"""Writes the current motor positions as zero."""

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.model import Model
from firmware.motors.motor import Motors


async def main() -> None:
    # motors = Model.motor_ids
    motors = Model.left_leg.motor_ids + Model.right_leg.motor_ids

    async with Motors(CanIP("can0")) as motor:
        # for i in motors:
        #     await motor.write_multi_turn_encoder_zero_offset(i)
        #     await asyncio.sleep(0.1)

        # for i in motors:
        #     await motor.reset(i)
        #     await asyncio.sleep(0.1)

        # for i in motors:
        #     print(i, await motor.read_multi_turn_angle(i))
        #     await asyncio.sleep(0.1)

        for i in motors:
            await motor.set_relative_location(i, 0)


if __name__ == "__main__":
    asyncio.run(main())
