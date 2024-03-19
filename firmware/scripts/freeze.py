#!/usr/bin/env python
# Tick all motors back and forth by 2 degrees.

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors


async def main() -> None:
    motors = [
        1,
        2,
        8,
        3,
        9,
        4,
        10,
        16,
        22,
        17,
        23,
        18,
        24,
        19,
        25,
        20,
        26,
        21,
        27,
    ]

    async with Motors(CanIP("can0")) as motor:
        for i in motors:
            await motor.reset(i)

        # for i in motors:
        #     await motor.set_relative_location(i, 5)


if __name__ == "__main__":
    asyncio.run(main())
