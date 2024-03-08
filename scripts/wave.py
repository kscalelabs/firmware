#!/usr/bin/env python
# Wave a hand.

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors

positions_a = {
    1: 45,
    3: -14,
    4: 91,
    5: 0,
}

positions_b = {
    1: 73,
    3: 74,
    4: 87,
    5: 45,
}


async def main() -> None:
    async with Motors(CanIP("can0")) as motor:
        # while True:
        #     # for i in positions_a.keys():
        #     #     position = await motor.read_multi_turn_angle(i)
        #     #     print("motor:", i, "position:", position)
        #     # print()
        #     # await asyncio.sleep(5.0)

        #     for p in (positions_a, positions_b):
        #         for i, v in p.items():
        #             await motor.set_position(i, v, max_dps=360)
        #         await asyncio.sleep(0.8)

        for i in positions_a.keys():
            await motor.shutdown_motor(i)


if __name__ == "__main__":
    asyncio.run(main())

