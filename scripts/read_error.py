#!/usr/bin/env python

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.model import Model
from firmware.motors.motor import Motors

async def main() -> None:
    async with Motors(CanIP("can0")) as motor:
        print(await motor.read_motor_status_and_errors(27))


if __name__ == "__main__":
    asyncio.run(main())
