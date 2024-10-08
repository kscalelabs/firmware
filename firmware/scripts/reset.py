#!/usr/bin/env python
"""Resets all the motors."""

import asyncio

from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors


async def main() -> None:
    async with Motors(CanIP("can0")) as motor:
        await motor.reset_all()


if __name__ == "__main__":
    asyncio.run(main())
