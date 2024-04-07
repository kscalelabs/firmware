#!/usr/bin/env python
"""Tick all motors based on client input."""

import asyncio
import json
import logging

from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors

logger = logging.getLogger(__name__)


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, motor: Motors) -> None:
    while True:
        request_data = await reader.readline()
        if not request_data:
            break
        request = json.loads(request_data.decode().strip())
        if request["type"] == "set_position":
            motor_id = request["motor_id"]
            position = request["position"]
            await motor.set_absolute_location(motor_id, position)
            response = {"status": "success"}
        else:
            response = {"status": "invalid_request"}
        response_data = json.dumps(response).encode() + b"\n"
        writer.write(response_data)
        await writer.drain()


async def main(host: str, port: int) -> None:
    logger.info("Starting")
    async with Motors(CanIP("can0")) as motor:
        await motor.reset_all()
        server = await asyncio.start_server(lambda r, w: handle_client(r, w, motor), host, port)
        async with server:
            await server.serve_forever()


if __name__ == "__main__":
    # python -m firmware.app.server
    host = "localhost"
    port = 8888
    asyncio.run(main(host, port))
