"""Defines a client interface for the motor controller."""

import asyncio
import json
import logging
import random

logger = logging.getLogger(__name__)


async def send_request(host: str, port: int, motor_id: int, position: float) -> None:
    reader, writer = await asyncio.open_connection(host, port)
    request = {"type": "set_position", "motor_id": motor_id, "position": position}
    request_data = json.dumps(request).encode() + b"\n"
    writer.write(request_data)
    await writer.drain()
    response_data = await reader.readline()
    writer.close()
    await writer.wait_closed()
    response = json.loads(response_data.decode().strip())
    logger.info("Server response: %s", response)


async def main() -> None:
    host = "localhost"
    port = 8888
    motor_id = 2
    # Random position between -10 and 10
    position = random.uniform(-10, 10)
    await send_request(host, port, motor_id, position)


if __name__ == "__main__":
    # python -m firmware.app.client
    asyncio.run(main())
