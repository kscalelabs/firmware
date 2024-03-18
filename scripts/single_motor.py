#!/usr/bin/env python
"""Control a single motor from the command line."""

import asyncio
import curses
from collections import deque
from typing import Deque

from tap import Tap

from firmware.motors.can.callback import CanWithCallback
from firmware.motors.can.dry_run import CanDryRun
from firmware.motors.can.ip import CanIP
from firmware.motors.motor import Motors


class ArgumentParser(Tap):
    can_bus: int = 0  # The CAN bus to use.
    motor_id: int | None = None  # The specific motor to control.
    dry_run: bool = False  # Use the dry run interface.
    width: int = 80  # The width of the Curses window.
    height: int = 80  # The height of the Curses window.

    def configure(self) -> None:
        self.add_argument("-c", "--can_bus")
        self.add_argument("--dry_run", action="store_true")
        self.add_argument("-d", "--dry_run")


async def main() -> None:
    args = ArgumentParser().parse_args()

    try:
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        curses.curs_set(0)
        stdscr.clear()

        # Window with border.
        stdscr.resize(args.height, args.width)
        stdscr.border(0)

        middle = args.width // 2
        # for y in range(args.height):
        #     stdscr.addch(y, middle, curses.ACS_VLINE)

        stdscr.addstr(1, middle // 2 - len("Pane 1") // 2, "Pane 1")
        stdscr.addstr(1, middle + (middle // 2 - len("Pane 2") // 2), "Pane 2")
        stdscr.refresh()
        stdscr.getkey()

        # Queues for sending and receiving messages.
        send_queue: Deque[tuple[int, bytes]] = deque()
        recv_queue: Deque[tuple[int, bytes]] = deque()

        async def send_callback(id: int, data: bytes) -> None:
            send_queue.append((id, data))

        async def recv_callback(id: int, data: bytes) -> None:
            recv_queue.append((id, data))

        base_can_interface = CanDryRun() if args.dry_run else CanIP(f"can{args.can_bus}")
        can_interface = CanWithCallback(base_can_interface, send_callback, recv_callback)

        async with Motors(can_interface) as motor:
            if args.dry_run:
                motor_id = 1
            elif args.motor_id is not None:
                motor_id = args.motor_id
            else:
                motor_id = await motor.get_single_motor_id()

            # Main loop.
            while True:
                stdscr.clear()
                stdscr.addstr(0, 0, f"Motor {motor_id}")
                stdscr.addstr(1, 0, "Press 'q' to quit")
                stdscr.addstr(2, 0, "Press 'w' to move forward")
                stdscr.addstr(3, 0, "Press 's' to move backward")
                stdscr.refresh()

                key = stdscr.getch()
                if key == ord("q"):
                    break

    finally:
        if "stdscr" in locals():
            stdscr.keypad(False)
            curses.echo()
            curses.nocbreak()
            curses.endwin()


if __name__ == "__main__":
    # python -m scripts.single_motor
    asyncio.run(main())
