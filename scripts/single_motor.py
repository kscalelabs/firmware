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
    height: int = 24  # The height of the Curses window.
    timeout: float = 1.0  # The timeout for each command.

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

        hmiddle = args.width // 2
        vmiddle = args.height // 2

        # Vertical line in the middle.
        stdscr.addch(0, hmiddle, curses.ACS_TTEE)
        for y in range(1, args.height - 1):
            stdscr.addch(y, hmiddle, curses.ACS_VLINE)
        stdscr.addch(args.height - 1, hmiddle, curses.ACS_BTEE)
        stdscr.addstr(1, hmiddle // 2 - len("Commands") // 2, "Commands")

        def write_commands(commands: list[str]) -> None:
            for y, command in enumerate(commands):
                command = command[:hmiddle - 3].ljust(hmiddle - 3)
                stdscr.addstr(y + 3, 2, command)

        write_commands(
            [
                "q: Quit the program",
                "m <n>: Move by N degrees",
                "v <n>: Set velocity to N RPM",
            ],
        )

        # Separate the TX and RX panes top-to-bottom.
        stdscr.addch(vmiddle, hmiddle, curses.ACS_LTEE)
        for x in range(hmiddle + 1, args.width - 1):
            stdscr.addch(vmiddle, x, curses.ACS_HLINE)
        stdscr.addch(vmiddle, args.width - 1, curses.ACS_RTEE)
        stdscr.addstr(1, hmiddle + (hmiddle // 2 - len("TX") // 2), "TX")
        stdscr.addstr(1 + vmiddle, hmiddle + (hmiddle // 2 - len("RX") // 2), "RX")

        tx_messages: Deque[str] = deque()
        rx_messages: Deque[str] = deque()

        def add_tx(message: str) -> None:
            tx_messages.appendleft(message)
            if len(tx_messages) > vmiddle - 2:
                tx_messages.pop()
            for y, message in enumerate(tx_messages):
                message = message[:hmiddle - 4].ljust(hmiddle - 4)
                stdscr.addstr(2 + y, hmiddle + 2, message)

        def add_rx(message: str) -> None:
            rx_messages.appendleft(message)
            if len(rx_messages) > vmiddle - 2:
                rx_messages.pop()
            for y, message in enumerate(rx_messages):
                message = message[:hmiddle - 4].ljust(hmiddle - 4)
                stdscr.addstr(2 + vmiddle + y, hmiddle + 2, message)

        # Segment at the base to enter commands.
        stdscr.addch(args.height - 4, 0, curses.ACS_LTEE)
        for x in range(1, hmiddle):
            stdscr.addch(args.height - 4, x, curses.ACS_HLINE)
        stdscr.addch(args.height - 4, hmiddle, curses.ACS_RTEE)
        stdscr.addstr(args.height - 3, 2, "Enter command:")

        def set_command(command: str) -> None:
            cmd_padded = command[: hmiddle - 3].ljust(hmiddle - 3)
            stdscr.addstr(args.height - 2, 2, cmd_padded)

        async def run_command(command: str) -> None:
            command = command.lower()
            if command == "q":
                raise KeyboardInterrupt
            try:
                async with asyncio.timeout(args.timeout):
                    if command.startswith("m "):
                        degrees = float(command[2:].strip())
                        await motor.set_position(motor_id, degrees)
                    elif command.startswith("v "):
                        rpm = float(command[2:].strip())
                        await motor.set_velocity(motor_id, rpm)
                    else:
                        raise ValueError
            except asyncio.TimeoutError:
                add_tx(f"Timeout: {command}")
            except Exception:
                add_tx(f"Bad command: {command}")

        # Writes the screen.
        stdscr.refresh()

        async def send_callback(id: int, data: bytes) -> None:
            data_str = data.hex()
            data_str = " ".join(data_str[i : i + 2] for i in range(0, len(data_str), 2))
            add_tx(f"{id:03x} {data_str}")

        async def recv_callback(id: int, data: bytes) -> None:
            data_str = data.hex()
            data_str = " ".join(data_str[i : i + 2] for i in range(0, len(data_str), 2))
            add_rx(f"{id:03x} {data_str}")

        base_can_interface = CanDryRun() if args.dry_run else CanIP(f"can{args.can_bus}")
        can_interface = CanWithCallback(base_can_interface, send_callback, recv_callback)

        command_str = ""

        async with Motors(can_interface) as motor:
            if args.dry_run:
                motor_id = 1
            elif args.motor_id is not None:
                motor_id = args.motor_id
            else:
                motor_id = await motor.get_single_motor_id()

            # Main loop.
            while True:
                key = stdscr.getch()
                if chr(key) in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ._-":
                    command_str += chr(key)
                    set_command(command_str)
                elif key in (curses.KEY_BACKSPACE, 127):
                    command_str = command_str[:-1]
                    set_command(command_str)
                elif key in (curses.KEY_ENTER, ord("\n")):
                    await run_command(command_str)
                    command_str = ""
                    set_command(command_str)
                else:
                    set_command(chr(key))

    finally:
        if "stdscr" in locals():
            stdscr.keypad(False)
            curses.echo()
            curses.nocbreak()
            curses.endwin()


if __name__ == "__main__":
    # python -m scripts.single_motor
    asyncio.run(main())
