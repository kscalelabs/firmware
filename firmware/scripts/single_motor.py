#!/usr/bin/env python
"""Control a single motor from the command line."""

import asyncio
import curses
import re
from collections import deque
from typing import Deque

from tap import Tap

from firmware.motors.can.callback import CanWithCallback
from firmware.motors.can.dry_run import CanDryRun
from firmware.motors.can.ip import CanIP
from firmware.motors.motor import MotionModeArgs, Motors


class ArgumentParser(Tap):
    can_bus: int = 0  # The CAN bus to use.
    can_read_bus: int = 1  # The CAN bus to use for reading.
    motor_id: int | None = None  # The specific motor to control.
    dry_run: bool = False  # Use the dry run interface.
    width: int = 80  # The width of the Curses window.
    height: int = 25  # The height of the Curses window.
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
        stdscr.nodelay(True)
        curses.start_color()
        curses.use_default_colors()
        curses.curs_set(0)
        stdscr.clear()

        # Window with border.
        stdscr.resize(args.height, args.width)

        # Color pairs.
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.init_pair(3, curses.COLOR_RED, -1)

        hmiddle = args.width // 2
        vmiddle = args.height // 2

        # Derived left and right panes.
        lwin = stdscr.derwin(args.height, hmiddle, 0, 0)
        rwin = stdscr.derwin(args.height, hmiddle, 0, args.width - hmiddle)

        # Command window.
        cmdwin = lwin.derwin(args.height - 3, hmiddle, 0, 0)
        cmdwin.border(0)
        cmdwin.addstr(0, hmiddle // 2 - len(" Commands ") // 2, " Commands ", curses.color_pair(1))

        # Current command window.
        curwin = lwin.derwin(3, hmiddle, args.height - 3, 0)
        curwin.border(0)
        curwin.addstr(0, hmiddle // 2 - len(" Enter command: ") // 2, " Enter command ", curses.color_pair(1))

        # TX window.
        txwin = rwin.derwin(vmiddle, hmiddle, 0, 0)

        # RX window.
        rxwin = rwin.derwin(args.height - vmiddle, hmiddle, vmiddle, 0)

        command_win_offset_h = 0
        command_win_offset_w = 0

        commands = {
            "q": "Quit the program",
            "c": "Clear the TX and RX panes",
            "n <p> (<v>) (<t>)": "Motor motion command",
            "m <n>": "Move by N degrees",
            "v <n>": "Set velocity to N degrees / second",
            "a <n>": "Set absolute position to N degrees",
            "t <n>": "Set tracking position to N degrees",
            "reset": "Reset the motor",
            "r a": "Read acceleration",
            "r i": "Read single-turn encoder",
            "r m": "Read multi-turn encoder angle",
            "r p": "Reads multi-turn encoder position",
            "r o": "Reads multi-turn encoder original position",
            "r z": "Reads multi-turn encoder zero offset",
            "r s": "Read motor status and errors",
            "r t": "Get system runtime",
            "r w": "Get the motor power",
            "r pid": "Read the PID values",
            "w z": "Write multi-turn encoder zero offset",
            "w z <n>": "Write multi-turn encoder zero offset value",
            "w pid <v/p/c> <ki/kp> <n>": "Write PID value",
            "version": "Gets the system version",
            "shutdown": "Shutdown the motor",
            "stop": "Stop the motor",
        }
        command_list = list(commands.items())

        def write_commands() -> None:
            height, width = cmdwin.getmaxyx()
            for y, (key, value) in enumerate(command_list[command_win_offset_h : command_win_offset_h + height - 2], 1):
                key = f"{key}: "
                cmdwin.addstr(y, 2, key, curses.color_pair(1))
                slen = width - len(key) - 4
                value = value[command_win_offset_w : slen + command_win_offset_w].ljust(slen)
                cmdwin.addstr(y, len(key) + 2, value)
            cmdwin.refresh()

        def move_command_win_up() -> None:
            nonlocal command_win_offset_h
            if command_win_offset_h > 0:
                command_win_offset_h -= 1
                write_commands()

        def move_command_win_down() -> None:
            nonlocal command_win_offset_h
            height, _ = cmdwin.getmaxyx()
            if command_win_offset_h < len(command_list) - height + 2:
                command_win_offset_h += 1
                write_commands()

        def move_command_win_left() -> None:
            nonlocal command_win_offset_w
            if command_win_offset_w > 0:
                command_win_offset_w -= 1
                write_commands()

        def move_command_win_right() -> None:
            nonlocal command_win_offset_w
            command_win_offset_w += 1
            write_commands()

        write_commands()

        tx_messages: Deque[tuple[int, str]] = deque()
        rx_messages: Deque[tuple[int, str]] = deque()
        tx_id = 0
        rx_id = 0
        tx_lock = asyncio.Lock()
        rx_lock = asyncio.Lock()

        def clear_tx() -> None:
            txwin.clear()
            txwin.border(0)
            txwin.addstr(0, hmiddle // 2 - len(" TX ") // 2, " TX ", curses.color_pair(2))
            txwin.refresh()
            tx_messages.clear()

        def clear_rx() -> None:
            rxwin.clear()
            rxwin.border(0)
            rxwin.addstr(0, hmiddle // 2 - len(" RX ") // 2, " RX ", curses.color_pair(2))
            rxwin.refresh()
            rx_messages.clear()

        clear_tx()
        clear_rx()

        def write_rx() -> None:
            height, width = rxwin.getmaxyx()
            while len(rx_messages) > height - 2:
                rx_messages.pop()
            for y, (i, message) in enumerate(rx_messages, 1):
                i_str = f"{i}: "
                rxwin.addstr(y, 2, i_str, curses.color_pair(2))
                slen = width - 4 - len(i_str)
                rxwin.addstr(y, 2 + len(i_str), message[:slen].ljust(slen))
            rxwin.refresh()

        def write_tx() -> None:
            height, width = txwin.getmaxyx()
            while len(tx_messages) > height - 2:
                tx_messages.pop()
            for y, (i, message) in enumerate(tx_messages, 1):
                i_str = f"{i}: "
                txwin.addstr(y, 2, i_str, curses.color_pair(2))
                slen = width - 4 - len(i_str)
                txwin.addstr(y, 2 + len(i_str), message[:slen].ljust(slen))
            txwin.refresh()

        async def add_tx(message: str) -> None:
            async with tx_lock:
                nonlocal tx_id
                message = re.sub(r"[\s\n\r]+", r" ", message, re.MULTILINE)
                tx_messages.appendleft((tx_id, message))
                tx_id += 1
                write_tx()

        async def add_rx(message: str) -> None:
            async with rx_lock:
                nonlocal rx_id
                message = re.sub(r"[\s\n\r]+", r" ", message, re.MULTILINE)
                rx_messages.appendleft((rx_id, message))
                rx_id += 1
                write_rx()

        def set_command(command: str) -> None:
            cmd_padded = command[: hmiddle - 3].ljust(hmiddle - 3)
            stdscr.addstr(args.height - 2, 2, cmd_padded)

        async def run_command(command: str) -> None:
            command = command.lower().strip()
            await add_tx(command)
            if command == "q":
                raise KeyboardInterrupt
            try:
                async with asyncio.timeout(args.timeout):
                    if command == "c":
                        clear_tx()
                        clear_rx()
                    elif command.startswith("n "):
                        parts = command[2:].strip().split()
                        motion_kwargs: MotionModeArgs = {}
                        if len(parts) < 1:
                            raise ValueError("Not enough arguments")
                        if len(parts) >= 1:
                            motion_kwargs["desired_position"] = float(parts[0])
                        if len(parts) >= 2:
                            motion_kwargs["desired_velocity"] = float(parts[1])
                        if len(parts) >= 3:
                            motion_kwargs["feedforward_torque"] = float(parts[2])
                        if len(parts) >= 4:
                            motion_kwargs["kp"] = int(parts[3])
                        if len(parts) >= 5:
                            motion_kwargs["kd"] = int(parts[4])
                        motion_info = await motor.motor_motion(motor_id, **motion_kwargs)
                        await add_rx(f"Pos.: {motion_info.position}")
                        await add_rx(f"Vel.: {motion_info.velocity}")
                        await add_rx(f"Torque: {motion_info.torque}")
                    elif command.startswith("a "):
                        degrees = float(command[2:].strip())
                        await motor.set_position(motor_id, degrees)
                    elif command.startswith("v "):
                        rpm = float(command[2:].strip())
                        await motor.set_velocity(motor_id, rpm)
                    elif command.startswith("t "):
                        degrees = float(command[2:].strip())
                        await motor.set_tracking_position(motor_id, degrees)
                    elif command.startswith("m "):
                        degrees = float(command[2:].strip())
                        await motor.set_relative_position(motor_id, degrees)
                    elif command == "reset":
                        await motor.reset(motor_id)
                        await asyncio.sleep(1.0)
                    elif command == "shutdown":
                        await motor.shutdown(motor_id)
                    elif command == "stop":
                        await motor.stop(motor_id)
                    elif command == "version":
                        version_date = await motor.get_system_version(motor_id)
                        version_str = version_date.strftime("%Y-%m-%d %H:%M:%S")
                        await add_rx(f"Version: {version_str}")
                    elif command.startswith("r "):
                        subcommand = command[2:].strip()
                        if subcommand == "a":
                            acceleration = await motor.read_acceleration(motor_id)
                            await add_rx(f"Acc.: {acceleration}")
                        elif subcommand == "i":
                            single_turn_data = await motor.read_single_turn_encoder(motor_id)
                            await add_rx(f"Orig. pos.: {single_turn_data.original_position}")
                            await add_rx(f"Current pos.: {single_turn_data.position}")
                            await add_rx(f"Zero bias: {single_turn_data.zero_bias}")
                        elif subcommand == "m":
                            multi_turn_angle = await motor.read_multi_turn_angle(motor_id)
                            await add_rx(f"Angle: {multi_turn_angle}")
                        elif subcommand == "p":
                            position = await motor.read_multi_turn_encoder_position(motor_id)
                            await add_rx(f"Pos.: {position}")
                        elif subcommand == "o":
                            original_position = await motor.read_multi_turn_encoder_original_position(motor_id)
                            await add_rx(f"Orig. pos.: {original_position}")
                        elif subcommand == "z":
                            zero_offset = await motor.read_multi_turn_encoder_zero_offset(motor_id)
                            await add_rx(f"Zero offset: {zero_offset}")
                        elif subcommand == "s":
                            motor_status = await motor.read_motor_status_and_errors(motor_id)
                            await add_rx(f"Temp.: {motor_status.temperature}")
                            await add_rx(f"Mos. temp.: {motor_status.mos_temperature}")
                            await add_rx(f"Brake lock: {motor_status.brake_is_locked}")
                            await add_rx(f"Voltage: {motor_status.voltage}")
                            for motor_status_error in motor_status.errors:
                                await add_rx(f"Error: {motor_status_error}")
                        elif subcommand == "t":
                            runtime = await motor.get_system_runtime(motor_id)
                            await add_rx(f"Runtime: {runtime}")
                        elif subcommand == "w":
                            power = await motor.get_power(motor_id)
                            await add_rx(f"Power: {power}")
                        elif subcommand == "pid":
                            pid = await motor.read_pid(motor_id)
                            await add_rx(f"Curr. Ki: {pid.current_ki}")
                            await add_rx(f"Curr. Kp: {pid.current_kp}")
                            await add_rx(f"Pos. Ki: {pid.position_ki}")
                            await add_rx(f"Pos. Kp: {pid.position_kp}")
                            await add_rx(f"Vel. Ki: {pid.speed_ki}")
                            await add_rx(f"Vel. Kp: {pid.speed_kp}")
                        else:
                            raise ValueError
                    elif command.startswith("w "):
                        subcommand = command[2:].strip()
                        if subcommand == "z":
                            await motor.write_multi_turn_encoder_zero_offset(motor_id)
                        elif subcommand.startswith("z "):
                            zero_offset = int(subcommand[2:].strip())
                            await motor.write_multi_turn_encoder_zero_offset(motor_id, zero_offset)
                        elif subcommand.startswith("pid "):
                            mode, kp_ki, value_str = subcommand[4:].strip().split()
                            assert mode in ("v", "p", "c")
                            assert kp_ki in ("ki", "kp")
                            pid_value = int(value_str)
                            pid = await motor.read_pid(motor_id)
                            match mode:
                                case "v":
                                    if kp_ki == "ki":
                                        pid.speed_ki = pid_value
                                    else:
                                        pid.speed_kp = pid_value
                                case "p":
                                    if kp_ki == "ki":
                                        pid.position_ki = pid_value
                                    else:
                                        pid.position_kp = pid_value
                                case "c":
                                    if kp_ki == "ki":
                                        pid.current_ki = pid_value
                                    else:
                                        pid.current_kp = pid_value
                                case _:
                                    raise ValueError
                            await motor.write_pid_to_ram(motor_id, pid)
                        else:
                            raise ValueError
                    else:
                        raise ValueError
            except asyncio.TimeoutError:
                await add_tx(f"Timeout: {command}")
            except Exception as e:
                await add_tx(f"Bad command: {command}")
                await add_tx(f"Error: {e}")

        # Writes the screen.
        stdscr.refresh()

        async def send_callback(id: int, data: bytes) -> None:
            data_str = data.hex()
            data_str = " ".join(data_str[i : i + 2] for i in range(0, len(data_str), 2))
            await add_tx(f"{id:03x} {data_str}")

        async def recv_callback(id: int, data: bytes) -> None:
            data_str = data.hex()
            data_str = " ".join(data_str[i : i + 2] for i in range(0, len(data_str), 2))
            await add_rx(f"{id:03x} {data_str}")

        base_can_interface = CanDryRun() if args.dry_run else CanIP(f"can{args.can_bus}")
        can_interface = CanWithCallback(base_can_interface, send_callback, recv_callback)

        command_str = ""
        last_command_str = ""

        async with Motors(can_interface) as motor:
            if args.dry_run:
                motor_id = 1
            elif args.motor_id is not None:
                motor_id = args.motor_id
            else:
                motor_id = await motor.get_single_motor_id()
                await add_rx(f"Motor ID: {motor_id}")

            # Main loop.
            while True:
                key = stdscr.getch()
                if key == curses.ERR:
                    await asyncio.sleep(0.01)
                elif key == curses.KEY_UP:
                    move_command_win_up()
                elif key == curses.KEY_DOWN:
                    move_command_win_down()
                elif key == curses.KEY_LEFT:
                    move_command_win_left()
                elif key == curses.KEY_RIGHT:
                    move_command_win_right()
                elif chr(key) in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ._-":
                    command_str += chr(key)
                    set_command(command_str)
                elif key in (curses.KEY_BACKSPACE, 127):
                    command_str = command_str[:-1]
                    set_command(command_str)
                elif key in (curses.KEY_ENTER, ord("\n")):
                    if command_str == "":
                        command_str = last_command_str
                    await run_command(command_str)
                    last_command_str, command_str = command_str, ""
                    set_command(command_str)
                else:
                    set_command(chr(key))

    finally:
        if "stdscr" in locals():
            stdscr.keypad(False)
            curses.echo()
            curses.nocbreak()
            curses.endwin()


def cli_entry_point() -> None:
    asyncio.run(main())


if __name__ == "__main__":
    # python -m firmware.scripts.single_motor
    cli_entry_point()
