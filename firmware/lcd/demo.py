"""Defines a simple script for interacting with the LCD screens."""

import board
import displayio

from firmware.lcd.gc9a01 import GC9A01


def main() -> None:
    # Pin Configuration
    CS_PIN = board.CE0
    DC_PIN = board.D25
    RESET_PIN = board.D24

    SCLK = board.SCLK
    MOSI = board.MOSI
    MISO = board.MISO

    # SPI Setup
    spi = board.SPI()
    while not spi.try_lock():
        pass
    # spi.configure(baudrate=24000000)
    spi.configure(baudrate=100000)
    spi.unlock()

    # Initialize Display
    displayio.release_displays()

    display_bus = displayio.FourWire(spi, command=DC_PIN, chip_select=CS_PIN, reset=RESET_PIN)
    display = GC9A01(display_bus, width=240, height=240)

    # Create a two color palette
    bitmap = displayio.Bitmap(display.width, display.height, 2)
    palette = displayio.Palette(2)
    palette[0] = 0x000000
    palette[1] = 0xffffff
    tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette)
    group = displayio.Group()
    group.append(tile_grid)
    display.root_group = group
    bitmap[80, 50] = 1
    for x in range(150, 170):
        for y in range(100, 110):
            bitmap[x, y] = 1

    print("Finished initializing the display.")
    while True:
        pass


if __name__ == "__main__":
    # python -m firmware.lcd.demo
    main()
