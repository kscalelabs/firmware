import board
import busio
import digitalio
import adafruit_mcp2515


def main() -> None:
    CS_PIN = 5
    cs = digitalio.DigitalInOut(board.D5)
    cs.direction = digitalio.Direction.OUTPUT
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    mcp = adafruit_mcp2515.MCP2515(spi, cs)


if __name__ == "__main__":
    # python -m firmware.scripts.simple
    main()
