import RPi.GPIO as GPIO
from time import sleep
import board


def main(pin: int = board.GPIO12) -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW)

    while True:
        GPIO.output(board.GPIO12, GPIO.HIGH)
        sleep(1)
        GPIO.output(board.GPIO12, GPIO.LOW)
        sleep(1)


if __name__ == "__main__":
    # python -m firmware.scripts.blink
    main()
