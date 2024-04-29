"""Test script for the MCP2515 CAN Controller."""
from build import can_controller
import time


def test_send_msg():
    device_path = "/dev/spidev0.0"
    spi_mode = 1
    bits_per_word = 8
    speed = 1000000

    try:
        can = can_controller.MCP_CAN(device_path, spi_mode, bits_per_word, speed)
        print("Resetting MCP2515 CAN Controller...")
        can.reset()
        print("Reset complete.")

        # Send a CAN message
        message_id = 0x00
        extended_frame = 0
        message_length = 5
        message_data = [0x01, 0x02, 0x03, 0x04, 0x05]
        print(f"Sending message {message_data} to the CAN bus...")
        can.send_msg_buf(message_id, extended_frame, message_length, bytes(message_data))
        print("Message sent successfully.")

    except Exception as e:
        print(f"An error occurred: {e}")


def test_motor_msg():
    device_path = "/dev/spidev0.0"
    spi_mode = 0
    bits_per_word = 8
    speed = 500000

    try:
        can = can_controller.MCP_CAN(device_path, spi_mode, bits_per_word, speed)
        print("Resetting MCP2515 CAN Controller...")
        can.reset()
        print("Reset complete.")
        can_id = 0x142
        data = [168, 0, 104, 1, 168, 253, 255, 255]
        data = [0xa8, 0x00, 0x68, 0x01, 0xa8, 0xfd, 0xff, 0xff]
        spi_message = [can_id & 0xFF, (can_id >> 8) & 0xFF] + data  # Splitting the CAN ID into two bytes and appending data

        spi_message = [0x142, 0xa8, 0x00, 0x68, 0x01, 0xa8, 0xfd, 0xff, 0xff]
        spi_message = [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            
        # Send a CAN message
        message_id = 0x00
        extended_frame = 0
        message_data = [0x01, 0x02, 0x03, 0x04, 0x05]
        message_data = [can_id & 0xFF, (can_id >> 8) & 0xFF] + [0xa8, 0x00, 0x68, 0x01, 0xa8, 0xfd, 0xff, 0xff]
        # message_data = [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message_length = len(message_data)

        while True:
            print(f"Sending message {message_data} to the CAN bus...")
            can.sendMsgBuf(message_id, extended_frame, message_length, bytes(message_data))
            time.sleep(1)
        print("Message sent successfully.")

    except Exception as e:
        print(f"An error occurred: {e}")


def test_register():
    device_path = "/dev/spidev0.0"
    spi_mode = 0
    bits_per_word = 8
    speed = 500000

    try:
        can = can_controller.MCP_CAN(device_path, spi_mode, bits_per_word, speed)
        print("Resetting MCP2515 CAN Controller...")
        can.reset()
        print("Reset complete.")

        # Example of setting and reading a register
        reg_address = 0x2D
        reg_value = 0x55
        can.set_rugister(reg_address, reg_value)
        print(f"Set register {reg_address} to {reg_value}.")

        read_value = can.read_register(reg_address)
        print(f"Read back value: {read_value}")

        if read_value != reg_value:
            print("Error: Read value does not match written value!")
        else:
            print("Register value verified successfully.")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    test_motor_msg()
