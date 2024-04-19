"""Test script for the MCP2515 CAN Controller."""
from build import can_controller


def main():
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
        can.set_register(reg_address, reg_value)
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
    main()
