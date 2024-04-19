#include "can.h"
#include <iostream>

MCP_CAN::MCP_CAN(
  const std::string &device, uint8_t spiMode,
  uint8_t bitsPerWord, uint32_t speed
): spiMode(spiMode), bitsPerWord(bitsPerWord), speed(speed) {
  spi_fd = open(device.c_str(), O_RDWR);
  if (spi_fd < 0)
    throw std::runtime_error("Can't open device: " + std::string(device));
  
  // Configure SPI.
  if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spiMode) < 0)
    throw std::runtime_error("Can't set SPI mode");
  if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0)
    throw std::runtime_error("Can't set bits per word");
  if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    throw std::runtime_error("Can't set speed");
}

MCP_CAN::~MCP_CAN() {
  if (close(spi_fd) < 0)
    throw std::runtime_error("Can't close device");
}

void MCP_CAN::transfer(uint8_t *tx, uint8_t *rx, size_t len) {
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = len,
      .speed_hz = speed,
      .delay_usecs = 0,
      .bits_per_word = bitsPerWord,
      .cs_change = 0,
      .pad = 0,
  };

  if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    throw std::runtime_error("Can't transfer data");
}

void MCP_CAN::reset(void) {
  uint8_t tx[] = {MCP_RESET};
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
}

uint8_t MCP_CAN::readRegister(uint8_t address) {
  uint8_t tx[] = {MCP_READ, address};
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
  return rx[1];
}

void MCP_CAN::setRegister(uint8_t address, uint8_t value) {
  uint8_t tx[] = {MCP_WRITE, address, value};
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
}

int main() {
    try {
        // Example device path - replace with the actual SPI device file
        std::string device = "/dev/spidev0.0";
        uint8_t spiMode = 0;           // SPI mode 0
        uint8_t bitsPerWord = 8;       // 8-bit word size
        uint32_t speed = 500000;       // 500 kHz SPI speed

        // Create an instance of MCP_CAN
        MCP_CAN canController(device, spiMode, bitsPerWord, speed);

        // Reset the MCP2515 CAN controller
        std::cout << "Resetting MCP2515 CAN Controller..." << std::endl;
        canController.reset();
        std::cout << "Reset complete." << std::endl;

        // Set a register, just as an example
        uint8_t registerAddress = 0x2D; // Example register address
        uint8_t registerValue = 0x55;   // Example value to write
        std::cout << "Setting register 0x" << std::hex << int(registerAddress)
                  << " to 0x" << int(registerValue) << std::endl;
        canController.setRegister(registerAddress, registerValue);

        // Read back the register
        std::cout << "Reading back the register..." << std::endl;
        uint8_t readValue = canController.readRegister(registerAddress);
        std::cout << "Read value: 0x" << std::hex << int(readValue) << std::endl;

        if (readValue != registerValue) {
            std::cerr << "Error: Read value does not match written value!" << std::endl;
        } else {
            std::cout << "Register value verified successfully." << std::endl;
        }

    } catch (const std::runtime_error& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

namespace py = pybind11;

using namespace pybind11::literals;


PYBIND11_MODULE(can_controller, m) {
    m.doc() = "Pybind11 interface for the MCP_CAN CAN controller";

    py::class_<MCP_CAN>(m, "MCP_CAN")
        .def(py::init<const std::string &, uint8_t, uint8_t, uint32_t>(),
             py::arg("device"), py::arg("spiMode") = SPI_MODE_0, py::arg("bitsPerWord") = 8, py::arg("speed") = 500000,
             "Constructor that initializes the MCP_CAN controller with specified SPI settings.\n"
             "\n"
             "Parameters:\n"
             "    device (str): Path to the SPI device (e.g., '/dev/spidev0.0').\n"
             "    spiMode (int, optional): SPI mode (default is SPI_MODE_0).\n"
             "    bitsPerWord (int, optional): Number of bits per word (default is 8).\n"
             "    speed (int, optional): SPI bus speed in Hz (default is 500000).")
        .def("reset", &MCP_CAN::reset, "Resets the MCP_CAN controller.")
        .def("read_register", &MCP_CAN::readRegister, py::arg("address"),
             "Reads a register from the MCP_CAN controller.\n"
             "\n"
             "Parameters:\n"
             "    address (int): Register address to read from.")
        .def("set_register", &MCP_CAN::setRegister, py::arg("address"), py::arg("value"),
             "Sets a register of the MCP_CAN controller.\n"
             "\n"
             "Parameters:\n"
             "    address (int): Register address to write to.\n"
             "    value (int): Value to write to the register.");
}