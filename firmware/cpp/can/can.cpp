#include "can.h"

MCP_CAN::MCP_CAN(const std::string &device, uint8_t spiMode,
                 uint8_t bitsPerWord, uint32_t speed)
    : spiMode(spiMode), bitsPerWord(bitsPerWord), speed(speed) {
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

namespace py = pybind11;

using namespace pybind11::literals;

PYBIND11_MODULE(can, m) {
  m.doc() = "CAN module for interfacing with MCP2515 CAN controller";

  py::class_<MCP_CAN>(m, "MCP_CAN")
      .def(py::init<const char *>(), "device"_a)
      .def("reset", &MCP_CAN::reset)
      .def("read_register", &MCP_CAN::readRegister, "address"_a)
      .def("set_register", &MCP_CAN::setRegister, "address"_a, "value"_a);
}
