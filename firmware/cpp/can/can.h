#pragma once

#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "can_dfs.h"

#if DEBUG_MODE
#define DEBUG_PRINT(fmt, args...) fprintf(stderr, fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

class MCP_CAN {
private:
  int spi_fd; // File descriptor for the SPI device.

  // SPI parameters.
  uint8_t spiMode;
  uint8_t bitsPerWord;
  uint32_t speed;

public:
  MCP_CAN(const std::string &device, uint8_t spiMode = SPI_MODE_0,
          uint8_t bitsPerWord = 8, uint32_t speed = 1000000);
  ~MCP_CAN();

  // Helper function for transmitting and receiving data.
  void transfer(uint8_t *tx, uint8_t *rx, size_t len);

  // Control registers.
  void reset(void);
  uint8_t readRegister(uint8_t address);
  void setRegister(uint8_t address, uint8_t value);
};
