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

  // buffer data
  uint8_t   m_nExtFlg;                                                  // Identifier Type
                                                                     // Extended (29 bit) or Standard (11 bit)
  uint32_t  m_nID;                                                      // CAN ID
  uint8_t   m_nDlc;                                                     // Data Length Code                              // Data array
  uint8_t   m_nRtr;                                                     // Remote request flag
  uint8_t   m_nfilhit;                                          // The number of the filter that matched the message
  uint8_t   m_nDta[CAN_MAX_CHAR_IN_MESSAGE];                    // Data array
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
  void modifyRegister(uint8_t address, uint8_t mask, uint8_t data);
  void writeId( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id);
  void writeCanMsg( const uint8_t buffer_sidh_addr);
  uint8_t getNextFreeTXBuf(uint8_t *txbuf_n);
  uint8_t setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData);
  uint8_t sendMsg();
  uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);
  void setRegisterS(uint8_t address, uint8_t values[], uint8_t n);
};
