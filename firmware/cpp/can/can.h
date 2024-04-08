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

namespace py = pybind11;

using namespace pybind11::literals;

#define MAX_CHAR_IN_MESSAGE 8

#if DEBUG_MODE
#define DEBUG_PRINT(fmt, args...) fprintf(stderr, fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

void errmsg(const char *msg) {
  perror(msg);
  exit(EXIT_FAILURE);
}

class MCP_CAN {
private:
  uint8_t m_nExtFlg;                   // Identifier Type
  uint32_t m_nID;                      // CAN ID
  uint8_t m_nDlc;                      // Data Length Code
  uint8_t m_nDta[MAX_CHAR_IN_MESSAGE]; // Data array
  uint8_t m_nRtr;                      // Remote request flag
  uint8_t m_nfilhit; // The number of the filter that matched the message
  uint8_t MCPCS;     // Chip Select pin number
  uint8_t mcpMode;   // Mode to return to after configurations are performed.
  int spi_fd;

private:
  void mcp2515_reset(void); // Soft Reset MCP2515

  uint8_t mcp2515_readRegister(const uint8_t address); // Read MCP2515 register

  void mcp2515_readRegisterS(
      const uint8_t address, // Read MCP2515 successive registers
      uint8_t values[], const uint8_t n);

  void mcp2515_setRegister(const uint8_t address, // Set MCP2515 register
                           const uint8_t value);

  void mcp2515_setRegisterS(
      const uint8_t address, // Set MCP2515 successive registers
      const uint8_t values[], const uint8_t n);

  void mcp2515_initCANBuffers(void);

  void mcp2515_modifyRegister(
      const uint8_t address, // Set specific bit(s) of a register
      const uint8_t mask, const uint8_t data);

  uint8_t mcp2515_readStatus(void);                       // Read MCP2515 Status
  uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode); // Set mode
  uint8_t mcp2515_requestNewMode(const uint8_t newmode);  // Set mode
  uint8_t mcp2515_configRate(const uint8_t canSpeed,      // Set baudrate

                             const uint8_t canClock);

  uint8_t mcp2515_init(const uint8_t canIDMode, // Initialize Controller
                       const uint8_t canSpeed, const uint8_t canClock);

  void mcp2515_write_mf(const uint8_t mcp_addr, // Write CAN Mask or Filter
                        const uint8_t ext, const uint32_t id);

  void mcp2515_write_id(const uint8_t mcp_addr, // Write CAN ID
                        const uint8_t ext, const uint32_t id);

  void mcp2515_read_id(const uint8_t mcp_addr, // Read CAN ID
                       uint8_t *ext, uint32_t *id);

  void
  mcp2515_write_canMsg(const uint8_t buffer_sidh_addr);     // Write CAN message
  void mcp2515_read_canMsg(const uint8_t buffer_sidh_addr); // Read CAN message
  uint8_t
  mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n); // Find empty transmit buffer

  uint8_t setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len,
                 uint8_t *pData); // Set message
  uint8_t clearMsg();             // Clear all message to zero
  uint8_t readMsg();              // Read message
  uint8_t sendMsg();              // Send message

public:
  MCP_CAN(const char *device);
  ~MCP_CAN();
  uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);
  uint8_t init_Mask(uint8_t num, uint8_t ext, uint32_t ulData);
  uint8_t init_Mask(uint8_t num, uint32_t ulData);
  uint8_t init_Filt(uint8_t num, uint8_t ext, uint32_t ulData);
  uint8_t init_Filt(uint8_t num, uint32_t ulData);
  void setSleepWakeup(uint8_t enable);
  uint8_t setMode(uint8_t opMode);
  uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);
  uint8_t sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf);
  uint8_t readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf);
  uint8_t readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf);
  uint8_t checkReceive(void);
  uint8_t checkError(void);
  uint8_t getError(void);
  uint8_t errorCountRX(void);
  uint8_t errorCountTX(void);
  uint8_t enOneShotTX(void);
  uint8_t disOneShotTX(void);
  uint8_t abortTX(void);
  uint8_t setGPO(uint8_t data);
  uint8_t getGPI(void);
};
