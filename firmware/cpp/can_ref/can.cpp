#include "can.h"

#define spi_readwrite mcpSPI->transfer
#define spi_read() spi_readwrite(0x00)

void MCP_CAN::mcp2515_reset(void) {
  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_RESET);
  MCP2515_UNSELECT();
  mcpSPI->endTransaction();
  delay(5); // If the MCP2515 was in sleep mode when the reset command was
            // issued then we need to wait a while for it to reset properly
}

uint8_t MCP_CAN::mcp2515_readRegister(const uint8_t address) {
  uint8_t ret;

  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_READ);
  spi_readwrite(address);
  ret = spi_read();
  MCP2515_UNSELECT();
  mcpSPI->endTransaction();

  return ret;
}

void MCP_CAN::mcp2515_readRegisterS(const uint8_t address, uint8_t values[],
                                    const uint8_t n) {
  uint8_t i;
  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_READ);
  spi_readwrite(address);
  // mcp2515 has auto-increment of address-pointer
  for (i = 0; i < n; i++)
    values[i] = spi_read();

  MCP2515_UNSELECT();
  mcpSPI->endTransaction();
}

void MCP_CAN::mcp2515_setRegister(const uint8_t address, const uint8_t value) {
  // mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // MCP2515_SELECT();
  // spi_readwrite(MCP_WRITE);
  // spi_readwrite(address);
  // spi_readwrite(value);
  // MCP2515_UNSELECT();
  // mcpSPI->endTransaction();

  uint8_t tx_buffer[] = {MCP_WRITE, address, value};
  uint8_t rx_buffer[sizeof(tx_buffer)] = {0};

  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx_buffer,
      .rx_buf = (unsigned long)rx_buffer,
      .len = sizeof(tx_buffer),
      .speed_hz = 10000000,
      .bits_per_word = 8,
  };

  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

void MCP_CAN::mcp2515_setRegisterS(const uint8_t address,
                                   const uint8_t values[], const uint8_t n) {
  uint8_t i;
  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_WRITE);
  spi_readwrite(address);

  for (i = 0; i < n; i++)
    spi_readwrite(values[i]);

  MCP2515_UNSELECT();
  mcpSPI->endTransaction();
}

void MCP_CAN::mcp2515_modifyRegister(const uint8_t address, const uint8_t mask,
                                     const uint8_t data) {
  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_BITMOD);
  spi_readwrite(address);
  spi_readwrite(mask);
  spi_readwrite(data);
  MCP2515_UNSELECT();
  mcpSPI->endTransaction();
}

uint8_t MCP_CAN::mcp2515_readStatus(void) {
  uint8_t i;
  mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  spi_readwrite(MCP_READ_STATUS);
  i = spi_read();
  MCP2515_UNSELECT();
  mcpSPI->endTransaction();
  return i;
}

void MCP_CAN::setSleepWakeup(const uint8_t enable) {
  mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

uint8_t MCP_CAN::setMode(const uint8_t opMode) {
  mcpMode = opMode;
  return mcp2515_setCANCTRL_Mode(mcpMode);
}

uint8_t MCP_CAN::mcp2515_setCANCTRL_Mode(const uint8_t newmode) {
  if ((mcp2515_readRegister(MCP_CANSTAT) & MODE_MASK) == MCP_SLEEP &&
      newmode != MCP_SLEEP) {
    bool wakeIntEnabled = (mcp2515_readRegister(MCP_CANINTE) & MCP_WAKIF);
    if (!wakeIntEnabled)
      mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

    // Set wake flag (this does the actual waking up)
    mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

    if (mcp2515_requestNewMode(MCP_LISTENONLY) != MCP2515_OK)
      return MCP2515_FAIL;

    // Turn wake interrupt back off if it was originally off
    if (!wakeIntEnabled)
      mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, 0);
  }

  // Clear wake flag
  mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, 0);

  return mcp2515_requestNewMode(newmode);
}

uint8_t MCP_CAN::mcp2515_requestNewMode(const uint8_t newmode) {
  mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
  uint8_t statReg = mcp2515_readRegister(MCP_CANSTAT);
  if ((statReg & MODE_MASK) == newmode)
    return MCP2515_OK;
  else
    return MCP2515_FAIL;
}

uint8_t MCP_CAN::mcp2515_configRate(const uint8_t canSpeed,
                                    const uint8_t canClock) {
  uint8_t set, cfg1, cfg2, cfg3;
  set = 1;
  switch (canClock & MCP_CLOCK_SELECT) {
  case (MCP_8MHZ):
    switch (canSpeed) {
    case (CAN_5KBPS): //   5KBPS
      cfg1 = MCP_8MHz_5kBPS_CFG1;
      cfg2 = MCP_8MHz_5kBPS_CFG2;
      cfg3 = MCP_8MHz_5kBPS_CFG3;
      break;

    case (CAN_10KBPS): //  10KBPS
      cfg1 = MCP_8MHz_10kBPS_CFG1;
      cfg2 = MCP_8MHz_10kBPS_CFG2;
      cfg3 = MCP_8MHz_10kBPS_CFG3;
      break;

    case (CAN_20KBPS): //  20KBPS
      cfg1 = MCP_8MHz_20kBPS_CFG1;
      cfg2 = MCP_8MHz_20kBPS_CFG2;
      cfg3 = MCP_8MHz_20kBPS_CFG3;
      break;

    case (CAN_31K25BPS): //  31.25KBPS
      cfg1 = MCP_8MHz_31k25BPS_CFG1;
      cfg2 = MCP_8MHz_31k25BPS_CFG2;
      cfg3 = MCP_8MHz_31k25BPS_CFG3;
      break;

    case (CAN_33K3BPS): //  33.33KBPS
      cfg1 = MCP_8MHz_33k3BPS_CFG1;
      cfg2 = MCP_8MHz_33k3BPS_CFG2;
      cfg3 = MCP_8MHz_33k3BPS_CFG3;
      break;

    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_8MHz_40kBPS_CFG1;
      cfg2 = MCP_8MHz_40kBPS_CFG2;
      cfg3 = MCP_8MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg1 = MCP_8MHz_50kBPS_CFG1;
      cfg2 = MCP_8MHz_50kBPS_CFG2;
      cfg3 = MCP_8MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_8MHz_80kBPS_CFG1;
      cfg2 = MCP_8MHz_80kBPS_CFG2;
      cfg3 = MCP_8MHz_80kBPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_8MHz_100kBPS_CFG1;
      cfg2 = MCP_8MHz_100kBPS_CFG2;
      cfg3 = MCP_8MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_8MHz_125kBPS_CFG1;
      cfg2 = MCP_8MHz_125kBPS_CFG2;
      cfg3 = MCP_8MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_8MHz_200kBPS_CFG1;
      cfg2 = MCP_8MHz_200kBPS_CFG2;
      cfg3 = MCP_8MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_8MHz_250kBPS_CFG1;
      cfg2 = MCP_8MHz_250kBPS_CFG2;
      cfg3 = MCP_8MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_8MHz_500kBPS_CFG1;
      cfg2 = MCP_8MHz_500kBPS_CFG2;
      cfg3 = MCP_8MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_8MHz_1000kBPS_CFG1;
      cfg2 = MCP_8MHz_1000kBPS_CFG2;
      cfg3 = MCP_8MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      return MCP2515_FAIL;
      break;
    }
    break;

  case (MCP_16MHZ):
    switch (canSpeed) {
    case (CAN_5KBPS): //   5Kbps
      cfg1 = MCP_16MHz_5kBPS_CFG1;
      cfg2 = MCP_16MHz_5kBPS_CFG2;
      cfg3 = MCP_16MHz_5kBPS_CFG3;
      break;

    case (CAN_10KBPS): //  10Kbps
      cfg1 = MCP_16MHz_10kBPS_CFG1;
      cfg2 = MCP_16MHz_10kBPS_CFG2;
      cfg3 = MCP_16MHz_10kBPS_CFG3;
      break;

    case (CAN_20KBPS): //  20Kbps
      cfg1 = MCP_16MHz_20kBPS_CFG1;
      cfg2 = MCP_16MHz_20kBPS_CFG2;
      cfg3 = MCP_16MHz_20kBPS_CFG3;
      break;

    case (CAN_33K3BPS): //  20Kbps
      cfg1 = MCP_16MHz_33k3BPS_CFG1;
      cfg2 = MCP_16MHz_33k3BPS_CFG2;
      cfg3 = MCP_16MHz_33k3BPS_CFG3;
      break;

    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_16MHz_40kBPS_CFG1;
      cfg2 = MCP_16MHz_40kBPS_CFG2;
      cfg3 = MCP_16MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg2 = MCP_16MHz_50kBPS_CFG2;
      cfg3 = MCP_16MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_16MHz_80kBPS_CFG1;
      cfg2 = MCP_16MHz_80kBPS_CFG2;
      cfg3 = MCP_16MHz_80kBPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_16MHz_100kBPS_CFG1;
      cfg2 = MCP_16MHz_100kBPS_CFG2;
      cfg3 = MCP_16MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_16MHz_125kBPS_CFG1;
      cfg2 = MCP_16MHz_125kBPS_CFG2;
      cfg3 = MCP_16MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_16MHz_200kBPS_CFG1;
      cfg2 = MCP_16MHz_200kBPS_CFG2;
      cfg3 = MCP_16MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_16MHz_250kBPS_CFG1;
      cfg2 = MCP_16MHz_250kBPS_CFG2;
      cfg3 = MCP_16MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_16MHz_500kBPS_CFG1;
      cfg2 = MCP_16MHz_500kBPS_CFG2;
      cfg3 = MCP_16MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_16MHz_1000kBPS_CFG1;
      cfg2 = MCP_16MHz_1000kBPS_CFG2;
      cfg3 = MCP_16MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      return MCP2515_FAIL;
      break;
    }
    break;

  case (MCP_20MHZ):
    switch (canSpeed) {
    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_20MHz_40kBPS_CFG1;
      cfg2 = MCP_20MHz_40kBPS_CFG2;
      cfg3 = MCP_20MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg1 = MCP_20MHz_50kBPS_CFG1;
      cfg2 = MCP_20MHz_50kBPS_CFG2;
      cfg3 = MCP_20MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_20MHz_80kBPS_CFG1;
      cfg2 = MCP_20MHz_80kBPS_CFG2;
      cfg3 = MCP_20MHz_80kBPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_20MHz_100kBPS_CFG1;
      cfg2 = MCP_20MHz_100kBPS_CFG2;
      cfg3 = MCP_20MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_20MHz_125kBPS_CFG1;
      cfg2 = MCP_20MHz_125kBPS_CFG2;
      cfg3 = MCP_20MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_20MHz_200kBPS_CFG1;
      cfg2 = MCP_20MHz_200kBPS_CFG2;
      cfg3 = MCP_20MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_20MHz_250kBPS_CFG1;
      cfg2 = MCP_20MHz_250kBPS_CFG2;
      cfg3 = MCP_20MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_20MHz_500kBPS_CFG1;
      cfg2 = MCP_20MHz_500kBPS_CFG2;
      cfg3 = MCP_20MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_20MHz_1000kBPS_CFG1;
      cfg2 = MCP_20MHz_1000kBPS_CFG2;
      cfg3 = MCP_20MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      return MCP2515_FAIL;
      break;
    }
    break;

  default:
    set = 0;
    return MCP2515_FAIL;
    break;
  }

  if (canClock & MCP_CLKOUT_ENABLE) {
    cfg3 &= (~SOF_ENABLE);
  }

  if (set) {
    mcp2515_setRegister(MCP_CNF1, cfg1);
    mcp2515_setRegister(MCP_CNF2, cfg2);
    mcp2515_setRegister(MCP_CNF3, cfg3);
    return MCP2515_OK;
  }

  return MCP2515_FAIL;
}

void MCP_CAN::mcp2515_initCANBuffers(void) {
  uint8_t i, a1, a2, a3;

  uint8_t std = 0;
  uint8_t ext = 1;
  uint32_t ulMask = 0x00, ulFilt = 0x00;

  mcp2515_write_mf(MCP_RXM0SIDH, ext, ulMask);
  mcp2515_write_mf(MCP_RXM1SIDH, ext, ulMask);

  mcp2515_write_mf(MCP_RXF0SIDH, ext, ulFilt);
  mcp2515_write_mf(MCP_RXF1SIDH, std, ulFilt);
  mcp2515_write_mf(MCP_RXF2SIDH, ext, ulFilt);
  mcp2515_write_mf(MCP_RXF3SIDH, std, ulFilt);
  mcp2515_write_mf(MCP_RXF4SIDH, ext, ulFilt);
  mcp2515_write_mf(MCP_RXF5SIDH, std, ulFilt);

  a1 = MCP_TXB0CTRL;
  a2 = MCP_TXB1CTRL;
  a3 = MCP_TXB2CTRL;
  for (i = 0; i < 14; i++) {
    mcp2515_setRegister(a1, 0);
    mcp2515_setRegister(a2, 0);
    mcp2515_setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  mcp2515_setRegister(MCP_RXB0CTRL, 0);
  mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

uint8_t MCP_CAN::mcp2515_init(const uint8_t canIDMode, const uint8_t canSpeed,
                              const uint8_t canClock) {

  uint8_t res;

  mcp2515_reset();

  mcpMode = MCP_LOOPBACK;

  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    DEBUG_PRINT("Entering Configuration Mode Failure...");
    return res;
  }
  DEBUG_PRINT("Entering Configuration Mode Successful!");

  // Set Baudrate
  if (mcp2515_configRate(canSpeed, canClock)) {
    DEBUG_PRINT("Setting Baudrate Failure...");
    return res;
  }
  DEBUG_PRINT("Setting Baudrate Successful!");

  if (res == MCP2515_OK) {

    mcp2515_initCANBuffers();
    mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

    // Sets BF pins as GPO
    mcp2515_setRegister(MCP_BFPCTRL, MCP_BxBFS_MASK | MCP_BxBFE_MASK);
    // Sets RTS pins as GPI
    mcp2515_setRegister(MCP_TXRTSCTRL, 0x00);

    switch (canIDMode) {
    case (MCP_ANY):
      mcp2515_modifyRegister(MCP_RXB0CTRL, MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                             MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
      mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_ANY);
      break;
      /*          The followingn two functions of the MCP2515 do not work, there
         is a bug in the silicon. case (MCP_STD):
                  mcp2515_modifyRegister(MCP_RXB0CTRL,
                  MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                  MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK );
                  mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                  MCP_RXB_RX_STD);
                  break;

                  case (MCP_EXT):
                  mcp2515_modifyRegister(MCP_RXB0CTRL,
                  MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                  MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK );
                  mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                  MCP_RXB_RX_EXT);
                  break;
      */
    case (MCP_STDEXT):
      mcp2515_modifyRegister(MCP_RXB0CTRL, MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                             MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
      mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);
      break;

    default:
      DEBUG_PRINT("`Setting ID Mode Failure...");
      return MCP2515_FAIL;
      break;
    }

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res) {
      DEBUG_PRINT("Returning to Previous Mode Failure...");
      return res;
    }
  }
  return res;
}

void MCP_CAN::mcp2515_write_id(const uint8_t mcp_addr, const uint8_t ext,
                               const uint32_t id) {
  uint16_t canid;
  uint8_t tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  if (ext == 1) {
    tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
    tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 5);
  } else {
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3);
    tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
    tbufdata[MCP_EID0] = 0;
    tbufdata[MCP_EID8] = 0;
  }

  mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

void MCP_CAN::mcp2515_write_mf(const uint8_t mcp_addr, const uint8_t ext,
                               const uint32_t id) {
  uint16_t canid;
  uint8_t tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  if (ext == 1) {
    tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
    tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 5);
  } else {
    tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
    tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3);
  }

  mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

void MCP_CAN::mcp2515_read_id(const uint8_t mcp_addr, uint8_t *ext,
                              uint32_t *id) {
  uint8_t tbufdata[4];

  *ext = 0;
  *id = 0;

  mcp2515_readRegisterS(mcp_addr, tbufdata, 4);

  *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

  if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M) {

    *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    *id = (*id << 8) + tbufdata[MCP_EID8];
    *id = (*id << 8) + tbufdata[MCP_EID0];
    *ext = 1;
  }
}

void MCP_CAN::mcp2515_write_canMsg(const uint8_t buffer_sidh_addr) {
  uint8_t mcp_addr;
  mcp_addr = buffer_sidh_addr;
  mcp2515_setRegisterS(mcp_addr + 5, m_nDta, m_nDlc);

  if (m_nRtr == 1)
    m_nDlc |= MCP_RTR_MASK;

  mcp2515_setRegister((mcp_addr + 4), m_nDlc);
  mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID);
}

void MCP_CAN::mcp2515_read_canMsg(const uint8_t buffer_sidh_addr) {
  uint8_t mcp_addr, ctrl;

  mcp_addr = buffer_sidh_addr;

  mcp2515_read_id(mcp_addr, &m_nExtFlg, &m_nID);

  ctrl = mcp2515_readRegister(mcp_addr - 1);
  m_nDlc = mcp2515_readRegister(mcp_addr + 4);

  if (ctrl & 0x08)
    m_nRtr = 1;
  else
    m_nRtr = 0;

  m_nDlc &= MCP_DLC_MASK;
  mcp2515_readRegisterS(mcp_addr + 5, &(m_nDta[0]), m_nDlc);
}

uint8_t MCP_CAN::mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n) {
  uint8_t res, i, ctrlval;
  uint8_t ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL,
                                       MCP_TXB2CTRL};

  res = MCP_ALLTXBUSY;
  *txbuf_n = 0x00;

  for (i = 0; i < MCP_N_TXBUFFERS; i++) {
    ctrlval = mcp2515_readRegister(ctrlregs[i]);
    if ((ctrlval & MCP_TXB_TXREQ_M) == 0) {
      *txbuf_n = ctrlregs[i] + 1;

      res = MCP2515_OK;
      return res;
    }
  }
  return res;
}

MCP_CAN::MCP_CAN(uint8_t _CS) {
  MCPCS = _CS;
  MCP2515_UNSELECT();
  pinMode(MCPCS, OUTPUT);
  mcpSPI = &SPI;
}

MCP_CAN::MCP_CAN(SPIClass *_SPI, uint8_t _CS) {
  MCPCS = _CS;
  MCP2515_UNSELECT();
  pinMode(MCPCS, OUTPUT);
  mcpSPI = _SPI;
}

uint8_t MCP_CAN::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset) {
  uint8_t res;

  mcpSPI->begin();
  res = mcp2515_init(idmodeset, speedset, clockset);
  if (res == MCP2515_OK)
    return CAN_OK;

  return CAN_FAILINIT;
}

uint8_t MCP_CAN::init_Mask(uint8_t num, uint8_t ext, uint32_t ulData) {
  uint8_t res = MCP2515_OK;
  DEBUG_PRINT("Starting to Set Mask!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    DEBUG_PRINT("Entering Configuration Mode Failure...");
    return res;
  }

  if (num == 0) {
    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);

  } else if (num == 1) {
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
  } else
    res = MCP2515_FAIL;

  res = mcp2515_setCANCTRL_Mode(mcpMode);
  if (res > 0) {
    DEBUG_PRINT("Entering Previous Mode Failure...");
    DEBUG_PRINT("Setting Mask Failure...");
    return res;
  }
  DEBUG_PRINT("Setting Mask Successful!");
  return res;
}

uint8_t MCP_CAN::init_Mask(uint8_t num, uint32_t ulData) {
  uint8_t res = MCP2515_OK;
  uint8_t ext = 0;
  DEBUG_PRINT("Starting to Set Mask!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    DEBUG_PRINT("Entering Configuration Mode Failure...");
    return res;
  }

  if ((ulData & 0x80000000) == 0x80000000)
    ext = 1;

  if (num == 0) {
    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);

  } else if (num == 1) {
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
  } else
    res = MCP2515_FAIL;

  res = mcp2515_setCANCTRL_Mode(mcpMode);
  if (res > 0) {
    DEBUG_PRINT("Entering Previous Mode Failure...");
    DEBUG_PRINT("Setting Mask Failure...");
    return res;
  }
  DEBUG_PRINT("Setting Mask Successful!");
  return res;
}

uint8_t MCP_CAN::init_Filt(uint8_t num, uint8_t ext, uint32_t ulData) {
  uint8_t res = MCP2515_OK;
  DEBUG_PRINT("Starting to Set Filter!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    DEBUG_PRINT("Enter Configuration Mode Failure...");
    return res;
  }

  switch (num) {
  case 0:
    mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
    break;

  case 1:
    mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
    break;

  case 2:
    mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
    break;

  case 3:
    mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
    break;

  case 4:
    mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
    break;

  case 5:
    mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
    break;

  default:
    res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(mcpMode);
  if (res > 0) {
    DEBUG_PRINT("Entering Previous Mode Failure...");
    DEBUG_PRINT("Setting Filter Failure...");
    return res;
  }
  DEBUG_PRINT("Setting Filter Successful!");

  return res;
}

uint8_t MCP_CAN::init_Filt(uint8_t num, uint32_t ulData) {
  uint8_t res = MCP2515_OK;
  uint8_t ext = 0;

  DEBUG_PRINT("Starting to Set Filter!");
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
    DEBUG_PRINT("Enter Configuration Mode Failure...");
    return res;
  }

  if ((ulData & 0x80000000) == 0x80000000)
    ext = 1;

  switch (num) {
  case 0:
    mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
    break;

  case 1:
    mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
    break;

  case 2:
    mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
    break;

  case 3:
    mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
    break;

  case 4:
    mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
    break;

  case 5:
    mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
    break;

  default:
    res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(mcpMode);
  if (res > 0) {
    DEBUG_PRINT("Entering Previous Mode Failure...");
    DEBUG_PRINT("Setting Filter Failure...");
    return res;
  }
  DEBUG_PRINT("Setting Filter Successful!");

  return res;
}

uint8_t MCP_CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len,
                        uint8_t *pData) {
  int i = 0;
  m_nID = id;
  m_nRtr = rtr;
  m_nExtFlg = ext;
  m_nDlc = len;
  for (i = 0; i < MAX_CHAR_IN_MESSAGE; i++)
    m_nDta[i] = *(pData + i);

  return MCP2515_OK;
}

uint8_t MCP_CAN::clearMsg() {
  m_nID = 0;
  m_nDlc = 0;
  m_nExtFlg = 0;
  m_nRtr = 0;
  m_nfilhit = 0;
  for (int i = 0; i < m_nDlc; i++)
    m_nDta[i] = 0x00;

  return MCP2515_OK;
}

uint8_t MCP_CAN::sendMsg() {
  uint8_t res, res1, txbuf_n;
  uint32_t uiTimeOut, temp;

  temp = micros();

  // 24 * 4 microseconds typical
  do {
    res = mcp2515_getNextFreeTXBuf(&txbuf_n);
    uiTimeOut = micros() - temp;
  } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

  if (uiTimeOut >= TIMEOUTVALUE) {
    return CAN_GETTXBFTIMEOUT;
  }
  uiTimeOut = 0;
  mcp2515_write_canMsg(txbuf_n);
  mcp2515_modifyRegister(txbuf_n - 1, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);

  temp = micros();
  do {
    res1 = mcp2515_readRegister(txbuf_n - 1);
    res1 = res1 & 0x08;
    uiTimeOut = micros() - temp;
  } while (res1 && (uiTimeOut < TIMEOUTVALUE));

  if (uiTimeOut >= TIMEOUTVALUE)
    return CAN_SENDMSGTIMEOUT;

  return CAN_OK;
}

uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len,
                            uint8_t *buf) {
  uint8_t res;

  setMsg(id, 0, ext, len, buf);
  res = sendMsg();

  return res;
}

uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf) {
  uint8_t ext = 0, rtr = 0;
  uint8_t res;

  if ((id & 0x80000000) == 0x80000000)
    ext = 1;

  if ((id & 0x40000000) == 0x40000000)
    rtr = 1;

  setMsg(id, rtr, ext, len, buf);
  res = sendMsg();

  return res;
}

uint8_t MCP_CAN::readMsg() {
  uint8_t stat, res;

  stat = mcp2515_readStatus();

  if (stat & MCP_STAT_RX0IF) {
    mcp2515_read_canMsg(MCP_RXBUF_0);
    mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
    res = CAN_OK;
  } else if (stat & MCP_STAT_RX1IF) {
    mcp2515_read_canMsg(MCP_RXBUF_1);
    mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
    res = CAN_OK;
  } else
    res = CAN_NOMSG;

  return res;
}

uint8_t MCP_CAN::readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len,
                            uint8_t buf[]) {
  if (readMsg() == CAN_NOMSG)
    return CAN_NOMSG;

  *id = m_nID;
  *len = m_nDlc;
  *ext = m_nExtFlg;
  for (int i = 0; i < m_nDlc; i++)
    buf[i] = m_nDta[i];

  return CAN_OK;
}

uint8_t MCP_CAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t buf[]) {
  if (readMsg() == CAN_NOMSG)
    return CAN_NOMSG;

  if (m_nExtFlg)
    m_nID |= 0x80000000;

  if (m_nRtr)
    m_nID |= 0x40000000;

  *id = m_nID;
  *len = m_nDlc;

  for (int i = 0; i < m_nDlc; i++)
    buf[i] = m_nDta[i];

  return CAN_OK;
}

uint8_t MCP_CAN::checkReceive(void) {
  uint8_t res;
  res = mcp2515_readStatus();
  if (res & MCP_STAT_RXIF_MASK)
    return CAN_MSGAVAIL;
  else
    return CAN_NOMSG;
}

uint8_t MCP_CAN::checkError(void) {
  uint8_t eflg = mcp2515_readRegister(MCP_EFLG);

  if (eflg & MCP_EFLG_ERRORMASK)
    return CAN_CTRLERROR;
  else
    return CAN_OK;
}

uint8_t MCP_CAN::getError(void) { return mcp2515_readRegister(MCP_EFLG); }

uint8_t MCP_CAN::errorCountRX(void) { return mcp2515_readRegister(MCP_REC); }

uint8_t MCP_CAN::errorCountTX(void) { return mcp2515_readRegister(MCP_TEC); }

uint8_t MCP_CAN::enOneShotTX(void) {
  mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
  if ((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
    return CAN_FAIL;
  else
    return CAN_OK;
}

uint8_t MCP_CAN::disOneShotTX(void) {
  mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
  if ((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
    return CAN_FAIL;
  else
    return CAN_OK;
}

uint8_t MCP_CAN::abortTX(void) {
  mcp2515_modifyRegister(MCP_CANCTRL, ABORT_TX, ABORT_TX);

  // Maybe check to see if the TX buffer transmission request bits are cleared
  // instead?
  if ((mcp2515_readRegister(MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
    return CAN_FAIL;
  else
    return CAN_OK;
}

uint8_t MCP_CAN::setGPO(uint8_t data) {
  mcp2515_modifyRegister(MCP_BFPCTRL, MCP_BxBFS_MASK, (data << 4));

  return 0;
}

uint8_t MCP_CAN::getGPI(void) {
  uint8_t res;
  res = mcp2515_readRegister(MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
  return (res >> 3);
}

PYBIND11_MODULE(can, m) {
  m.doc() = "CAN module for interfacing with MCP2515 CAN controller";
}
