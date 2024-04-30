/*
    mcp_can.h
    2012 Copyright (c) Seeed Technology Inc.  All right reserved.

    Author:Loovee (loovee@seeed.cc)
    2014-1-16

    Contributor:

    Cory J. Fowler
    Latonita
    Woodward1
    Mehtajaghvi
    BykeBlast
    TheRo0T
    Tsipizic
    ralfEdmund
    Nathancheek
    BlueAndi
    Adlerweb
    Btetz
    Hurvajs
    ttlappalainen

    The MIT License (MIT)

    Copyright (c) 2013 Seeed Technology Inc.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can.h"
#include "mcp2515_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

class mcp2515_can : public MCP_CAN
{
public:
    // mcp2515_can(uint8_t _CS) : MCP_CAN(_CS), nReservedTx(0){};
    // pfb30
    mcp2515_can(const std::string &device, uint8_t spiMode = SPI_MODE_0,
                uint8_t bitsPerWord = 8, uint32_t speed = 1000000) : MCP_CAN(device, spiMode, bitsPerWord, speed), nReservedTx(0){};
  // Helper function for transmitting and receiving data.
    void transfer(uint8_t *tx, uint8_t *rx, size_t len);

public:
    virtual void enableTxInterrupt(bool enable = true); // enable transmit interrupt
    virtual void reserveTxBuffers(uint8_t nTxBuf = 0)
    {
        nReservedTx = (nTxBuf < MCP_N_TXBUFFERS ? nTxBuf : MCP_N_TXBUFFERS - 1);
    }
    virtual uint8_t getLastTxBuffer()
    {
        return MCP_N_TXBUFFERS - 1; // read index of last tx buffer
    }
    virtual uint8_t begin(uint32_t speedset, const uint8_t clockset = MCP_16MHz);                                                                                 // init can
    virtual uint8_t init_Mask(uint8_t num, uint8_t ext, unsigned long ulData);                                                                                   // init Masks
    virtual uint8_t init_Filt(uint8_t num, uint8_t ext, unsigned long ulData);                                                                                   // init filters
    virtual void setSleepWakeup(uint8_t enable);                                                                                                           // Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity, making it send only)
    virtual uint8_t sleep();                                                                                                                               // Put the MCP2515 in sleep mode
    virtual uint8_t wake();                                                                                                                                // Wake MCP2515 manually from sleep
    virtual uint8_t setMode(uint8_t opMode);                                                                                                                  // Set operational mode
    virtual uint8_t getMode();                                                                                                                             // Get operational mode
    virtual uint8_t checkError(uint8_t* err_ptr = NULL);                                                                                                   // if something error

    virtual uint8_t checkReceive(void);                                                                                                                    // if something received
    virtual uint8_t readMsgBufID(uint8_t status, volatile unsigned long *id, volatile uint8_t *ext, volatile uint8_t *rtr, volatile uint8_t *len, volatile uint8_t *buf); // read buf with object ID
    /* wrapper */
    uint8_t readMsgBufID(unsigned long *ID, uint8_t *len, uint8_t *buf) {
        return readMsgBufID(readRxTxStatus(), ID, &ext_flg, &rtr, len, buf);
    }
    uint8_t readMsgBuf(uint8_t *len, uint8_t *buf) {
        return readMsgBufID(readRxTxStatus(), &can_id, &ext_flg, &rtr, len, buf);
    }

    virtual uint8_t trySendMsgBuf(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, uint8_t iTxBuf = 0xff);                                 // as sendMsgBuf, but does not have any wait for free buffer
    virtual uint8_t sendMsgBuf(uint8_t status, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, volatile const uint8_t *buf);                                  // send message buf by using parsed buffer status
    virtual uint8_t sendMsgBuf(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, bool wait_sent = true);                                 // send buf
    using MCP_CAN::sendMsgBuf; // make other overloads visible

    virtual void clearBufferTransmitIfFlags(uint8_t flags = 0);                                                                                            // Clear transmit flags according to status
    virtual uint8_t readRxTxStatus(void);                                                                                                                  // read has something send or received
    virtual uint8_t checkClearRxStatus(uint8_t *status);                                                                                                      // read and clear and return first found rx status bit
    virtual uint8_t checkClearTxStatus(uint8_t *status, uint8_t iTxBuf = 0xff);                                                                                  // read and clear and return first found or buffer specified tx status bit
    virtual bool mcpPinMode(const uint8_t pin, const uint8_t mode);                                                                                           // switch supported pins between HiZ, interrupt, output or input
    virtual bool mcpDigitalWrite(const uint8_t pin, const uint8_t mode);                                                                                      // write HIGH or LOW to RX0BF/RX1BF
    virtual uint8_t mcpDigitalRead(const uint8_t pin);

private:
    void mcp2515_reset(void); // reset mcp2515

    uint8_t mcp2515_readRegister(const uint8_t address); // read mcp2515's register

    void mcp2515_readRegisterS(const uint8_t address,
                               uint8_t values[],
                               const uint8_t n);
    void mcp2515_setRegister(const uint8_t address, // set mcp2515's register
                             const uint8_t value);

    void mcp2515_setRegisterS(const uint8_t address, // set mcp2515's registers
                              const uint8_t values[],
                              const uint8_t n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const uint8_t address, // set bit of one register
                                const uint8_t mask,
                                const uint8_t data);

    uint8_t mcp2515_readStatus(void);                                  // read mcp2515's Status
    uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode);               // set mode
    uint8_t mcp2515_requestNewMode(const uint8_t newmode);                // Set mode
    uint8_t mcp2515_configRate(const uint8_t canSpeed, const uint8_t clock); // set baudrate
    uint8_t mcp2515_init(const uint8_t canSpeed, const uint8_t clock);       // mcp2515init

    void mcp2515_write_id(const uint8_t mcp_addr, // write can id
                          const uint8_t ext,
                          const unsigned long id);

    void mcp2515_read_id(const uint8_t mcp_addr, // read can id
                         uint8_t *ext,
                         unsigned long *id);

    void mcp2515_write_canMsg(const uint8_t buffer_sidh_addr, unsigned long id, uint8_t ext, uint8_t rtr, uint8_t len,
                              volatile const uint8_t *buf); // read can msg
    void mcp2515_read_canMsg(const uint8_t buffer_load_addr, volatile unsigned long *id, volatile uint8_t *ext,
                             volatile uint8_t *rtr, volatile uint8_t *len, volatile uint8_t *buf); // write can msg
    void mcp2515_start_transmit(const uint8_t mcp_addr);                                     // start transmit
    uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n);                                         // get Next free txbuf
    uint8_t mcp2515_isTXBufFree(uint8_t *txbuf_n, uint8_t iBuf);                                   // is buffer by index free

    /*
        can operator function
    */

    uint8_t sendMsg(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, bool wait_sent = true); // send message
private:
    uint8_t nReservedTx; // Count of tx buffers for reserved send

          
    int spi_fd; // File descriptor for the SPI device.

    // SPI parameters.
    uint8_t spiMode;
    uint8_t bitsPerWord;
    uint32_t speed;

};

#endif
/*********************************************************************************************************
    END FILE
*********************************************************************************************************/