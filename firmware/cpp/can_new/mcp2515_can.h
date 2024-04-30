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
#pragma once

#include <linux/spi/spidev.h>
#include <inttypes.h>
#include <stdint.h>

#define CAN_OK              (0)
#define CAN_FAILINIT        (1)
#define CAN_FAILTX          (2)
#define CAN_MSGAVAIL        (3)
#define CAN_NOMSG           (4)
#define CAN_CTRLERROR       (5)
#define CAN_GETTXBFTIMEOUT  (6)
#define CAN_SENDMSGTIMEOUT  (7)
#define CAN_FAIL            (0xff)

#include "mcp2515_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

// clock
typedef enum {
    MCP_NO_MHz,
    /* apply to MCP2515 */
    MCP_16MHz,
    MCP_12MHz,
    MCP_8MHz,
    // /* apply to MCP2518FD */
    // MCP2518FD_40MHz = MCP_16MHz /* To compatible MCP2515 shield */,
    // MCP2518FD_20MHz,
    // MCP2518FD_10MHz,
} MCP_CLOCK_T;

typedef enum {
    CAN_NOBPS,
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_25KBPS,
    CAN_31K25BPS,
    CAN_33KBPS  ,
    CAN_40KBPS  ,
    CAN_50KBPS  ,
    CAN_80KBPS  ,
    CAN_83K3BPS ,
    CAN_95KBPS  ,
    CAN_95K2BPS ,
    CAN_100KBPS ,
    CAN_125KBPS ,
    CAN_200KBPS ,
    CAN_250KBPS ,
    CAN_500KBPS ,
    CAN_666KBPS ,
    CAN_800KBPS ,
    CAN_1000KBPS
} MCP_BITTIME_SETUP;



class mcp2515_can {
public:
    // mcp2515_can(uint8_t _CS) : MCP_CAN(_CS), nReservedTx(0){};
    // pfb30
    mcp2515_can(const std::string &device, uint8_t spiMode = SPI_MODE_0,
                uint8_t bitsPerWord = 8, uint32_t speed = 1000000);
  // Helper function for transmitting and receiving data.
    void transfer(uint8_t *tx, uint8_t *rx, size_t len);

    void enableTxInterrupt(bool enable = true); // enable transmit interrupt
    void reserveTxBuffers(uint8_t nTxBuf = 0)
    {
        nReservedTx = (nTxBuf < MCP_N_TXBUFFERS ? nTxBuf : MCP_N_TXBUFFERS - 1);
    }
    uint8_t getLastTxBuffer()
    {
        return MCP_N_TXBUFFERS - 1; // read index of last tx buffer
    }
    uint8_t begin(void); //uint32_t speedset, const uint8_t clockset = MCP_16MHz);                                                                                 // init can
    uint8_t init_Mask(uint8_t num, uint8_t ext, unsigned long ulData);                                                                                   // init Masks
    uint8_t init_Filt(uint8_t num, uint8_t ext, unsigned long ulData);                                                                                   // init filters
    void setSleepWakeup(uint8_t enable);                                                                                                           // Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity, making it send only)
    uint8_t sleep();                                                                                                                               // Put the MCP2515 in sleep mode
    uint8_t wake();                                                                                                                                // Wake MCP2515 manually from sleep
    uint8_t setMode(uint8_t opMode);                                                                                                                  // Set operational mode
    uint8_t getMode();                                                                                                                             // Get operational mode
    uint8_t checkError(uint8_t* err_ptr = NULL);                                                                                                   // if something error

    uint8_t checkReceive(void);                                                                                                                    // if something received
    uint8_t readMsgBufID(uint8_t status, volatile unsigned long *id, volatile uint8_t *ext, volatile uint8_t *rtr, volatile uint8_t *len, volatile uint8_t *buf); // read buf with object ID
    /* wrapper */
    uint8_t readMsgBufID(unsigned long *ID, uint8_t *len, uint8_t *buf) {
        return readMsgBufID(readRxTxStatus(), ID, &ext_flg, &rtr, len, buf);
    }
    uint8_t readMsgBuf(uint8_t *len, uint8_t *buf) {
        return readMsgBufID(readRxTxStatus(), &can_id, &ext_flg, &rtr, len, buf);
    }

    uint8_t trySendMsgBuf(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, uint8_t iTxBuf = 0xff);                                 // as sendMsgBuf, but does not have any wait for free buffer
    // uint8_t sendMsgBuf(uint8_t status, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, volatile const uint8_t *buf);                                  // send message buf by using parsed buffer status
    uint8_t sendMsgBuf(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, bool wait_sent = true);                                 // send buf
    // using MCP_CAN::sendMsgBuf; // make other overloads visible

    void clearBufferTransmitIfFlags(uint8_t flags = 0);                                                                                            // Clear transmit flags according to status
    uint8_t readRxTxStatus(void);                                                                                                                  // read has something send or received
    uint8_t checkClearRxStatus(uint8_t *status);                                                                                                      // read and clear and return first found rx status bit
    uint8_t checkClearTxStatus(uint8_t *status, uint8_t iTxBuf = 0xff);                                                                                  // read and clear and return first found or buffer specified tx status bit
    bool mcpPinMode(const uint8_t pin, const uint8_t mode);                                                                                           // switch supported pins between HiZ, interrupt, output or input
    bool mcpDigitalWrite(const uint8_t pin, const uint8_t mode);                                                                                      // write HIGH or LOW to RX0BF/RX1BF
    uint8_t mcpDigitalRead(const uint8_t pin);

    void singleByteTransfer(uint8_t value);
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
    uint8_t mcp2515_init(void); //const uint8_t canSpeed, const uint8_t clock);       // mcp2515init

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
    uint8_t ext_flg; // identifier xxxID
    // // either extended (the 29 LSB) or standard (the 11 LSB)
    unsigned long can_id; // can id
    uint8_t rtr;             // is remote frame
    // uint8_t SPICS;
    // // SPIClass *pSPI;
    uint8_t mcpMode;     // Current controller mode

};


/*********************************************************************************************************
    END FILE
*********************************************************************************************************/