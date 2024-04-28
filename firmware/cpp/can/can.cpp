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
  /* Equivalent
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
    mcpSPI->endTransaction();
  */  
  uint8_t tx[] = {MCP_WRITE, address, value};
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
}

void MCP_CAN::modifyRegister(uint8_t address, uint8_t mask, uint8_t data)
{
    // mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    // MCP2515_SELECT();
    // spi_readwrite(MCP_BITMOD);
    // spi_readwrite(address);
    // spi_readwrite(mask);
    // spi_readwrite(data);
    // MCP2515_UNSELECT();
    // mcpSPI->endTransaction();
  uint8_t tx[] = {MCP_BITMOD, address, mask, data};
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
}

void MCP_CAN::writeId( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    
    setRegisterS( mcp_addr, tbufdata, 4 );
}

void MCP_CAN::writeCanMsg( const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
	
    if (m_nRtr == 1)                                                   /* if RTR set bit in byte       */
        m_nDlc |= MCP_RTR_MASK;  

    setRegister((mcp_addr+4), m_nDlc );                         /* write the RTR and DLC        */
    writeId(mcp_addr, m_nExtFlg, m_nID );                      /* write CAN id                 */
}


void MCP_CAN::setRegisterS(uint8_t address, uint8_t values[], uint8_t n)
{
  /*
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);

    for (i=0; i<n; i++) 
        spi_readwrite(values[i]);
	
    MCP2515_UNSELECT();
    mcpSPI->endTransaction();
  */
  uint8_t i;
  uint8_t tx[n + 2];

  // Fill the transmission buffer
  tx[0] = MCP_WRITE; // Command byte to write
  tx[1] = address;   // Starting address for the write operation
  // Copy the values into the transmission buffer starting from the third byte
  for (uint8_t i = 0; i < n; ++i) {
      tx[i + 2] = values[i];
  }
  uint8_t rx[sizeof(tx)];
  transfer(tx, rx, sizeof(tx));
}

// new messages
uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, buf);
    return sendMsg();
}

uint8_t MCP_CAN::setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData)
{
  // this needs to be adapted and passed differently
    int i = 0;
    m_nID     = id;
    m_nDlc    = len;
    m_nExtFlg = ext;
    for(i = 0; i<m_nDlc && i<CAN_MAX_CHAR_IN_MESSAGE; i++)
        m_nDta[i] = *(pData+i);

    return MCP2515_OK;
}

uint8_t MCP_CAN::getNextFreeTXBuf(uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffer*/
            
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

uint8_t MCP_CAN::sendMsg()
{
    uint8_t res, res1, txbuf_n;
    uint32_t uiTimeOut, temp;

    // pfb30 - add temp
    // temp = micros();
    // 24 * 4 microseconds typical
    do {
        res = getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        // uiTimeOut = micros() - temp;
    } while (res == MCP_ALLTXBUSY ); //&& (uiTimeOut < TIMEOUTVALUE));

    // if(uiTimeOut >= TIMEOUTVALUE) 
    // {   
    //     return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    // }
    uiTimeOut = 0;
    writeCanMsg(txbuf_n);
    modifyRegister(txbuf_n-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);
  
    // temp = micros();
    do
    {       
        res1 = readRegister(txbuf_n-1);                         /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;
        // uiTimeOut = micros() - temp;
    } while (res1);  //&& (uiTimeOut < TIMEOUTVALUE));   
    
    // if(uiTimeOut >= TIMEOUTVALUE)                                       /* send msg timeout             */	
    //     return CAN_SENDMSGTIMEOUT;
    
    return CAN_OK;
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
             "device"_a, "spiMode"_a = SPI_MODE_0, "bitsPerWord"_a = 8, "speed"_a = 500000,
             "Constructor that initializes the MCP_CAN controller with specified SPI settings.")
        .def("reset", &MCP_CAN::reset, "Resets the MCP_CAN controller.")
        .def("sendMsgBuf", [](MCP_CAN &self, uint32_t id, uint8_t ext, uint8_t len, py::buffer buf) {
            py::buffer_info info = buf.request();
            if (info.ndim != 1) {
                throw std::runtime_error("Buffer must be 1-dimensional");
            }
            if (info.size < len) {
                throw std::runtime_error("Buffer size is smaller than the specified length");
            }
            return self.sendMsgBuf(id, ext, len, static_cast<uint8_t *>(info.ptr));
        }, "id"_a, "ext"_a, "len"_a, "buf"_a, "Sends a message buffer over the CAN network.")
        .def("set_register", &MCP_CAN::setRegister, "address"_a, "value"_a,
             "Sets a register of the MCP_CAN controller.")
        .def("read_register", &MCP_CAN::readRegister, "address"_a,
             "Reads a register from the MCP_CAN controller.");
}