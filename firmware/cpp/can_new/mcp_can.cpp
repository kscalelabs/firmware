#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "mcp_can.h"

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Constructor
*********************************************************************************************************/
// MCP_CAN::MCP_CAN(uint8_t _CS)
// {
//     // pSPI = &SPI;
//     init_CS(_CS);
// }

MCP_CAN::MCP_CAN(const std::string &device, uint8_t spiMode, uint8_t bitsPerWord, uint32_t speed)
    spiMode(spiMode), bitsPerWord(bitsPerWord), speed(speed) {
    // begin starts here:
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
/*********************************************************************************************************
** Function name:           init_CS
** Descriptions:            init CS pin and set UNSELECTED
*********************************************************************************************************/
void MCP_CAN::init_CS(uint8_t _CS)
{
    // This is a 9 in arduino cases?
    if (_CS == 0)
    {
        return;
    }
    SPICS = _CS;
    // pinMode(SPICS, OUTPUT);
    // digitalWrite(SPICS, HIGH);
}

// void MCP_CAN::setSPI(SPIClass *_pSPI)
// {
//     pSPI = _pSPI; // define SPI port to use before begin()
// }


