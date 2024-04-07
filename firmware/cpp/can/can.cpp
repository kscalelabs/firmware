#include "can.h"

int main() {
    int spi_fd = open("/dev/spidev0.0", O_RDWR);

    if (spi_fd < 0) {
        perror("SPI Device open failed");
        return EXIT_FAILURE;
    }

    // SPI parameters
    uint8_t spi_mode = SPI_MODE_0;
    uint8_t bits_per_word = 8;
    uint32_t speed = 500000; // 500 kHz

    // Configure SPI
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        perror("SPI Mode Change failure");
        close(spi_fd);
        return EXIT_FAILURE;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        perror("SPI BPW Change failure");
        close(spi_fd);
        return EXIT_FAILURE;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI Speed Change failure");
        close(spi_fd);
        return EXIT_FAILURE;
    }

    uint8_t tx_buffer[] = { 0x01, 0x02, 0x03 };
    uint8_t rx_buffer[sizeof(tx_buffer)] = { 0 };

    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long) tx_buffer,
        .rx_buf = (unsigned long) rx_buffer,
        .len = sizeof(tx_buffer),
        .speed_hz = speed,
        .bits_per_word = bits_per_word,
    };

    // Transmit and receive
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
        perror("SPI Transaction failure");
        close(spi_fd);
        return EXIT_FAILURE;
    }

    printf("Received: ");
    for (int i = 0; i < (int) sizeof(tx_buffer); i++) {
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n");

    // Close SPI device
    close(spi_fd);

    return 0;
}

PYBIND11_MODULE(can, m) {
    m.doc() = "CAN module for interfacing with MCP2515 CAN controller";

    m.def("main", &main, "Main function for SPI communication");
}
