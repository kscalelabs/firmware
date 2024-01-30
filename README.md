# firmware

## Resources

- [Demo we are replicating](https://www.youtube.com/watch?v=EMWync-BGmo&ab_channel=Skyentific)
- [CAN Bus Library this script uses](https://github.com/adafruit/Adafruit_CircuitPython_MCP2515/tree/main)
- [Enable SPI devices and CAN Bus shield](https://hub.libre.computer/t/waveshare-rs485-can-hat-on-aml-s905x-cc-le-potato/84)
- [List board info](https://hub.libre.computer/t/libre-computer-wiring-tool/40)
- [Alternative CAN Bus library to use](https://python-can.readthedocs.io/en/stable/bus.html)
- [MyActuator documents](https://www.myactuator.com/dowload)

## Notes

### Le Potato

#### Commands

```bash
# Device Overlays
sudo ldto list  # List available device overlays
sudo ldto reset  # Reset device overlays

# CAN Bus
sudo ip link set can0 up type can bitrate 1000000  # Bring up CAN Bus
sudo ip link set can0 down  # Bring down CAN Bus
```

#### Notes

Device overlays to merge:

```bash
sudo ldto merge spicc spicc-mcp2515-can0 i2c-ao i2c-b uart-a w1-gpio
```

Checks:

```bash
ls /sys/bus/spi/devices/spi0.0
ls /sys/bus/spi/devices/spi0.0/net
ls /dev/spidev0.0
```
