# firmware

## Resources

- [Demo we are replicating](https://www.youtube.com/watch?v=EMWync-BGmo&ab_channel=Skyentific)
- [CAN Bus Library this script uses](https://github.com/adafruit/Adafruit_CircuitPython_MCP2515/tree/main)
- [Enable SPI devices and CAN Bus shield](https://hub.libre.computer/t/waveshare-rs485-can-hat-on-aml-s905x-cc-le-potato/84)
- [List board info](https://hub.libre.computer/t/libre-computer-wiring-tool/40)
- [Alternative CAN Bus library to use](https://python-can.readthedocs.io/en/stable/bus.html)
- [MyActuator documents](https://www.myactuator.com/dowload)
- [CAN bus on Raspberry Pi using MCP2515](https://forums.raspberrypi.com/viewtopic.php?t=141052)

## Notes

### Le Potato

#### Commands

```bash
# Device Overlays
ldto list  # List available device overlays
ldto reset  # Reset device overlays

# CAN Bus
ip link set can0 up  # Bring up CAN Bus
ip link set can0 down  # Bring down CAN Bus
ip link set can0 type can bitrate 1000000 loopback on  # Enable loopback mode
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

### Raspberry Pi 4 (Old)

```bash
ls /dev/spidev0.0  # This should exist before the device overlay is added.
ls /sys/bus/spi/devices/spi0.0/net/can0  # This should exist after, if the CAN board is connected correctly.
```

Add this to `/boot/firmware/config.txt`:

```bash
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=12
dtoverlay=spi0-1cs
```

### Raspberry Pi 4 (New)

Install Blinka:

```bash
python -m firmware.install_blinka
```

Set up the CAN bus correctly:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
```
