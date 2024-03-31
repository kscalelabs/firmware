# firmware

## Resources

- [Demo we are replicating](https://www.youtube.com/watch?v=EMWync-BGmo&ab_channel=Skyentific)
- [MyActuator documents](https://www.myactuator.com/dowload)
- [CAN Bus hat Wiki](https://www.waveshare.com/wiki/2-CH_CAN_HAT)

## Notes

### Install
```bash
pip install -e .
```

### Raspberry Pi 4

Set up the CAN bus:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536
```

Run the console for interfacing with motors:

```bash
python -m firmware.scripts.single_motor
```
