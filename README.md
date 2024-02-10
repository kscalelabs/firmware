# firmware

## Resources

- [Demo we are replicating](https://www.youtube.com/watch?v=EMWync-BGmo&ab_channel=Skyentific)
- [MyActuator documents](https://www.myactuator.com/dowload)
- [CAN Bus hat Wiki](https://www.waveshare.com/wiki/2-CH_CAN_HAT)

## Notes

### Raspberry Pi 4 (New)

Set up the CAN bus:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536
```
