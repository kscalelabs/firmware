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

### Install for Jetson
#### Install Conda
```bash
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
chmod +x Miniforge3-Linux-aarch64.sh
./Miniforge3-Linux-aarch64.sh
source ~/.bashrc
```

#### Create Conda environment and install package
```bash
conda create --name firmware python=3.11
conda activate firmware
make install-dev
```


### Raspberry Pi 4

Set up the CAN bus (this might already be happening in a `systemctl` service):

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536
```

Test that you can send a command from `can0` and receive it from `can1`:

```bash
# In one terminal
candump can1
# In another terminal
cansend can0 123#DEADBEEF
```

Run the console for interfacing with motors:

```bash
python -m firmware.scripts.single_motor
```
