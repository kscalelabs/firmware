# Device Tree Overlays

1. Compile:

```bash
dtc -@ -I dts -O dtb -o enable-spi.dtbo enable-spi.dts
```

2. Move the compiled to the `/boot/overlays/` directory:

```bash
sudo mv enable-spi.dtbo /boot/overlays/enable-spi.dtbo
```

3. Add to `/boot/firmware/config.txt`:

```bash
dtoverlay=enable-spi
```
