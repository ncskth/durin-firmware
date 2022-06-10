Firmware for durin.

uses platformio to build and upload. It's probably a bad practice but you can download it with pip. otherwise see their documentation
All durins will configure mDNS so using ```durin[id].local``` as the IP should work

### set wifi config
Durin can connect to two networks, One is hard coded to ```SSID: peopleNCS password: NCSpeople``` and the other is dynamic. To set the dynamic one use

```python durin_configurator/main.py [DURIN IP] --wifi [ssid] [password]```

### set node id
sets the node id. used for mDNS and beacon

```python durin_configurator/main.py [DURIN IP] -id [id]```

### runtime firmware update and build (recommended)
Durin will power off when it is done. you MUST power it on to verify the update

```pio run && python durin_configurator/main.py [DURIN IP] --firmware .pio/build/esp32dev/firmware.bin --verify```

### normal platformio bootloader build and upload
enter bootloader mode and run this to build and upload (might have to change the upload_port in ```platformio.ini```).

```pio run -t upload -e esp32dev```

### only build
```pio run```

binaries will be put in ```.pio/build/esp32dev/```

### bootloader firmware update
run this while in the ```.pio/build/esp32/dev``` directory.

```esptool.py --chip esp32 --port /dev/ttySS0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin```