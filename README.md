# oraboxcd

## Useful commands

### Connect to Board

```
python -m serial.tools.miniterm COM5 115200
```

### Setup und build firmware project

```
idf.py set-target esp32
idf.py build
```

### Flash project to board

Change Serial Port (COM5) and project directory bin (firmware/test_play_mp3/build/play_mp3.bin)


```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_play_mp3/build/bootloader/bootloader.bin 0x8000 firmware/test_play_mp3/build/partition_table/partition-table.bin 0x10000 firmware/test_play_mp3/build/play_mp3.bin
 ```

```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_softAP/build/bootloader/bootloader.bin 0x8000 firmware/test_softAP/build/partition_table/partition-table.bin 0x10000 firmware/test_softAP/build/wifi_softAP.bin
 ```

```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_file_server/build/bootloader/bootloader.bin 0x8000 firmware/test_file_server/build/partition_table/partition-table.bin 0x10000 firmware/test_file_server/build/file_server.bin
 ```

```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_sd_card/build/bootloader/bootloader.bin 0x8000 firmware/test_sd_card/build/partition_table/partition-table.bin 0x10000 firmware/test_sd_card/build/sd_card.bin
 ```

 ```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_pn532/build/bootloader/bootloader.bin 0x8000 firmware/test_pn532/build/partition_table/partition-table.bin 0x10000 firmware/test_pn532/build/hello-world.bin
 ```

```
esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 firmware/test_rc522/build/bootloader/bootloader.bin 0x8000 firmware/test_rc522/build/partition_table/partition-table.bin 0x10000 firmware/test_rc522/build/hello-world.bin
 ```