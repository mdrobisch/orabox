# orabox

python -m serial.tools.miniterm COM5 115200

idf.py set-target esp32
idf.py build


 esptool -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/play_mp3.bin