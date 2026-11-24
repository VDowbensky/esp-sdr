python -m esptool --chip esp32c5 -p COM39 -b 921600 write_flash @flash_args --force
rem python -m esptool -p COM38 -b 921600 write_flash @flash_args_2