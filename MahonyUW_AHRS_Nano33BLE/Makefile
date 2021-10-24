OPENOCD=/usr/bin/openocd
OUTPUT_BINARY=MahonyUW_AHRS_Nano33BLE.ino.with_bootloader.ARDUINO_NANO33BLE.bin

#
# Before calling 'make flash' don't forget to export the binary using the
# Arduino IDE: open the ino file, and "Sketch -> Export Compiled Binary"
#
flash:
	$(OPENOCD) -d2 -f openocd_nrf52.cfg -c 'init_reset halt; program $(OUTPUT_BINARY) verify; reset; exit'
