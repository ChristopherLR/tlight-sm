# Makefile to automate the usage of avra and avrdude in the programming of the atmega328p

BUILD_DIR=./build/
AVR_CONF=../avrdude.conf
USB_MODEM := $(shell ls /dev/tty.usb*)
INCLUDE_DIR=./includes

build:
	mkdir build
	avra main.asm -I $(INCLUDE_DIR)
	mv *.hex $(BUILD_DIR)
	mv *.obj $(BUILD_DIR)

run:
	avrdude -C $(AVR_CONF) -v -p atmega328p -c arduino -P $(USB_MODEM) -b 115200 -D -U flash:w:$(BUILD_DIR)main.hex:i

clean:
	rm -rf build/
	rm -f *.obj *.hex

install: clean build run
