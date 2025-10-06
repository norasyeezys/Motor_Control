On Linux do ``avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -o file.elf file.c
avr-objcopy -O ihex -R .eeprom file.elf file.hex
avrdude -v -c arduino -p m328p -P /dev/ttyUSB0 -b 115200 -D -U flash:w:file.hex:i``

You might need `sudo` for that last command. On Windows or Mac good luck lol.

Also `basic_PD.c` is a fake PD controller (hence that one line) so there is no sensor dependence. This code only works for one particular type of board. Your board may have different pins for the motors.
