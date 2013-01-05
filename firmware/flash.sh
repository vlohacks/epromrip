sudo avrdude -c avrispmkII -p m32 -P usb -U flash:w:eepromripper.hex -U lfuse:w:0xe4:m -U hfuse:w:0xd9:m 
