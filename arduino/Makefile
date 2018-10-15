all:
	avr-gcc -mmcu=atmega328p ./main.c -o ./main
	avrdude -c avrisp2 -p m328p -U flash:w:main:e -v
