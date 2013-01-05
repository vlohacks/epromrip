#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>


#define F_CPU 8000000UL  // 8 MHz
#define BAUD_RATE 19200
#define UBRR_VAL ((F_CPU+BAUD_RATE*8)/(BAUD_RATE*16)-1) 


void waitms(int ms);
void read_eprom(void);

int usart_putchar (char c) {
	loop_until_bit_is_set(UCSRA, UDRE);
	//Ausgabe des Zeichens
	UDR = c;
	return (0);
}

void usart_init (void) {
	//Teiler wird gesetzt
	UBRRL=(F_CPU / (BAUD_RATE * 16L) - 1);
	//UBRRH = UBRR_VAL >> 8;
	//UBRRL = UBRR_VAL & 0xFF;

	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // Async 8n1
	UCSRB = (1 << TXEN) | (1 << RXEN) | (1 << RXCIE); // tx, rx, int
	//STDOUT für printf
	fdevopen (usart_putchar, NULL);
}

void io_init(void) {
	// Data
	DDRA	= 0b00000000;
	// Addr Lo
	DDRB	= 0b11111111;
	// Addr Hi
	DDRC	= 0b11111111;
	// USART & Control
	DDRD	= 0b11000011;
}


#define EEPROM_ENABLE 0b00000001
#define EEPROM_VPP    0b00000010

#define USART_IN_BUFFER_SIZE 80

volatile unsigned char usart_in_buffer[USART_IN_BUFFER_SIZE];
volatile uint8_t usart_in_buffer_pos;
volatile uint8_t usart_in_command_complete;

uint16_t eprom_start = 0x00;
uint16_t eprom_end = 0xff;

void process_command(void) {
	usart_in_command_complete = 0;
	//printf("you said: %s\n", usart_in_buffer);

	switch (usart_in_buffer[0]) {
	case 's': 
		sscanf(&usart_in_buffer[1], "%x", &eprom_start);
		printf ("ack: set start to %x\n", eprom_start);
		break;

	case 'e': 
		sscanf(&usart_in_buffer[1], "%x", &eprom_end);
		printf ("ack: set end to %x\n", eprom_end);
		break;

	case 'r': 
		cli();	// no interrupts while reading
		read_eprom();

		printf("done\n");
		sei();
		break;

	default:
		printf("invalid command\n");
		


	}
}

void read_eprom(void) {
	int i;
	unsigned char c;
	unsigned char buf[32];

	for (i = eprom_start; i <= eprom_end; i++) {

		// disable output
		PORTD |= EEPROM_VPP;
	
		// set address	
		PORTB = i & 255;
		PORTC = (i >> 8);

		// enable output
		PORTD &= ~EEPROM_VPP;

		//waitms(1);
		//waitms(10);

		// read output
		c = PINA;

		printf("r:%04x:%02x\n", i, c);
		
		if (i == eprom_end)
			break;
	}

}

int main (void) {
	unsigned int i;
	unsigned int j = 0;

	usart_init();
	io_init();
	sei();

	PORTD |= EEPROM_VPP;
	
	printf("ack:i woke up\n");

	for(;;) {
		if (usart_in_command_complete) {
			process_command();
		}
	}

}

void waitms(int ms) {
	int i;
	for (i = 0; i < ms; i++) _delay_ms(1);
}

ISR(USART_RXC_vect) {
	unsigned char c;

	c = UDR;

	if (usart_in_buffer_pos == USART_IN_BUFFER_SIZE - 2)
		c = '\n';
	
	usart_in_buffer[usart_in_buffer_pos++] = c;

	if (c == '\n' || c == '\r') {
		usart_in_buffer[usart_in_buffer_pos] = '\0';
		usart_in_buffer_pos = 0;
		usart_in_command_complete = 1;
	}
	
}

