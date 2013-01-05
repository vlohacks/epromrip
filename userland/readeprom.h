#ifndef _READEPROM_H
#define _READEPROM_H

#define BAUDRATE B19200
#define LINEBUFFER_SIZE 81

typedef enum {
	eprom_char = 1,
	ack = 2,
	done = 8,
	unknown = 9,
	error = 10,
} linetype_t;

typedef struct {
	linetype_t type;
	unsigned char data;
	int offset;
} line_t;


#endif


