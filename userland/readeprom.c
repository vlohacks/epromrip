#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "readeprom.h" 

void hl(unsigned char * buffer, unsigned int offset, int count) {
        int i;

        printf("%04x: ", offset);

        for (i=0; i< 16; i++) {
		if (i > count)
			printf ("   ");
		else
	                printf ("%02x ", (unsigned)buffer[i]);
        }

        printf ("| ");

        for (i=0; i< 16; i++) {
		if (i > count)
			printf(" ");
		else
	                printf("%c", (buffer[i] >= 32 && buffer[i] < 127) ? buffer[i] : '.');
        }

        printf("\n");

}


int set_interface_attribs(int fd, int speed, int parity) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
                fprintf (stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
                fprintf (stderr, "error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking(int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
                fprintf (stderr, "error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf (stderr, "error %d setting term attributes", errno);
}

void usage(char * prog) {
	fprintf(stderr, "usage: %s -d <serial_device> [options]\n\n", prog);
	fprintf(stderr, "options:\n");
	fprintf(stderr, "  -d <device>          serial device to use (for example /dev/ttyS0)\n");
	fprintf(stderr, "  -o <filename>        file to dump eprom content to\n");
	fprintf(stderr, "  -s <hex>             start offset in hex (for example 00a0)\n");
	fprintf(stderr, "  -e <hex>             end offset in hex (for example 0fff)\n");
	fprintf(stderr, "  -q                   be quiet (no output to stdout)\n");
	fprintf(stderr, "  -x                   output hexdump of ripped data\n");
	fprintf(stderr, "  -h                   this text\n\n");
	fprintf(stderr, "example (dumps chars 0 to 65535 to file test.bin from ripper connected to ttyUSB0):\n");
	fprintf(stderr, "  %s -d /dev/ttyUSB0 -o ./test.bin -s 0 -e ffff\n\n", prog);
}


void process_line(line_t * line, char * buf) {
	char * pch;
	int tmp;
	//printf(">>%s<<\n", buf);

	line->type = unknown;
	switch (buf[0]) {
	case 'a':
		line->type = ack;
		break;

	case 'r':
		pch = strtok(buf, ":");
		pch = strtok(NULL, ":");
		if (pch != NULL) {
			sscanf(pch, "%x", &(line->offset));	
		} else {
			line->type = error;
			break;
		}
		pch = strtok(NULL, ":");

		if (pch != NULL) {
			sscanf(pch, "%x", &tmp);
			line->data = (unsigned char)tmp;
		} else {
			line->type = error;
			break;
		}
	

		line->type = eprom_char;
		break;
		
	case 'd':
		line->type = done;
		break;

	}
}

int main (int argc, char ** argv) {
	int fd;
	int i;
	//struct termios tio_old, tio_new;
	int res;
	char linebuffer[LINEBUFFER_SIZE];
	int linebuffer_offset;


	unsigned char hexbuffer[16];
	int hexbuffer_offset;

	line_t line;

	char buffer[256];

	// arguments
	int dumphex = 0;
	int quiet = 0;
	char * outfilename = 0;
	char * dev = 0;
	char * start = 0;
	char * end = 0;
	int c;

	FILE * f = 0;
	

	//char * dev = "/dev/ttyUSB0";


	while ((c = getopt(argc, argv, "xhqs:e:d:o:")) != -1) {
		switch (c) {
		case 'x':
			dumphex = 1;
			break;
		case 'q':
			quiet = 1;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
			break;
		case 's':
			start = optarg;
			break;
		case 'e':
			end = optarg;
			break;
		case 'd':
			dev = optarg;
			break;
		case 'o':
			outfilename = optarg;
			break;
		
		case '?':
			if (optopt == 's' || optopt == 'e' || optopt == 'd' || optopt == 'o') {
				fprintf(stderr, "Option -%c requires an argument\n", optopt);
			} else {
				fprintf(stderr, "Unknown option: -%c\n", optopt);
			}
			return 1;
			break;
		default:
			usage(argv[0]);
			return 1;
			break;

		}
	}

	if (dev == 0) {
		fprintf(stderr, "You have to specify a device with -d\n");
		return 1;
	}
	
	fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "ERROR opening device: %s\n", dev);
		return 1;
	}

	if (outfilename != 0) {
		f = fopen(outfilename, "wb");
		if (f == NULL) {
			fprintf(stderr, "ERROR opening file: %s\n", outfilename);
			return 1;
		}
	}


	//recvbytes = 0;
	//recvcount = 0;

	int istart = 0;
	int iend = 0xff;

	set_interface_attribs(fd, BAUDRATE, 0);
	set_blocking(fd, 0);
	
	if (start) {
		write(fd, "s", 1);
		write(fd, start, strlen(start));
		write(fd, "\n", 1); 
		sscanf(start, "%x", &istart);
	}

	while(res = read(fd, buffer, 255));

	if (end) {
		write(fd, "e", 1);
		write(fd, end, strlen(end));
		write(fd, "\n", 1); 
		sscanf(end, "%x", &iend);
	}

	while(res = read(fd, buffer, 255));

	write(fd, "r\n", 2);
	
	linebuffer_offset = 0;
	hexbuffer_offset = 0;
		//printf("read");
	int running = 1;
	int chars_recv = 0;
	while(running) {
		res = read(fd, buffer, 255);
		for (i = 0; i < res; i++) {	
			if (buffer[i] == '\n' || buffer[i] == '\r') {
				linebuffer[linebuffer_offset++] = '\0';
				process_line(&line, linebuffer);
				if (line.type == eprom_char) {
					if (f) 
						fputc(line.data, f);

					if (!quiet && !dumphex) {
						chars_recv ++;
						printf("\rread char %i of %i (%6.2f%%)                 ", chars_recv, (iend - istart)+1, ((float)chars_recv / (float)((iend - istart) + 1) * 100.0f));
					}
					if (dumphex) {
						hexbuffer[hexbuffer_offset++] = line.data;
						if (hexbuffer_offset == 16) {
							hl(hexbuffer, line.offset-15, 16);
							hexbuffer_offset = 0;
						}
					}
				}
				if (line.type == done)
					running = 0;
				linebuffer_offset = 0;
				//printf ("processed line: type=%i\n", line.type);
			} else {
				if (linebuffer_offset >= LINEBUFFER_SIZE - 1) {
					fprintf(stderr, "ERROR line too long\n");
					linebuffer_offset = 0;
				}
				linebuffer[linebuffer_offset++] = buffer[i];
			}
		}

	}
	// last partial line
	if (dumphex) {
		if (hexbuffer_offset)
			hl(hexbuffer, line.offset-(hexbuffer_offset-1), hexbuffer_offset-1);
	}

	if (!quiet && !dumphex)
		printf ("\n");

	if (f)
		fclose(f);
	
}
