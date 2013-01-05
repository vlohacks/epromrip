CC	= gcc
CPP	= g++
OBJ	= readeprom.o 
BIN	= readeprom

all: $(OBJ)
	$(CC) $(CCFLAGS) $(LDFLAGS) -o $(BIN) $(OBJ)

.PHONY: clean
clean: 
	rm -f $(BIN) $(OBJ)

%.o: %.c
	$(CC) $(CCFLAGS) -c -o $@ $<






