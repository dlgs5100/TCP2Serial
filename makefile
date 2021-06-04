CC = gcc
CFLAGS = -g -w
CFLAGS_THREAD = -g -w

serial: serial_server

serial_server: serial_server.o
	$(CC) $(CFLAGS_THREAD) -o serial_server serial_server.o
serial_server.o: serial_server.c
	$(CC) $(CFLAGS_THREAD) -c serial_server.c
clean:
	rm -f serial_server serial_server.o
