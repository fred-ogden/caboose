CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -O2

CLI_PROGRAMS = h8_probe h8_read h8_launch
GUI_PROGRAM  = caboose

.PHONY: all gui clean

all: $(CLI_PROGRAMS)

h8_probe: h8_probe.c
	$(CC) $(CFLAGS) -o $@ $<

h8_read: h8_read.c
	$(CC) $(CFLAGS) -o $@ $<

h8_launch: h8_launch.c
	$(CC) $(CFLAGS) -o $@ $<

gui: $(GUI_PROGRAM)

$(GUI_PROGRAM): caboose.c h8_protocol.c h8_protocol.h
	$(CC) $(CFLAGS) $(shell pkg-config --cflags gtk+-3.0) \
		-o $@ caboose.c h8_protocol.c \
		$(shell pkg-config --libs gtk+-3.0) -lm

clean:
	rm -f $(CLI_PROGRAMS) $(GUI_PROGRAM) *.o
