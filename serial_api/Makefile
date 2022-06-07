

DIALECT = -std=c11
CFLAGS += $(DIALECT) -Og -g -W -D_GNU_SOURCE -Wall
LIBS = -lm -pthread -lpthread -lcurl -lz -lpanel -lncurses  -lrt
LDFLAGS =


all: serial_dump
	
%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

serial_dump: serial-driver.o bits.c gpsutils.o subframe.o gpsd.o driver_ubx.o serial_dump.o $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR)

clean:
	rm -f *.o  serial_dump
