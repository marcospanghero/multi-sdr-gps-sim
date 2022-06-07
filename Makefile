HACKRFSDR ?= no
PLUTOSDR ?= no
LIMESDR ?= no


DIALECT = -std=c11
CFLAGS += $(DIALECT) -O0 -g -W -Wall -D_GNU_SOURCE
LIBS = -lm -pthread -lpthread -lcurl -lz -lpanel -lncurses -lrt
LDFLAGS =
SDR_OBJ = sdr_iqfile.o

ifeq ($(HACKRFSDR), yes)
    SDR_OBJ += sdr_hackrf.o
    CPPFLAGS += -DENABLE_HACKRFSDR
    CFLAGS += $(shell pkg-config --cflags libhackrf)
    LIBS_SDR += $(shell pkg-config --libs libhackrf)
endif

ifeq ($(PLUTOSDR), yes)
    SDR_OBJ += sdr_pluto.o
    CPPFLAGS += -DENABLE_PLUTOSDR
    CFLAGS += $(shell pkg-config --cflags libiio libad9361)
    LIBS_SDR += $(shell pkg-config --libs libiio libad9361)
endif

ifeq ($(LIMESDR), yes)
    SDR_OBJ += sdr_lime.o
    CPPFLAGS += -DENABLE_LIMESDR
    CFLAGS += $(shell pkg-config --cflags LimeSuite)
    LIBS_SDR += $(shell pkg-config --libs LimeSuite)
endif

all: gps-sim
	
%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

gps-sim: fifo.o almanac.o curl_station.o gps-core.o serial_api/serial-driver.o serial_api/gpsd.o serial_api/driver_ubx.o serial_api/bits.o serial_api/gpsutils.o serial_api/pps_core.o serial_api/subframe.o serial.o gui.o sdr.o gps-sim.o $(SDR_OBJ) $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR)

clean:
	rm -f *.o serial_api/*.o gps-sim
