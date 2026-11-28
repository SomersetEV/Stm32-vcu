##
## Host unit tests for the S-Box + SimpBMS firmware logic.
## The real ZombieVerter S-Box driver (src/bmw_sbox.cpp) is linked in, so
## the emulator is tested against the exact frames the Zombie sends and the
## exact decoder it uses.
##
CPP       = g++
LD        = g++
CPPFLAGS  = -std=c++17 -Wall -Wextra -ggdb -I../include -I../../include -I../../libopeninv/include
BINARY    = test_sboxbms
OBJS      = test_sboxbms.o meb_bms.o soc.o simpbms_tx.o sbox_emu.o bmw_sbox.o canhardware.o
VPATH     = . ../src ../../src ../../libopeninv/src

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) -o $(BINARY) $(OBJS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY) $(BINARY).exe

.PHONY: all clean
