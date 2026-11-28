##
## ZombieVerter hardware as a BMW S-Box + SimpBMS (VW MEB modules).
##
## Build:      make -C sboxbms
## Host tests: make -C sboxbms test
##
## Reuses libopeninv, libopencm3, the linker script and a few modules from the
## VCU tree. Headers in sboxbms/include override the VCU ones of the same name
## (param_prj.h, errormessage_prj.h).
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##

BINARY      = stm32_sboxbms
OUT_DIR     = obj
TOP         = ..
PREFIX     ?= arm-none-eabi
SIZE        = $(PREFIX)-size
CC          = $(PREFIX)-gcc
CPP         = $(PREFIX)-g++
LD          = $(PREFIX)-gcc
OBJCOPY     = $(PREFIX)-objcopy
MKDIR_P     = mkdir -p
INCLUDES    = -Iinclude -I$(TOP)/include -I$(TOP)/libopeninv/include -I$(TOP)/libopencm3/include
CFLAGS      = -Os -Wall -Wextra $(INCLUDES) \
              -fno-common -fno-builtin -pedantic -DSTM32F1 -DMAX_USER_MESSAGES=30 \
              -mcpu=cortex-m3 -mthumb -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
CPPFLAGS    = -Os -Wall -Wextra $(INCLUDES) \
              -fno-common -std=c++17 -pedantic -DSTM32F1 -DMAX_USER_MESSAGES=30 \
              -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions \
              -fno-unwind-tables -mcpu=cortex-m3 -mthumb -ggdb3
LDSCRIPT    = $(TOP)/stm32_vcu.ld
LDFLAGS     = -L$(TOP)/libopencm3/lib -T$(LDSCRIPT) -march=armv7 -nostartfiles \
              -Wl,--gc-sections,-Map,$(BINARY).map

# This firmware
OBJSL       = main.o sysclk.o meb_bms.o soc.o simpbms_tx.o sbox_emu.o power.o
# Reused from the VCU tree
OBJSL      += terminal_prj.o isa_shunt.o bmw_sbox.o
# libopeninv
OBJSL      += stm32scheduler.o params.o terminal.o my_string.o digio.o my_fp.o \
              printf.o anain.o canhardware.o canmap.o param_save.o errormessage.o \
              stm32_can.o terminalcommands.o cansdo.o sdocommands.o

OBJS        = $(patsubst %.o,$(OUT_DIR)/%.o, $(OBJSL))
vpath %.c src/ $(TOP)/src/ $(TOP)/libopeninv/src/
vpath %.cpp src/ $(TOP)/src/ $(TOP)/libopeninv/src/

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

# Suppress the rwx segment warning on binutils 2.39 or later
comma := ,
try-run = $(shell set -e; if ($(1)) >/dev/null 2>&1; then echo "$(2)"; else echo "$(3)"; fi)
LDFLAGS += $(call try-run, $(PREFIX)-ld --no-warn-rwx-segments -v,-Wl$(comma)--no-warn-rwx-segments)

all: directories images

images: get-deps $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	@printf "  OBJCOPY $(BINARY).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(BINARY) $(BINARY).hex
	$(Q)$(SIZE) $(BINARY)

directories: $(OUT_DIR)

$(OUT_DIR):
	$(Q)$(MKDIR_P) $(OUT_DIR)

$(BINARY): $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(@)\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -lopencm3_stm32f1 -lm

$(OUT_DIR)/%.o: %.c Makefile | $(OUT_DIR)
	@printf "  CC      $(@)\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile | $(OUT_DIR)
	@printf "  CPP     $(@)\n"
	$(Q)$(CPP) $(CPPFLAGS) -MMD -MP -o $@ -c $<

DEP = $(OBJS:%.o=%.d)
-include $(DEP)

get-deps:
ifneq ($(shell test -s $(TOP)/libopencm3/lib/libopencm3_stm32f1.a && echo -n yes),yes)
	@printf "  GIT SUBMODULE\n"
	$(Q)cd $(TOP) && git submodule update --init
	@printf "  MAKE libopencm3\n"
	$(Q)$(MAKE) -C $(TOP)/libopencm3 TARGETS=stm32/f1
endif

test:
	$(MAKE) -C test
	test/test_sboxbms

clean:
	@printf "  CLEAN\n"
	$(Q)rm -rf $(OUT_DIR) $(BINARY) $(BINARY).bin $(BINARY).hex $(BINARY).map
	$(Q)$(MAKE) -C test clean

.PHONY: all directories get-deps images clean test
