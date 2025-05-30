# === Device Map ===
DEVICE_MAP_m16     = atmega16
DEVICE_MAP_m324pa  = atmega324pa
DEVICE_MAP_m324pb  = atmega324pb

# === User Input ===
EXAMPLE ?= $(firstword $(MAKECMDGOALS))
DEVICE  ?=
F_CPU   ?=

# === Validation ===
MCU = $(DEVICE_MAP_$(DEVICE))

ifeq ($(MCU),)
$(error Unknown or missing DEVICE. Use DEVICE=m16, DEVICE=m324pa, or DEVICE=m324pb)
endif

ifeq ($(F_CPU),)
$(error Missing F_CPU value. Usage: make <example> DEVICE=<device> F_CPU=<frequency>. Example: make hello_world DEVICE=m16 F_CPU=8000000)
endif

# === Compiler and Tools ===
CC = avr-g++
OBJCOPY = avr-objcopy
SIZE = avr-size

# === Flags ===
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Iinclude

ifeq ($(MCU), $(DEVICE_MAP_m324pb))
CFLAGS += \
	-I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\2.2.509\include" \
	-B"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\2.2.509\gcc\dev\atmega324pb"
endif

# === Paths ===
SRC_DIR = examples
BIN_DIR = bin

SRC_FILE = $(SRC_DIR)/$(EXAMPLE).cpp
OUT_BASE = $(BIN_DIR)/$(EXAMPLE)_$(DEVICE)
ELF_FILE = $(OUT_BASE).elf
HEX_FILE = $(OUT_BASE).hex

# === Targets ===
.PHONY: all clean $(EXAMPLE)

all:
	@echo "Usage: make <example> DEVICE=<device> F_CPU=<frequency>"
	@echo "Example: make hello_world DEVICE=m324pb F_CPU=8000000"

$(EXAMPLE):
	@test -d $(BIN_DIR) || mkdir $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(ELF_FILE) $(SRC_FILE)
	$(OBJCOPY) -O ihex -R .eeprom $(ELF_FILE) $(HEX_FILE)
	$(SIZE) $(ELF_FILE)
	@echo "Built: $(HEX_FILE)"

clean:
	rm -rf $(BIN_DIR)
