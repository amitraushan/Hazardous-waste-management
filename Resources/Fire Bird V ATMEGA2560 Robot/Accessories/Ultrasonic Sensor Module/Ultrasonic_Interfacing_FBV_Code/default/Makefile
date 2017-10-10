###############################################################################
# Makefile for the project Ultrasonic_Interfacing_FBV
###############################################################################

## General Flags
PROJECT = Ultrasonic_Interfacing_FBV
MCU = atmega2560
TARGET = Ultrasonic_Interfacing_FBV.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99    -DF_CPU=14745600UL -O0 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=Ultrasonic_Interfacing_FBV.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = Ultrasonic_Interfacing_FBV.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Ultrasonic_Interfacing_FBV.hex Ultrasonic_Interfacing_FBV.eep Ultrasonic_Interfacing_FBV.lss size

## Compile
Ultrasonic_Interfacing_FBV.o: ../Ultrasonic_Interfacing_FBV.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) Ultrasonic_Interfacing_FBV.elf dep/* Ultrasonic_Interfacing_FBV.hex Ultrasonic_Interfacing_FBV.eep Ultrasonic_Interfacing_FBV.lss Ultrasonic_Interfacing_FBV.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

