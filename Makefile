#Derived from Atollic True Studio Makefile by Prof. Greg Egan 2012

#SHELL=bash

# System configuration - UNCOMMENT AS DESIRED

#Code Sourcery current gcc 4.6.x
#Specify full path below including trailing / to your arm-gcc toolchain unless it's in PATH
TCHAIN=
CC = $(TCHAIN)arm-none-eabi-gcc
OPT = -Os
OBJCOPY = $(TCHAIN)arm-none-eabi-objcopy

#Atollic TrueStudio
#CC = "C:\Program Files (x86)\Atollic\TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0\ARMTools\bin\arm-atollic-eabi-gcc"
# OBJCOPY NOT PERMITTED IN FREEBY! 
#OBJCOPY = "C:\Program Files (x86)\Atollic\TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0\ARMTools\bin\arm-atollic-eabi-objcopy"
#OPT = -Os

#Yagarto currently gcc 4.6.2
#CC = "C:\Program Files (x86)\yagarto\bin\arm-none-eabi-gcc"
#OBJCOPY = "C:\Program Files (x86)\yagarto\bin\arm-none-eabi-objcopy"
#OPT = -Os

RM = rm -rf

# Define output directory
OBJECT_DIR = obj
BIN_DIR = $(OBJECT_DIR)
LINK_SCRIPT=stm32_flash.ld

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS=-lm -mthumb -mcpu=cortex-m3 -Wl,--gc-sections -T$(LINK_SCRIPT) -static -Wl,-cref "-Wl,-Map=$(BIN_DIR)/baseflight.map" -Wl,--defsym=malloc_getpagesize_P=0x1000
ASSEMBLER_FLAGS=-c $(OPT) -mcpu=cortex-m3 -mthumb -x assembler-with-cpp -Isrc -Ilib/STM32F10x_StdPeriph_Driver/inc -Ilib/CMSIS/CM3/CoreSupport -Ilib/CMSIS/CM3/DeviceSupport/ST/STM32F10x
COMPILER_FLAGS=-c -mcpu=cortex-m3 $(OPT) -Wall -ffunction-sections -fdata-sections -mthumb -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -Isrc -Ilib/STM32F10x_StdPeriph_Driver/inc -Ilib/CMSIS/CM3/CoreSupport -Ilib/CMSIS/CM3/DeviceSupport/ST/STM32F10x

# Define sources and objects
#
#condition more to eliminate support folder files to be included
#fixes build error reported by nicodh in post#1697
SRC := 	$(wildcard lib/*/*/*/*/*/*.c) \
	$(wildcard lib/*/*/*/*/*.c) \
	$(wildcard lib/*/*/*/*.c) \
	$(wildcard lib/*/*/*.c) \
	$(wildcard lib/*/*.c) \
	$(wildcard src/*.c)

SRCSASM := $(wildcard src/*/startup_stm32f10x_md_gcc.S)

OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)

all: buildelf
	$(OBJCOPY) -O ihex "$(BIN_DIR)/baseflight.elf" "$(BIN_DIR)/baseflight.hex"

buildelf: $(OBJS) 
	$(CC) -o "$(BIN_DIR)/baseflight.elf" $(OBJS) $(LINKER_FLAGS)

clean:
	$(RM) $(OBJS) "$(BIN_DIR)/*.*"
#	$(RM) $(OBJS) "$(BIN_DIR)/baseflight.elf" "$(BIN_DIR)/baseflight.map" "$(BIN_DIR)/src/*.*" "$(BIN_DIR)/lib/*.*"

$(OBJECT_DIR)/main.o: main.c
	@mkdir -p $(dir $@) 
	$(CC) $(COMPILER_FLAGS) main.c -o $(OBJECT_DIR)/main.o 

$(OBJECT_DIR)/%.o: %.c
	@mkdir -p $(dir $@) 
	$(CC) $(COMPILER_FLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir -p $(dir $@) 
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@

