LD_FILE = $(FAMILY_PATH)/ch32f205.ld

SRC_S += \
	$(FAMILY_PATH)/startup_gcc_ch32f20x_d8c.s

CFLAGS += \
	-DCH32F20x_D8C
