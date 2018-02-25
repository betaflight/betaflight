
F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/dma_spi.c \
            drivers/accgyro/accgyro_imuf9001.c \
            drivers/max7456.c