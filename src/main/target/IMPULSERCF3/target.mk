F3_TARGETS  += $(TARGET)

FEATURES    = VCP ONBOARDFLASH

FEATURE_CUT_LEVEL = 10

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c 
