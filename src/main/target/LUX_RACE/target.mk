F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

ifeq ($(TARGET), LUXV2_RACE)
FEATURE_CUT_LEVEL = 8
else
FEATURE_CUT_LEVEL = 0
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c
