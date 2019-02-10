F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

FEATURE_CUT_LEVEL = 10

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c
