F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

ifeq ($(TARGET), AIORACERF3)
FEATURE_CUT_LEVEL = 4
else
FEATURE_CUT_LEVEL = 8
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c 

