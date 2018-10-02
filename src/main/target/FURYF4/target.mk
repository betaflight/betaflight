F405_TARGETS    += $(TARGET)
ifeq ($(TARGET), FURYF4OSD)
FEATURES        += VCP ONBOARDFLASH
else
FEATURES        += VCP ONBOARDFLASH SDCARD_SPI
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm20689.c

ifeq ($(TARGET), FURYF4OSD)
TARGET_SRC += \
            drivers/max7456.c
else
TARGET_SRC += \
            drivers/barometer/barometer_ms5611.c
endif
