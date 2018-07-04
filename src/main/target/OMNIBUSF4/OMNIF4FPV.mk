# ...this clone has an MPU6000 (SPI), an OSD but no SDCard Slot
F4_TARGETS   += $(TARGET)

FEATURES       += VCP ONBOARDFLASH
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c
			