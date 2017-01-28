F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
           drivers/accgyro_mpu6500.c \
           drivers/accgyro_spi_mpu6500.c \
           drivers/barometer_ms5611.c
