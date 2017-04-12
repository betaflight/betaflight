F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
           drivers/accgyro/accgyro_mpu6500.c \
           drivers/accgyro/accgyro_spi_mpu6500.c \
           drivers/barometer/barometer_ms5611.c
