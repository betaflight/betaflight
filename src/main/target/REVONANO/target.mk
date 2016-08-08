F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
		   drivers/accgyro_spi_mpu9250.c \
		   drivers/barometer_ms5611.c