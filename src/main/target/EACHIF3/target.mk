F3_TARGETS  += $(TARGET)

FEATURES    = VCP

FEATURE_CUT_LEVEL = 0

TARGET_SRC = \
			drivers/accgyro/accgyro_mpu.c \
			drivers/accgyro/accgyro_mpu6500.c \
			drivers/accgyro/accgyro_spi_mpu6500.c \
			drivers/barometer/barometer_bmp280.c \
			drivers/barometer/barometer_ms5611.c \
			drivers/rx/rx_a7105.c \
			rx/a7105_flysky.c
