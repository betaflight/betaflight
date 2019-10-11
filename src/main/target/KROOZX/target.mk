F405_TARGETS   += $(TARGET)

FEATURES       += VCP SDCARD_SPI

FEATURE_CUT_LEVEL = 0

HSE_VALUE    = 16000000

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/max7456.c
