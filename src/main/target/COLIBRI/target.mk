F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH
HSE_VALUE       = 16000000

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_ms5611.c \
            drivers/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c \

