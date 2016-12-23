F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c\
            drivers/max7456.c \
            io/osd.c

