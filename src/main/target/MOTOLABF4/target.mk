F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

ifeq ($(TARGET), MLTEMPF4)
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c
else
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c
endif
