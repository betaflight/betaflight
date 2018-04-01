F411_TARGETS    += $(TARGET)

FEATURES        += VCP SDCARD

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            drivers/rx/rx_cc2500.c \
            rx/cc2500_frsky_shared.c \
            rx/cc2500_frsky_d.c \
            rx/cc2500_frsky_x.c
