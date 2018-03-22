F3_TARGETS  += $(TARGET)
FEATURES  = VCP 
TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
			drivers/rx/rx_a7105.c \
			rx/flysky.c \
            drivers/max7456.c