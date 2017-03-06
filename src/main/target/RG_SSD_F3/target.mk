F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c 

HSE_VALUE      = 12000000


