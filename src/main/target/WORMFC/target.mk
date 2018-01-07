F405_TARGETS    += $(TARGET)
FEATURES        += VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_lps.c \
            drivers/max7456.c \
            io/osd.c
            
            
TARGET_SRC += \
            drivers/sdio_stdlib.c \
            drivers/sdcard_sdio_stdlib.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c