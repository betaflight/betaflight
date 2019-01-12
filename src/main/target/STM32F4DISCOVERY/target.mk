F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SPI MSC

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c

TARGET_SRC += \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_msc_desc.c
