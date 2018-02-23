F3_TARGETS  += $(TARGET)
FEATURES    = ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c

ifeq ($(TARGET), FLIP32F3OSD)
TARGET_SRC += \
            drivers/accgyro/accgyro_mpu6500.c
else
ifeq ($(TARGET), ZCOREF3)
TARGET_SRC += \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c
else
TARGET_SRC += \
            drivers/accgyro/accgyro_mpu6050.c
endif
endif
