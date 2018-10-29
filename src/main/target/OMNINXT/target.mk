ifeq ($(TARGET), OMNINXT4)
F405_TARGETS    += $(TARGET)
else
ifeq ($(TARGET), OMNINXT7)
F7X2RE_TARGETS  += $(TARGET)
else
# Nothing to do for generic target
endif
endif

FEATURES       += VCP ONBOARDFLASH

# XXX Remove fake drivers for final production
TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/barometer/barometer_lps.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/max7456.c
