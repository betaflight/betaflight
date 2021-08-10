H743xI_TARGETS += $(TARGET)


ifeq ($(TARGET), NUCLEOH743_RAMBASED)
FEATURES       += VCP ONBOARDFLASH
RAM_BASED = yes
else
FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO
endif

# Top level Makefile adds, if not defined, HSE_VALUE, as default for F4 targets.
# We don't want to assume any particular value until de facto design is established,
# so we set the value here.
#
# However, HSE_VALUE is currently a global build option and can not be changed from
# board to board. Generic target will have to store this value as a PG variable and
# change clock on the fly after the PG became readable.

HSE_VALUE    = 8000000 # For NUCLEO-H743ZI with STLINK, HSE is 8MHz from STLINK

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_bmp388.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/max7456.c \
