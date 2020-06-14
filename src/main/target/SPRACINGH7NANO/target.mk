H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

EXST = yes
EXST_ADJUST_VMA = 0x97CE0000

FEATURES       += VCP ONBOARDFLASH

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_bmp388.c \
            drivers/barometer/barometer_ms5611.c \

