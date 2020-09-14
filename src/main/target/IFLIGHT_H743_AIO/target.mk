H743xI_TARGETS += $(TARGET)

HSE_VALUE   = 8000000

FEATURES    += VCP ONBOARDFLASH

TARGET_SRC += \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/barometer/barometer_dps310.c \
