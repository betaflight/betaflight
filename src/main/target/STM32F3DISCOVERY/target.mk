F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_adxl345.c \
            drivers/accgyro/accgyro_bma280.c \
            drivers/accgyro/accgyro_l3gd20.c \
            drivers/accgyro/accgyro_l3g4200d.c \
            drivers/accgyro/accgyro_lsm303dlhc.c \
            drivers/accgyro/accgyro_adxl345.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_bma280.c \
            drivers/accgyro/accgyro_mma845x.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu3050.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_bmi160.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_fake.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_fake.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c
