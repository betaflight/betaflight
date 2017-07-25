F3_TARGETS  += $(TARGET)
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            io/osd.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/barometer/barometer_spi_ms56xx.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/pitotmeter_ms4525.c \
            drivers/pitotmeter_adc.c
