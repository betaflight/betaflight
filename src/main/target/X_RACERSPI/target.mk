F3_TARGETS   += $(TARGET)
FEATURES     = ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
            drivers/display_ug2864hsweg01.h \
            drivers/flash_m25p16.c

