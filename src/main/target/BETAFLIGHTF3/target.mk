
F3_TARGETS   += $(TARGET)
FEATURES     = VCP SDCARD
TARGET_FLAGS = -DSPRACINGF3

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/display_ug2864hsweg01.h \
            drivers/flash_m25p16.c \
            drivers/max7456.c \
            io/osd.c