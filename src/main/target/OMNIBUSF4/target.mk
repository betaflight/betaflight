F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_ms5611.c \
            drivers/compass_hmc5883l.c \
            drivers/max7456.c \
            io/vtx_smartaudio.c \
            io/osd.c \
            io/canvas.c \
            io/cms.c \
            io/cms_imu.c \
            io/cms_blackbox.c \
            io/cms_vtx.c \
            io/cms_ledstrip.c

