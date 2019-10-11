F3_TARGETS  += $(TARGET)

FEATURES    = ONBOARDFLASH VCP

FEATURE_CUT_LEVEL = 0

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c
