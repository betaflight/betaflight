F3_TARGETS  += $(TARGET)

FEATURES    = VCP ONBOARDFLASH

FEATURE_CUT_LEVEL = 0

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/vtx_rtc6705.c
