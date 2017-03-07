F3_TARGETS  += $(TARGET)
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/vtx_rtc6705.c \
            io/vtx.c

