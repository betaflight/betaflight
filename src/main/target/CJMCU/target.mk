F1_TARGETS  += $(TARGET)
FLASH_SIZE  = 64

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/compass/compass_hmc5883l.c \
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            telemetry/telemetry.c \
            telemetry/ltm.c
