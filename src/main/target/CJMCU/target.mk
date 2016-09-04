F1_TARGETS  += $(TARGET)
FLASH_SIZE  = 64

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/compass_hmc5883l.c \
            hardware_revision.c \
            telemetry/telemetry.c \
            telemetry/ltm.c

