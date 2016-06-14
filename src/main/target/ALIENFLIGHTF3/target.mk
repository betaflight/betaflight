FEATURES    = VCP
F3_TARGETS += $(TARGET)

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/compass_ak8963.c \
            hardware_revision.c \
            drivers/sonar_hcsr04.c 
