F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD 

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
			drivers/compass_ak8963.c \
            drivers/flash_m25p16.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_stm32f30x.c \
            io/transponder_ir.c

ifeq ($(TARGET), TINYBEEF3)
TARGET_SRC += \
            drivers/accgyro_spi_mpu6500.c
endif
