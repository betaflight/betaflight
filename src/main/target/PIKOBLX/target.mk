F3_TARGETS  += $(TARGET)
FEATURES    = VCP

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_stm32f30x.c \
            io/transponder_ir.c

