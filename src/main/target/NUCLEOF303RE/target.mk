F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD_SPI
FLASH_SIZE  = 512
LD_SCRIPT   = $(ROOT)/src/main/target/$(TARGET)/stm32_flash_f303_512k.ld


TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c
