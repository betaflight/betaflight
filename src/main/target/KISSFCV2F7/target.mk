F7X2RE_TARGETS += $(TARGET)
LD_SCRIPT       = $(ROOT)/src/main/target/$(TARGET)/stm32_flash_f722_no_split.ld
CFLAGS +=         -D'CUSTOM_VECT_TAB_OFFSET=0x8000' \
                  -DCLOCK_SOURCE_USE_HSI

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c 
