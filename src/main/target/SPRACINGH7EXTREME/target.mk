H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

EXST = yes
EXST_ADJUST_VMA = 0x97CE0000


TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c \
           	$(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
	        $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
            