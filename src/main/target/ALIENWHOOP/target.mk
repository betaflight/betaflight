#  
#  
#     \   |   _ _| __|  \ |\ \      /|  |  _ \  _ \ _ \
#    _ \  |     |  _|  .  | \ \ \  / __ | (   |(   |__/
#  _/  _\____|___|___|_|\_|  \_/\_/ _| _|\___/\___/_|
#  
#  
ifeq ($(TARGET), ALIENWHOOPF4)
F405_TARGETS    += $(TARGET) # STM32F405RGT
else
ifeq ($(TARGET), ALIENWHOOPF7)
F7X2RE_TARGETS  += $(TARGET) # STM32F722RET
else
# Nothing to do for generic ALIENWHOOP... an MCU arch should be specified
endif
endif

FEATURES    += ONBOARDFLASH VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c
