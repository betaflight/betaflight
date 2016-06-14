FEATURES     = VCP ONBOARDFLASH
F3_TARGETS  += $(TARGET)

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/serial_softserial.c \
            drivers/vtx_rtc6705.c


