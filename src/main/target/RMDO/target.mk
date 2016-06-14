FEATURES     = VCP ONBOARDFLASH
F3_TARGETS  += $(TARGET)
TARGET_FLAGS = -DSPRACINGF3

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/barometer_bmp280.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/serial_softserial.c \
            drivers/sonar_hcsr04.c 

