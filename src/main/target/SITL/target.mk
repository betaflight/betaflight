SITL_TARGETS += $(TARGET)
#FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro_fake.c \
            drivers/barometer_fake.c \
            drivers/compass_fake.c \
            drivers/serial_tcp.c

