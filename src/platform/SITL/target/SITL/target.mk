TARGET_MCU        := SITL
TARGET_MCU_FAMILY := SITL
SIMULATOR_BUILD    = yes

TARGET_SRC = \
            drivers/accgyro/accgyro_virtual.c \
            drivers/barometer/barometer_virtual.c \
            drivers/compass/compass_virtual.c \
            drivers/serial_tcp.c
