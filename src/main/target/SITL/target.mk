TARGET_MCU        := SITL
TARGET_MCU_FAMILY := SITL
SIMULATOR_BUILD    = yes

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c
