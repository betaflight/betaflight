TARGET_MCU := SITL
SIMULATOR_BUILD = yes

FEATURES       += #SDCARD_SPI VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c
