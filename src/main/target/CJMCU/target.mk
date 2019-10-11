F1_TARGETS  += $(TARGET)
FLASH_SIZE  = 64

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/rx/rx_nrf24l01.c \
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c \
            telemetry/telemetry.c \
            telemetry/ltm.c
