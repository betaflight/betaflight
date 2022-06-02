F411_TARGETS    += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/rx/expresslrs_driver.c \
            drivers/rx/rx_sx127x.c \
            drivers/rx/rx_sx1280.c \
            rx/expresslrs_telemetry.c \
            rx/expresslrs_common.c \
            rx/expresslrs.c

