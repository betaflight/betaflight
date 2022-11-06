RX_SRC = \
    drivers/rx/expresslrs_driver.c \
    drivers/rx/rx_sx127x.c \
    drivers/rx/rx_sx1280.c \
    rx/expresslrs.c \
    rx/expresslrs_common.c \
    rx/expresslrs_telemetry.c \

G47X_TARGETS += $(TARGET)

FEATURES       += VCP SDCARD_SPI ONBOARDFLASH

TARGET_SRC = \
    $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
    $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
    $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
    $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
    drivers/max7456.c \
    drivers/vtx_rtc6705.c \
    drivers/vtx_rtc6705_soft_spi.c \
    $(RX_SRC)
