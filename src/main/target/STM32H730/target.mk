RX_SRC = \
    rx/cc2500_common.c \
    rx/cc2500_frsky_shared.c \
    rx/cc2500_frsky_d.c \
    rx/cc2500_frsky_x.c \
    rx/cc2500_sfhss.c \
    rx/cc2500_redpine.c \
    rx/a7105_flysky.c \
    rx/cyrf6936_spektrum.c \
    drivers/rx/expresslrs_driver.c \
    rx/expresslrs.c \
    rx/expresslrs_common.c \
    rx/expresslrs_telemetry.c \
    drivers/rx/rx_cc2500.c \
    drivers/rx/rx_a7105.c \
    drivers/rx/rx_cyrf6936.c \
    drivers/rx/rx_sx127x.c \
    drivers/rx/rx_sx1280.c

H730xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

ifneq ($(EXST),)
EXST = yes
ifeq ($(EXST_ADJUST_VMA),)
EXST_ADJUST_VMA = 0x90100000
endif
endif

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

TARGET_SRC = \
    msc/usbd_storage_sd_spi.c \
    drivers/sdcard_spi.c \
    drivers/bus_quadspi_hal.c \
    drivers/bus_quadspi.c \
    $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
    $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
    $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
    $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
    drivers/max7456.c \
    drivers/vtx_rtc6705.c \
    drivers/vtx_rtc6705_soft_spi.c \
    $(RX_SRC)
