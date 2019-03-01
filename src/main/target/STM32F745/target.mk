F7X5XG_TARGETS += $(TARGET)
FEATURES       += SDCARD_SPI VCP ONBOARDFLASH

TARGET_SRC = \
	$(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
	$(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
	$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
	drivers/max7456.c
