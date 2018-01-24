F7X2RE_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = $(subst ./src/main/,,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c)))
