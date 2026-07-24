TARGET_MCU        := RP2350A

DEVICE_FLAGS    += -DPICO_RP2350A=1

include $(TARGET_PLATFORM_DIR)/target/common/target_RP2350.mk
