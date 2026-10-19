TARGET_MCU        := RP2350B
TARGET_MCU_FAMILY := RP2350

# In pico-sdk, PICO_RP2350A=0 means RP2350B family.
DEVICE_FLAGS    += -DPICO_RP2350A=0

include $(TARGET_PLATFORM_DIR)/target/common/target_RP2350.mk
