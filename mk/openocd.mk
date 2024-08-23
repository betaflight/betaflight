OPENOCD ?= openocd
OPENOCD_IF ?= interface/stlink-v2.cfg

ifeq ($(TARGET_MCU_FAMILY),STM32F4)
OPENOCD_CFG := target/stm32f4x.cfg

else ifeq ($(TARGET_MCU_FAMILY),STM32G4)
OPENOCD_CFG := target/stm32g4x.cfg

else ifeq ($(TARGET_MCU_FAMILY),STM32F7)
OPENOCD_CFG := target/stm32f7x.cfg

else ifeq ($(TARGET_MCU_FAMILY),STM32H7)
OPENOCD_CFG := target/stm32h7x.cfg
endif

ifneq ($(OPENOCD_CFG),)
OPENOCD_COMMAND = $(OPENOCD) -f $(OPENOCD_IF) -f $(OPENOCD_CFG)
endif
