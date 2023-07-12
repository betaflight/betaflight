OPENOCD ?= openocd
OPENOCD_IF ?= interface/stlink-v2.cfg

ifeq ($(TARGET_MCU),STM32F4)
OPENOCD_CFG := target/stm32f4x.cfg

else ifeq ($(TARGET_MCU),STM32F7)
OPENOCD_CFG := target/stm32f7x.cfg
else
endif

ifneq ($(OPENOCD_CFG),)
OPENOCD_COMMAND = $(OPENOCD) -f $(OPENOCD_IF) -f $(OPENOCD_CFG)
endif
