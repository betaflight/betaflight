
INCLUDE_DIRS += $(PLATFORM_DIR)/common/stm32

# Portable common/stm32 sources shared with the APM32/AT32/X32 families.
include $(PLATFORM_DIR)/common/stm32/mcu_common_src.mk

# ST-only additions on top of the shared set.
MCU_COMMON_SRC += \
            common/stm32/can_hw.c \
            common/stm32/can_pinconfig.c \
            common/stm32/mco.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/camera_control.c \
            common/stm32/rx_pwm_hw.c \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c \
            STM32/pwm_output_hw.c \
            STM32/gyro_clkin_stm32.c

SIZE_OPTIMISED_SRC += \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/can_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/serial_uart_pinconfig.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/pwm_output_dshot_shared.c \
            STM32/pwm_output_hw.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/io_impl.c

