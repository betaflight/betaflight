
INCLUDE_DIRS += $(PLATFORM_DIR)/common/stm32

MCU_COMMON_SRC += \
            common/stm32/system.c \
            common/stm32/config_flash.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/mco.c \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/io_impl.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/dshot_dpwm.c \
            STM32/pwm_output_hw.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/debug_pin.c \
            common/stm32/adc_impl.c

SIZE_OPTIMISED_SRC += \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/serial_uart_pinconfig.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/pwm_output_dshot_shared.c \
            STM32/pwm_output_hw.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/io_impl.c

