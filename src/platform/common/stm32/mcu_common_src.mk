#
# Source files under common/stm32 that are portable across every STM32-style
# MCU family — the genuine ST parts (STM32*) and the ST-derivative vendors
# (APM32, AT32, X32). These used to be hand-listed in each family's MCU_COMMON_SRC,
# which meant a newly added common/stm32 driver had to be remembered in every
# family or it would silently go missing on some boards. Centralise them here so
# every family that includes this file picks them up automatically.
#
# Files that are NOT universal (e.g. can_hw.c, mco.c, bus_spi_hw.c,
# camera_control.c, rx_pwm_hw.c) stay in the per-family .mk that actually needs
# them. SPEED/SIZE optimisation lists also stay per-family because those sets
# legitimately differ between MCUs.

ifndef MCU_COMMON_STM32_SRC_INCLUDED
MCU_COMMON_STM32_SRC_INCLUDED := 1

MCU_COMMON_SRC += \
            common/stm32/system.c \
            common/stm32/io_impl.c \
            common/stm32/config_flash.c \
            common/stm32/adc_impl.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/ledstrip_ws2811_stm32.c \
            common/stm32/debug_pin.c \
            common/stm32/expresslrs_driver_hw.c \
            common/stm32/fault_handlers.c

endif
