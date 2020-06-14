NUCLEOF303RE target for use with ST Nucleo-F303RE.

- VCP is not working with STM32F303RE chip, so UART5 on PD2(RX) and PC12(TX), which is not available on CC package are reserved for configurator MSP connection. UART4, which is also not available on CC package may be configured for configurator MSP connection.

- Unfortunately, UART5 (or UART4) are not handled by embedded boot loader, so flashing through these ports is not possible.

- If PA11(USB-DM) and PA12(USB-DP) are properly wired, USB DFU can be used to flash the board.

- Target definition itself is a modified copy of SPRACINGF3EVO.

It is easy to build other targets to run on Nucleo-F303RE:

1. Add MCU_FLASH_SIZE line to target.mk (This will select stm32_flash_f303_512k.ld as linker script)
    MCU_FLASH_SIZE = 512

2. Modify target.h to define extra pins used for UART5 if necessary.
    #define TARGET_IO_PORTC 0xffff
    #define TARGET_IO_PORTD BIT(2)

3. Modify target.h to define target configuration if UART5 should be used as configurator MSP connection.
    #define USE_TARGET_CONFIG

   And then add UART5 as MSP in config.c (see config.c for NUCLEOF303RE target)

