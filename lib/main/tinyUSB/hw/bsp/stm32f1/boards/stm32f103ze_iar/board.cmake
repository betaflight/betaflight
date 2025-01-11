set(MCU_VARIANT stm32f103xe)
set(JLINK_DEVICE stm32f103ze)
#set(JLINK_OPTION "-USB 320000338")

string(TOUPPER ${MCU_VARIANT} MCU_VARIANT_UPPER)

set(LD_FILE_GNU ${ST_CMSIS}/Source/Templates/gcc/linker/${MCU_VARIANT_UPPER}_FLASH.ld)
set(LD_FILE_IAR ${ST_CMSIS}/Source/Templates/iar/linker/${MCU_VARIANT}_flash.icf)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F103xE
    HSE_VALUE=8000000U
    )
endfunction()
