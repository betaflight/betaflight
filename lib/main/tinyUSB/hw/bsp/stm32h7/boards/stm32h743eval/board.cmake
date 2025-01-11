set(MCU_VARIANT stm32h743xx)
set(JLINK_DEVICE stm32h743xi)
#set(JLINK_OPTION "-USB jtrace")

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../linker/${MCU_VARIANT}_flash.ld)

set(RHPORT_SPEED OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED)

# device default to PORT 1 High Speed
if (NOT DEFINED RHPORT_DEVICE)
  set(RHPORT_DEVICE 1)
endif()
if (NOT DEFINED RHPORT_HOST)
  set(RHPORT_HOST 0)
endif()

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${ST_MFXSTM32L152}/mfxstm32l152.c
    ${ST_MFXSTM32L152}/mfxstm32l152_reg.c
    )
  target_include_directories(${TARGET} PUBLIC
    ${ST_MFXSTM32L152}
    )
  target_compile_definitions(${TARGET} PUBLIC
    STM32H743xx
    HSE_VALUE=25000000
    )
endfunction()
