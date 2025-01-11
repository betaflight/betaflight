set(MCU_VARIANT MIMXRT1064)

set(JLINK_DEVICE MIMXRT1064xxx6A)
set(PYOCD_TARGET mimxrt1064)
set(NXPLINK_DEVICE MIMXRT1064xxxxA:EVK-MIMXRT1064)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1064_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1064DVL6A
    BOARD_TUD_RHPORT=0
    BOARD_TUH_RHPORT=1
    )
endfunction()
