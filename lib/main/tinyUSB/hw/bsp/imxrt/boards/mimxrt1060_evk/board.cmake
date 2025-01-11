set(MCU_VARIANT MIMXRT1062)

set(JLINK_DEVICE MIMXRT1062xxx6A)
#set(JLINK_OPTION "-USB 000726129165")
set(PYOCD_TARGET mimxrt1060)
set(NXPLINK_DEVICE MIMXRT1062xxxxA:EVK-MIMXRT1060)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1060_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1062DVL6A
    BOARD_TUD_RHPORT=0
    BOARD_TUH_RHPORT=1
    )
endfunction()
