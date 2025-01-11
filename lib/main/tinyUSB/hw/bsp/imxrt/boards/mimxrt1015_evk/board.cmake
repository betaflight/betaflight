set(MCU_VARIANT MIMXRT1015)

set(JLINK_DEVICE MIMXRT1015DAF5A)
set(PYOCD_TARGET mimxrt1015)
set(NXPLINK_DEVICE MIMXRT1015xxxxx:EVK-MIMXRT1015)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1015_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1015DAF5A
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
