set(MCU_VARIANT MIMXRT1011)

set(JLINK_DEVICE MIMXRT1011xxx5A)
set(PYOCD_TARGET mimxrt1010)
set(NXPLINK_DEVICE MIMXRT1011xxxxx:EVK-MIMXRT1010)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1010_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1011DAE5A
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
