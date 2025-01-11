set(MCU_VARIANT MIMXRT1024)

set(JLINK_DEVICE MIMXRT1024xxx5A)
set(PYOCD_TARGET mimxrt1024)
set(NXPLINK_DEVICE MIMXRT1024xxxxx:MIMXRT1024-EVK)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1024_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1024DAG5A
    CFG_EXAMPLE_VIDEO_READONLY
    #-Wno-error=array-bounds
    )
endfunction()
