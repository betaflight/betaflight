set(MCU_VARIANT MIMXRT1021)

set(JLINK_DEVICE MIMXRT1021xxx5A)
set(PYOCD_TARGET mimxrt1020)
set(NXPLINK_DEVICE MIMXRT1021xxxxx:EVK-MIMXRT1020)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkmimxrt1020_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1021DAG5A
    )
endfunction()
