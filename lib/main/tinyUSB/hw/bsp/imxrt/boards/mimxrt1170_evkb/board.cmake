set(MCU_VARIANT MIMXRT1176)

if (M4 STREQUAL "1")
  set(MCU_CORE _cm4)
  set(JLINK_CORE _M4)
  set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/${MCU_VARIANT}xxxxx${MCU_CORE}_ram.ld)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
else ()
  set(MCU_CORE _cm7)
  set(JLINK_CORE _M7)
endif()

set(JLINK_DEVICE MIMXRT1176xxxA${JLINK_CORE})
set(PYOCD_TARGET mimxrt1170${MCU_CORE})
set(NXPLINK_DEVICE MIMXRT1176xxxxx:MIMXRT1170-EVK)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/evkbmimxrt1170_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1176DVMAA${MCU_CORE}
    BOARD_TUD_RHPORT=0
    BOARD_TUH_RHPORT=1
    )
endfunction()
