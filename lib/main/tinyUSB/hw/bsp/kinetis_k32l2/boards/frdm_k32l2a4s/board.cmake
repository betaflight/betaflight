set(MCU_VARIANT K32L2A41A)

set(JLINK_DEVICE K32L2A41xxxxA)
set(PYOCD_TARGET K32L2A)

set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/K32L2A41xxxxA_flash.ld)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/clock_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_K32L2A41VLH1A
    )
endfunction()
