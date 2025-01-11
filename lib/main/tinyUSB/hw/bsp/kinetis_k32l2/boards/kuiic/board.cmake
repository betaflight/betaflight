set(MCU_VARIANT K32L2B31A)

set(JLINK_DEVICE K32L2B31xxxxA)
set(PYOCD_TARGET K32L2B)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/kuiic.ld)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/clock_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_K32L2B31VLH0A
    )
endfunction()
