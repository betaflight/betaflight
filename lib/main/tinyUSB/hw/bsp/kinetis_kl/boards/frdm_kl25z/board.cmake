set(MCU_VARIANT MKL25Z4)

set(JLINK_DEVICE MKL25Z128xxx4)
set(PYOCD_TARGET mkl25zl128)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../gcc/MKL25Z128xxx4_flash.ld)
set(STARTUP_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../gcc/startup_MKL25Z4.S)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/clock_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MKL25Z128VLK4
    CFG_EXAMPLE_MSC_READONLY
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
