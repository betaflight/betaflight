set(MCU_VARIANT MK64F12)

set(JLINK_DEVICE MK64FX512xxx12)
set(TEENSY_MCU TEENSY35)

set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/MK64FX512xxx12_flash.ld)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/board/pin_mux.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/board/clock_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MK64FX512VMD12
    )
endfunction()
