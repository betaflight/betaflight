set(MCU_VARIANT mm32f327x)
set(JLINK_DEVICE MM32F3273G8P)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    HSE_VALUE=12000000
    )
endfunction()
