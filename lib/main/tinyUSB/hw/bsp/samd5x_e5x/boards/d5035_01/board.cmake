set(SAM_FAMILY same51)

set(JLINK_DEVICE ATSAME51J19)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/same51j19a_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAME51J19A__
    SVC_Handler=SVCall_Handler
    CONF_CPU_FREQUENCY=80000000
    CONF_GCLK_USB_FREQUENCY=48000000
    )
endfunction()
