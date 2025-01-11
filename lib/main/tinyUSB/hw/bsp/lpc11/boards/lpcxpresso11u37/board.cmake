set(LPC_FAMILY 11xx)
set(JLINK_DEVICE LPC11U37/401)
set(PYOCD_TARGET lpc11u37)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/lpc11u37.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CFG_EXAMPLE_MSC_READONLY
    CFG_EXAMPLE_VIDEO_READONLY
    )
  target_sources(${TARGET} PRIVATE
    ${SDK_DIR}/src/gpio_${LPC_FAMILY}_1.c
    ${SDK_DIR}/src/sysctl_${LPC_FAMILY}.c
    )
  target_compile_options(${TARGET} PRIVATE
    -Wno-error=unused-parameter
    )
endfunction()
