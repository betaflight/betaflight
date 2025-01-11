set(MCU_VARIANT XMC4700)

set(JLINK_DEVICE XMC4700-2048)
set(LD_FILE_GNU ${SDK_DIR}/CMSIS/Infineon/COMPONENT_${MCU_VARIANT}/Source/TOOLCHAIN_GCC_ARM/XMC4700x2048.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    XMC4700_F144x2048
    )
endfunction()
