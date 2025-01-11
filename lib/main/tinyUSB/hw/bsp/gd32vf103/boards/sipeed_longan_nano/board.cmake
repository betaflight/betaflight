set(JLINK_DEVICE gd32vf103cbt6)

set(SDK_BSP_DIR ${SOC_DIR}/Board/gd32vf103c_longan_nano)
set(LD_FILE_GNU ${SDK_BSP_DIR}/Source/GCC/gcc_gd32vf103xb_flashxip.ld)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${SDK_BSP_DIR}/Source/gd32vf103c_longan_nano.c
    )
  target_include_directories(${TARGET} PUBLIC
    ${SDK_BSP_DIR}/Include
    )
endfunction()
