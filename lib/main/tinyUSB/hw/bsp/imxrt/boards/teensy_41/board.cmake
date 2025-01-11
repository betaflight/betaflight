set(MCU_VARIANT MIMXRT1062)

set(JLINK_DEVICE MIMXRT1062xxx6A)
set(PYOCD_TARGET mimxrt1060)
set(NXPLINK_DEVICE MIMXRT1062xxxxA:EVK-MIMXRT1060)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/teensy41_flexspi_nor_config.c
    )
  target_compile_definitions(${TARGET} PUBLIC
    CPU_MIMXRT1062DVL6A
    BOARD_TUD_RHPORT=0
    BOARD_TUH_RHPORT=1
    )
endfunction()

# flash by using teensy_loader_cli https://github.com/PaulStoffregen/teensy_loader_cli
# Make sure it is in your PATH
# flash: $(BUILD)/$(PROJECT).hex
# teensy_loader_cli --mcu=imxrt1062 -v -w $<
