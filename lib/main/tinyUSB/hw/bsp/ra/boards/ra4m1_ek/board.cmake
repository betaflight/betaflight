set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
set(MCU_VARIANT ra4m1)

set(JLINK_DEVICE R7FA4M1AB)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CFG_EXAMPLE_VIDEO_READONLY
    )
#  target_sources(${TARGET} PRIVATE)
#  target_include_directories(${BOARD_TARGET} PUBLIC)
endfunction()
