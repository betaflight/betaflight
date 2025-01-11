set(CMAKE_SYSTEM_PROCESSOR arm1176jzf-s CACHE INTERNAL "System Processor")
#set(SUFFIX "")

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    BCM_VERSION=2835
    )
endfunction()
