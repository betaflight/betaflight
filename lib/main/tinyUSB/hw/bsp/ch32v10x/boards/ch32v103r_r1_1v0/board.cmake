set(LD_FLASH_SIZE 64K)
set(LD_RAM_SIZE 20K)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CFG_EXAMPLE_MSC_DUAL_READONLY
    )
endfunction()
