set(MCU_VARIANT D6)

# 64KB zero-wait, 224KB total flash
#set(LD_FLASH_SIZE 64K)
set(LD_FLASH_SIZE 224K)
set(LD_RAM_SIZE 20K)

# set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../linker/${CH32_FAMILY}_tinyuf2.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    SYSCLK_FREQ_144MHz_HSE=144000000
    CFG_EXAMPLE_MSC_DUAL_READONLY
    )
endfunction()
