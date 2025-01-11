set(MCU_VARIANT D6)

# 32KB zero-wait, 224KB total flash
#set(LD_FLASH_SIZE 32K)
set(LD_FLASH_SIZE 224K)
set(LD_RAM_SIZE 10K)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    SYSCLK_FREQ_144MHz_HSI=144000000
    CFG_EXAMPLE_MSC_DUAL_READONLY
    )
endfunction()
