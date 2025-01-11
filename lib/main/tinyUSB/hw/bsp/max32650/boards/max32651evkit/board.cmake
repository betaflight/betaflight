# Use the secure linker file
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/max32651.ld)

function(update_board_extras TARGET)
  # for the signed target, need to add the __SLA_FWK__ define
  target_compile_definitions(${TARGET} PUBLIC
    __SLA_FWK__
    )
endfunction()

function(prepare_image TARGET_IN)
  #For the signed target, set up a POST_BUILD command to sign the elf file once
  #created
  if((WIN32) OR (MINGW) OR (MSYS))
    set(SIGN_EXE "sign_app.exe")
  else()
    set(SIGN_EXE "sign_app")
  endif()
  set(MCU_PATH "${TOP}/hw/mcu/analog/max32/")

  # Custom POST_BUILD command
  add_custom_command(
    TARGET ${TARGET_IN} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${TARGET_IN}>  -R .sig -O binary $<TARGET_FILE_DIR:${TARGET_IN}>/${TARGET_IN}.bin
    COMMAND ${MCU_PATH}/Tools/SBT/bin/${SIGN_EXE} -c MAX32651 key_file=${MCU_PATH}/Tools/SBT/devices/MAX32651/keys/maximtestcrk.key
            ca=$<TARGET_FILE_DIR:${TARGET_IN}>/${TARGET_IN}.bin sca=$<TARGET_FILE_DIR:${TARGET_IN}>/${TARGET_IN}.sbin
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${TARGET_IN}> --update-section .sig=$<TARGET_FILE_DIR:${TARGET_IN}>/${TARGET_IN}.sig
    VERBATIM
    )
endfunction()
