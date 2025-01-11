# TODO more docs and example on how to use this file
# Usage: requires target tinyusb_config which expose tusb_config.h file
# TINYUSB_TARGET_PREFIX and TINYUSB_TARGET_SUFFIX can be used to change the name of the target

cmake_minimum_required(VERSION 3.17)

# Add tinyusb to a target, if user don't want to compile tinyusb as a library
function(add_tinyusb TARGET)
  target_sources(${TARGET} PRIVATE
    # common
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/tusb.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/common/tusb_fifo.c
    # device
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/device/usbd.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/device/usbd_control.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/audio/audio_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/cdc/cdc_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/dfu/dfu_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/dfu/dfu_rt_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/hid/hid_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/midi/midi_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/msc/msc_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/net/ecm_rndis_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/net/ncm_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/usbtmc/usbtmc_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/vendor/vendor_device.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/video/video_device.c
    # host
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/host/usbh.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/host/hub.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/cdc/cdc_host.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/hid/hid_host.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/msc/msc_host.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/class/vendor/vendor_host.c
    # typec
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/typec/usbc.c
    )
  target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    # TODO for net driver, should be removed/changed
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../lib/networking
    )

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${TARGET} PRIVATE
      -Wall
      -Wextra
      -Werror
      -Wfatal-errors
      -Wdouble-promotion
      -Wstrict-prototypes
      -Wstrict-overflow
      -Werror-implicit-function-declaration
      -Wfloat-equal
      -Wundef
      -Wshadow
      -Wwrite-strings
      -Wsign-compare
      -Wmissing-format-attribute
      -Wunreachable-code
      -Wcast-align
      -Wcast-function-type
      -Wcast-qual
      -Wnull-dereference
      -Wuninitialized
      -Wunused
      -Wunused-function
      -Wreturn-type
      -Wredundant-decls
      -Wmissing-prototypes
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "IAR")

  endif ()
endfunction()

#------------------------------------
# TinyUSB as library target
#------------------------------------
if (NOT DEFINED TINYUSB_TARGET)
  set(TINYUSB_TARGET "tinyusb")
endif ()

set(TINYUSB_CONFIG_TARGET "${TINYUSB_TARGET}_config")

if (DEFINED TINYUSB_TARGET_PREFIX)
  set(TINYUSB_TARGET "${TINYUSB_TARGET_PREFIX}${TINYUSB_TARGET}")
  set(TINYUSB_CONFIG_TARGET "${TINYUSB_TARGET_PREFIX}${TINYUSB_CONFIG_TARGET}")
endif ()

if (DEFINED TINYUSB_TARGET_SUFFIX)
  set(TINYUSB_TARGET "${TINYUSB_TARGET}${TINYUSB_TARGET_SUFFIX}")
  set(TINYUSB_CONFIG_TARGET "${TINYUSB_CONFIG_TARGET}${TINYUSB_TARGET_SUFFIX}")
endif ()

add_library(${TINYUSB_TARGET} STATIC)
add_tinyusb(${TINYUSB_TARGET})

# Check if tinyusb_config target is defined
if (NOT TARGET ${TINYUSB_CONFIG_TARGET})
  message(FATAL_ERROR "${TINYUSB_CONFIG_TARGET} target is not defined")
endif()

# Link with tinyusb_config target
target_link_libraries(${TINYUSB_TARGET} PUBLIC
  ${TINYUSB_CONFIG_TARGET}
  )
