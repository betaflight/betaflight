idf_build_get_property(target IDF_TARGET)

set(srcs)
set(includes_public)
set(compile_options)
set(tusb_src "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../src")

string(TOUPPER OPT_MCU_${target} tusb_mcu)
list(APPEND compile_definitions
  CFG_TUSB_MCU=${tusb_mcu}
  CFG_TUSB_OS=OPT_OS_FREERTOS
  BOARD_TUD_RHPORT=${RHPORT_DEVICE}
  BOARD_TUD_MAX_SPEED=${RHPORT_DEVICE_SPEED}
  BOARD_TUH_RHPORT=${RHPORT_HOST}
  BOARD_TUH_MAX_SPEED=${RHPORT_HOST_SPEED}
  )

if (target STREQUAL esp32p4)
  # P4 change alignment to 64 (DCache line size) for possible DMA configuration
  list(APPEND compile_definitions
    CFG_TUD_MEM_ALIGN=__attribute__\(\(aligned\(64\)\)\)
    CFG_TUH_MEM_ALIGN=__attribute__\(\(aligned\(64\)\)\)
    )
endif ()

list(APPEND srcs
  # common
  ${tusb_src}/tusb.c
  ${tusb_src}/common/tusb_fifo.c
  # device
  ${tusb_src}/device/usbd.c
  ${tusb_src}/device/usbd_control.c
  ${tusb_src}/class/audio/audio_device.c
  ${tusb_src}/class/cdc/cdc_device.c
  ${tusb_src}/class/dfu/dfu_device.c
  ${tusb_src}/class/dfu/dfu_rt_device.c
  ${tusb_src}/class/hid/hid_device.c
  ${tusb_src}/class/midi/midi_device.c
  ${tusb_src}/class/msc/msc_device.c
  ${tusb_src}/class/net/ecm_rndis_device.c
  ${tusb_src}/class/net/ncm_device.c
  ${tusb_src}/class/usbtmc/usbtmc_device.c
  ${tusb_src}/class/vendor/vendor_device.c
  ${tusb_src}/class/video/video_device.c
  ${tusb_src}/portable/synopsys/dwc2/dcd_dwc2.c
  ${tusb_src}/portable/synopsys/dwc2/hcd_dwc2.c
  ${tusb_src}/portable/synopsys/dwc2/dwc2_common.c
  # host
  ${tusb_src}/host/usbh.c
  ${tusb_src}/host/hub.c
  ${tusb_src}/class/cdc/cdc_host.c
  ${tusb_src}/class/hid/hid_host.c
  ${tusb_src}/class/msc/msc_host.c
  ${tusb_src}/class/vendor/vendor_host.c
  )

# use max3421 as host controller
if (MAX3421_HOST STREQUAL "1")
  list(APPEND srcs ${tusb_src}/portable/analog/max3421/hcd_max3421.c)
  list(APPEND compile_definitions CFG_TUH_MAX3421=1)
endif ()

if (DEFINED LOG)
  list(APPEND compile_definitions CFG_TUSB_DEBUG=${LOG})
  if (LOG STREQUAL "4")
    # no inline for debug level 4
    list(APPEND compile_definitions TU_ATTR_ALWAYS_INLINE=)
  endif ()
endif()

if(DEFINED CFLAGS_CLI)
  separate_arguments(CFLAGS_CLI)
  list(APPEND compile_definitions ${CFLAGS_CLI})
endif()


idf_component_register(SRCS ${srcs}
  INCLUDE_DIRS ${tusb_src}
  REQUIRES src
  PRIV_REQUIRES esp_mm
  )

target_compile_definitions(${COMPONENT_LIB} PUBLIC ${compile_definitions})
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
