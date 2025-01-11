cmake_minimum_required(VERSION 3.5)

# Apply board specific content i.e IDF_TARGET must be set before project.cmake is included
include("${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake")
string(TOUPPER ${IDF_TARGET} FAMILY_MCUS)

# Device port default to Port1 for P4 (highspeed), Port0 for others (fullspeed)
set(RHPORT_SPEED OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED)

if (NOT DEFINED RHPORT_DEVICE)
  if (IDF_TARGET STREQUAL "esp32p4")
    set(RHPORT_DEVICE 1)
  else ()
    set(RHPORT_DEVICE 0)
  endif ()
endif()

if (NOT DEFINED RHPORT_HOST)
  if (IDF_TARGET STREQUAL "esp32p4")
    set(RHPORT_HOST 1)
  else ()
    set(RHPORT_HOST 0)
  endif ()
endif()

if (NOT DEFINED RHPORT_DEVICE_SPEED)
  list(GET RHPORT_SPEED ${RHPORT_DEVICE} RHPORT_DEVICE_SPEED)
endif ()
if (NOT DEFINED RHPORT_HOST_SPEED)
  list(GET RHPORT_SPEED ${RHPORT_HOST} RHPORT_HOST_SPEED)
endif ()

# Add example src and bsp directories
set(EXTRA_COMPONENT_DIRS "src" "${CMAKE_CURRENT_LIST_DIR}/boards" "${CMAKE_CURRENT_LIST_DIR}/components")

# set SDKCONFIG for each IDF Target
set(SDKCONFIG ${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
