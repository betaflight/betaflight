cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Set up the project
project(usbx
    LANGUAGES C ASM
)    

if(NOT DEFINED THREADX_ARCH)
    message(FATAL_ERROR "Error: THREADX_ARCH not defined")
endif()
if(NOT DEFINED THREADX_TOOLCHAIN)
    message(FATAL_ERROR "Error: THREADX_TOOLCHAIN not defined")
endif()

# Define our target library and an alias for consumers
add_library(${PROJECT_NAME})
add_library("azrtos::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Define any required dependencies between this library and others
target_link_libraries(${PROJECT_NAME} PUBLIC 
    "azrtos::threadx"
    "azrtos::filex"
    "azrtos::netxduo"
)

# A place for generated/copied include files
set(CUSTOM_INC_DIR ${CMAKE_CURRENT_BINARY_DIR}/custom_inc)

# Pick up the port specific stuff first
if(DEFINED USBX_CUSTOM_PORT)
    add_subdirectory(${USBX_CUSTOM_PORT} usbx_port)
else()
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/ports/${THREADX_ARCH}/${THREADX_TOOLCHAIN})
endif()

# Then the common files
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/common)




# If the user provided an override, copy it to the custom directory
if (NOT UX_USER_FILE)
    message(STATUS "Using default ux_user.h file")
    set(UX_USER_FILE ${CMAKE_CURRENT_LIST_DIR}/common/core/inc/ux_user_sample.h)
else()
    message(STATUS "Using custom ux_user.h file from ${UX_USER_FILE}")
endif()
configure_file(${UX_USER_FILE} ${CUSTOM_INC_DIR}/ux_user.h COPYONLY)
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
    ${CUSTOM_INC_DIR}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "UX_INCLUDE_USER_DEFINE_FILE" )

# Enable a build target that produces a ZIP file of all sources
set(CPACK_SOURCE_GENERATOR "ZIP")
set(CPACK_SOURCE_IGNORE_FILES
  \\.git/
  \\.github/
  _build/
  \\.git
  \\.gitattributes
  \\.gitignore
  ".*~$"
)
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)