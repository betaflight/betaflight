cmake_minimum_required (VERSION 3.6)

project(CMSISDSPDistance)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPDistance STATIC ${SRC})

configLib(CMSISDSPDistance ${ROOT})
configDsp(CMSISDSPDistance ${ROOT})

### Includes
target_include_directories(CMSISDSPDistance PUBLIC "${DSP}/Include")
target_include_directories(CMSISDSPDistance PRIVATE ".")



