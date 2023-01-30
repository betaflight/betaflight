cmake_minimum_required (VERSION 3.6)

project(CMSISDSPMatrix)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPMatrix STATIC ${SRC})

configLib(CMSISDSPMatrix ${ROOT})
configDsp(CMSISDSPMatrix ${ROOT})

### Includes
target_include_directories(CMSISDSPMatrix PUBLIC "${DSP}/Include")



