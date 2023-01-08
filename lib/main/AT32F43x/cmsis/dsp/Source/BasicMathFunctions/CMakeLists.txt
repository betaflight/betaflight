cmake_minimum_required (VERSION 3.6)

project(CMSISDSPBasicMath)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPBasicMath STATIC ${SRC})

configLib(CMSISDSPBasicMath ${ROOT})
configDsp(CMSISDSPBasicMath ${ROOT})

### Includes
target_include_directories(CMSISDSPBasicMath PUBLIC "${DSP}/Include")



