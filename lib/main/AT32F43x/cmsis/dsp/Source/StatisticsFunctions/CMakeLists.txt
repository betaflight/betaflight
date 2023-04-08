cmake_minimum_required (VERSION 3.6)

project(CMSISDSPStatistics)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPStatistics STATIC ${SRC})

configLib(CMSISDSPStatistics ${ROOT})
configDsp(CMSISDSPStatistics ${ROOT})

### Includes
target_include_directories(CMSISDSPStatistics PUBLIC "${DSP}/Include")



