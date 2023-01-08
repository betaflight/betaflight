cmake_minimum_required (VERSION 3.6)

project(CMSISDSPBayes)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPBayes STATIC ${SRC})

configLib(CMSISDSPBayes ${ROOT})
configDsp(CMSISDSPBayes ${ROOT})

### Includes
target_include_directories(CMSISDSPBayes PUBLIC "${DSP}/Include")



