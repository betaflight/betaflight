cmake_minimum_required (VERSION 3.6)

project(CMSISDSPSupport)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPSupport STATIC ${SRC})

configLib(CMSISDSPSupport ${ROOT})
configDsp(CMSISDSPSupport ${ROOT})

### Includes
target_include_directories(CMSISDSPSupport PUBLIC "${DSP}/Include")



