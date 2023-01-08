cmake_minimum_required (VERSION 3.6)

project(CMSISDSPCommon)

include(configLib)
include(configDsp)

add_library(CMSISDSPCommon STATIC arm_common_tables.c)

configLib(CMSISDSPCommon ${ROOT})
configDsp(CMSISDSPCommon ${ROOT})

if (CONFIGTABLE AND ALLFFT)
    target_compile_definitions(CMSISDSPCommon PUBLIC ARM_ALL_FFT_TABLES) 
endif()

if (CONFIGTABLE AND ALLFAST)
    target_compile_definitions(CMSISDSPCommon PUBLIC ARM_ALL_FAST_TABLES) 
endif()

include(fft)
fft(CMSISDSPCommon)

include(interpol)
interpol(CMSISDSPCommon)

target_sources(CMSISDSPCommon PRIVATE arm_const_structs.c)


### Includes
target_include_directories(CMSISDSPCommon PUBLIC "${DSP}/Include")

if (NEON OR NEONEXPERIMENTAL)
    target_sources(CMSISDSPCommon PRIVATE "${DSP}/ComputeLibrary/Source/arm_cl_tables.c")
endif()

if (HELIUM OR MVEF)
    target_sources(CMSISDSPCommon PRIVATE "${DSP}/Source/CommonTables/arm_mve_tables.c")
endif()


