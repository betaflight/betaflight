cmake_minimum_required (VERSION 3.6)

project(CMSISDSPFastMath)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPFastMath STATIC)
configLib(CMSISDSPFastMath ${ROOT})
configDsp(CMSISDSPFastMath ${ROOT})

include(interpol)
interpol(CMSISDSPFastMath)

if (CONFIGTABLE AND ALLFAST)
    target_compile_definitions(CMSISDSPFastMath PUBLIC ARM_ALL_FAST_TABLES)  
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_COS_F32)
target_sources(CMSISDSPFastMath PRIVATE arm_cos_f32.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_COS_Q15)
target_sources(CMSISDSPFastMath PRIVATE arm_cos_q15.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_COS_Q31)
target_sources(CMSISDSPFastMath PRIVATE arm_cos_q31.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_SIN_F32)
target_sources(CMSISDSPFastMath PRIVATE arm_sin_f32.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_SIN_Q15)
target_sources(CMSISDSPFastMath PRIVATE arm_sin_q15.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_SIN_Q31)
target_sources(CMSISDSPFastMath PRIVATE arm_sin_q31.c)
endif()

target_sources(CMSISDSPFastMath PRIVATE arm_sqrt_q15.c)
target_sources(CMSISDSPFastMath PRIVATE arm_sqrt_q31.c)
target_sources(CMSISDSPFastMath PRIVATE arm_vlog_f32.c)
target_sources(CMSISDSPFastMath PRIVATE arm_vexp_f32.c)


### Includes
target_include_directories(CMSISDSPFastMath PUBLIC "${DSP}/Include")



