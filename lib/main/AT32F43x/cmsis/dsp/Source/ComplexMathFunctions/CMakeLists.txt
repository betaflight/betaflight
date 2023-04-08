cmake_minimum_required (VERSION 3.6)

project(CMSISDSPComplexMath)

include(configLib)
include(configDsp)

file(GLOB SRC "./*_*.c")

add_library(CMSISDSPComplexMath STATIC)

configLib(CMSISDSPComplexMath ${ROOT})
configDsp(CMSISDSPComplexMath ${ROOT})


include(interpol)
interpol(CMSISDSPFastMath)

if (CONFIGTABLE AND ALLFAST)
    target_compile_definitions(CMSISDSPComplexMath PUBLIC ARM_ALL_FAST_TABLES)  
endif()

# MVE code is using a table for computing the fast sqrt arm_cmplx_mag_q31
# There is the possibility of not compiling this function and not including
# the table.
if (NOT CONFIGTABLE OR ALLFAST OR ARM_CMPLX_MAG_Q31 OR (NOT HELIUM AND NOT MVEI))
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_q31.c)
endif()

if (NOT CONFIGTABLE OR ALLFAST OR ARM_CMPLX_MAG_Q15 OR (NOT HELIUM AND NOT MVEI))
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_q15.c)
endif()

target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_conj_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_conj_q15.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_conj_q31.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_dot_prod_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_dot_prod_q15.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_dot_prod_q31.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_squared_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_squared_q15.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mag_squared_q31.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_cmplx_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_cmplx_q15.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_cmplx_q31.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_real_f32.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_real_q15.c)
target_sources(CMSISDSPComplexMath PRIVATE arm_cmplx_mult_real_q31.c)


### Includes
target_include_directories(CMSISDSPComplexMath PUBLIC "${DSP}/Include")
