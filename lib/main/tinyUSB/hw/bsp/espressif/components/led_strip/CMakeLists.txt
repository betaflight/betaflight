include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(srcs "src/led_strip_api.c")

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    if(CONFIG_SOC_RMT_SUPPORTED)
        list(APPEND srcs "src/led_strip_rmt_dev.c" "src/led_strip_rmt_encoder.c")
    endif()
else()
    list(APPEND srcs "src/led_strip_rmt_dev_idf4.c")
endif()

# the SPI backend driver relies on something that was added in IDF 5.1
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.1")
    if(CONFIG_SOC_GPSPI_SUPPORTED)
        list(APPEND srcs "src/led_strip_spi_dev.c")
    endif()
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include" "interface"
                       REQUIRES "driver")
