cmake_minimum_required(VERSION 3.13)
include_guard(GLOBAL)

if (NOT BOARD)
	message("BOARD not specified, defaulting to pico_sdk")
	set(BOARD pico_sdk)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

#if (TOOLCHAIN STREQUAL "clang")
#	set(PICO_COMPILER "pico_arm_clang")
#else()
#	set(PICO_COMPILER "pico_arm_gcc")
#endif()

# add the SDK in case we are standalone tinyusb example (noop if already present)
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

# include basic family CMake functionality
set(FAMILY_MCUS RP2040)

if (PICO_PLATFORM STREQUAL "rp2040")
	set(JLINK_DEVICE rp2040_m0_0)
	set(OPENOCD_TARGET rp2040)
elseif (PICO_PLATFORM STREQUAL "rp2350-arm-s" OR PICO_PLATFORM STREQUAL "rp2350")
	set(JLINK_DEVICE rp2350_m33_0)
	set(OPENOCD_TARGET rp2350)
elseif (PICO_PLATFORM STREQUAL "rp2350-riscv")
	set(JLINK_DEVICE rp2350_riscv_0)
	set(OPENOCD_TARGET rp2350-riscv)
endif()

set(OPENOCD_OPTION "-f interface/cmsis-dap.cfg -f target/${OPENOCD_TARGET}.cfg -c \"adapter speed 5000\"")

if (NOT PICO_TINYUSB_PATH)
	set(PICO_TINYUSB_PATH ${TOP})
endif()

if (NOT TINYUSB_OPT_OS)
	set(TINYUSB_OPT_OS OPT_OS_PICO)
endif()

#------------------------------------
# Base config for both device and host; wrapped by SDK's tinyusb_common
#------------------------------------
add_library(tinyusb_common_base INTERFACE)

target_sources(tinyusb_common_base INTERFACE
	${TOP}/src/tusb.c
	${TOP}/src/common/tusb_fifo.c
	)

target_include_directories(tinyusb_common_base INTERFACE
	${TOP}/src
	)

if(DEFINED LOG)
	set(TINYUSB_DEBUG_LEVEL ${LOG})
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
	message("Compiling TinyUSB with CFG_TUSB_DEBUG=1")
	set(TINYUSB_DEBUG_LEVEL 1)
else ()
	set(TINYUSB_DEBUG_LEVEL 0)
endif()

target_compile_definitions(tinyusb_common_base INTERFACE
	CFG_TUSB_MCU=OPT_MCU_RP2040
	CFG_TUSB_OS=${TINYUSB_OPT_OS}
	CFG_TUSB_DEBUG=${TINYUSB_DEBUG_LEVEL}
)

target_link_libraries(tinyusb_common_base INTERFACE
	hardware_structs
	hardware_irq
	hardware_resets
	pico_sync
	)

#------------------------------------
# Base config for device mode; wrapped by SDK's tinyusb_device
#------------------------------------
add_library(tinyusb_device_base INTERFACE)
target_sources(tinyusb_device_base INTERFACE
		${TOP}/src/portable/raspberrypi/rp2040/dcd_rp2040.c
		${TOP}/src/portable/raspberrypi/rp2040/rp2040_usb.c
		${TOP}/src/device/usbd.c
		${TOP}/src/device/usbd_control.c
		${TOP}/src/class/audio/audio_device.c
		${TOP}/src/class/cdc/cdc_device.c
		${TOP}/src/class/dfu/dfu_device.c
		${TOP}/src/class/dfu/dfu_rt_device.c
		${TOP}/src/class/hid/hid_device.c
		${TOP}/src/class/midi/midi_device.c
		${TOP}/src/class/msc/msc_device.c
		${TOP}/src/class/net/ecm_rndis_device.c
		${TOP}/src/class/net/ncm_device.c
		${TOP}/src/class/usbtmc/usbtmc_device.c
		${TOP}/src/class/vendor/vendor_device.c
		${TOP}/src/class/video/video_device.c
		)

#------------------------------------
# Base config for host mode; wrapped by SDK's tinyusb_host
#------------------------------------
add_library(tinyusb_host_base INTERFACE)
target_sources(tinyusb_host_base INTERFACE
		${TOP}/src/portable/raspberrypi/rp2040/hcd_rp2040.c
		${TOP}/src/portable/raspberrypi/rp2040/rp2040_usb.c
		${TOP}/src/host/usbh.c
		${TOP}/src/host/hub.c
		${TOP}/src/class/cdc/cdc_host.c
		${TOP}/src/class/hid/hid_host.c
		${TOP}/src/class/msc/msc_host.c
		${TOP}/src/class/vendor/vendor_host.c
		)

# Sometimes have to do host specific actions in mostly common functions
target_compile_definitions(tinyusb_host_base INTERFACE
		RP2040_USB_HOST_MODE=1
)

#------------------------------------
# Host MAX3421
#------------------------------------
add_library(tinyusb_host_max3421 INTERFACE)
target_sources(tinyusb_host_max3421 INTERFACE
	${TOP}/src/portable/analog/max3421/hcd_max3421.c
	)
target_compile_definitions(tinyusb_host_max3421 INTERFACE
	CFG_TUH_MAX3421=1
	)
target_link_libraries(tinyusb_host_max3421 INTERFACE
	hardware_spi
	)

#------------------------------------
# BSP & Additions
#------------------------------------
add_library(tinyusb_bsp INTERFACE)
target_sources(tinyusb_bsp INTERFACE
	${TOP}/hw/bsp/rp2040/family.c
	)
target_include_directories(tinyusb_bsp INTERFACE
	${TOP}/hw
	)
target_link_libraries(tinyusb_bsp INTERFACE
	pico_unique_id
	hardware_clocks
	)

# tinyusb_additions will hold our extra settings for examples
add_library(tinyusb_additions INTERFACE)

target_compile_definitions(tinyusb_additions INTERFACE
	PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
	PICO_RP2040_USB_DEVICE_UFRAME_FIX=1
)

if(LOGGER STREQUAL "RTT" OR LOGGER STREQUAL "rtt")
	target_compile_definitions(tinyusb_additions INTERFACE
			LOGGER_RTT
			#SEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
	)

	target_sources(tinyusb_additions INTERFACE
			${TOP}/lib/SEGGER_RTT/RTT/SEGGER_RTT.c
	)

	set_source_files_properties(${TOP}/lib/SEGGER_RTT/RTT/SEGGER_RTT.c
		PROPERTIES
		COMPILE_FLAGS "-Wno-cast-qual -Wno-cast-align -Wno-sign-conversion")

	target_include_directories(tinyusb_additions INTERFACE
			${TOP}/lib/SEGGER_RTT/RTT
	)
endif()

#------------------------------------
# Functions
#------------------------------------

function(family_configure_target TARGET RTOS)
	if (RTOS STREQUAL noos OR RTOS STREQUAL "")
		set(RTOS_SUFFIX "")
	else()
		set(RTOS_SUFFIX _${RTOS})
	endif()
	# export RTOS_SUFFIX to parent scope
	set(RTOS_SUFFIX ${RTOS_SUFFIX} PARENT_SCOPE)

	# compile define from command line
	if(DEFINED CFLAGS_CLI)
		separate_arguments(CFLAGS_CLI)
		target_compile_options(${TARGET} PUBLIC ${CFLAGS_CLI})
	endif()

	pico_add_extra_outputs(${TARGET})
	pico_enable_stdio_uart(${TARGET} 1)
	target_link_libraries(${TARGET} PUBLIC pico_stdlib tinyusb_board${RTOS_SUFFIX} tinyusb_additions)

	family_flash_openocd(${TARGET})
	family_flash_jlink(${TARGET})
endfunction()


function(rp2040_family_configure_example_warnings TARGET)
	if (NOT PICO_TINYUSB_NO_EXAMPLE_WARNINGS)
		family_add_default_example_warnings(${TARGET})
	endif()
	if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
		target_compile_options(${TARGET} PRIVATE -Wno-unreachable-code)
	endif()
	suppress_tinyusb_warnings()
endfunction()


function(family_configure_device_example TARGET RTOS)
	family_configure_target(${TARGET} ${RTOS})
	target_link_libraries(${TARGET} PUBLIC pico_stdlib tinyusb_device${RTOS_SUFFIX})
	rp2040_family_configure_example_warnings(${TARGET})
endfunction()


function(family_add_pico_pio_usb TARGET)
	target_link_libraries(${TARGET} PUBLIC tinyusb_pico_pio_usb)
endfunction()


# since Pico-PIO_USB compiler support may lag, and change from version to version, add a function that pico-sdk/pico-examples
# can check (if present) in case the user has updated their TinyUSB
function(is_compiler_supported_by_pico_pio_usb OUTVAR)
	if ((NOT CMAKE_C_COMPILER_ID STREQUAL "GNU"))
		SET(${OUTVAR} 0 PARENT_SCOPE)
	else()
		set(${OUTVAR} 1 PARENT_SCOPE)
	endif()
endfunction()

function(family_configure_host_example TARGET RTOS)
	family_configure_target(${TARGET} ${RTOS})
	target_link_libraries(${TARGET} PUBLIC pico_stdlib tinyusb_host${RTOS_SUFFIX})
	rp2040_family_configure_example_warnings(${TARGET})

	# For rp2040 enable pico-pio-usb
	if (TARGET tinyusb_pico_pio_usb)
		# Pico-PIO-USB does not compile with all pico-sdk supported compilers, so check before enabling it
		is_compiler_supported_by_pico_pio_usb(PICO_PIO_USB_COMPILER_SUPPORTED)
		if (PICO_PIO_USB_COMPILER_SUPPORTED)
			family_add_pico_pio_usb(${PROJECT})
		endif()
	endif()

	# for max3421 host
	if (MAX3421_HOST STREQUAL "1")
		target_link_libraries(${TARGET} PUBLIC tinyusb_host_max3421)
	endif()
endfunction()


function(family_configure_dual_usb_example TARGET RTOS)
	family_configure_target(${TARGET} ${RTOS})
	# require tinyusb_pico_pio_usb
	target_link_libraries(${TARGET} PUBLIC pico_stdlib tinyusb_device tinyusb_host tinyusb_pico_pio_usb )
	rp2040_family_configure_example_warnings(${TARGET})
endfunction()


function(check_and_add_pico_pio_usb_support)
	# check for pico_generate_pio_header (as depending on environment we may be called before SDK is
	# initialized in which case it isn't available yet), and only do the initialization once
	if (COMMAND pico_generate_pio_header AND NOT TARGET tinyusb_pico_pio_usb)
		#------------------------------------
		# PIO USB for both host and device
		#------------------------------------

		if (NOT DEFINED PICO_PIO_USB_PATH)
			set(PICO_PIO_USB_PATH "${TOP}/hw/mcu/raspberry_pi/Pico-PIO-USB")
		endif()

		if (EXISTS ${PICO_PIO_USB_PATH}/src/pio_usb.c)
			add_library(tinyusb_pico_pio_usb INTERFACE)
			target_sources(tinyusb_device_base INTERFACE
					${TOP}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
					)
			target_sources(tinyusb_host_base INTERFACE
					${TOP}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
					)

			target_sources(tinyusb_pico_pio_usb INTERFACE
					${PICO_PIO_USB_PATH}/src/pio_usb.c
					${PICO_PIO_USB_PATH}/src/pio_usb_host.c
					${PICO_PIO_USB_PATH}/src/pio_usb_device.c
					${PICO_PIO_USB_PATH}/src/usb_crc.c
					)

			target_include_directories(tinyusb_pico_pio_usb INTERFACE
					${PICO_PIO_USB_PATH}/src
					)

			target_link_libraries(tinyusb_pico_pio_usb INTERFACE
					hardware_dma
					hardware_pio
					pico_multicore
					)

			target_compile_definitions(tinyusb_pico_pio_usb INTERFACE
					PIO_USB_USE_TINYUSB
					)

			pico_generate_pio_header(tinyusb_pico_pio_usb ${PICO_PIO_USB_PATH}/src/usb_tx.pio)
			pico_generate_pio_header(tinyusb_pico_pio_usb ${PICO_PIO_USB_PATH}/src/usb_rx.pio)
		endif()
	endif()
endfunction()

# Try to add Pico-PIO_USB support now for the case where this file is included directly
# after Pico SDK initialization, but without using the family_ functions (as is the case
# when included by the SDK itself)
check_and_add_pico_pio_usb_support()


function(family_initialize_project PROJECT DIR)
	# call the original version of this function from family_common.cmake
	_family_initialize_project(${PROJECT} ${DIR})
	enable_language(C CXX ASM)
	pico_sdk_init()

	# now re-check for adding Pico-PIO_USB support now SDK is definitely available
	check_and_add_pico_pio_usb_support()
endfunction()


# This method must be called from the project scope to suppress known warnings in TinyUSB source files
function(suppress_tinyusb_warnings)
	# some of these are pretty silly warnings only occurring in some older GCC versions 9 or prior
	if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
		if (CMAKE_C_COMPILER_VERSION VERSION_LESS 10.0)
			set(CONVERSION_WARNING_FILES
				${PICO_TINYUSB_PATH}/src/tusb.c
				${PICO_TINYUSB_PATH}/src/common/tusb_fifo.c
				${PICO_TINYUSB_PATH}/src/device/usbd.c
				${PICO_TINYUSB_PATH}/src/device/usbd_control.c
				${PICO_TINYUSB_PATH}/src/host/usbh.c
				${PICO_TINYUSB_PATH}/src/class/cdc/cdc_device.c
				${PICO_TINYUSB_PATH}/src/class/cdc/cdc_host.c
				${PICO_TINYUSB_PATH}/src/class/hid/hid_device.c
				${PICO_TINYUSB_PATH}/src/class/hid/hid_host.c
				${PICO_TINYUSB_PATH}/src/class/audio/audio_device.c
				${PICO_TINYUSB_PATH}/src/class/dfu/dfu_device.c
				${PICO_TINYUSB_PATH}/src/class/dfu/dfu_rt_device.c
				${PICO_TINYUSB_PATH}/src/class/midi/midi_device.c
				${PICO_TINYUSB_PATH}/src/class/usbtmc/usbtmc_device.c
				${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040/hcd_rp2040.c
				)
			foreach(SOURCE_FILE IN LISTS CONVERSION_WARNING_FILES)
				set_source_files_properties(
						${SOURCE_FILE}
						PROPERTIES
						COMPILE_FLAGS "-Wno-conversion")
			endforeach()
		endif()
		if (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 11.0)
			set_source_files_properties(
					${PICO_TINYUSB_PATH}/lib/fatfs/source/ff.c
					COMPILE_FLAGS "-Wno-stringop-overflow -Wno-array-bounds")
		endif()
		set_source_files_properties(
				${PICO_TINYUSB_PATH}/lib/fatfs/source/ff.c
				PROPERTIES
				COMPILE_FLAGS "-Wno-conversion -Wno-cast-qual")

		set_source_files_properties(
				${PICO_TINYUSB_PATH}/lib/lwip/src/core/tcp_in.c
				${PICO_TINYUSB_PATH}/lib/lwip/src/core/tcp_out.c
				PROPERTIES
				COMPILE_FLAGS "-Wno-conversion")

		set_source_files_properties(
				${PICO_TINYUSB_PATH}/lib/networking/dnserver.c
				${PICO_TINYUSB_PATH}/lib/networking/dhserver.c
				${PICO_TINYUSB_PATH}/lib/networking/rndis_reports.c
				PROPERTIES
				COMPILE_FLAGS "-Wno-conversion -Wno-sign-conversion")

		if (TARGET tinyusb_pico_pio_usb)
			set_source_files_properties(
					${PICO_TINYUSB_PATH}/hw/mcu/raspberry_pi/Pico-PIO-USB/src/pio_usb_device.c
					${PICO_TINYUSB_PATH}/hw/mcu/raspberry_pi/Pico-PIO-USB/src/pio_usb.c
					${PICO_TINYUSB_PATH}/hw/mcu/raspberry_pi/Pico-PIO-USB/src/pio_usb_host.c
					${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
					PROPERTIES
					COMPILE_FLAGS "-Wno-conversion -Wno-cast-qual -Wno-attributes")
		endif()
	elseif(CMAKE_C_COMPILER_ID STREQUAL "Clang")
		set_source_files_properties(
				${PICO_TINYUSB_PATH}/src/class/cdc/cdc_device.c
				COMPILE_FLAGS "-Wno-unreachable-code")
		set_source_files_properties(
				${PICO_TINYUSB_PATH}/src/class/cdc/cdc_host.c
				COMPILE_FLAGS "-Wno-unreachable-code-fallthrough")
		set_source_files_properties(
				${PICO_TINYUSB_PATH}/lib/fatfs/source/ff.c
				PROPERTIES
				COMPILE_FLAGS "-Wno-cast-qual")
	endif()
endfunction()
