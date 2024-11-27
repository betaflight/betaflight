#
# H7 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),H7)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver
STDPERIPH_SRC   = \
            stm32h7xx_hal_adc.c \
            stm32h7xx_hal_adc_ex.c \
            stm32h7xx_hal.c \
            stm32h7xx_hal_cordic.c \
            stm32h7xx_hal_cortex.c \
            stm32h7xx_hal_dac.c \
            stm32h7xx_hal_dac_ex.c \
            stm32h7xx_hal_dfsdm_ex.c \
            stm32h7xx_hal_dma.c \
            stm32h7xx_hal_dma_ex.c \
            stm32h7xx_hal_dts.c \
            stm32h7xx_hal_exti.c \
            stm32h7xx_hal_flash.c \
            stm32h7xx_hal_flash_ex.c \
            stm32h7xx_hal_fmac.c \
            stm32h7xx_hal_gfxmmu.c \
            stm32h7xx_hal_gpio.c \
            stm32h7xx_hal_i2c.c \
            stm32h7xx_hal_i2c_ex.c \
            stm32h7xx_hal_ospi.c \
            stm32h7xx_hal_otfdec.c \
            stm32h7xx_hal_pcd.c \
            stm32h7xx_hal_pcd_ex.c \
            stm32h7xx_hal_pssi.c \
            stm32h7xx_hal_pwr.c \
            stm32h7xx_hal_pwr_ex.c \
            stm32h7xx_hal_qspi.c \
            stm32h7xx_hal_rcc.c \
            stm32h7xx_hal_rcc_ex.c \
            stm32h7xx_hal_rng_ex.c \
            stm32h7xx_hal_rtc_ex.c \
            stm32h7xx_hal_sd.c \
            stm32h7xx_hal_spi_ex.c \
            stm32h7xx_hal_tim.c \
            stm32h7xx_hal_tim_ex.c \
            stm32h7xx_hal_uart.c \
            stm32h7xx_hal_uart_ex.c \
            stm32h7xx_ll_cordic.c \
            stm32h7xx_ll_crs.c \
            stm32h7xx_ll_dma.c \
            stm32h7xx_ll_fmac.c \
            stm32h7xx_ll_sdmmc.c \
            stm32h7xx_ll_spi.c \
            stm32h7xx_ll_tim.c \
            stm32h7xx_ll_usb.c

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = \
            usbd_core.c \
            usbd_ctlreq.c \
            usbd_ioreq.c

USBCDC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = usbd_cdc.c

USBHID_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = usbd_hid.c

USBMSC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = \
            usbd_msc_bot.c \
            usbd_msc.c \
            usbd_msc_data.c \
            usbd_msc_scsi.c

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBHID_DIR)/Src:$(USBMSC_DIR)/Src:$(STDPERIPH_DIR)/src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32H7xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBHID_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32H7/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
                   $(TARGET_PLATFORM_DIR)/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

#
# H743xI : 2M FLASH, 512KB AXI SRAM + 512KB D2 & D3 SRAM (H753xI also)
# H743xG : 1M FLASH, 512KB AXI SRAM + 512KB D2 & D3 SRAM (H753xG also)
# H7A3xI : 2M FLASH, 1MB   AXI SRAM + 160KB AHB & SRD SRAM
# H750xB : 128K FLASH, 1M RAM
#
ifeq ($(TARGET_MCU),STM32H743xx)
DEVICE_FLAGS       += -DSTM32H743xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h743_2m.ld
STARTUP_SRC         = startup/startup_stm32h743xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h743.ld
endif

else ifeq ($(TARGET_MCU),STM32H7A3xxQ)
DEVICE_FLAGS       += -DSTM32H7A3xxQ
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_2m.ld
STARTUP_SRC         = startup/startup_stm32h7a3xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_ram_based.ld
endif

else ifeq ($(TARGET_MCU),STM32H7A3xx)
DEVICE_FLAGS       += -DSTM32H7A3xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_2m.ld
STARTUP_SRC         = startup/startup_stm32h7a3xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_ram_based.ld
endif

else ifeq ($(TARGET_MCU),STM32H723xx)
DEVICE_FLAGS       += -DSTM32H723xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h723_1m.ld
STARTUP_SRC         = startup/startup_stm32h723xx.s
DEFAULT_TARGET_FLASH := 1024
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(TARGET_FLASH),)
MCU_FLASH_SIZE := $(DEFAULT_TARGET_FLASH)
endif

ifeq ($(EXST),yes)
FIRMWARE_SIZE      := 1024
# TARGET_FLASH now becomes the amount of MEMORY-MAPPED address space that is occupied by the firmware
# and the maximum size of the data stored on the external flash device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h723_exst.ld
LD_SCRIPTS          = $(LINKER_DIR)/stm32_h723_common.ld $(LINKER_DIR)/stm32_h723_common_post.ld
endif

else ifeq ($(TARGET_MCU),STM32H725xx)
DEVICE_FLAGS       += -DSTM32H725xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h723_1m.ld
STARTUP_SRC         = startup/startup_stm32h723xx.s
MCU_FLASH_SIZE     := 1024
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

else ifeq ($(TARGET_MCU),STM32H730xx)
DEVICE_FLAGS       += -DSTM32H730xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h730_128m.ld
STARTUP_SRC         = startup/startup_stm32h730xx.s
DEFAULT_TARGET_FLASH := 128
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16


ifeq ($(TARGET_FLASH),)
MCU_FLASH_SIZE := $(DEFAULT_TARGET_FLASH)
endif

ifeq ($(EXST),yes)
FIRMWARE_SIZE      := 1024
# TARGET_FLASH now becomes the amount of MEMORY-MAPPED address space that is occupied by the firmware
# and the maximum size of the data stored on the external flash device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h730_exst.ld
LD_SCRIPTS          = $(LINKER_DIR)/stm32_h730_common.ld $(LINKER_DIR)/stm32_h730_common_post.ld
endif


else ifeq ($(TARGET_MCU),STM32H750xx)
DEVICE_FLAGS       += -DSTM32H750xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h750_128k.ld
STARTUP_SRC         = startup/startup_stm32h743xx.s
DEFAULT_TARGET_FLASH := 128

ifeq ($(TARGET_FLASH),)
MCU_FLASH_SIZE := $(DEFAULT_TARGET_FLASH)
endif

ifeq ($(EXST),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h750_exst.ld
endif

ifeq ($(EXST),yes)
# Upper 8 regions are reserved for a boot loader in EXST environment
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=8
else
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      := -Os
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

else
$(error Unknown MCU for H7 target)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

ifneq ($(FIRMWARE_SIZE),)
DEVICE_FLAGS   += -DFIRMWARE_SIZE=$(FIRMWARE_SIZE)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHSE_STARTUP_TIMEOUT=1000 -DSTM32

VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf_stm32h7xx.c \
            vcp_hal/usbd_cdc_hid.c \
            vcp_hal/usbd_cdc_interface.c \
            serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            stm32/system.c \
            drivers/bus_i2c_timing.c \
            drivers/bus_quadspi.c \
            drivers/dshot_bitbang_decode.c \
            drivers/pwm_output_dshot_shared.c \
            adc_stm32h7xx.c \
            audio_stm32h7xx.c \
            bus_i2c_hal_init.c \
            bus_i2c_hal.c \
            bus_spi_ll.c \
            bus_quadspi_hal.c \
            bus_octospi_stm32h7xx.c \
            debug.c \
            dma_reqmap_mcu.c \
            dma_stm32h7xx.c \
            dshot_bitbang_ll.c \
            dshot_bitbang.c \
            exti.c \
            io_stm32.c \
            light_ws2811strip_hal.c \
            memprot_hal.c \
            memprot_stm32h7xx.c \
            persistent.c \
            pwm_output.c \
            pwm_output_dshot_hal.c \
            rcc_stm32.c \
            sdio_h7xx.c \
            serial_uart_hal.c \
            serial_uart_stm32h7xx.c \
            system_stm32h7xx.c \
            timer_hal.c \
            timer_stm32h7xx.c \
            transponder_ir_io_hal.c \
            camera_control_stm32.c \
            drivers/adc.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_pinconfig.c \
            startup/system_stm32h7xx.c

MSC_SRC = \
            usb_msc_hal.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
