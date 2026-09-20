
PLATFORM_SDK := arm

#CMSIS
CMSIS_DIR      := $(LIB_MAIN_DIR)/UM324xx/Drivers/CMSIS
#STDPERIPH
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/UM324xx/Drivers/UM324xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = \
                um324xx_hal_acmp.c \
                um324xx_hal_aes.c \
                um324xx_hal_aes_ex.c \
                um324xx_hal_can.c \
                um324xx_hal_canfd.c \
                um324xx_hal_cordic.c \
                um324xx_hal_crc.c \
                um324xx_hal_ctm.c \
                um324xx_hal_dac.c \
                um324xx_hal_emac.c \
                um324xx_hal_i2c.c \
                um324xx_hal_i2s.c \
                um324xx_hal_lptim.c \
                um324xx_hal_lpuart.c \
                um324xx_hal_iwdt.c \
                um324xx_hal_pcd.c \
                um324xx_hal_pcd_ex.c \
                um324xx_hal_rng.c \
                um324xx_hal_spi.c \
                um324xx_hal_uart.c \
                um324xx_hal_vref.c \
                um324xx_hal_wwdt.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
VPATH       := $(VPATH):$(STDPERIPH_DIR)/Src


#USB
USBCHERY_DIR := $(ROOT)/lib/main/UM324xx/Middlewares/UM/CherryUSB-1.6.0

USBCORE_DIR = $(USBCHERY_DIR)/core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/*.c))
EXCLUDES    =
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(USBCHERY_DIR)/class/cdc
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/*.c))
EXCLUDES   =
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR =
USBHID_SRC = 

USBMSC_DIR = $(USBCHERY_DIR)/class/msc
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/*.c))
EXCLUDES   = usbh_msc.c \
            msc_ram_template.c
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

USBPORT_DIR = $(USBCHERY_DIR)/port/um
USBPORT_SRC = $(notdir $(wildcard $(USBPORT_DIR)/*.c))
EXCLUDES   =
USBPORT_SRC := $(filter-out ${EXCLUDES}, $(USBPORT_SRC))

VPATH := $(VPATH):$(USBCDC_DIR):$(USBCORE_DIR):$(USBHID_DIR):$(USBMSC_DIR):$(USBPORT_DIR)

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBMSC_SRC) \
                        $(USBPORT_SRC)

#CMSIS
VPATH := $(VPATH):$(LIB_MAIN_DIR)/UM324xx/Drivers/CMSIS/Device/UM/UM324xx

CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCHERY_DIR)/common \
                   $(USBCORE_DIR) \
                   $(USBCDC_DIR)  \
                   $(USBHID_DIR)  \
                   $(USBMSC_DIR)  \
                   $(USBPORT_DIR) \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/UM/UM324xx/Include \
                   $(LIB_MAIN_DIR)/CMSIS/Core/Include \
                   $(TARGET_PLATFORM_DIR)/vcp_hal \
                   $(PLATFORM_DIR)/common/stm32

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16

DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

ifneq (,$(filter $(TARGET_MCU),UM3241F UM3247F))
DEVICE_FLAGS    += -DUM324xF
STARTUP_SRC     = UM32/startup/startup_um324xf.s
# QSPI XIP code expansion: yes = 512K internal + 512K QSPI (1024K code
# space, files listed in EX_FLASH_SRC below run from the QSPI XIP window);
# no = internal FLASH1 only, EX_FLASH_SRC is emptied and the listed files
# fall back to .text automatically. Drives -DUSE_QSPI_XIP (C code and the
# linker-script preprocessor) and MCU_FLASH_SIZE (feature gating).
QSPI_XIP ?= yes
ifeq ($(QSPI_XIP),yes)
DEVICE_FLAGS    += -DUSE_QSPI_XIP
MCU_FLASH_SIZE  = 1024
else
MCU_FLASH_SIZE  = 512
endif
# Linker script is preprocessed from the .ld.in source so it can follow
# USE_QSPI_XIP (EXFLASH region, .ex_flash section, .tcm_code LMA). The mode
# is encoded in the generated filename — make cannot see that the output
# depends on the QSPI_XIP variable, so a shared name would silently keep a
# stale script when switching modes between builds.
LD_SCRIPT       = $(OBJECT_DIR)/um32_flash_4xf_$(if $(filter yes,$(QSPI_XIP)),qspi,int).ld
$(LD_SCRIPT): $(LINKER_DIR)/um32_flash_4xf.ld.in
	@mkdir -p $(OBJECT_DIR)
	$(V1) $(CROSS_CC) -E -P -xc $(if $(filter yes,$(QSPI_XIP)),-DUSE_QSPI_XIP) -o $@ $<
else
$(error Unknown MCU for target)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

VCP_SRC = \
            UM32/vcp_hal/cdc_acm.c \
            UM32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            UM32/rcc_um32.c \
            UM32/io_um32.c \
            UM32/exti.c \
            UM32/debug.c \
            UM32/persistent.c \
            UM32/adc_um324xx.c \
            UM32/bus_i2c_hal.c \
            UM32/bus_i2c_um324xx.c \
            UM32/bus_spi_hal.c \
            UM32/bus_quadspi_hal.c \
            UM32/dma_reqmap_mcu.c \
            UM32/dma_um324xx.c \
            UM32/serial_uart_hal.c \
            UM32/serial_uart_um324xx.c \
            UM32/timer_hal.c \
            UM32/timer_um324xx.c \
            UM32/dshot_bitbang.c \
            UM32/dshot_bitbang_hal.c \
            UM32/pwm_output_dshot_hal.c \
            UM32/pwm_output_hw.c \
            UM32/light_ws2811strip_hal.c \
            UM32/transponder_ir_io_hal.c \
            UM32/sdio_um324xx.c \
            UM32/camera_control_um32.c \
            UM32/exflash_um324xx_hal.c \
            UM32/sysfunc_um324xx.c \
            UM32/startup/system_um324xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/bus_i2c_timing.c \
            drivers/usb_msc_common.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            common/stm32/system.c \
            common/stm32/io_impl.c \
            common/stm32/config_flash.c \
            common/stm32/mco.c \
            common/stm32/rx_pwm_hw.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/camera_control.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/ledstrip_ws2811_stm32.c \
            common/stm32/debug_pin.c \
            common/stm32/adc_impl.c \
            common/stm32/expresslrs_driver_hw.c \
            common/stm32/fault_handlers.c

MSC_SRC = \
            UM32/usb_msc_hal.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sdio.c \
            msc/usbd_storage_sd_spi.c

SIZE_OPTIMISED_SRC += \
            UM32/serial_usb_vcp.c \
            drivers/inverter.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/serial_uart_pinconfig.c \
            io/gps.c \
            io/ledstrip.c \
            msp/msp.c \
            cms/cms_menu_imu.c \
            telemetry/crsf.c \
            telemetry/mavlink.c \
            blackbox/blackbox.c \
            config/config.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4

# QSPI XIP (EXFLASH) file-level placement: code+rodata of these files go to
# the .ex_flash section in the preprocessed linker script (matched by object
# path suffix — keep the two lists in sync). Compiled -Os -fno-lto via the
# NOT_OPTIMISED_SRC path so the flags win over the default -flto profile.
# Entries MUST be full paths with "./" prefix (raw $< match, no
# normalisation). Never list boot-path files or files containing IRQ
# handlers. Empty when QSPI_XIP=no — files fall back to .text (FLASH1).
ifeq ($(QSPI_XIP),yes)
EX_FLASH_SRC := ./src/platform/UM32/debug.c \
                ./src/platform/UM32/sysfunc_um324xx.c
endif

NOT_OPTIMISED_SRC += $(EX_FLASH_SRC)
$(foreach f,$(EX_FLASH_SRC),$(eval SRC_CFLAGS_$(notdir $(f)) += -Os -fno-lto))
