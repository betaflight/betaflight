#
# NXP i.MX95 Make file include, Cortex-M7 core
#

PLATFORM_SDK := mcux_imx
NXP_FAMILY := imx95

ifeq ($(TARGET_MCU),MIMX9596)
MCUX_CPU        ?= MIMX9596AVZXN_cm7
MCUX_RT_DIR     := $(LIB_MODULES_DIR)/mcux-devices-imx
MCUX_RT_SERIES  := i.MX95
DEVICE_FLAGS    += -DMIMX9596 -DIMX95 -DCPU_$(MCUX_CPU)
# mcuboot slot 0 on the NOR, 7 MB from 0x28020000.
MCU_FLASH_SIZE  ?= 7168
LD_SCRIPT       ?= $(LINKER_DIR)/imx95_flexspi_nor.ld
else
$(error Unknown MCU $(TARGET_MCU) for i.MX95 platform)
endif

# OpenAMP and libmetal for rpmsg; libmetal's cmake header templates are filled in for its generic system.
OPENAMP_DIR       := $(LIB_MODULES_DIR)/open-amp
LIBMETAL_DIR      := $(LIB_MODULES_DIR)/libmetal
METAL_INCLUDE_DIR := $(OBJECT_DIR)/libmetal/include
METAL_CONFIG_H    := $(METAL_INCLUDE_DIR)/metal/config.h

PLATFORM_SDK_STAMP := $(MCUX_IMX_STAMP) $(OPENAMP_STAMP) $(LIBMETAL_STAMP) $(METAL_CONFIG_H)

METAL_VERSION_PART = $(shell sed -n 's/^VERSION_$(1) *= *//p' $(LIBMETAL_DIR)/VERSION)

$(METAL_CONFIG_H): $(LIBMETAL_STAMP)
	$(V1) tmp=$(abspath $(METAL_INCLUDE_DIR)).$$$$ && rm -rf $$tmp && mkdir -p $$tmp && \
	(cd $(LIBMETAL_DIR)/lib && find . -name '*.h' | while read -r h; do \
	    mkdir -p $$tmp/metal/$$(dirname $$h) && \
	    sed -e 's/@PROJECT_SYSTEM@/generic/g' -e 's/@PROJECT_SYSTEM_UPPER@/GENERIC/g' \
	        -e 's/@PROJECT_PROCESSOR@/generic/g' -e 's/@PROJECT_PROCESSOR_UPPER@/GENERIC/g' \
	        -e 's/@PROJECT_MACHINE@/template/g' -e 's/@PROJECT_MACHINE_UPPER@/TEMPLATE/g' \
	        -e 's/@PROJECT_VERSION_MAJOR@/$(call METAL_VERSION_PART,MAJOR)/g' \
	        -e 's/@PROJECT_VERSION_MINOR@/$(call METAL_VERSION_PART,MINOR)/g' \
	        -e 's/@PROJECT_VERSION_PATCH@/$(call METAL_VERSION_PART,PATCH)/g' \
	        -e 's/@PROJECT_VERSION@/$(call METAL_VERSION_PART,MAJOR).$(call METAL_VERSION_PART,MINOR).$(call METAL_VERSION_PART,PATCH)/g' \
	        -e 's/^#cmakedefine HAVE_STDATOMIC_H/#define HAVE_STDATOMIC_H/' \
	        -e 's/^#cmakedefine \(.*\)/\/* #undef \1 *\//' $$h > $$tmp/metal/$$h || exit 1; \
	done) && mkdir -p $(dir $(METAL_INCLUDE_DIR)) && rm -rf $(METAL_INCLUDE_DIR) && mv $$tmp $(METAL_INCLUDE_DIR)

DEVICE_FLAGS     += -DVIRTIO_DRIVER_SUPPORT=0 -DVIRTIO_DEVICE_SUPPORT=1 -DVQ_RX_EMPTY_NOTIFY=0
SYS_INCLUDE_DIRS += $(OPENAMP_DIR)/lib/include $(METAL_INCLUDE_DIR)
%/libmetal/lib/io.o %/libmetal/lib/system/generic/io.o %/libmetal/lib/system/generic/init.o: CFLAGS += -DMETAL_INTERNAL

include $(TARGET_PLATFORM_DIR)/mk/nxp_common.mk

# No USB device controller reaches the M7.
DEVICE_STDPERIPH_SRC :=
MSC_SRC :=

OPENAMP_SRC := \
            open-amp/lib/remoteproc/remoteproc_virtio.c \
            open-amp/lib/rpmsg/rpmsg.c \
            open-amp/lib/rpmsg/rpmsg_virtio.c \
            open-amp/lib/utils/utilities.c \
            open-amp/lib/virtio/virtio.c \
            open-amp/lib/virtio/virtqueue.c \
            libmetal/lib/io.c \
            libmetal/lib/system/generic/init.c \
            libmetal/lib/system/generic/io.c

# drivers/usb_io.c serves the USB cable calls for the rpmsg tty VCP.
MCU_COMMON_SRC += \
            $(OPENAMP_SRC) \
            drivers/usb_io.c \
            NXP/blackbox_file_imx95.c \
            NXP/dma_imx95.c \
            NXP/exti_imx95.c \
            NXP/flexspi_nor_imx95.c \
            NXP/io_imx95.c \
            NXP/rpmsg_imx95.c \
            NXP/rpmsgfs_imx95.c \
            NXP/scmi_imx95.c \
            NXP/serial_flexio_nxp.c \
            NXP/serial_rpmsg_imx95.c \
            NXP/system_imx95.c \
            NXP/tpm_imx95.c

# i.MXRT drivers for blocks this part lacks or has differently.
MCU_EXCLUDES = \
            NXP/dma_imxrt.c \
            NXP/exti_imxrt.c \
            NXP/flexpwm_imxrt.c \
            NXP/flexpwm_pins_imxrt.c \
            NXP/flexspi_nor_imxrt.c \
            NXP/io_imxrt.c \
            NXP/light_ws2811strip_imxrt.c \
            NXP/pwm_beeper_imxrt.c \
            NXP/pwm_motor_imxrt.c \
            NXP/pwm_servo_imxrt.c \
            NXP/romapi_imxrt.c \
            NXP/sdio_usdhc.c \
            NXP/serial_usb_vcp_imxrt.c \
            NXP/system.c \
            NXP/usb_io_imxrt.c \
            NXP/usb/usb_cdc.c \
            NXP/usb/usb_descriptors.c \
            NXP/usb/usb_msc_imxrt.c \
            NXP/xip/image_vector_table.c

# mcuboot image: hash only signature, 0x800 byte header, 7 MB slot.
DEFAULT_OUTPUT   := bin
IMGTOOL          ?= imgtool
IMGTOOL_SIGN     := sign --header-size 0x800 --pad-header --slot-size 0x700000 --align 2 --version 0.0.0
BIN_FROM_ELF_CMD = $(OBJCOPY) -O binary $< $@.raw && \
    if ! command -v $(IMGTOOL) >/dev/null 2>&1; then \
        echo "$(IMGTOOL) not found: mcuboot only boots a signed image, run pip install imgtool or set IMGTOOL=<path>" >&2; \
        exit 1; \
    fi && \
    $(IMGTOOL) $(IMGTOOL_SIGN) $@.raw $@
