# Shared by every i.MXRT board target. Included at the end of a target.mk,
# after it has named its MCU, family and flash size.

# Builds the boot ROM's FlexSPI configuration block from the flash profile and
# the wiring values in target.h.
TARGET_SRC += \
            NXP/xip/flexspi_nor_profiles.c
