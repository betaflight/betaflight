# ─────────────────────────────────────────────────────────────────────────────
# target.mk — NavQ95 (NXP i.MX 95, Cortex-M7) base target
#
# Discovered by the top-level Makefile via the presence of this file at
#   src/platform/NAVQ95/target/NAVQ95/target.mk
# The Makefile reads TARGET_MCU_FAMILY from here, then includes
#   src/platform/NAVQ95/mk/$(TARGET_MCU_FAMILY).mk  == mk/NAVQ95.mk
# ─────────────────────────────────────────────────────────────────────────────

# TARGET_MCU_FAMILY is mandatory (locates mk/<family>.mk); TARGET_MCU names the
# specific device/core.
TARGET_MCU        := IMX95_M7
TARGET_MCU_FAMILY := NAVQ95

# FlexSPI1 NOR XIP image budget for the m_flash region.
MCU_FLASH_SIZE    := 8192

# The i.MX 95 real-time core is a Cortex-M7.
DEVICE_FLAGS      += -DIMX95_M7 -DNAVQ95
