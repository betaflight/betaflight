# DroneCAN stack + libcanard transport.
#
# Each MCU family that wants the stack opts in from its own .mk file with
# one line:
#
#   LIB_SUBMODULES += $(DRONECAN_LIB_DIR)
#
# When this file is sourced (after the MCU .mk has populated
# LIB_SUBMODULES) the presence of that path triggers the rest of the
# wiring: source list, size-optimised flags and include directory. MCUs
# that don't opt in never reference libcanard, so neither the external
# library nor the Betaflight-side glue is compiled for non-CAN targets.

ifneq ($(filter $(DRONECAN_LIB_DIR),$(LIB_SUBMODULES)),)

MCU_COMMON_SRC += \
            io/dronecan/dronecan.c \
            io/dronecan/dronecan_gnss.c \
            io/dronecan/dronecan_node.c \
            dronecan/libcanard/canard.c

SIZE_OPTIMISED_SRC += \
            dronecan/libcanard/canard.c

INCLUDE_DIRS       += $(ROOT)/$(DRONECAN_LIB_DIR)

endif
