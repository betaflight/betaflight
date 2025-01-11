CPU_CORE = cortex-m85
MCU_VARIANT = ra8m1

# For flash-jlink target
JLINK_DEVICE = R7FA8M1AH

# Port 1 is highspeed
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 0

flash: flash-jlink
