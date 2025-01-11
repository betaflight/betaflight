CPU_CORE = cortex-m33
MCU_VARIANT = ra6m5

# For flash-jlink target
JLINK_DEVICE = R7FA6M5BH

# Port 1 is highspeed
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 0

flash: flash-jlink
