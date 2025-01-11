CPU_CORE = cortex-m33
MCU_VARIANT = ra6m5

# Port 1 is highspeed
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 0

JLINK_DEVICE = R7FA6M5BH
DFU_UTIL_OPTION = -d 2341:0368 -a 0

flash: flash-dfu-util
