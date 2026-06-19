TARGET_MCU        := ESP32S3
TARGET_MCU_FAMILY := ESP32S3

# Default for the bare ESP32S3 target; a per-board config.mk may override it
# (e.g. CUSTS3AIO uses an N4R2 module with 4 MB flash). ?= so config.mk wins.
MCU_FLASH_SIZE  ?= 8192
