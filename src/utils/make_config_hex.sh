#!/bin/bash

# Create hex file from custom defaults in order to flash separately
#
# This will only work if the target was built with 'CUSTOM_DEFAULTS_EXTENDED'
#
# Usage: make_config_hex <input file> <output directory> <config area start address>
# Choose the config area start address from:
#
# STM32F405: 0x080FC000
# STM32F411 / STM32F7X2: 0x0807C000
# STM32F74X: 0x080F8000

INPUT_FILE=$1
DESTINATION_DIR=$2
TARGET_ADDRESS=$3

srec_cat ${INPUT_FILE} -binary -offset ${TARGET_ADDRESS} \
    -generate '(' -maximum-address ${INPUT_FILE} -binary -maximum-address ${INPUT_FILE} -binary -offset 1 ')' \
    -constant 0x00 -offset ${TARGET_ADDRESS} \
    -output ${DESTINATION_DIR}/$(basename ${INPUT_FILE}).hex -intel
