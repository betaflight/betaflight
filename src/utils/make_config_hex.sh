#!/bin/bash

INPUT_FILE=$1
DESTINATION_DIR=$2

TARGET_ADDRESS=0x080FC000

srec_cat ${INPUT_FILE} -binary -offset ${TARGET_ADDRESS} \
    -generate '(' -maximum-address ${INPUT_FILE} -binary -maximum-address ${INPUT_FILE} -binary -offset 1 ')' \
    -constant 0x00 -offset ${TARGET_ADDRESS} \
    -output ${DESTINATION_DIR}/$(basename ${INPUT_FILE}).hex -intel
