#!/bin/bash

# Check if the input file was provided
if [ -z dos2unix  "$1" ]; then
  echo "Usage: $0 <input-file>"
  exit 1
fi

# Convert line endings of the script file to LF
dos2unix "$0" >/dev/null 2>&1

# Loop through each line in the input file
while read -r line; do
  # Extract the device name from the current line
  device=$(echo "$line" | cut -d',' -f1)

  # Skip the header row
  if [ "$device" == "DEVICE_NAME" ]; then
    continue
  fi

  # Run the make command for the current device
  echo "Building firmware for $device"
  make "$device" EXTRA_FLAGS="-D'RELEASE_NAME=4.4.1' -DCLOUD_BUILD -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_OSD -DUSE_OSD_HD -DUSE_OSD_SD -DUSE_PINIO -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_SERIALRX_DEFAULT -DUSE_SERIALRX_GHST -DUSE_SERIALRX_SBUS -DUSE_SERIALRX_SRXL2 -DUSE_SERIALRX_SPEKTRUM -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_TELEMETRY_GHST -DUSE_VTX"

done < "$1"