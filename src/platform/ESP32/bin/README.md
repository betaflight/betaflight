# Prebuilt ESP32 second-stage bootloaders

These are the ESP-IDF second-stage bootloaders, flashed at offset `0x0`. The
ROM loads this bootloader, which then reads the partition table (`0x8000`) and
loads the Betaflight app from the factory partition (`0x10000`).

They are vendored as binaries rather than built here: the IDF bootloader is a
self-contained sub-project (its own linker script, startup and ~dozens of
sources, driven by CMake/Kconfig) and reproducing that in the Betaflight
Makefile is not worthwhile. The bootloader is chip-specific but otherwise
stable, and esptool patches the flash mode/size/freq into its header at flash
time, so one binary per chip is sufficient.

The build (`make ESP32S3`) generates the partition table from
`../partitions.csv` and merges bootloader + partition table + app into the single
bootable `.bin` (via a transient `*_tmp.bin` app image that is then deleted),
flashable in one step at `0x0`.

## Provenance / how to regenerate

Built with ESP-IDF v5.4 (the `lib/modules/esp-idf` submodule) for the chip,
default bootloader config, flash mode DIO:

    idf.py --preview set-target esp32s3
    idf.py bootloader
    # -> build/bootloader/bootloader.bin

| File                     | Chip       |
|--------------------------|------------|
| `bootloader_esp32s3.bin` | ESP32-S3   |
