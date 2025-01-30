/**
 * \defgroup pico_standard_binary_info pico_standard_binary_info
 * \brief Includes default information about the binary that can be displayed by picotool
 *
 * Information is included only if `PICO_NO_BINARY_INFO` and `PICO_NO_PROGRAM_INFO` are both false.
 *
 * This library adds the following information to the binary:
 *
 * * The program name if defined (unless `PICO_NO_BINARY_SIZE=1`). The value is `PICO_PROGRAM_NAME` or `PICO_TARGET_NAME` if the former isn't defined
 * * The value of PICO_BOARD (unless `PICO_NO_BI_PICO_BOARD=1`)
 * * The SDK version (unless `PICO_NO_BI_SDK_VERSION=1`)
 * * The program version string if defined (unless `PICO_NO_BI_PROGRAM_VERSION_STRING=1`). The value is `PICO_PROGRAM_VERSION_STRING``
 * * The program description if defined (unless `PICO_NO_BI_PROGRAM_DESCRIPTION=1`). The value is `PICO_PROGRAM_DESCRIPTION`
 * * The program url if defined (unless `PICO_NO_BI_PROGRAM_URL=1`). The value is `PICO_PROGRAM_URL`
 * * The boot stage 2 used if any (unless `PICO_NO_BI_BOOT_STAGE2_NAME=1`). The value is `PICO_BOOT_STAGE2_NAME`
 * * The program build date (unless `PICO_NO_BI_PROGRAM_BUILD_DATE=1). The value defaults to the C preprocessor value `__DATE__`, but can be overridden with `PICO_PROGRAM_BUILD_DATE`. Note you should do a clean build if you want to be sure this value is up to date.
 * * The program build type (unless `PICO_NO_BI_BUILD_TYPE=1`). The value is `PICO_CMAKE_BUILD_TYPE` which comes from the CMake build - e.g. Release, Debug, RelMinSize
 * * The binary size (unless `PICO_NO_BI_BINARY_SIZE=1`)
 */
