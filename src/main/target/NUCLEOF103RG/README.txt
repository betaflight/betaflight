NUCLEOF103RG is a target for Nucleo-F103RG (Nucleo-F103RB transplanted with STM32F103RG which has 1MB of FLASH).
Such hardware with this target comes in handy when a firmware that doesn't fit in smaller FLASH variant when compiled with DEBUG option must be debugged with a debugger.

The target definition files are straight copy of NAZE, except LED0_PIN has been redefined to use Nucleo's LD2 (User LED).

It is also easy to convert exisiting F1 targets to be built to run on the Nucleo-F103RG board:

- Add
    #define FLASH_PAGE_SIZE 0x800
    to target.h

- Also add
#undef USE_DSHOT
#undef USE_LED_STRIP
#undef USE_TRANSPONDER
#undef USE_CAMERA_CONTROL
    to target.h to avoid non-F1 compatible code from getting in.

- Add
MCU_FLASH_SIZE  = 1024
    to target.mk
