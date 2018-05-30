# Code Memory Usage Debugging

This document is primarily aimed at seasoned contributors

## Rationale

With the recent amount of new features memory overflows became a problem not only on F3 targets, but also on F7 with their precious **ITCM-RAM**.
It becomes increasingly hard for developers and contributors to deal with linker errors due to **FLASH**/**AXIM-FLASH**/**ITCM-RAM** overflows as memory usage changes are not always trivial and sometimes mostly hidden behind **Link Time Optimization** (LTO).
To give you a hand in such situations a simple helper script and this document were written.

## Description

So you've run into a problem with your changes causing an overflow, and the numbers don't add up? Chances are this is due to **LTO**, **Inter-Procedural Optimizations** and **Dead Code Elimination**. To further identify which functions have changed in size you will follow a few steps.

For example we'll be using the OMNIBUS target and the Acro Trainer functionality. It currently fits into memory, but let's assume it doesn't for the sake of example.

Edit the corresponding linker script to increase the amount of available memory.
In case of F3 targets you will likely have to open `src/main/target/link/stm32_flash_f303_256k.ld`.
On F3 only **FLASH** is used for code, let's increase the amount from 256K to 512K:
```
--- FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 252K
--- FLASH_CONFIG (r)  : ORIGIN = 0x0803F000, LENGTH = 4K
+++ FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 508K
+++ FLASH_CONFIG (r)  : ORIGIN = 0x0804F000, LENGTH = 4K
```
In case of F7, you may have to increase the amount of ITCM-RAM:
```
--- ITCM_RAM (rx)           : ORIGIN = 0x00000000, LENGTH = 16K
+++ ITCM_RAM (rx)           : ORIGIN = 0x00000000, LENGTH = 128K
```

Build your target again, linking should now succeed:
Use `dump-functions-memory-usage.sh` script on the newly created `.elf` file and save result to file:
```
make OMNIBUS
./dump-functions-memory-usage.sh obj/main/betaflight_OMNIBUS.elf > functions-new.log
```

Run `dump-functions-memory-usage.sh` script against original binary, without the changes that caused an overflow.
For this example, navigate into `fc_core.c` and comment out the call to `pidSetAcroTrainerState()` in `processRx()`, then rebuild and run script again: 
```
make OMNIBUS
./dump-functions-memory-usage.sh obj/main/betaflight_OMNIBUS.elf > functions-old.log
```

Compare the two files by using diff:
```
diff functions-old.log functions-new.log > functions-old-new.diff
```

Study the resulting diff to gain insight on what exactly changes and what happens to memory usage:
```
610c610
< pidController 0x00000ad8
---
> pidController 0x00000cc4
638c638
< processRx.part.5 0x00002658
---
> processRx.part.5 0x000026a0
```
You can notice that the biggest change in memory usage came from the `pidController()` function, even though we have modified `processRx` only.
Compiler has identified that code in `pidController()` can no longer be reached and subject to Dead Code Elimination.
