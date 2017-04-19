# Configuration

The configuration in INAV is stored at the end of the flash ram, currently it uses 2KB of flash.

Sometimes it's necesaary to erase this during development.

## Erasing

Generate a 2kb blank file.

```
dd if=/dev/zero of=obj/blankconfig.bin bs=1024 count=2
```

Overwrite configuration using JLink

Run JLink (OSX: `/Applications/SEGGER/JLink/JLinkExe`)

Execute commands
`device <your device>`, e.g. `STM32F303CB`
`r`
`h` 
`loadbin obj/blankconfig.bin, <address>`, address 128k device = 0x801F800, 256k device = 0x803f800
`r` to Reboot FC.
`q` to quit

Example session

```
$ /Applications/SEGGER/JLink/JLinkExe
SEGGER J-Link Commander V4.90c ('?' for help)
Compiled Aug 29 2014 09:52:38
DLL version V4.90c, compiled Aug 29 2014 09:52:33
Firmware: J-Link ARM-OB STM32 compiled Aug 22 2012 19:52:04
Hardware: V7.00
S/N: -1 
Feature(s): RDI,FlashDL,FlashBP,JFlash,GDBFull 
VTarget = 3.300V
Info: Could not measure total IR len. TDO is constant high.
Info: Could not measure total IR len. TDO is constant high.
No devices found on JTAG chain. Trying to find device on SWD.
Info: Found SWD-DP with ID 0x2BA01477
Info: Found Cortex-M4 r0p1, Little endian.
Info: FPUnit: 6 code (BP) slots and 2 literal slots
Info: TPIU fitted.
Info: ETM fitted.
Cortex-M4 identified.
Target interface speed: 100 kHz
J-Link>device STM32F303CC
Info: Device "STM32F303CC" selected (256 KB flash, 32 KB RAM).
Reconnecting to target...
Info: Found SWD-DP with ID 0x2BA01477
Info: Found SWD-DP with ID 0x2BA01477
Info: Found Cortex-M4 r0p1, Little endian.
Info: FPUnit: 6 code (BP) slots and 2 literal slots
Info: TPIU fitted.
Info: ETM fitted.
J-Link>r
Reset delay: 0 ms
Reset type NORMAL: Resets core & peripherals via SYSRESETREQ & VECTRESET bit.
J-Link>h
PC = 08001154, CycleCnt = 00000000
R0 = 00000000, R1 = 00000000, R2 = 00000000, R3 = 00000000
R4 = 00000000, R5 = 00000000, R6 = 00000000, R7 = 00000000
R8 = 00000000, R9 = 00000000, R10= 00000000, R11= 00000000
R12= 00000000
SP(R13)= 2000A000, MSP= 2000A000, PSP= 00000000, R14(LR) = FFFFFFFF
XPSR = 01000000: APSR = nzcvq, EPSR = 01000000, IPSR = 000 (NoException)
CFBP = 00000000, CONTROL = 00, FAULTMASK = 00, BASEPRI = 00, PRIMASK = 00
FPU regs: FPU not enabled / not implemented on connected CPU.
J-Link>loadbin obj/blankconfig.bin, 0x803f800
Downloading file [obj/blankconfig.bin]...
WARNING: CPU is running at low speed (7989 kHz).
Info: J-Link: Flash download: Flash programming performed for 1 range (2048 bytes)
Info: J-Link: Flash download: Total time needed: 1.254s (Prepare: 0.698s, Compare: 0.009s, Erase: 0.075s, Program: 0.405s, Verify: 0.005s, Restore: 0.059s)
O.K.
J-Link>r
Reset delay: 0 ms
Reset type NORMAL: Resets core & peripherals via SYSRESETREQ & VECTRESET bit.
J-Link>q
```


