# Flight controller hardware

The current focus is geared towards flight controller hardware that use the STM32F303 and legacy STM32F103 series processors.  The core logic is separated from the hardware drivers, porting to other processors is possible.

If you want a fully featured version of Cleanflight then the recommendation for new purchases is an STM32 F3 based board with 256KB of flash memory.  The F3 processor is faster, has built in USB support and supports more hardware without the need for additional adaptors and cables.

The core set of recommended boards are:

* [Seriously Pro SPRacingF3Mini](boards/Board%20-%20SPRacingF3MINI.md)
* [Seriously Pro SPRacingF3](boards/Board%20-%20SPRacingF3.md)
* [Seriously Pro SPRacingF3EVO](boards/Board%20-%20SPRacingF3EVO.md)
* [TBS Colibri Race](boards/Board%20-%20ColibriRace.md)
* [AlienFlightF3](boards/Board%20-%20AlienFlight.md)
* [TauLabs Sparky](boards/Board%20-%20Sparky.md)

The core set of legacy boards are:

* [AlienFlightF1](boards/Board%20-%20AlienFlight.md)
* [OpenPilot CC3D](boards/Board%20-%20CC3D.md)
* [CJMCU](boards/Board%20-%20CJMCU.md)
* Flip32+
* [AbuseMark Naze32](boards/Board%20-%20Naze32.md)
* [RMRC Dodo](boards/Board%20-%20RMDO.md)

Cleanflight also runs on the following developer boards:

* STM32F3Discovery - Recommended for developers.
* Port103R - Recommended for F1 developers.

There is also limited support for the following boards which may be removed due to lack of users or commercial availability.
 
* STM32F3Discovery with Chebuzz F3 shield.

NOTE: Users are advised against purhasing boards that have CPUs with less than 256KB of EEPROM space - available features may be limited.
NOTE: Hardware developers should not design new boards that have CPUs with less than 256KB EEPROM space. 

Each board has it's pros and cons, before purchasing hardware the main thing to check is if the board offers enough serial ports and input/output pins for the hardware you want to use with it and that you can use them at the same time.  On some boards some features are mutually exclusive.

Please see the board-specific chapters in the manual for wiring details.

There are off-shoots (forks) of the project that support the STM32F4 processors as found on the Revo and Quanton boards.

Where applicable the chapters also provide links to other hardware that is known to work with Cleanflight, such as receivers, buzzers, etc.
