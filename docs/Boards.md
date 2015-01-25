# Flight controller hardware

The current focus is geared towards flight controller hardware that use the STM32F103 and STM32F303 series processors.  The core logic is separated from the hardware drivers, porting to other processors is possible.

The core set of supported flyable boards are:

* CC3D
* CJMCU
* Flip32+
* Naze32
* Sparky
* AlienWii32

Cleanflight also runs on the following developer boards:

* STM32F3Discovery
* Port103R
* EUSTM32F103RB

There is also limited support for the following boards which may be removed due to lack of users or commercial availability.
 
* Olimexino
* Naze32Pro
* STM32F3Discovery with Chebuzz F3 shield.

Each board has it's pros and cons, before purchasing hardware the main thing to check is if the board offers enough serial ports and input/output pins for the hardware you want to use with it and that you can use them at the same time.  On some boards some features are mutually exclusive.

Please see the board-specific chapters in the manual for wiring details.

There are off-shoots (forks) of the project that support the STM32F4 processors as found on the Revo and Quanton boards.

Where applicable the chapters also provide links to other hardware that is known to work with Cleanflight, such as receivers, buzzers, etc.
