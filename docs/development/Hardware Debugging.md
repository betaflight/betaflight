# Hardware debugging

The code can be compiled with debugging information, you can then upload a debug version to a board via a JLink/St-Link debug adapter and step through the code in your IDE.

More information about the necessary hardware and setting up the eclipse IDE can be found [here](Hardware Debugging in Eclipse.md)

A guide for visual studio can be found here:
http://visualgdb.com/tutorials/arm/st-link/

This video is also helpful in understanding the proces:
https://www.youtube.com/watch?v=kjvqySyNw20

## Compilation options

use `DEBUG=GDB` make argument.

You may find that if you compile all the files with debug information on that the program is too big to fit on the target device.  If this happens you have some options:

* Compile all files without debug information (`make clean`, `make ...`), then re-save or `touch` the files you want to be able to step though and then run `make DEBUG=GDB`.  This will then re-compile the files you're interested in debugging with debugging symbols and you will get a smaller binary file which should then fit on the device.
* You could use a development board such as an Olimexino or an EUSTM32F103RB, development boards often have more flash rom.

## OSX

### Install OpenOCD via Brew

ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

brew install openocd

