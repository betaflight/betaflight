# Hardware Debugging In Eclipse

Build a binary with debugging information using command line or via Eclipse make target.

Example Eclipse make target

![](https://raw.github.com/wiki/hydra/cleanflight/images/eclipse-gdb-debugging/make 1 - OLIMEXINO GDB.PNG)

# GDB and OpenOCD

start openocd

Create a new debug configuration in eclipse :
![connect to openocd](http://i.imgur.com/somJLnq.png)
![use workspace default](http://i.imgur.com/LTtioaF.png)

you can control openocd with a telnet connection:

     telnet localhost 4444

stop the board, flash the firmware, restart:

     reset halt
     wait_halt 
     sleep 100
     poll
     flash probe 0
     flash write_image erase /home/user/git/inav/obj/inav_NAZE.hex 0x08000000
     sleep 200
     soft_reset_halt
     wait_halt
     poll
     reset halt

A this point you can launch the debug in Eclispe.
![](http://i.imgur.com/u7wDgxv.png)

# GDB and J Link

Here are some screenshots showing Hydra's configuration of Eclipse (Kepler)

If you use cygwin to build the binaries then be sure to have configured your common `Source Lookup Path`, `Path Mappings` first, like this:

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 7.PNG)


Create a new `GDB Hardware Debugging` launch configuration from the `Run` menu

It's important to have build the executable compiled with GDB debugging information first.
Select the appropriate .elf file (not hex file) - In these examples the target platform is an OLIMEXINO, not a naze32.

DISABLE auto-build

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 1.PNG)

Choose the appropriate gdb executable - ideally from the same toolchain that you use to build the executable.

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 2.PNG)

Configure Startup as follows

Initialization commands

```
target remote localhost:2331
monitor interface SWD
monitor speed 2000
monitor flash device = STM32F103RB
monitor flash download = 1
monitor flash breakpoints = 1
monitor endian little
monitor reset
```


![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 3.PNG)

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 4.PNG)

It may be useful to specify run commands too:

```
monitor reg r13 = (0x00000000)
monitor reg pc = (0x00000004)
continue
```

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 13.PNG)

If you use cygwin an additional entry should be shown on the Source tab (not present in this screenshot)

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 5.PNG)

Nothing to change from the defaults on the Common tab

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 6.PNG)

Start up the J-Link server in USB mode

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 9.PNG)

If it connects to your target device it should look like this

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 10.PNG)

From Eclipse launch the application using the Run/Debug Configurations..., Eclipse should upload the compiled file to the target device which looks like this
 
![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 11.PNG)

When it's running the J-Link server should look like this.

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 12.PNG)

Then finally you can use Eclipse debug features to inspect variables, memory, stacktrace, set breakpoints, step over code, etc.

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/debugging.PNG)

If Eclipse can't find your breakpoints and they are ignored then check your path mappings (if using cygwin) or use the other debugging launcher as follows.  Note the 'Select other...' at the bottom of the configuration window.

![](https://raw.github.com/wiki/cleanflight/cleanflight/images/eclipse-gdb-debugging/config 8 - If breakpoints do not work.PNG)

