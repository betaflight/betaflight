# Analog Devices MAX32650/1/2

This BSP is for working with the Analog Devices
[MAX32650](https://www.analog.com/en/products/max32650.html),
[MAX32651](https://www.analog.com/en/products/max32651.html) and
[MAX32652](https://www.analog.com/en/products/max32652.html)
microcontrollers.  The following boards are supported:
 * [MAX32650EVKIT](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max32650-evkit.html)
 * [MAX32650FTHR](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max32650fthr.html)
 * [MAX32651EVKIT](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max32651-evkit.html) (Secure Bootloader)

This part family leverages the Maxim Microcontrollers SDK (MSDK) for the device
interfaces and hardware abstraction layers. This source code package is fetched
as part of the get-deps script.

The microcontrollers utilize the standard GNU ARM toolchain.  If this toolchain
is not already available on your build machine, it can be installed by using the
bundled MSDK installation.  Details on downloading and installing can be found
in the [User's Guide](https://analogdevicesinc.github.io/msdk//USERGUIDE/).

## Flashing

### MAX32650 and MAX32652

The default flashing behavior in this BSP for the MAX32650 and MAX32652 is to
utilize JLink.  This can be done by running the `flash` or `flash-jlink` rule
for Makefiles, or the `<target>-jlink` target for CMake.

Both the Evaluation Kit and Feather boards are shipped with a CMSIS-DAP
compatible debug probe. However, at the time of writing, the necessary flashing
algorithms for OpenOCD have not yet been incorporated into the OpenOCD master
branch.  To utilize the provided debug probes, please install the bundled MSDK
package which includes the appropriate OpenOCD modifications.   To leverage this
OpenOCD instance, run the `flash-msdk` Makefile rule, or `<target>-msdk` CMake
target.

### MAX32651

The MAX32651 features an integrated secure bootloader which requires the
application image be signed prior to flashing.  Both the Makefile and CMake
scripts account for this signing automatically when building for the
MAX32651EVKIT.

To flash the signed image, the MSDK's OpenOCD variant must be used.  To flash
the MAX32651EVKIT please install the bundled MSDK, and utilize the `flash-msdk`
and `<target>-msdk` rule and target.
