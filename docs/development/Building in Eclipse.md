# Short Version
Install the latest Eclipse Standard/SDK and install the **C/C++ developments Tools** plugins
![plugin eclipse](http://i.imgur.com/IdJ8ki1.png)

Import the project using the wizard **Existing Code as Makefile Project**
![](http://i.imgur.com/XsVCwe2.png)

Adjust your build option if necessary
![](https://camo.githubusercontent.com/64a1d32400d6be64dd4b5d237df1e7f1b817f61b/687474703a2f2f692e696d6775722e636f6d2f6641306d30784d2e706e67)

Make sure you have a valid ARM toolchain and Ruby in the path
![](http://i.imgur.com/dAbscJo.png)

# Long version
* First you need an ARM toolchain. Good choices are **GCC ARM Embedded** (https://launchpad.net/gcc-arm-embedded) or **Yagarto** (http://www.yagarto.de).
* Install Ruby (see the document for your operating system).
* Now download Eclipse and unpack it somewhere. At the time of writing Eclipse 4.2 was the latest stable version.
* To work with ARM projects in Eclipse you need a few plugins:
	+ **Eclipse C Development Tools** (CDT) (available via *Help > Install new Software*).
	+ **Zylin Embedded CDT Plugin** (http://opensource.zylin.com/embeddedcdt.html).
	+ **GNU ARM Eclipse** (http://sourceforge.net/projects/gnuarmeclipse/).
	+ If you want to hook up an SWD debugger you also need the **GDB Hardware Debugging** plugin (Also available via *Install new Software*).
* Now clone the project to your harddrive.
* Create a new C project in Eclipse and choose ARM Cross Target Application and your ARM toolchain.
* Import the Git project into the C project in Eclipse via *File > Import > General > File System*.
* Activate Git via *Project > Team > Share Project*.
* Switch to the development branch in Eclipse (*Project > Team > Switch To > development*).
* The next thing you need to do is adjust the project configuration. There is a Makefile included that works but you might want to use GNU ARM Eclipse's automatic Makefile generation. Open the Project configuration and go to *C/C++ Build > Settings*
	* Under *Target Processor* choose "cortex-m3"
	* Under *ARM Yagarto [Windows/Mac OS] Linker > General* (or whatever toolchain you chose)
		+ Browse to the Script file *stm32_flash.ld*
		+ Uncheck "Do not use standard start files"
		+ Check "Remove unused sections"	
	* Under *ARM Yagarto [Windows/Mac OS] Linker > Libraries*
		+ Add "m" for the math library
	* Under *ARM Yagarto [Windows/Mac OS] Compiler > Preprocessor* add the following 2 items to "Defined Symbols":
		+ STM32F10X_MD
		+ USE_STDPERIPH_DRIVER
	* Under *ARM Yagarto [Windows/Mac OS] Compiler > Directories* add the following 3 items
		+ ${workspace_loc:/${ProjName}/lib/CMSIS/CM3/CoreSupport}
		+ ${workspace_loc:/${ProjName}/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x}
		+ ${workspace_loc:/${ProjName}/lib/STM32F10x_StdPeriph_Driver/inc}
	* Under *ARM Yagarto [Windows/Mac OS] Compiler > Miscellaneous* add the following item to "Other flags":
		+ -fomit-frame-pointer
* The code in the support directory is for uploading firmware to the board and is meant for your host machine. Hence, it must not be included in the build process. Just right-click on it to open its properties and choose "Exclude from build" under *C/C++ Build > Settings*
* The last thing you need to do is adding your toolchain to the PATH environment variable.
	+ Go to *Project > Properties > C/C++ Build > Environment*, add a variable named "PATH" and fill in the full path of your toolchain's binaries.
	+ Make sure "Append variables to native environment" is selected.		   
* Try to build the project via *Project > Build Project*.
* **Note**: If you're getting "...could not be resolved" errors for data types like int32_t etc. try to disable and re-enable the Indexer under *Project > Properties > C/C++ General > Indexer*.
