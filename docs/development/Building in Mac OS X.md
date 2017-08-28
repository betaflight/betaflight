# Building in Mac OS X

Building in Mac OS X can be accomplished in just a few steps:

* Install general development tools (clang, make, git)
* Install ARM GCC 4.9 series compiler
* Checkout INAV sourcecode through git
* Build the code

## Install general development tools (clang, make, git)

Open up a terminal and run `make`. If it is installed already, you should see a message like this, which means that you
already have the required development tools installed:

```
make: *** No targets specified and no makefile found.  Stop.
```

If it isn't installed yet, you might get a popup like this. If so, click the "install" button to install the commandline
developer tools:

![Prompt to install developer tools](assets/mac-prompt-tools-install.png)

If you just get an error like this instead of a helpful popup prompt:

```
-bash: make: command not found
```

Try running `xcode-select --install` instead to trigger the popup.

If that doesn't work, you'll need to install the XCode development environment [from the App Store][]. After
installation, open up XCode and enter its preferences menu. Go to the "downloads" tab and install the
"command line tools" package.

[from the App Store]: https://itunes.apple.com/us/app/xcode/id497799835

## Install ARM GCC 4.9 series compiler

INAV is built using the 4.9 series GCC compiler provided by the [GNU Tools for ARM Embedded Processors project][].

Hit the "all downloads" link on the right side of the GNU Tools for ARM page to view [the older releases][]. Grab the
Mac installation tarball for the latest version in the 4.9 series (e.g. 4.9-2015q2). Move it somewhere useful
such as a `~/development` folder (in your home directory) and double click it to unpack it. You should end up with a
folder called `~/development/gcc-arm-none-eabi-4_9-2015q2/`.

Now you just need to add the `bin/` directory from inside the GCC directory to your system's path. Run `nano ~/.profile`. Add a
new line at the end of the file which adds the path for the `bin/` folder to your path, like so:

```
export PATH=$PATH:~/development/gcc-arm-none-eabi-4_9-2015q2/bin
```

Press CTRL+X to exit nano, and answer "y" when prompted to save your changes.

Now *close this terminal window* and open a new one. Try running:

```
arm-none-eabi-gcc --version
```

You should get output similar to:

```
arm-none-eabi-gcc.exe (GNU Tools for ARM Embedded Processors) 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 224288]
Copyright (C) 2014 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

If `arm-none-eabi-gcc` couldn't be found, go back and check that you entered the correct path in your `~/.profile` file.

[GNU Tools for ARM Embedded Processors project]: https://launchpad.net/gcc-arm-embedded
[the older releases]: https://launchpad.net/gcc-arm-embedded/+download

## Ruby

Ruby is installed by default on macOS.

## Checkout INAV sourcecode through git

Enter your development directory and clone the [INAV repository][] using the "HTTPS clone URL" which is shown on
the right side of the INAV GitHub page, like so:

```
git clone https://github.com/iNavFlight/inav
```

This will download the entire INAV repository for you into a new folder called "inav".

[INAV repository]: https://github.com/iNavFlight/inav.git

## Build the code

Enter the inav directory and run `make TARGET=NAZE` to build firmware for the Naze32. When the build completes,
the .hex firmware should be available as `obj/inav_NAZE.hex` for you to flash using the INAV
Configurator.

## Updating to the latest source

If you want to erase your local changes and update to the latest version of the INAV source, enter your
inav directory and run these commands to first erase your local changes, fetch and merge the latest
changes from the repository, then rebuild the firmware:

```
git reset --hard
git pull

make clean TARGET=NAZE
make TARGET=NAZE
```
