# Using devcontainers

NOTE: Unless otherwise specified all commands below should be run from the main workspace folder of the betaflight repository.

NOTE: All examples here are using a Moblite7 1S tiny whoop based on STM32F411CEU6.
The betaflight target is CRAZYBEEF4DX.

## Why use devcontainers?

Devcontainers provide a consistent, reproducible development environment that works the same across machines.
They eliminate “works on my machine” issues by packaging tools, dependencies, and configurations inside a container.
This makes onboarding faster, simplifies collaboration, and ensures your development setup stays clean and isolated.

## How is this different from the betaflight/cloudbuild repository?

The betaflight/cloudbuild repository provides a containerized CI/CD environment for building and testing Betaflight firmware in the cloud.
It is optimized for automated builds and may not include all the tools and configurations needed for local development.
This devcontainer includes DFU utilities and OpenOCD debugger for example.
While similar to the cloudbuild container it is tailored for interactive development and debugging and avoids disruptions caused by changes in the cloudbuild repo.

## Defining UDEV rules on the host

Devcontainers for HW development are a bit special, as they require HW access to the device.
It can get tricky to make this universal, because not all Linux distributions use the same group numbers to control permissions.
Typically to do HW development on an STM32 board the user would need at least `dialout` and `plugdev` permissions.
The `dialout` groups for example can be number 11, 18 or 20 on Void Linux, Fedora and Debian, respectively.

The easiest way to deal with this is to define custom `udev` rules.
To allow the container access to the HW, the device would need to be configured with the right permissions when plugged in.
Here are the steps required:

1. plug in the device and check its IDs and default permissions
    ```bash
    # List the USB connected devices
    $ lsusb
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    Bus 001 Device 002: ID 0483:5740 STMicroelectronics Virtual COM Port   <==== this is our FC! Note the `{idVendor}` and `{idProduct}` IDs!
    Bus 001 Device 003: ID 1fd2:8005 Melfas LGDisplay Incell Touch
    Bus 001 Device 004: ID 04f2:b681 Chicony Electronics Co., Ltd ThinkPad T490 Webcam
    Bus 001 Device 005: ID 06cb:00bd Synaptics, Inc. Prometheus MIS Touch Fingerprint Reader
    Bus 001 Device 006: ID 8087:0aaa Intel Corp. Bluetooth 9460/9560 Jefferson Peak (JfP)
    Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub

    # Check the default permissions
    $ ls -la /dev/ttyACM0
    crw-rw---- 1 root dialout 166, 0 Jan 24 09:35 /dev/ttyACM0     <==== device is owned by root with access via the `dialout` group.
    ```

2. create a `udev` rule for the device by creating this file: `sudo nano /etc/udev/rules.d/99-betaflight.rules`
    ```bash
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", TAG+="uaccess"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666"
    ```
    The `{idVendor}` and `{idProduct}` IDs must match those of the device.
    The two `SUBSYSTEM` rules cover both the normal operation and DFU mode of the flight controller.

3. plug the device out and in again. You should see:
    ```bash
    $ ls -la /dev/ttyACM0
    crw-rw-rw- 1 root dialout 166, 0 Jan 24 09:39 /dev/ttyACM0   <==== note the third `rw` allowing access to all
    ```

NOTE: this example is with a ready made flight controller on UART, but the same procedure works for STLink debuggers for example.

## Installation of tools

Two examples are given here - one using Docker and the other using Podman.
For the Docker example we will use Microsoft's VSCode with the Dev Containers extension.
For the Podman example we will use fully open source development chain with `devpod` and VSCodium.

### Installing Podman

Podman is a rootless, daemonless alternative to Docker that provides a compatible command‑line interface for managing containers.
Installation instructions for all supported platforms are available in the [official Podman documentation](https://podman.io/docs/installation).
There are no special daemons to be run or group permissions to be added.
Validate the installation with:
```bash
    podman run docker.io/library/hello-world
```

### Installing Docker (CLI)

Docker provides the docker CLI for building and running containers.

Follow the [official installation guide](https://docs.docker.com/engine/install/).

After installation usually the docker service needs to be started and the user should be added to the `docker` group.
Details on how to do this is linux distro dependent - refer to your distribution manual for details on how to achieve this.
Below is an example that should work on distributions with `systemd` (Debian and derivatives, RedHat and derivatives, etc.).

```bash
    sudo systemctl enable --now docker  # Enable and start the service
    sudo usermod -aG docker $USERNAME  # you might need to log out and back in after this step
    docker run docker.io/library/hello-world  # Run an example to validate the install
```

### Installing VS Code and the Dev Containers Extension

To use devcontainers more effectively, you can install Visual Studio Code along with the Dev Containers extension.

You can install VS Code by following the [official instructions provided](https://code.visualstudio.com/docs/setup/setup-overview) for your operating system.

Once VS Code is installed, add the Dev Containers extension using the [official guide](https://code.visualstudio.com/docs/devcontainers/containers).

With these tools installed, you can open the betaflight repository in VS Code and use the Dev Containers extension to build and run the devcontainer defined in the `.devcontainer` folder.

NOTE: Dev Containers extension in VSCode comes pre-configured for the `docker` binary.
    You can simply type `podman` in the extension setting to change the container provider.

### Fully open-source solution via devpod and VSCodium

VSCode from Microsoft and its Dev Containers Extension are not fully open-source.

The [VSCodium project](https://vscodium.com/) builds from the open-source repository of VSCode stripping Microsoft proprietary code.
Similarly the [devpod project](https://devpod.sh/) provides a wrapper similar to the Microsoft Dev Containers Extension around the open [development container specification](https://containers.dev/).
The two can be used together to achieve the same as Microsoft's proprietary solution.

Example of running with `devpod`:

```bash
    $ podman --version  # check podman version
    podman version 5.7.1
    $ devpod version  # check devpod version
    v0.6.15
    $ devpod provider list  # check that the podman provider is configured

     NAME  | VERSION | DEFAULT | INITIALIZED |   DESCRIPTION
  ---------+---------+---------+-------------+-------------------
    docker | v0.0.1  | false   | true        | DevPod on Docker
    podman | v0.0.1  | true    | true        | DevPod on Podman
    $ devpod ide list  # check that the VSCodium IDE is configured

         NAME       | DEFAULT
  ------------------+----------
    clion           | false
    codium          | true
    cursor          | false
    dataspell       | false
    fleet           | false
    goland          | false
    intellij        | false
    jupyternotebook | false
    none            | false
    openvscode      | false
    phpstorm        | false
    positron        | false
    pycharm         | false
    rider           | false
    rstudio         | false
    rubymine        | false
    rustrover       | false
    vscode          | false
    vscode-insiders | false
    webstorm        | false
    zed             | false
    $ devpod up .  # start the devcontainer in the current folder
```
The last command will open VSCodium in the devcontainer context.

## Example usage inside the container

Once inside the devcontainer terminal you can check the device access:

```bash
    devpod@bdaab82778ae:/workspaces/betaflight$ ls -la /dev/ttyACM0
    crw-rw-rw- 1 nobody nogroup 166, 0 Jan 24 09:24 /dev/ttyACM0
```

You can then build and flash the firmware as usual:

```bash
    devpod@bdaab82778ae:/workspaces/betaflight$ make clean
    devpod@bdaab82778ae:/workspaces/betaflight$ make CRAZYBEEF4DX
        Building target config CRAZYBEEF4DX
        make --no-print-directory fwo CONFIG=CRAZYBEEF4DX
        EF HASH -> ./obj/main/STM32F411_CRAZYBEEF4DX/.efhash_4e3f673b7beeb9bf243f745d779b10e0
        %% startup_stm32f411xe.s
        %% (speed optimised) ./src/platform/common/stm32/system.c
        .
        .
        .
        %% (optimised) ./src/main/drivers/usb_io.c
        Linking STM32F411_CRAZYBEEF4DX
        Memory region         Used Size  Region Size  %age Used
                FLASH:        8268 B        16 KB     50.46%
            FLASH_CONFIG:           0 B        16 KB      0.00%
                FLASH1:      371710 B       480 KB     75.62%
        SYSTEM_MEMORY:           0 B        29 KB      0.00%
                    RAM:       81960 B       128 KB     62.53%
            MEMORY_B1:           0 B          0 B
        text	   data	    bss	    dec	    hex	filename
        374254	   5724	  75824	 455802	  6f47a	./obj/main/betaflight_STM32F411_CRAZYBEEF4DX.elf
        Creating HEX ./obj/betaflight_2025.12.0-beta_STM32F411_CRAZYBEEF4DX.hex
        Building target config CRAZYBEEF4DX succeeded.
    devpod@bdaab82778ae:/workspaces/betaflight$ make dfu_flash CONFIG=CRAZYBEEF4DX
        # potentially this is because the MCU already is in DFU mode, try anyway
        echo -n 'R' > /dev/ttyACM0
        sleep 1
        make --no-print-directory ./obj/betaflight_2025.12.0-beta_STM32F411_CRAZYBEEF4DX.dfu
        Creating DFU ./obj/betaflight_2025.12.0-beta_STM32F411_CRAZYBEEF4DX.dfu
        dfu-util -a 0 -D ./obj/betaflight_2025.12.0-beta_STM32F411_CRAZYBEEF4DX.dfu -s :leave
        dfu-util 0.11

        Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
        Copyright 2010-2021 Tormod Volden and Stefan Schmidt
        This program is Free Software and has ABSOLUTELY NO WARRANTY
        Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

        Match vendor ID from file: 0483
        Match product ID from file: df11
        Multiple alternate interfaces for DfuSe file
        Opening DFU capable USB device...
        Device ID 0483:df11
        Device DFU version 011a
        Claiming USB DFU Interface...
        Setting Alternate Interface #0 ...
        Determining device status...
        DFU state(10) = dfuERROR, status(10) = Device's firmware is corrupt. It cannot return to run-time (non-DFU) operations
        Clearing status
        Determining device status...
        DFU state(2) = dfuIDLE, status(0) = No error condition is present
        DFU mode device DFU version 011a
        Device returned transfer size 2048
        DfuSe interface name: "Internal Flash  "
        File contains 1 DFU images
        Parsing DFU image 1
        Target name: ST...
        Image for alternate setting 0, (2 elements, total size = 379994)
        Setting Alternate Interface #0 ...
        Parsing element 1, address = 0x08000000, size = 8268
        Erase   	[=========================] 100%         8268 bytes
        Erase    done.
        Download	[=========================] 100%         8268 bytes
        Download done.
        Parsing element 2, address = 0x08008000, size = 371710
        Erase   	[=========================] 100%       371710 bytes
        Erase    done.
        Download	[=========================] 100%       371710 bytes
        Download done.
        Done parsing DfuSe file
        Submitting leave request...
        Transitioning to dfuMANIFEST state

```