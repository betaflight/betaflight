# Configuration


Cleanflight is configured primarilty using the Cleanflight Configurator GUI.

Both the command line interface and gui are accessible by connecting to a serial port on the target,
be it a USB virtual serial port, physical hardware UART port or a softserial port.

The GUI cannot currently configure all aspects of the system, the CLI must be used to enable or configure
some features and settings.

See the Serial section for more information.
See the Board specific sections for details of the serial ports available on the board you are using.

## GUI

The GUI tool is the preferred way of configuration.  The GUI tool also includes a terminal which
can be used to interact with the CLI.

https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb

## CLI

Cleanflight can also be configured by a command line interface.

The CLI can be accessed via the GUI tool or by sending a single '#' character to the main serial port.

To exit the CLI without saving power off the flight controller.

To see a list of commands type in 'help' and press return.

To dump your configuration (including the current profile), use the 'dump' command.

See the other documentation sections for details of the cli commands and settings that are available.
