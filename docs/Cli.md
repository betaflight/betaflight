# Command Line Interface (CLI)

Cleanflight has a command line interface (CLI) that can be used to change settings and configure the FC.

## Accessing the CLI.

The CLI can be accessed via the GUI tool or via a terminal emulator connected to the CLI serial port.

1. Connect your terminal emulator to the CLI serial port (which, by default, is the same as the MSP serial port)
2. Use the baudrate specified by cli_baudrate (115200 by default).
3. Send a `#` character.

To save your settings type in 'save', saving will reboot the flight controller.

To exit the CLI without saving power off the flight controller or type in 'exit'.

To see a list of other commands type in 'help' and press return.

To dump your configuration (including the current profile), use the 'dump' command.

See the other documentation sections for details of the cli commands and settings that are available.

## Backup via CLI

Disconnect main power, connect to cli via USB/FTDI.

dump using cli

`rate profile 0`
`profile 0`
`dump`

dump profiles using cli if you use them

`profile 1`
`dump profile`

`profile 2`
`dump profile`

dump rate profiles using cli if you use them

`rate profile 1`
`dump rates`

`rate profile 2`
`dump rates`

copy screen output to a file and save it.

## Restore via CLI.

Use the cli `defaults` command first.

When restoring from a backup it is a good idea to do a dump of the latest defaults so you know what has changed - if you do this each time a firmware release is created youwill be able to see the cli changes between firmware versions.  For instance, in December 2014 the default GPS navigation PIDs changed.  If you blindly restore your backup you would not benefit from these new defaults.

Use the CLI and send all the output from the saved from the backup commands.

Do not send the file too fast, if you do the FC might not be able to keep up when using USART adapters (including built in ones) since there is no hardware serial flow control.

You may find you have to copy/paste a few lines at a time.

Repeat the backup process again!

Compare the two backups to make sure you are happy with your restored settings.

Re-apply any new defaults as desired.


