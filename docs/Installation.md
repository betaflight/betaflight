# Installation

## Using configurator

Use the firmware flasher in the cleanflight configurator.

## Manually

See the board specific flashing instructions.

# Upgrading

When upgrading be sure to backup / dump your existing settings.  Some firmware releases are not backwards compatible and default settings are restored when the FC detects an out of date configuration.

## Backup process

disconnect main power, connect to cli via USB/FTDI.

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

## Restore process

Use the cli `defaults` command first.

When restoring from a backup it is a good idea to do a dump of the latest defaults so you know what has changed - if you do this each time a firmware release is created youwill be able to see the cli changes between firmware versions.  For instance, in December 2014 the default GPS navigation PIDs changed.  If you blindly restore your backup you would not benefit from these new defaults.

Use the CLI and send all the output from the saved from the backup commands.

Do not send the file too fast, if you do the FC might not be able to keep up when using USART adapters (including built in ones) since there is no hardware serial flow control.

You may find you have to copy/paste a few lines at a time.

Repeat the backup process again!

Compare the two backups to make sure you are happy with your restored settings.

Re-apply any new defaults as desired.
