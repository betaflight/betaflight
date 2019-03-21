# How to Create a Unified Target Configuration

These instructions explain how to create a Unified Target configuration for an existing Betaflight target, starting out with the latest (4.0 or newer) firmware built for this target.


## 1. Flash the firmware onto your board

(Theoretically there is no need for the board to match the firmware that is flashed in this step, but there is a chance that the board configuration is setting an input pin on the board to be an output pin, thus leading to a short and potential hardware damage.)

- the important part is that this is the firmware for the target you want to convert to a Unified Target configuration;

- make sure to enable 'Full chip erase' before flashing, or reset to default with `defaults` in CLI;

- verify that your board is properly reset to defaults: The output of a `diff hardware` in CLI should show `board_name`, `manufacturer_id`, and lines starting with a `#`.


## 2. Get a dump of your board configuration

- re-start CLI (Disconnect / Connect), then do a `dump hardware`, save the output into a file with 'Save to File';

- edit the resulting file and verify that `board_name` is set to the target name, and `manufacturer_id` is set to the manufacturer's id as listed in [this document](Manufacturers.md). If the manufacturer is not listed, open an [issue](https://github.com/betaflight/betaflight/issues) and ask for a new id to be assigned. For boards that are homebrew and / or not planned for commercial availability, use `CUST` as the `manufacturer_id`.

## 3. Flash the Unified Target firmware

- find the correct Unified Target for your board based on the MCU type, according to the table below. If your target's MCU is not listed, please open an [issue](https://github.com/betaflight/betaflight/issues) about this to let us know there is demand. Currently, MCU types with only one or two boards using them are not released as unified targets.

|Unified Target|MCU Type|
|-|-|
|STM32F405|STM32F405|
|STM32F411|STM32F411|
|STM32F7X2|STM32F722|
|STM32F745|STM32F745|

- find and install the firmware (4.0 or newer) for the Unified Target identified above. The firmware is available from the board type drop-down in configurator. Be aware that after flashing this firmware, the LEDs on your board will not be working - this is normal.


## 4. Do the initial setup for your Unified Target configuration

- connect to your board running the Unified Target firmware, enter CLI;

- copy / paste the contents of the dump created in 2. into CLI, then enter `save`;

- when the board reboots now, the LEDs should start working again - this is a sign that the previous step was successful.


## 5. Add custom settings for your board to the Unified Target configuration

- this step is optional. Omitting it will give users of your board a minimal but working board configuration when using a Unified Target;

- set all the custom settings that are specific to your board (e.g. presets for serial RX on a specific port if the board's instructions are for users to use this port for the serial RX). Changes can be done in the UI or in CLI;

- try to only include extra settings if you are certain that most / all of your users will want them - unwanted extra changes just make it harder for your users to use your board;

- save the changes;


## 6. Create a Unified Target configuration file for your board

- re-start CLI (Disconnect / Connect), then do a `diff all bare`, save the output into a file named `<board_name>.config` with 'Save to File';

- edit the resulting file and verify that there are no extra lines before the one starting with `# Betaflight`, and no extra lines after the last line starting with `set`;

- open a [pull request](https://github.com/betaflight/betaflight/pulls) to put your target configuration into `unified_targets/configs`. If you were assigned a new manufacturer id that is not yet listed, also edit `unified_targets/docs/Manufacturers.md` and add this id (the list is in alphabetical order).
