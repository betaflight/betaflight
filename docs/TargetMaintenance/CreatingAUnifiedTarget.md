# How to Create a Unified Target Configuration

## 1. Considerations for Unified Target Configurations

### 1.1. General

Unified Target Configurations are very flexible - anything that can be configured in Betaflight can be added to a Unified Target Configuration. This is intentional, as it allows Unified Target Configurations to be adapted to many different kinds of targets, from 'general purpose' flight controller boards over 'all in one' boards with on-board ESC and RX all the way to targets for 'ready to fly' craft that come pre-configured and pre-tuned.

However, this flexibility also puts the onus on target designers to be reasonable about what they add to the Unified Target Configurations that they create and propose for publication. In general, Betaflight tries to minimise the risk of damage or injury to users, and to avoid configurations that will create problems and potential support issues for a majority of users of a particular target. We reserve the right to refuse to add Unified Target Configurations that are in breach of these principles. Furthermore, target creators are advised to use common sense, and refrain from using configuration that is likely to annoy most users of their hardware - users are not likely to consider products from a manufacturer that has annoyed them in the past for future purchases.

Another expectation is that Unified Target Configurations that are submitted for publication are **tested** - please, in the interest of your customers, do not submit untested configurations. Instructions on how to install Betaflight with a new and unpublished Unified Target Configuration can be found [here](https://youtu.be/gO7HyoXYYLY).


### 1.2. Minimum configuration

The expectation is that a Unified Target Configuration contains at least all of the settings to make the hardware on a board work for the user out of the box. The instructions in 2. can be used to create such a configuration from a legacy target. For any hardware that is not configured to work in the Unified Target Configuration, Betaflight's answer to support requests for this hardware will be 'target X does not support this hardware'.


### 1.3. Don'ts

These are things that will result in a Unified Target Configuration being rejected by Betaflight, such as:
- any settings that are equal to the defaults in the firmware (e.g. `resource ... NONE`). Having such settings included results in targets that are more likely to break with new versions of the firmware, and are therefore harder to maintain and more likely to cause support issues;
- pre-setting any calibration settings that are dependent on and different for every board, like `acc_calibration` - these are designed for the user to be prompted to calibrate them when connecting to Betaflight Configurator, in order to get the correct calibration for their hardware;
- Unified Target Configurations that result in safety issues or a setup that is potentially illegal to be used where the user is, such as:
  - setting `motor_pwm_protocol` to any digital protocol - if this default configuration is used with ESCs that do not support digital protocols, it is likely to result in motors spinning up as soon as a battery is connected. The default of `DISABLED` is safe, and the user will be asked to select a motor protocol that is appropriate for their ESCs when they connect to Betaflight Configurator. (Exception: If a Unified Target Configuration is to be used for an all-in-one / RTF product only, an appropriate value for `motor_pwm_protocol` for the on-board ESCs can be selected.)
  - pre-setting any VTX table settings - allowable values for these are governed by law in most countries, and it is up to the manufacturer / supplier of the VTX hardware and the user to make sure that they are complying with the laws in their country.
- setting any values that default to auto-detection (like `baro_hardware`, `mag_hardware`) to `NONE` - the default of `AUTO` is designed to work with no hardware present, and setting the value to `NONE` will result in support issues from users wanting to add their own hardware. (Exception: Boards that do not have any pins or pads for an SPI or I2C bus exposed _can_ define these settings as `NONE`.)
- pre-setting tuning settings for general purpose boards. Betaflight puts a lot of effort into shipping firmware with a default tune that is designed to work reliably across a wide range of hardware, and is extensively tested before every release. All of our documentation is based on this default tune. Adding a different tune to a Unified Target Configuration will result in additional support issues from users for whom the instructions provided by Betaflight do not work. Furthermore, these custom defaults are likely to become outdated with future releases of Betaflight. (Exception: For Unified Target Configurations that are used with only one RTF offering with fixed hardware, it may make sense to add a tune that is optimised for the hardware.)


### 1.4 Discouraged

Betaflight advises target creators against adding any setting to their Unified Target Configuration if it is not strictly related hardware configuration. Examples for discouraged settings are:
- OSD element configuration;
- craft name;
- rates

All of these are really user preferences, and adding them as part of a Unified Target Configuration just means that most customers of this hardware will have to spend extra time on reverting the presets before they can set up their own preferences. There are no strict rules here, and what is reasonable will be dependent on what type of target it is, but manufacturers are advised to use common sense.

## 2. Creating a Unified Target Configuration for a new firmware

These instructions explain how to create a new Unified Target configuration without an existing Betaflight target. If your target already exists in non-unified format please skip to "Creatings a Unified Target Configuration for an existing firmware" 
Creating Unified Targets for a new board means you must start with a generic firmware for your MCU and build up the required pin mappings. Start with a generic firmware and then validate pin mappings and hardware functionality.

### 2.1 Obtaining a unified target firmware for your MCU

Betaflight Configuratior distributes customised targets featuring settings specific to each board. To create firmware for a new board it is recommended to build a generic firmware for your MCU type - STM32F411, STM32F405, STM32F7X2, and so on.
These firmware types can be created by building locally and specifying the MCU type such as `make STM32F05`. Firmware files are created in the `obj` directory and can be flashed by choosing `Load local file` in Betaflight Configurator.

### 2.2 Flashing the unified target firmware

(Theoretically there is no need for the board to match the firmware that is flashed in this step, but there is a chance that the board configuration is setting an input pin on the board to be an output pin, thus leading to a short and potential hardware damage.)

- the important part is that this is the firmware for the MCU type your board uses.

- make sure to enable 'Full chip erase' before flashing, or reset to default with `defaults` in CLI.

- verify that your board is properly reset to defaults: The output of a `diff hardware` in CLI should show `board_name`, `manufacturer_id`, and lines starting with a `#`.

### 2.3 Resource Mappings

In CLI enter the command `resource show` and observe the pin mappings. Betaflight shows pin mappings as a single letter and two digits - `PA2` is shown as `A02`.

Validate that the key mappings for UARTs, motors, SPI, ADC and other common resources are correct. Then check that any unique hardware on your board is added. 

Configure changed or additional hardware with the command `resource <DEVICE> <ID> <PIN>` where device refers to the type of (resource)[https://github.com/betaflight/betaflight/blob/master/src/main/drivers/resource.c] and ID allows for multiple resources of the same type and represents the number of the motor or UART port.

Once all resources are present type `save` and the board configuration will be persisted.

### 2.4 Validate resources

After saving your resource please test your board to ensure all hardware is fully operational. If hardware is not fully working repeat the "Resource Mappings" step.

Once testing is completed proceed to "Get a dump of your board configuration"

## 3. Creating a Unified Target Configuration for an existing firmware

These instructions explain how to create a Unified Target configuration for an existing Betaflight target, starting out with the latest (4.0 or newer) firmware built for this target.

### 3.1. Flash the firmware onto your board

(Theoretically there is no need for the board to match the firmware that is flashed in this step, but there is a chance that the board configuration is setting an input pin on the board to be an output pin, thus leading to a short and potential hardware damage.)

- the important part is that this is the firmware for the target you want to convert to a Unified Target configuration.

- make sure to enable 'Full chip erase' before flashing, or reset to default with `defaults` in CLI.

- verify that your board is properly reset to defaults: The output of a `diff hardware` in CLI should show `board_name`, `manufacturer_id`, and lines starting with a `#`.


### 3.2. Get a dump of your board configuration

- re-start CLI (Disconnect / Connect), then do a `dump hardware`, save the output into a file with 'Save to File'.


### 3.3. Add #defines for hardware drivers

- edit the file from the previous step, and add #defines for accelerometer, barometer, gyro and magnetometer hardware drivers to be included for the cloud build API.

For example on [this target](https://github.com/betaflight/unified-targets/blob/master/configs/default/AIRB-NOX.config) this will look like:

    # Betaflight / STM32F411 (S411) 4.1.0 Oct 16 2019 / 11:57:34 (c37a7c91a) MSP API: 1.42

    #define USE_GYRO_SPI_MPU6000
    #define USE_GYRO_SPI_MPU6500
    #define USE_ACC_SPI_MPU6000
    #define USE_ACC_SPI_MPU6500
    #define USE_MAX7456

    board_name NOX
    manufacturer_id AIRB


For more information see reference at the [Hardware specification](https://github.com/betaflight/betaflight/blob/master/docs/Manufacturer%20Design%20Guidelines.md#42-definitions-for-unified-targets)


### 3.4. Add the board and manufacturer information

- edit the file from the previous step, and verify that `board_name` is set to the target name, and `manufacturer_id` is set to the manufacturer's id as listed in [this document](https://github.com/betaflight/unified-targets/tree/master/Manufacturers.md);
- if the manufacturer is not listed, open an [issue](https://github.com/betaflight/betaflight/issues) and ask for a new id to be assigned. This issue needs to contain the following: name of the company selling the board and supporting customers of it; a website URL for this company. In a future release of the Betaflight configurator, the listing for every board will include this company name, and users will be able to open the company's website from within Betaflight configurator. Allow for a day or two for the Betaflight team to respond to your issue and assign a manufacturer it;
- for boards that are homebrew and / or not planned for commercial availability, use `CUST` as the `manufacturer_id`.


### 3.5. Flash the Unified Target firmware

- find the correct Unified Target for your board based on the MCU type, according to the table below. If your target's MCU is not listed, please open an [issue](https://github.com/betaflight/betaflight/issues) about this to let us know there is demand. Currently, MCU types with only one or two boards using them are not released as unified targets.

|Unified Target|MCU Type|
|-|-|
|STM32F405|STM32F405|
|STM32F411|STM32F411|
|STM32G47X|STM32G471|
|STM32F7X2|STM32F722|
|STM32F745|STM32F745|
|STM32H743|STM32H743|
|STM32H750|STM32H750|

- find and install the firmware (4.0 or newer) for the Unified Target identified above. The firmware is available from the board type drop-down in configurator. Be aware that after flashing this firmware, the LEDs on your board will not be working - this is normal.


### 3.6. Do the initial setup for your Unified Target configuration

- connect to your board running the Unified Target firmware, enter CLI;

- copy / paste the contents of the dump created in 2. into CLI, then enter `save`;

- when the board reboots now, the LEDs should start working again - this is a sign that the previous step was successful.


### 3.7. Add custom settings for your board to the Unified Target configuration

- enter the command: `feature OSD` in CLI, just before the #master section (to switch on OSD by default);

- the following steps are optional (omitting it will give users of your board a minimal but working board configuration when using a Unified Target):;

	- set all the custom settings that are specific to your board (e.g. presets for serial RX on a specific port if the board's instructions are for users to use this port for the serial RX). Changes can be done in the UI or in CLI;

	- try to only include extra settings if you are certain that most / all of your users will want them - unwanted extra changes just make it harder for your users to use your board;

- save the changes;


### 3.8. Create a Unified Target configuration file for your board

- re-start CLI (Disconnect / Connect), then do a `diff all bare`, save the output into a file named `<manufacturer_id>-<board_name>.config` with 'Save to File'. It is crucial that the name exactly matches the manufacturer id and board name specified in the file, or else checking of the pull request you are going to open in a subsequent step will fail;

- edit the resulting file and identify the first line in the file starting with `# Betaflight`. This line (called the banner line) should be left untouched, and it has to be the first line in the file. Whatever content is found above this line,  delete it. Likewise, there must be no extra lines after the last line starting with `set` - delete all subsequnet lines otherwise;

### 3.9. Test

- thoroughly test your new Unified Target configuration with the actual hardware (or a prototype of it). Make sure you test all peripherals (gyro, OSD, flash, SDcard, ...) and all motor outputs (with analog and digital protocols). Ideally you want to flight test as well. Remember that Betaflight does no testing of your Unified Target configuration, and once it has been accepted it will become available to your customers through Betaflight Configurator immediately - if it does not work then this will reflect badly onto your company and its products;

### 4.0. Get it added to Betaflight
- open a [pull request](https://github.com/betaflight/unified-targets/pulls) to put your target configuration into `configs/default`.
