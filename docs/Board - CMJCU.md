# Board - CMJCU

The CMJCU is a tiny (80mm) board running a STM32F103, which contains a 3-Axis Compass (HMC5883L) 
and an Accelerometer/Gyro (MPU6050).

This board does not have an onboard USB-Serial converter, so an external adapter is needed.

# Pins

RX Connections

| Pin Label | Description |
| --------- | ----------- |
| PA0       | Channel 1   |
| PA1       | Channel 2   |
| PA2       | Channel 3   |
| PA3       | Channel 4   |
| VCC       | Power +3.3v |
| GND       | Ground      |

Serial Connections

| Pin Label | Description     |
| --------- | --------------- |
| TX1       | Serial Transmit |
| RX1       | Serial Receive  |
| GND       | Ground          |
| 3V3       | Power +3.3v     |
| 5V        | Power +5v       |

Power Connections

| Pin Label | Description             |
| --------- | ----------------------- |
| Power +   | Power - 1 Cell 3.7v Max |
| Power -   | Ground                  |

Motor Connections
In standard QUADX configuration, the motors are mapped:

| Cleanflight | CMJCU  |
| ----------- | ------ |
| Motor 1     | Motor3 |
| Motor 2     | Motor2 |
| Motor 3     | Motor4 |
| Motor 4     | Motor1 |

It is therefore simplest to wire the motors:
 * Motor 1 -> Clockwise
 * Motor 2 -> Anti-Clockwise
 * Motor 3 -> Clockwise
 * Motor 4 -> Anti-Clockwise

If you are using the Hubsan x4/Ladybird motors, clockwise are Blue(GND)/Red(VCC) wires, anticlockwise
are Black(GND)/White(VCC)

If you have wired as above, Motor1/Motor2 on the board will be forward.

# Connecting a Serial-USB Adapter

You will need a USB -> Serial UART adapter. Connect:

| Adapter           | CMJCU                      |
| ----------------- | -------------------------- |
| Either 3.3v OR 5v | The correct 3.3v OR 5v pin |
| RX                | TX                         |
| TX                | RX                         |

When first connected this should power up the board, and will be in bootloader mode. If this does not happen, check 
the charge switch is set to POW.
After the flashing process has been completed, this will allow access via the cleanflight configurator to change 
settings or flash a new firmware.

WARNING: If the motors are connected and the board boots into the bootloader, they will start 
to spin after around 20 seconds, it is recommended not to connect the motors until the board
is flashed.

# Flashing

To flash the board:
 * Open Cleanflight Configurator
 * Download the CMJCU firmware binary (https://github.com/cleanflight/cleanflight/tree/master/obj/cleanflight_CJMCU.hex)
 * Select "Load Firmware [Local]" and choose the CMJCU binary
 * Tick "No Reboot Sequence" and "Full Chip Erase"
 * Connect the USB->Serial adapter to the board
 * Select the USB-UART adapter from the top left box
 * Click "Flash Firmware"
 * You should see "Programming: SUCCESSFUL" in the log box
 * Click "Connect" -> This should open the "Initial Setup" tab and you should see sensor data from the quad shown
 * Unplug the quad and solder across the 2 "BOOT0" pins - This prevents the board from going into bootloader mode on next
   boot, if anything goes wrong, simply unsolder these pins and the bootloader will start, allowing you to reflash. You cannot
   overwrite the bootloader.

# Helpful Hints

 * If you are only using a 4 channel RX, in the auxiliary configuration tab, you can add a "Horizon" mode range around 1500 
 for one of the the AUX channels which will result in it being always on
 * Enabling the feature MOTOR_STOP helps with crashes so it doesn't try to keep spinning on its back
 * When the power runs low, the quad will start jumping around a bit, if the flight behaviour seems strange, check your batteries charge

