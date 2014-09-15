# Board - CC3D

The OpenPilot Copter Control 3D aka CC3D is a board more tuned to Acrobatic flying or GPS based
auto-piloting.  It only has one sensor, the MPU6000 SPI based Accelerometer/Gyro.
It also features a 16mbit SPI based EEPROM chip.  It has 6 ports labelled as inputs (one pin each)
and 6 ports labelled as motor/servo outputs (3 pins each).

If issues are found with this board please report via the github issue tracker.

# Serial Ports

| Value | Identifier   | Board Markings | Notes                                    |
| ----- | ------------ | -------------- | -----------------------------------------|
| 1     | USART1       | MAIN PORT      | Has a hardware inverter for SBUS         |
| 2     | USART3       | FLEX PORT      |                                          |

Software serial is not supported yet due to timer and pin configuration mappings.

