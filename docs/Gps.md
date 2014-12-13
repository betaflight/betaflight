# GPS

Two GPS protocols are supported. NMEA text and UBLOX Binary.

## Configuration

Enable the GPS from the CLI as follows:

1) first enable the `feature GPS`
2) set the `gps_provider`
3) if required, set your GPS baud rate using `gps_baudrate`
4) connect your GPS to a serial port that supports GPS and set an approprate `serial_port_x_scenario` to `2`. where `x` is a serial port number.
5) `save`.

Note:  GPS packet loss has been observed at 115200.  Try using 57600 if you experience this.

For the last step check the Board documentation for pins and port numbers and check the Serial documentation for details on serial port scenarios where you will also find some example configurations. 

### GPS Provider

Set the `gps_provider` appropriately.

| Value | Meaning  |
| ----- | -------- |
| 0     | NMEA     |
| 1     | UBLOX    |

### SBAS

When using a UBLOX GPS the SBAS mode can be configured using `gps_sbas_mode`.

The default is AUTO.

| Value | Meaning  | Region        |
| ----- | -------- | ------------- |
| 0     | AUTO     | Global        |
| 1     | EGNOS    | Europe        |
| 2     | WAAS     | North America |
| 3     | MSAS     | Asia          |
| 4     | GAGAN    | India         |

If you use a regional specific setting you may achieve a faster GPS lock than using AUTO.

This setting only works when `gps_auto_config=1`

## GPS Receiver Configuration

UBlox GPS units can either be configured using the FC or manually.

### UBlox GPS manual configuration

Use UBox U-Center and connect your GPS to your computer.  The cli `gpspassthough` command may be of use if you do not have a spare USART to USB adapter.

Display the Packet Console (so you can see what messages your receiver is sending to your computer).

Display the Configation View.

Navigate to CFG (Configuration)

Select `Revert to default configuration`.
Click `Send`.

At this point you might need to disconnect and reconnect at the default baudrate - probably 9600 baud.

Navigate to PRT (Ports)

Set `Target` to `1 - Uart 1`
Set `Protocol In` to `0+1+2`
Set `Protocol Out` to `0+1`
Set `Buadrate` to `57600` `115200`
Press `Send`

This will immediatly "break" communication to the GPS. Since you haven't saved the new baudrate setting to the non-volatile memory you need to change the baudrate you communicate to the GPS without resetting the GPS. So `Disconnect`, Change baud rate to match, then `Connect`. 

Click on `PRT` in the Configuration view again and inspect the packet console to make sure messages are being sent and acknowledged.

Next, to ensure the FC doesn't waste time processing messages it does not need you must disable all messages on except:

    NAV-POSLLH
    NAV-DOP
    NAV-SOL
    NAV-VELNED
    NAV-TIMEUTC

The above messages should each be enabled with a rate of `1`.

    NAV-SVINFO

The above messages should each be enabled with a rate of `5` to reduce bandwidth and load on the FC.

When changing message target and rates remember to click `Send` after changing each message.

Next change the global update rate, click `Rate (Rates)` in the Configuration view.

Set `Measurement period` to `100` ms.
Set `Navigation rate` to `1`.
Click `Send`.

This will cause the GPS receive to send the require messages out 10 times a second.  If your GPS receiver cannot be set to use `100`ms try `200`ms (5hz) - this is less precise.

Next change the mode, click `NAV5 (Navigation 5)` in the Configuration View.

Set to `Dynamic Model` to `airborne <1g` and click `Send`.

Next change the SBAS settings.  Click `SBAS (SBAS Settings)` in the Configuration View.

Set `Subsystem` to `Enabled`.
Set `PRN Codes` to `Auto-Scan`.
Click `Send`.

Finally, we need to save the configuration.

Click `CFG (Configuration` in the Configuration View.

Select `Save current configuration` and click `Send`.
