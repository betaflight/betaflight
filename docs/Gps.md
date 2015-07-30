# GPS

GPS features in Cleanflight are experimental.  Please share your findings with the developers.

GPS works best if the GPS receiver is mounted above and away from other sources of interference.

The compass/mag sensor should be well away from sources of magnetic interference, e.g. keep it away from power wires, motors, ESCs.

Two GPS protocols are supported. NMEA text and UBLOX binary.

## Configuration

Enable the GPS from the CLI as follows:

1. configure a serial port to use for GPS.
1. set your GPS baud rate
1. enable the `feature GPS`
1. set the `gps_provider`
1. connect your GPS to the serial port configured for GPS.
1. save and reboot.

Note:  GPS packet loss has been observed at 115200.  Try using 57600 if you experience this.

For the connections step check the Board documentation for pins and port numbers.

### GPS Provider

Set the `gps_provider` appropriately.

| Value | Meaning  |
| ----- | -------- |
| 0     | NMEA     |
| 1     | UBLOX    |

### GPS Auto configuration

When using UBLOX it is a good idea to use GPS auto configuration so your FC gets the GPS messages it needs.

Enable GPS auto configuration as follows `set gps_auto_config=1`.

If you are not using GPS auto configuration then ensure your GPS receiver sends out the correct messages at the right frequency.  See below for manual UBlox settings.

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

Use UBox U-Center and connect your GPS to your computer.  The CLI `gpspassthrough` command may be of use if you do not have a spare USART to USB adapter.

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

Next, to ensure the FC doesn't waste time processing unneeded messages, click on `MSG` and enable the following on UART1 alone with a rate of 1. When changing message target and rates remember to click `Send` after changing each message.:

    NAV-POSLLH
    NAV-DOP
    NAV-SOL
    NAV-VELNED
    NAV-TIMEUTC

Enable the following on UART1 with a rate of 5, to reduce bandwidth and load on the FC.

    NAV-SVINFO

All other message types should be disabled.

Next change the global update rate, click `Rate (Rates)` in the Configuration view.

Set `Measurement period` to `100` ms.
Set `Navigation rate` to `1`.
Click `Send`.

This will cause the GPS receive to send the require messages out 10 times a second.  If your GPS receiver cannot be set to use `100`ms try `200`ms (5hz) - this is less precise.

Next change the mode, click `NAV5 (Navigation 5)` in the Configuration View.

Set to `Dynamic Model` to `Pedestrian` and click `Send`.

Next change the SBAS settings.  Click `SBAS (SBAS Settings)` in the Configuration View.

Set `Subsystem` to `Enabled`.
Set `PRN Codes` to `Auto-Scan`.
Click `Send`.

Finally, we need to save the configuration.

Click `CFG (Configuration` in the Configuration View.

Select `Save current configuration` and click `Send`.

### UBlox Navigation model

Cleanflight will use `Pedestrian` when gps auto config is used.

From the UBlox documentation:

* Pedestrian - Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed. MAX Altitude [m]: 9000, MAX Velocity [m/s]: 30, MAX Vertical, Velocity [m/s]: 20, Sanity check type: Altitude and Velocity, Max Position Deviation: Small.
* Portable - Applications with low acceleration, e.g. portable devices. Suitable for most situations. MAX Altitude [m]: 12000, MAX Velocity [m/s]: 310, MAX Vertical Velocity [m/s]: 50, Sanity check type: Altitude and Velocity, Max Position Deviation: Medium.
* Airborne < 1G - Used for applications with a higher dynamic range and vertical acceleration than a passenger car. No 2D position fixes supported. MAX Altitude [m]: 50000, MAX Velocity [m/s]: 100, MAX Vertical Velocity [m/s]: 100, Sanity check type: Altitude, Max Position Deviation: Large

## Hardware

There are many GPS receivers available on the market. 
Below are some examples of user-tested hardware. 

### Ublox

###U-Blox

#### NEO-M8
Module | Comments
-------|--------
U-blox Neo-M8N w/Compass |
Reyax RY825AI | NEO-M8N, 18Hz UART USB interface GPS Glonass BeiDou QZSS antenna module flash. [eBay](http://www.ebay.com/itm/RY825AI-18Hz-UART-USB-interface-GPS-Glonass-BeiDou-QZSS-antenna-module-flash/181566850426)

#### NEO-7
Module | Comments
-------|--------
U-blox Neo-7M w/Compass | [HobbyKing](http://www.hobbyking.com/hobbyking/store/__55558__Ublox_Neo_7M_GPS_with_Compass_and_Pedestal_Mount.html)

#### NEO-6
Module | Comments
-------|--------
Ublox NEO-6M GPS with Compass | [eBay](http://www.ebay.com/itm/111585855757)


### Serial NMEA

#### MediaTek
Module | Comments
-------|--------
MTK 3329 | Tested on hardware serial at 115200 baud (default) and on softserial at 19200 baud. The baudrate and refresh rate can be adjusted using the MiniGPS software (recommended if you lower the baudrate). The software will estimate the percentage of UART bandwidth used for your chosen baudrate and update rate.
