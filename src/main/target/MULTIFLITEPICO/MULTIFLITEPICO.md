# MultiFlite PICO-B-FC

http://www.multiflite.co.uk

The multiFlite PICO-B-FC is a micro sized brushed FC, sporting an F3 chip, quad >8A fets, 5V out, and upgradable to hex. This FC is designed for power delivery and high performance.

The FC is available from the following store(s):

- http://store.multiflite.co.uk/index.php?route=product/product&path=59&product_id=71

The features of the FC are detailed below:

- Cleanflight and Betaflight firmware compatible
- 22.5 x 25mm
- 2.25g
- Single side populated (for super easy mounting)
- MPU6050
- Brushed motor support for 4 motors (all populated with fets)
- M5 and M6 broken out (used when converting into a HEX)
- 8A N Channel fets (10A <5secs)
- Flight battery voltage monitoring
- Spektrum Satellite* (3.3V) or PPM (VBATT) RX input
- Spektrum bind button (hold on boot to bind)
- Many pads broken out for mods
- 5V Buck/Boost regulator (no brownouts)
- 3.3V regulator powered from 5V
- 1S (4.2V) only
- Battery can be connected at same time as USB for testing (when battery is not connected, the motors will NOT spin. USB does not provide power to motors)
- WS2812B LED solder pad
- Buzzer led for low battery alarm
- Buzzer transistor either for physical buzzer or an LED
- Connectors all around edge and on both sides with labels

(*) Spektrum Compatible DSM2 satellites are supported out of the box. DSMX sat will work with DSM2 protocol with default settings (DSM2, 11bit, 11ms is preset). This is chosen for maximum compatibility. For optimal connection it is recommended to adjust settings to match the capabilities of your transmitter and satellite receiver. If possible it is recommended to use the DSMX protocol since it is known as more reliable. Also to make use of additional channels you should adjust the following two parameters with the BetaFlight Configurator.

    set serialrx_provider = 1   (0 for 1024bit, 1 for 2048bit)
    set spektrum_sat_bind = 5

For more detail of the different bind modes please refer the CleanFlight Spektrum Bind document.

Deltang receivers in serial mode will work like any other Spektrum satellite receiver (10bit, 22ms) only the bind process will be different.

The pin layout for the MULTIFLITEPICO is very similar to SPRACINGF3.

## Flashing the firmware

The firmware can be updated with the BetaFlight configurator as for any other target. All multiFlite boards have a boot jumper which need to be closed for initial flashing or for recovery from a broken firmware.
