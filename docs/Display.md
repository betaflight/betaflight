# Display

Cleanflight supports displays to provide information to you about your aircraft and cleanflight state.

When the aircraft is armed the display does not update so flight is not affected.  When disarmed the display
cycles between various pages.

There is currently no way to change the information on the pages, the list of pages or the time between pages - Code submissions via pull-requests are welcomed!

##Supported Hardware

At this time no other displays are supported other than the SSD1306 / UG-2864HSWEG01.

## Configuration

From the CLI enable the `DISPLAY` feature

```
feature DISPLAY
```


### SSD1306 OLED displays

The SSD1306 display is a 128x64 OLED display that is visible in full sunlight, small and consumes very little current.  
This makes it ideal for aircraft use.

There are various models of SSD1306 boards out there, they are not all equal and some require addtional modifications
before they work.  Choose wisely!

Links to screens:

http://www.banggood.com/0_96-Inch-I2C-IIC-SPI-Serial-128-X-64-OLED-LCD-LED-Display-Module-p-922246.html
http://www.wide.hk/products.php?product=I2C-0.96%22-OLED-display-module-%28-compatible-Arduino-%29
http://witespyquad.gostorego.com/accessories/readytofly-1-oled-128x64-pid-tuning-display-i2c.html
http://www.multiwiicopter.com/products/1-oled

The banggood.com screen is the cheapest at the time fo writing and will correctly send I2C ACK signals.

#### Crius CO-16

This screen is best avoided but will work if you modify it.

Step 1

As supplied the I2C ack signal is not sent because the manufacturer did not bridge D1 and D2 together.  To fix this solder
the two pins together as they enter the screen.  Failure to do this will result is a screen that doesn't display anything.

Step 2

Pin 14 must be disconnected from the main board using a scalpel. Then connect a 10nF or 100nF capacitor between pins 30 and the
lifted pin 14.

Step 3

Connect a 100K resistor between Pin 9 and the lifted Pin 14.

Failure to perform steps 2 and 3 will result in a display that only works on power up some of the time any may display random dots
or other display corruption.

More can be read about this procedure here: http://www.multiwii.com/forum/viewtopic.php?f=6&t=2705&start=10

![Crius CO-16 Diagram](Wiring/Crius CO-16 OLED diagram.png)
![Crius CO-16 Modification](Wiring/Crius CO-16 OLED modifications.jpg)

## Connections

Connect +5v, Ground, I2C SDA and I2C SCL from the flight controller to the display.

On Naze32 rev 5 boards the SDA and SCL pins are underneath the board.


