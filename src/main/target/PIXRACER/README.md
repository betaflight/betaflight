# Pixracer

* Target owner: @DigitalEntity
* The Pixracer is the first autopilot of the new FMUv4 Pixhawk generation. It has Wifi built-in, comes with upgraded sensors and more flash.
* Website: https://pixhawk.org/modules/pixracer
* Purchase from: http://www.auav.co/product-p/xr-v1.htm

## HW info

* STM32F427VIT6
* ICM-20608 SPI
* MPU9250 SPI
* HMC5983 compass
* MS5611 baro
* 6 pwm outputs + PPM/SBus input

## Warnings

* PicRacers native Flight controller (PX4 & ArduPilot) motor layout is different to iNAV
  * Either Swap your ESC PWM cables to match iNav
  * Or make a custom mix
