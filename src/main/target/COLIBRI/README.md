# COLIBRI/QUANTON

* COLIBRI is a flight controller of TBS Gemini mini-hex (http://team-blacksheep.com/products/prod:gemini)
* QUANTON is another FC with the same architecture (http://www.readytoflyquads.com/quanton-flight-control)

## HW info

* STM32F405RGT
* MPU6000 SPI
* MS5611 baro
* HMC5883L compass
* Lots of motor/servo outputs (8 motor + 7 servo on FW or 11 motor + 4 servo of MC)
* No BOOT button (if you brick the board, you'll need SWD to bring it back to life)