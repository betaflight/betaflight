# AlienFlight (ALIENFLIGHTF1, ALIENFLIGHTF3, ALIENFLIGHTF4 and ALIENFLIGHTNGF7 target)

AlienWii is now AlienFlight. This designs are released partially for public (CC BY-SA 4.0) and some for noncommercial use (CC BY-NC-SA 4.0) at:

http://www.alienflight.com

AlienFlight F3 Eagle files are available at:

https://github.com/MJ666/Flight-Controllers

AlienFlightNG (Next Generation) designs are released for noncommercial use (CC BY-NC-SA 4.0) or (CC BY-NC-ND 4.0) can be found here:

http://www.alienflightng.com

This targets supports various variants of brushed and brushless flight controllers. All published designs are flight tested by various people. The intention here is to make these flight controllers available and enable skilled users and in some cases RC vendors to build them.

Some variants of the AlienFlight controllers will be available for purchase from:

http://www.microfpv.eu
https://micro-motor-warehouse.com

Here are the general hardware specifications for this boards:

- STM32F103CBT6 MCU (ALIENFLIGHTF1)
- STM32F303CCT6 MCU (ALIENFLIGHTF3)
- STM32F405RGT6 MCU (ALIENFLIGHTF4)
- STM32F711RET6 MCU (ALIENFLIGHTNGF7)
- MPU6050/6500/9250/ICM-20602 accelerometer/gyro(/mag) sensor unit
- The MPU sensor interrupt is connected to the MCU for all published designs and enabled in the firmware
- 4-8 x 4.2A to 9.5A brushed ESCs, integrated, to run the strongest micro motors (brushed variants)
- extra-wide traces on the PCB, for maximum power throughput (brushed variants)
- some new F4 boards using a 4-layer PCB for better power distribution
- USB port, integrated
- (*) serial connection for external DSM2/DSMX sat receiver (e.g. Spektrum SAT, OrangeRx R100, Lemon RX or Deltang Rx31) and SBUS
- CPPM input
- ground and 3.3V for the receiver, some boards have also the option to power an 5V receiver
- hardware bind plug for easy binding
- motor connections are at the corners for a clean look with reduced wiring
- small footprint
- direct operation from a single cell LIPO battery for brushed versions
- 3.3V LDO power regulator (older prototypes)
- 3.3V buck-boost power converter (all new versions)
- 5V buck-boost power converter for FPV (some versions)
- brushless versions are designed for 4S operation and also have an 5V power output
- battery monitoring with an LED or buzzer output (for some variants only)
- current monitoring (F4/F7 V1.1 versions) 
- SDCard Reader for black box monitoring (F4/F7 V1.1 versions)
- (**) integrated OpenSky (FrSky compatible) receiver with FrSky hub telemetry (F4/F7 V2 versions)
- hardware detection of brushed and brushless versions with individual defaults

(*) Spektrum Compatible DSM2 satellites are supported out of the box. DSMX sat will work with DSM2 protocol with default settings (DSM2, 11bit, 11ms is preset). This is chosen for maximum compatibility. For optimal connection it is recommended to adjust settings to match the capabilities of your transmitter and satellite receiver. If possible it is recommended to use the DSMX protocol since it is known as more reliable. Also to make use of additional channels you should adjust the following two parameters with the Cleanflight Configurator.

    set serialrx_provider = 1   (0 for 1024bit, 1 for 2048bit) 
    set spektrum_sat_bind = 5

(**) This receiver is based on the uSky and OpenSky projects. http://www.fishpepper.de 
    
For more detail of the different bind modes please refer the CleanFlight Spektrum Bind document.

Deltang receivers in serial mode will work like any other Spektrum satellite compatible receiver (10bit, 22ms) only the bind process will be different. 

The pin layout for the AlienFlight F1 is very similar to NAZE32 or the related clones (MW32, Flip32, etc.). The pin layout for the AlienFlight F3 is similar to Sparky. The new AlienFlightF3 V2 design have the sensor connected via SPI and some slightly different pin layout. All AlienFlight F3 flight controllers running the same firmware which takes care on the differences with a hardware detection. The AlienFlight F4 and F7 have their individual pin layouts and are independent designs.

(**) OpenSky receiver with telemetry is enabled by default if present on the board.

The AlienFlight firmware will be built as target ALIENFLIGHTF1, ALIENFLIGHTF3, ALIENFLIGHTF4 or ALIENFLIGHTNGF7. The firmware image will come with alternative default settings which will give the user a plug and play experience. There is no computer needed to get this into the air with a small Quadcopter. A preconfigured custom mixer for an Octocopter is part of the default settings to allow clean straight wiring with the AlienFlight. The mixer can be activated with "mixer custom" in the CLI. To use the AlienFlight controller in a Hexa- or Octocopter or to do some more tuning additional configuration changes can be done as usual in the CLI or the BetaFlight configurator. 

## Flashing the firmware

The firmware can be updated with the BetaFlight configurator as for any other target. All AlienFlight boards have a boot jumper which need to be closed for initial flashing or for recovery from a broken firmware.

The firmware for the OpenSky receiver can be updated via serial pass-through and the embedded boot loader. The initial flashing need to be done with the ISP programming pins. The target for the embedded AlienFlight OpenSky receiver is "AFF4RX". Please refer to the OpenSky project for more details.

https://github.com/fishpepper/OpenSky/blob/master/README.md
