# Board - AlienFlight (ALIENFLIGHTF1 and ALIENFLIGHTF3 target)

AlienWii is now AlienFlight. This target supports various variants of brushed and brusless flight controllers. The designs for them are released for public use at:

http://www.alienflight.com

All published designs are flight tested by various people. The intention here is to make this flight controllers available and enable skilled users or RC vendors to build this designs.

Here are the general hardware specifications for this boards:

- STM32F103CBT6 MCU (ALIENFLIGHTF1)
- STM32F303CCT6 MCU (ALIENFLIGHTF3)
- MPU6050/6500/9250 accelerometer/gyro(/mag) sensor unit
- The MPU sensor interrupt is connected to the MCU for all new F3 designs and enabled in the firmware
- 4-8 x 4.2A to 9.5A brushed ESCs, integrated, to run the strongest micro motors
- extra-wide traces on the PCB, for maximum power throughput
- USB port, integrated
- (*) serial connection for external DSM2/DSMX sat receiver (e.g. Spektrum SAT, OrangeRx R100, Lemon RX or Deltang Rx31)
- CPPM input
- ground and 3.3V for the receiver
- hardware bind plug for easy binding
- motor connections are at the corners for a clean look with reduced wiring
- small footprint
- direct operation from an single cell lipoly battery
- 3.3V LDO power regulator (older prototypes)
- 3.3V buck-boost power converter (all new versions)
- 5V buck-boost power converter for FPV (some versions)
- battery monitoring with an LED for buzzer functionality (actually for some ALIENFLIGHTF3 variants only)

(*) Spektrum Compatible DSM2 satellites are supported out of the box. DSMX sat will work with DSM2 protocol with default settings (DSM2, 11bit, 11ms is preset). This is chosen for maximum compatibility. For optimal connection it is recommended to adjust settings to match the capabilities of your transmitter and satellite receiver. If possible it is recommended to use the DSMX protocol since it is known as more reliable. Also to make use of additional channels you should adjust the following two parameters with the Cleanflight Configurator.

    set serialrx_provider = 1   (0 for 1024bit, 1 for 2048bit) 
    set spektrum_sat_bind = 5
    
For more detail of the different bind modes please refer the [Spektrum Bind](Spektrum%20bind.md) document

Deltang receivers in serial mode will work like any other Spektrum satellite receiver (10bit, 22ms) only the bind process will be different. 

The pin layout for the ALIENFLIGHTF1 is very similar to NAZE32 or the related clones (MW32, Flip32, etc.). The hardware bind pin is connected to pin 41 (PB5). The pin layout for the ALIENFLIGHTF3 is similar to Sparky. The hardware bind pin is connected to pin 25 (PB12). The new AlienFlightF3 V2 design have the sensor connected via SPI and some slightly different pin layout. All AlienFlight/AlienWii F3 layouts running the same firmware which takes care on the differences with an hardware detection.

The AlienFlight firmware will be built as target ALIENFLIGHTF1 or ALIENFLIGHTF3. The firmware image will come with alternative default settings which will give the user a plug and play experience. There is no computer needed to get this into the air with an small Quadcopter. An preconfigured custom mixer for an Octocopter is part of the default settings to allow clean straight wiring with the AlienFlight. The mixer can be activated with "mixer custom" in the CLI. To use the AlienFlight in an Hexa- or Octocopter or to do some more tuning. Additional configuration changes can be done as usual in the CLI or the Cleanflight configurator. 

## Flashing the firmware

The firmware can be updated with the Cleanflight configurator as for any other target. All AlienFlight boards have an boot jumper which need to be closed for the initial flashing or for recovery from an broken firmware.