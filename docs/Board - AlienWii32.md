# Board - AlienWii32 (ALIENWIIF1 and ALIENWIIF3 target)

The AlienWii32 is actually in prototype stage and few samples exist. There are some different variants and field testing with some users is ongoing. The information below is preliminary and will be updated as needed.

Here are the hardware specifications:

- STM32F103CBT6 MCU (ALIENWIIF1)
- STM32F303CCT6 MCU (ALIENWIIF3)
- MPU6050 accelerometer/gyro sensor unit
- 4-8 x 4.2A brushed ESCs, integrated, to run the strongest micro motors
- extra-wide traces on the PCB, for maximum power throughput
- USB port, integrated
- (*) serial connection for external DSM2/DSMX sat receiver (e.g. Spektrum SAT, OrangeRx R100, Lemon RX or Deltang Rx31)
- ground and 3.3V for the receiver
- hardware bind plug for easy binding
- motor connections are at the corners for a clean look with reduced wiring
- dimensions: 29x33mm
- direct operation from an single cell lipoly battery
- 3.3V LDO power regulator (older prototypes)
- 3.3V buck-boost power converter (newer prototypes and production versions)
- battery monitoring with an LED for buzzer functionality (actualy for an ALIENWIIF3 variant)

(*) Spektrum Compatible DSM2 satellites are supported out of the box. DSMX sat will work with DSM2 protocol with default settings (DSM2, 11bit, 11ms is preset). This is chosen for maximum compatibility. For optimal connection it is recommended to adjust settings to match the capabilities of your transmitter and satellite receiver. If possible it is recommended to use the DSMX protocol since it is known as more reliable. Also to make use of additional channels you should adjust the following two parameters with the Cleanflight Configurator.

    set serialrx_provider = 1   (0 for 1024bit, 1 for 2048bit) 
    set spektrum_sat_bind = 5
    
For more detail of the different bind modes please refer the [Spektrum Bind](Spektrum bind.md) document

Deltang receivers in serial mode will work like any other Spektrum satellite receiver (10bit, 22ms) only the bind process will be different. 

The pin layout for the ALIENWIIF1 is very similar to NAZE32 or the related clones (MW32, Flip32, etc.). The hardware bind pin is connected to pin 41 (PB5). The pin layout for the ALIENWIIF3 is similar to Sparky. The hardware bind pin is connected to pin 25 (PB12). The AlienWii32 firmware will be built as target ALIENWIIF1 or ALIENWIIF3. The firmware image will come with alternative default settings which will give the user a plug and play experience. There is no computer needed to get this into the air with an small Quadcopter. An preconfigured custom mixer for an Octocopter is part of the default settings to allow clean straight wiring with the AlienWii32. The mixer can be activated with "mixer custom" in the CLI. To use the AlienWii32 in an Hexa- or Octocopter or to do some more tuning additional configuration changes can be done as usual in the CLI or the Cleanflight configurator. 

## Flashing the firmware

The AlienWii32 F1 board can be flashed like the Naze board or the related clones. All the different methods will work in the same way.

The AlienWii32 F3 board needs to be flashed via the USB port in DFU mode. Flashing via the Cleanflight GUI is not possible yet. The DFU mode can be activated via setting the BOOT0 jumper during power on of the board. The second method is to connect with an terminal program (i.e. Putty) to the board and enter the character "R" immediately after connecting. Details about the flashing process can be found in the related section of the [Sparky](Board - Sparky.md) documentation. The BOOT0 jumper should be removed and the board needs to be repowerd after firmware flashing. Please be aware, during reboot of the AlienWii F3 board, the GUI will disconnect and an manual reconnect is required.