# Board - AlienWii32

The AlienWii32 is actually in prototype stage only a few samples exist. There is some more field testing ongoing. The information below is preliminary an will be updated as needed.

Here are the hardware specifications:

- STM32F103CBT6 MCU
- MPU6050 accelerometer/gyro sensor unit
- 8x 4.2A brushed ESCs, integrated, to run the strongest micro motors
- extra-wide traces on the PCB, for maximum power throughput
- USB port, integrated (subject to change)
- * serial connection for external DSM2/DSMX sat receiver (e.g. Spektrum SAT, OrangeRx R100 or Lemon RX)
- alternatively PPM receiver connection (i.e. Deltag Rx31)
- ground and 3.3V for the receiver
- hardware bind plug for easy binding
- motor connections are at the corners for a clean look with reduced wiring
- dimensions: 30x32mm
- direct operation from an single cell lipoly battery

*  Spektrum Compatible DSM2 satellites are supported out of the box. DSMX sat will work with DSM2 protocol with default settings. Changes can be done as usual via CLI or the Cleanflight configurator.

The pin layout is very similar as the NAZE32 or the related clones (MW32, Flip32 etc). The hardware bind pin is connected to pin 41 (PB5). The AlienWii32 firmware will be build with TARGET=NAZE and OPTIONS="AlienWii32". The firmware image will come with alternative default settings which will give the user an plug and play experience. There is no computer need to get this into the air with an smal quadcopter. Anyhow to use this in an Hexa- or Octocopter or to do some more tuning additional CLI changes are required as usual.