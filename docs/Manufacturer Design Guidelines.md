# Betaflight Flight Controller Manufacturer Design Guidelines


![Betaflight](assets/images/bf_logo.png)


Version Change Register

| Version # | Revision Date | Changes, Reasons, and Notes |
| :--- | :--- | :--- | 
| Draft 0.1 | 14 May 2022 | Initial Draft Format |
| Draft 0.2 | 04 June 2022 | Revise format to Final Format |
| Draft 0.3 | 12 June 2022 | Update Visual Media and Tables |
| Draft 0.4 | 21 October 2022 | Update format, add information |
| Draft 0.5 | 24 October 2022 | Add additional information |
| Draft 0.6 | 06 November 2022 | Add cloud build information |
| Draft 0.7 | 17 November 2022 | Remove off-board hardware defines |


Thank you for considering or continuing your development of Betaflight capable flight control hardware.  

Betaflight is an open source project that is free to use and does not incur a license cost, however for the most successful release of a new flight controller or complete ready-to-fly product that is using Betaflight, it remains immensely beneficial to provide representative production samples or pre-production testing units to the Betaflight development team for testing and development feedback.  

In order to have hardware added to the Betaflight approved hardware list, hardware samples representative of the final configuration must be provided to designated members of the betaflight development team.  It is strongly recommended that samples of development hardware or production-representative examples are evaluated positively before accepting pre-orders or releasing products.  Many of the same benefits can be provided for inclusive hardware packages, including flight control stacks, or ready-to-fly craft.

Sharing schematics/layouts with the reviewers will also be beneficial and improve the quality of the review.

This provides the Betaflight team with an opportunity to ensure that the hardware and firmware behave 100% as expected in the representative configuration, as well as support verification of custom defaults required for firmware operation.  After-sales support to customers from members of the Betaflight team are also made possible for reproducing end user issues.

Additional benefits are also present in the form of allowing experienced active pilots with backgrounds in engineering of these systems to assist with aspects of the development process particularly in respect to real world use the products will be subjected to. 

Finally, we will offer a ’Betaflight approved’ product list on the Betaflight github to advise the userbase on electronics which both follow our ‘best practice’ guidance, and which have been tested by our development team. This will be available for flight controller hardware, as well as electronic speed controller stacks, AIOs, and ready-to-fly craft. This strategy is designed to help both developers optimise their hardware and our user base get directed to optimal hardware, and reduce support requests for the Betaflight team who serve a user base of over a hundred thousand users.


# 1 Important Terms and Conditions

## 1.1 Intent

The goal of achieving a certified hardware ecosystem for Betaflight Flight Controller hardware and firmware targets is to ensure that hardware design supports correct operation of Betaflight features and supports an improved user experience for ‘connecting’, flashing, programming, and flying.

The cutting edge flight performance achieved by Betaflight relies on proper hardware design, microcontroller resource allocation, and the ability of end users to correctly configure the software with Betaflight. 

Achieving state of the art performance requires minimizing latency in craft response to perturbations and command inputs, efficient filtering to remove oscillations from flight control calculations, and leveraging the capabilities of peripheral components to enhance the flight experience. For example, features such as advanced RPM filtering, RPM Dynamic Idle and Multi-Dynamic Notch rely on achieving microsecond scale timing jitter for best performance while simultaneously communicating with electronic speed controllers using bidirectional DShot, determining craft attitude and calculating optimal mixer outputs, as well as operating the myriad of desired user peripherals. This symphony of delicately scheduled operations can be achieved reliably with proper flight controller design.


## 1.2 Important Terms and Glossary

| Term | Definition | Reference |
| :--- | :--- | :--- |
| ADC | Analog to Digital Converter |
| BEC | Battery Elimination Circuit | Electronic voltage regulator circuit that provides a specified voltage to a single output power rail |
| Bidirectional DSHOT | Two-Way DSHOT communication protocol to enable Electronic Speed Controller telemetry to be sent back to the flight controller | https://github.com/betaflight/betaflight/wiki/Bidirectional-DSHOT-and-RPM-Filter |
| Bit Bang | Bit-banging refers to any instance of utilizing GPIO functionality to digitally create signals in place of dedicated hardware. This can alleviate some specific data buffer timing requirements |
| DSHOT | Digital Shot Communication Protocol used for Flight Controller to Electronic Speed Controller | https://github.com/betaflight/betaflight/wiki/DSHOT-ESC-Protocol |
| ESC | Electronic Speed Controller | https://us.aspina-group.com/en/learning-zone/columns/what-is/021/ |
| FPV | First Person View | This can also refer to the complete avionics-telemetry supplied via a camera & video transmission system |
| GPIO | General Purpose Input/Output | Reconfigurable digital signal pin that can be selected from an MCU pin |
| IMU | Inertial Motion Unit | Inertial Navigation hardware systems using combined gyroscope and accelerometer unit intended to provide accurate estimates of angular rates, acceleration, and orientation. These can also incorporate magnetometers, barometers, and GPS signals |
| I2C | Inter-Integrated Circuit synchronous serial communication protocol used for connecting microcontroller CPU to peripheral devices |
| MCU | Micro Controller Unit | Single integrated circuit microprocessor with integrated processor, memory, and programmable input/output modules |
| SPI Bus | Serial Peripheral Interface that enables MCU to interface with ADC, DAC, registers, RAM storage, and GPIO | https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html |
| PCB | Printed Circuit Board | May also refer to a populated PCB with components
| PDB | Power Distribution Board | A PCB intended to provide power distribution to electronic speed controller, flight controller, and other peripheral elements |
| VDD | Voltage - Operating Voltage for a particular chip |
| VDD_IO | Voltage range permissible as I/O Supply voltage (Typically 3.3V or less) |


# 2 Engaging with the Betaflight Development Team

## 2.1 Hardware Approval Process - Example

- Manufacturer initiates contact with Betaflight developer(s) at hardware@betaflight.com

- Betaflight team will establish a closed Discord channel for ongoing private discussion between key members of the development team and manufacturer designees 
        Work in progress schematics, PCB renders, and similar documentation prior to initial production provides opportunities for early feedback

- Initial Submission
    - This will require key information to be available, such as specific MCU arrangement (e.g. SPI Bus allocations) may be required in order to support complete feature sets.

    - Desired Target name, MCU type, and which unified target architecture to be used required.

        - Optionally, if any Official Betaflight Presets are to be requested with this hardware, what specific configurations will be used, and a rough plan of what complete system hardware will be provided to support these efforts.

    - Initial PR for target implementation will need to also include a maintainer, with the intention of providing permission to make future edits on the target.

- Request for specific targets
    - If the flight controller design is intended to have a dedicated Betaflight target, then a new target submission can accompany the process
        - For additional information, see here: https://betaflight.com/news/new-target-requirements/ 

- Production Representative Samples
    - To complete hardware certification, representative hardware samples of initial production run, or pre-production batches which are representative of the final release design must be furnished to designated Betaflight development team member(s) to conduct final validation and certification operations.  

    - This will indirectly provide access to the adjacent benefits of leveraging Betaflight developer expertise by using this hardware in complete systems, including flight testing as desired and comprehensive blackbox log analysis.  For configurations requiring specific hardware to be tested (e.g. a Bind & Fly or Plug & Play UAS), providing complete representative hardware systems is best.

- Reporting of performance 	
    - With sufficient test time using the final product, developers can agree to recommend products which meet our guidelines and performance expectations. Products will be noted on Betaflight Github.

- Submission
    - Once hardware has been approved, it will be added to the list of Betaflight approved hardware
        - If new target(s) and/or presets have been added, pull requests will be evaluated, completed, and merged for inclusion in subsequent Betaflight releases

- Target Maintenance
    - After approval and release, the designated contact is expected to provide any continuing support that may be required in order to keep the approved hardware working well for end users


## 2.2 Adjacent Benefits and Opportunities

Betaflight developers are likely to provide significant indirect benefits for manufacturers adhering to this program. These benefits could include reviews of board designs, pad location recommendations, evaluation of EMI behaviors in builds, analysis of Blackbox logging outputs, and even insights into forward compatibility with other subsystems used in complete builds.

Improved customer experience with software interactions, development of presets for BNF multicopter configurations, and validation with specific FPV and RC segments of craft to ensure customer experience will be positive.  The key integration aspects of supporting various remote control and video combinations can only be fully validated with testing hardware samples. 

This thorough testing by expert team members, will help verify and then allow recommendation from the Betaflight development team. Betaflight official recommendation will be highly valuable promotion for the manufacturer so we urge manufacturers to take up this opportunity and work with us.


# 3.0 Flight Controller Design Guidelines

These guidelines provide best practices for physical, electrical, and documentation support of flight controllers. These recommendations are provided as guidelines, however deviating from these provided suggestions should only be undertaken through a collaborative effort with Betaflight developers during the design and prototyping process.


## 3.1 Best Practices for Flight Controller Design and Performance

When asked to review hardware our first action will be to review schematics/layouts against the application notes for the applicable data sheets. The specifics of how the hardware is interconnected is of course going to be driven by a number of constraints, such as recommended pinouts, as outlined below, but is it essential that good design practises are followed with respect to providing good quality power regulation, appropriate use of ground/power planes, decoupling component positioning etc.

### 3.1.1 Physical Configuration

Primary configurations that place the inertial motion unit and other EMI-sensitive components on the side of the PCB that should be mounted away from the electronic speed controller performs best in vertically stacked configurations.  Similarly, physically larger components such as inverters in more protected locations from impacts or adjacent PCB boards in stacked build configurations is recommended.

The commonly accepted and used board layouts include 16x16mm for M2 hardware (perpendicularly aligned), 25.4x25.4mm for M2 hardware (typically diagonally aligned), 20x20mm for M2 or M3 hardware (perpendicularly aligned), and 30.5x30.5mm for M3 hardware (perpendicularly aligned).  In the case of 20x20 boards, selecting a hardware mounting size should be informed by the ability to use threaded inserts and shock-absorbing grommets in oversized holes, while dedicated M2 mounting limits mounting options if hardware is capable of being used in larger craft that tend to utilize M3 mounting.

Boards with oversized holes (e.g. M4 holes and silicone inserts for M3 stack mounting) experience longer service life and experience fewer failures due to PCB deformation during impacts.  For standalone flight controllers, integrated ‘AIO’ flight controllers, and even adjacent configurations provided as ‘stacks’ with additional PCBs, this architecture has been proven to be the most robust over time.

Board layouts should provide redundancy for all critical functions. For example, solder locations with identical pin arrangements to JST connector located adjacent to the JST header mounting is strongly recommended, providing end users significant added value and opportunities to produce more robust craft.  
Castellations and use of mounting pads with through holes or edge continuity are also strongly recommended, especially with more compact flight controller designs.

For board layouts implementing transistor PINIO functionality for ‘pit switch’ or similar behaviors, solder bridge options are strongly recommended to enable users to select output voltage, or physically bypass the switching functionality by connecting to voltage sources.  Particularly for video systems, it is strongly recommended to avoid performing this switching on the ground, due to interference concerns demonstrated with inconsistent ground planes.

If providing direct mounting support for receivers, the following specification should be followed: the pin sequence must be GND, 5V, UART RX, UART TX with a 2.54mm pin pitch, and permit receivers sized up to 12x20mm.  This mirrors the standard mounting (Gnd/5V/Tx/Rx) of CRSF Nano and ELRS Nano receivers, with mirrored UART communication allowing for Tx and Rx to be paired to the same UART.


### 3.1.2 Inertial Measurement Unit (IMU) Selection

Selecting an appropriate IMU for flight controller operation is critical to the resulting flight performance of systems. Proven examples of hardware using single Invensense MPU-6000, single or dual ICM-20602, and also Bosch BMI-270 and BMI-180 units have been successfully demonstrated to operate with Betaflight, although the latter two examples have required significand development effort to bring performance of the IMU gyroscope and accelerometer sensing behaviors up to the standards required to maximize flight performance.


### 3.1.3 Future IMU Options and how to select preferred options

Future IMU selection should be carried out with close involvement of the Betaflight development group.  In cases such as the ICM-42688-P, ICM-42688-V, or ICM-42605, early hardware validation samples should be explored in collaboration with Betaflight developers to determine the suitability of these IMU units in relevant environments.
The ability to customize IMU lowpass filtering and operate within the same GRMS/Shock environment allows for maximum portability of existing filtering and tune schemes, but this development must occur with complete hardware samples and flown in representative flight regimes in order to replicate the EMI environment end users will experience. 

The IMU sensors, designed for applications outside of sUAS, are typically subjected to very harsh electromagnetic environments. Ensuring electromagnetic compatibility when using these immediately adjacent to ultrasonically switched power MOSFET devices, constantly operational radio frequency devices (such as remote control and FPV video systems), under thermal stresses of moving over 1kW through the complete flight stack, are a nontrivial operation.  In order to minimize risks of flyaway and brownout behaviors which can be observed if IMU data filtering and power delivery are inadequate, proper circuit design and validation testing must be performed.

// Mention examples such as the ICM-42688-P and similar options that offer the same high precision and GRMS/Shock tolerances required.  

// Also reiterate the importance of these sensors being able to operate in the sometimes harsh EMI environment of being mounted immediately adjacent to, or on the same PCB as a system that is pushing 1kW or more power via ultrasonic power MOSFET switching through lightweight flight control stacks.


### 3.1.4 Electrical Isolation for Sensor Components

Separate VDD from VDD_IO
Implement additional filtering on VDD if using a single 3v reg for MCU and Sensor


#### 3.1.4.1 Regulated Power and LDO Power Configurations

A key aspect of flight controller performance and longevity is design of the low powered rails that supply power to inertial motion units and the STM microcontroller.  Providing robust low-ripple power to these devices provides the maximum performance potential and hardware longevity for operation in the challenging EMI environments present on these craft.

Similarly, 3.3V, 5V, and 9-12V BEC power needs to provide consistent power at the intended current draw.  For example, 3.3V 500mA is a recommended minimum current.  ‘4.5V’ (5V USB supplied power) should be capable of powering a receiver and GPS unit, which may require over 700mA.

Standard battery powered 5V rails should provide at least 1A, preferably 1.5A to 2A in order to provide higher current if anticipated to be used with a large number of peripherals (such as LED strips, 5V powered FPV Video Systems, or as source power to HD cameras).  

Providing a 10V 2A BEC is also strongly recommended with flight controller designs, as this supports high definition video systems, and even enables better analog video system power options.  
Each of these can be optionally connected to PINIO driven pit switches, and/or jumper pad setups that enable end users to select constant-on or transistor switched behavior, particularly if located to support video transmission systems.

Such 10V regulators should function at full rated current down to 10V input voltage to support the BEC output from some ESCs.

Again, providing robust low-ripple power to these devices provides the maximum performance potential and hardware longevity for operation in the challenging EMI environments present on these craft.


#### 3.1.4.2 ADC Circuitry (e.g. for Current Sensors)

Using ADC measurements on the flight controller to actively monitor current information from ESCs, or if using integral FC-PDB, minimizing noise on the circuit improves the accuracy and jitter of these signals.  To reduce noise from current reading, instead of transfering voltage to FC, send current to FC then move RL Load resistor associated with INA31 onto FC with capacitor in parallel.
https://www.ti.com/lit/ds/symlink/ina139.pdf

Ideally there should be a pair of bridgeable pads on the ESC in series with the load resistor so that with legacy FCs one can bridge it to include the resistor. Correspondingly on FCs we should have a 82K resistor (and parallel 100nF cap), again with series bridging pads which would be left unbridged when paired with legacy ESCs.

When using an ESC/FC stack supporting the above bridging pads, one would leave the ESC pads open and short those on the FC for noise free current reading. In all other cases the ESC pads would be bridged or the FC’s left open to support the current configuration at the expense of higher noise.

Note that the above recommendation is based on the fact that if current if fed into one end of a wire, that same current will always flow out of the other end, whereas with a voltage there will be a voltage drop due to resistance in both that wire and the ground return, and the currents flowing through them. Converting the current to a voltage at the input of the ADC will result in significant noise reduction.


#### 3.1.4.3 Supporting Additional Features

Numerous standard features have become common with flight controller design, for example chips such as MAX7456 to enable monochrome On-Screen Display (OSD) functionality, including Barometers to supplement IMU functionality to provide more accurate altitude estimates, PINIO transistor switched output pads, LED pads, or additional PWM/Motor outputs.

In order to fully support these additional features, it remains strongly recommended that hardware manufacturers incorporate early developer feedback to ensure complete functionality.


#### 3.1.4.4 SWD Debug Support

In order to aid development of Betaflight firmware, and to debug FC specific issues, it is highly beneficial to have test points for the SWDIO/SWCLK lines together with 3V3 and ground connections to enable connection of a debugger such as a Segger JLink or ST-Link. This speeds up resolution of issues hugely, so if at all possible do not use those pins (PA13/PA14) for other purposes.

#### 3.1.4.5 Blackbox Support

Black box of at least 8mb should also be standard on all fcs as it’s literally impossible to problem solve a tune or flight issues with out black box.


## 3.2 Resource Selection Considerations

### 3.2.1 Assigning Resource by Priority

Good resource allocation ensures maximum flexibility in the selection of the modes available to users (for example with DSHOT) and also minimizes conflict in timer and DMA stream allocation.  

Assign motor channels with highest priority …


#### 3.2.1.1 G4 and H7 Resource Selection

G4 and H7 have very flexible DMA controls, therefore we have no specific requirement for these processors.


#### 3.2.1.2 F7 Resource Selection

F7X2 MCUs provide greater flexibility and do not require inverters in order to support protocols such as SBUS, SmartPort, or F.Port.  They also do not exhibit the SPI 1 DMA limitations of F4 processors.

Bitbang Dshot communcation protocol will always use Timer 1 and Timer 8 - Do NOT use these timers for any other functions


#### 3.2.1.3 F4 Resource Selection

Due to F4 MCU management of serial inversion, any pins that implement inversion should be clearly marked, and ideally not result in reduced capability of that UART when used with un-inverted peripheral connections.

For Betaflight 4.4 and later versions, the expected default configuration will take advantage of Bidirectional DShot, therefore default Looprates and DShot of 8k/4k/DShot-300 are anticipated to be the stock configuration.  This does specifically require Motor Resource allocation to enable proper bidirectional DShot communication.

If using Bitbang DShot, when SPI Bus #1 is to be used for the gyro, care must be taken to ensure that motor pins are assigned to appropriate timers.  This is because Bitbang DSHOT uses DMA2 to write to GPIO ports. If this is enabled, it is not possible to enable DMA on an SPI bus using DMA2.
Practically speaking this means that we can’t support DMA on SPI bus 1 (which uses DMA2) on F405 and F411 processors. It is better to put multiple devices on other SPI busses that use SPI bus 1, which is typically used for the gyro.
Bitbang Dshot communcation protocol will always use Timer 1 and Timer 8 - do NOT use these pins for any other functions.  

Further reading: Section 2.1.10 of the errata at 
https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf 

Corruption may occurs on DMA2 if AHB peripherals (e.g. GPIO ports) are accessed concurrently with APB peripherals (eg SPI busses).
Practically, this means that all pins should be on the same port, or at most two ports, so that only one (or two) DMA streams are required for bitbanged operation.

* Additional Recommendations:

    Use A00 for LED Strip (diatone fury resources insert as example)

    Use pins PA13 and 14 for debug options. Including test point pads (these can by tiny) as well as the necessary 3.3v/Ground pads is strongly recommended.

    The DIAT-FURYF4OSD is a good example configuration for F405 boards, because it uses pins with timers that do not experience any conflicts. Motors are on port pins with associated timers and use neither TIM 1 nor N Channels.

    As an additional reference design, see the Fenix F405: https://oshwlab.com/jyesmith/fenix-f405 


## 3.3 Markings, version numbers, and documentation

It is highly recommended that the flight controller Manufacturer Name, Board Name, and Board Design Revision be marked clearly on the flight controller itself.  Where possible, indicating the firmware target name, or selecting a firmware target name to minimize confusion reduces the likelihood of an end user attempting to configure the flight controller with an incorrect target.


### 3.3.1 Creating Design Revisions and Communicating Changes

Providing sufficient marking and documentation will be required for hardware approval.  Screen printing on the flight controller critical information - pin identifiers, board name, revision, and manufacturer will be necessary, with sufficient detail to enable end users to properly connect devices to the flight controller.

When creating flight controller revisions and improvements, it is strongly recommended that indications and documentations are made available, particularly when pinout changes, output rating changes, or hardware bill of material changes. Any board configuration change requiring a user to change configuration behavior must have an accompanying change in marking to indicate (e.g. changing an IMU, or altering the pinout of solder pads).

Following standard marking practices (e.g. V, VCC, or VBAT for battery voltage, G or GND for ground, T for ESC Telemetry, C for Current Sensing ADC, etc.) is also strongly recommended.  Similarly, providing consistent color selections for pin header JST wiring looms, and consistent marking on the PCB is always preferable for robust building and troubleshooting.

Providing a CLI dump file that enables users to reset their flight controller to stock configuration is needed if a dedicated target is not provided.  This is particularly relevant if multiple versions of a board which share targets but have significant hardware changes are present, a CLI dump file for each flight controller revision must be made available to users.


### 3.3.2 Implementing Flight Controller Designs for Ready-To-Fly Craft

#### 3.3.2.1 General Recommendations and Documentation

// A good example of properly documenting and supporting ready-to-fly craft would be the way EMAX makes entire CLI dumps of every craft which enables users to revert systems to the as-shipped software configuration

#### 3.3.2.2 Leveraging the Betaflight Preset System
One of the most powerful tools for end users that can save significant time and reduce the likelihood of misconfiguration is the Preset system.  Developing an Official Betaflight preset and generating a pull request with the assistance of the Betaflight development team allows for end users to quickly and easily configure their craft. The versatility of selectable options, RC Link presets, and even Tune & Filter presets allow for diverse configurations of RTF craft products to be supported.
Applying and using presets not only saves users significant time, but reduces the likelihood of data entry errors or incorrect configurations to be applied, which can be of particular importance for products aimed at less experienced users.

For manufacturers providing configuration presets for flight controllers (strongly recommended), providing documentation of suggested locations to connect peripheral devices, match the provided preset menu selections can greatly assist end users with proper configuration of their craft.  These should include selecting appropriate PID loop rates, DShot data rates, IMU-specific Gyro Lowpass Filtering, Receiver installation (including options with the correct Port, protocol, SerialRX_Inversion, and SerialRX_HalfDuplex), HD FPV MSP port installation, VTX control (including protocol(s), port, and optional pit mode states), GPS (port and protocols), LEDs, and any PINIO type peripheral connections where applicable.

Working with the Betaflight development team provides opportunities to develop presets that can bring RTF craft back to stock configuration, enable users to rapidly configure ‘Plug & Play’ systems based on selected RC link hardware installed per documentation, and even permit Betaflight developers with extensive experience developing precise tunes for craft to generate a range of tunes for that specific product.


## 3.4 Electronic Speed Controller Compatibility

A significant amount of the added performance available in Betaflight 4.X and beyond is based on leveraging ESC RPM telemetry data in order to use RPM Notch Filters and Dynamic Idle features. 


### 3.4.1 For 32 Bit ESCs (BLHeli_32 and AM32)

Betaflight supports all 32-bit ESCs currently available, with BLHeli_32 and AM32 configurations, as well as APD configurations being capable of supporting bidirectional DShot, and user-configured operation with bidirectional DShot disabled.
Additional DShot extended telemetry will be implemented over time as demonstrated stable, however current extended telemetry options will only be enabled by user selection.

Providing 32b ESCs with firmware prior to 32.66 will require end users to reflash ESCs. Craft will not arm due to the RPMFILTER error that will be present due to a lack of RPM Telemetry.  The required solution will be disabling Bidirectional DShot (not recommended) or reflashing ESC (strongly recommended).


### 3.4.2 For 8 Bit ESCs (BLHeli_S)

Betaflight will continue to support all current 8-bit ESC configurations, however these will rely on bidirectional DShot enabled by default.

For Betaflight 4.4 and subsequent releases, the Betaflight team will NO LONGER support BLHeli_S as a default configuration.  The enhanced flight performance made possible by operating with Bidirectional DShot features enabled will become the default behavior for all Betaflight craft..

8-bit ESCs can run **BlueJay**, **JESC**.

For hardware, such as AIO boards which incorporate ESC and FC, the expectation will be that hardware comes with installed firmware meeting these requirements.  The preferred option in this case is **BlueJay**, due to the ability to adjust PWM frequencies and ease of end user support for other functionality across MCU layouts.

Failure to comply with this requirement will require end users to perform firmware reflash of ESCs, and without reflashing craft will not arm due to the RPMFILTER error that will be present due to a lack of RPM Telemetry.
The required solution will be disabling Bidirectional DShot (not recommended) or reflashing ESC (strongly recommended).


### 3.4.3 For Legacy ESCs

For legacy ESCs that are only capable of OneShot and Multishot utilization, end users will be required to disable DShot in order to continue with operations.  This situation only applies to pre-BLHeli_S ESC architectures, and is not anticipated to be an issue for the vast majority of users. The lack of DShot capability has been part of Betaflight operation since 3.2 (in 2017), therefore legacy support for this obsolescent hardware will require additional end user configuration, however these are still indirectly supported and may exhibit improved performance with the most recent Betaflight versions despite being unable to take advantage of bidirectional DShot features.


# 4 Reference Tables


## 4.1 Rated Looptime and Performance

Rated Performance of specific MCU, IMU, and ESC DShot Protocol Combinations

These are the **strongly recommended** default configurations 

For stock configurations, and implementations of ready-to-fly craft, the following configurations are the officially recommended configurations.

Importantly, although the Bidirectional DShot ENABLED may require lower PID Loop Rates for F411 and F405 flight controllers, the difference in loop time is 125us or 250us.  These are microseconds.  The improvements in filtering using RPM Notches alongside Sliding-DFT Multi-Dynamic notches can provide improvements on the scale of milliseconds for phase latency when processing IMU signals to post-filter information that can be used for PID/Mixer calculations. The flight performance of using bidirectional DShot is absolutely worth the PID Looptime Tradeoff, due to that order of magnitude improvement in cumulative signal pathway delay enabled with more targeted notch filtering schema.

Looptime and Performance Recommendation Table: 

| MCU | IMU | Sampling Rate | Bidirectional DShot Status | PID Loop Rate | DShot Protocol |
| :--- | :--- | :--- | :--- | :--- | :--- |
| F7X2, H7XX, G4XX, and similar | MPU60X0,  ICM2060X, ICM42688P | 8 kHz | Enabled or Disabled | 8 kHz | DShot 600 |
| | BMI-270 | 3.2 kHz | Enabled or Disabled | 3.2 kHz | DShot 300 |
| F405 | MPU6000, MPU6050, ICM20601, ICM42688P | 8 kHz | Enabled | 4 kHz | DShot 300 |
| | MPU6000, MPU6050, ICM20601, ICM42688P | 8 kHz | Disabled (not recommended) | 8 kHz | DShot 600 |
| | BMI-270 | 3.2 kHz | Enabled or Disabled | 3.2 kHz | DShot 300 |
| F411 UART Rx ** | MPU6000, MPU6050, ICM20601 | 8 kHz | Enabled | 4 kHz | DShot 300 |
| | MPU6000, MPU6050, ICM20601 | 8 kHz | Disabled (not recommended) | 8 kHz | DShot 600 |
| | BMI-270 | 3.2 kHz | Enabled | 3.2 kHz | DShot 300 |
| F411 SPI Rx *** | MPU6000, MPU6050, ICM20601 | 8 kHz | Enabled | 2 kHz | DShot 300 |
| | MPU6000, MPU6050, ICM20601 | 8 kHz | Disabled  (not recommended) | 4 kHz | DShot 300 |
| | BMI-270 | 3.2 kHz | Enabled | 1.6 kHz | DShot 300 |
| | BMI-270 | 8 kHz | Disabled  (not recommended) | 3.2 kHz | DShot 300 |


** For F411 UART Rx applications, using both available UARTs AND enabling SoftSerial, Accelerometer, large numbers of OSD elements, and using a larger number of filters, stability may require lowering looprate to 2kHz

*** There are no SPI Rx solutions that are strongly recommended for future development due to challenges in resource allocation and scheduler inconsistency that consistently emerge with SPI Rx designs.
Additionally, there are no RC ecosystems that are actively developing a supported SPI Rx solution (ExpressLRS 3.0 and later do not support SPI; FrSky does not support SPI Rx over any protocol, and other SPI Rx solutions have been fully deprecated).

Note that the use of gyros such as the BMI270 lowers the gyro loop rate from 8kHz to 3.2kHz and is therefore advantageous for F411 designs.


## 4.2 Definitions for unified targets

As reference please choose the defines for your target from this list as applicable for the target to select appropiate hardware for the cloud build.


### 4.2.1 Defines for GYRO and ACC

Define at least one gyro and one accelerometer.

    #define USE_GYRO_SPI_MPU6000
    #define USE_ACC_SPI_MPU6000
    #define USE_GYRO_SPI_MPU6500
    #define USE_ACC_SPI_MPU6500
    #define USE_GYRO_SPI_ICM20689
    #define USE_ACC_SPI_ICM20689
    #define USE_ACCGYRO_BMI270
    #define USE_GYRO_SPI_ICM42605
    #define USE_ACC_SPI_ICM42605
    #define USE_GYRO_SPI_ICM42688P
    #define USE_ACC_SPI_ICM42688P

### 4.2.2 Defines for FLASH

Define correct flash driver(s) only if physical present on the board.

    #define USE_FLASH_M25P16           // 16MB Micron M25P16 and others (https://github.com/betaflight/betaflight/blob/master/src/main/drivers/flash_m25p16.c#L68)
    #define USE_FLASH_W25N01G          // 1Gb NAND flash support
    #define USE_FLASH_W25M             // 16, 32, 64 or 128MB Winbond stacked die support
    #define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
    #define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
    #define USE_FLASH_W25Q128FV        // 16MB Winbond 25Q128 and the 8MB Winbond W25Q8 types

### 4.2.3 Defines for BARO

Define a barometer only if physical present on the board.

    #define USE_BARO_MS5611
    #define USE_BARO_SPI_MS5611
    #define USE_BARO_BMP280
    #define USE_BARO_SPI_BMP280
    #define USE_BARO_BMP388
    #define USE_BARO_SPI_BMP388
    #define USE_BARO_LPS
    #define USE_BARO_SPI_LPS
    #define USE_BARO_QMP6988
    #define USE_BARO_SPI_QMP6988
    #define USE_BARO_DPS310
    #define USE_BARO_SPI_DPS310

### 4.2.4 Defines for MAG

Define a magnetometer only if physical present of the board.

    #define USE_MAG_DATA_READY_SIGNAL
    #define USE_MAG_HMC5883
    #define USE_MAG_SPI_HMC5883
    #define USE_MAG_QMC5883
    #define USE_MAG_LIS3MDL
    #define USE_MAG_AK8963
    #define USE_MAG_MPU925X_AK8963
    #define USE_MAG_SPI_AK8963
    #define USE_MAG_AK8975

### 4.2.5 Defines for SX1280

For SPI based SX1280 target designs add the following defines:

    #define USE_RX_SPI
    #define USE_RX_EXPRESSLRS
    #define USE_RX_EXPRESSLRS_TELEMETRY
    #define USE_RX_SX1280
    #define RX_CHANNELS_AETR

### 4.2.6 Defines for OSD

    #define USE_MAX7456

### 4.2.7 Defines for SDCARD

    #define USE_SDCARD

### 4.2.8 Defines for CC2500

For SPI based CC2500 target designs add the following defines:

    #define USE_RX_SPI
    #define USR_RX_CC2500

## 4.3 Usage of the cloud build API

See reference to [cloud build API](https://github.com/betaflight/betaflight/blob/master/docs/Cloud%20build%20API.md)


# 5 Information for Marketing Purposes

* Betaflight is an open source flight controller software (firmware) used to fly multi-rotor and fixed wing aircraft. 

* Betaflight is a fork of Baseflight and Cleanflight, with an emphasis focused on flight performance, leading-edge feature additions, and wide target support.  Combining cutting edge flight performance with diverse hardware support, Betaflight is the leading solution for high performance small unmanned aircraft.

* This project is operated and maintained by volunteers, with community support from pilots with a diverse range of flight goals.  

* Pilots flying Betaflight have won every major FAI, MultiGP, and other major FPV multirotor racing event since 2019.
