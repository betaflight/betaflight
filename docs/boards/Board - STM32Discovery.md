# Boards - STM32Discovery

STM32Discovery boards are relatively cheap boards featuring STM32 MCUs manufactured and distributed by [STMicroelectronics](https://www.st.com/). They are known as "tinkerer boards", not specifically as flight controllers, as they can be used for a variety of things. They are bigger and heavier than standard flight controllers, feature plug pins for ease of use and quick changes, no soldering required.

They come with different MCUs and in different sizes, however pin-outs are very similar for compatibility with external boards which can be used as extensions.

Solid documentation is provided by STMicroelectronics.


## Targets

| Target               | MCU           | Documentation |
| ---                  | ---           | --- |
| `STM32F3DISCOVERY`   | `STM32F303VC` | [User Manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/8a/56/97/63/8d/56/41/73/DM00063382.pdf/files/DM00063382.pdf/jcr:content/translations/en.DM00063382.pdf) \| [MCU Datasheet](www.st.com/resource/en/datasheet/stm32f303vc.pdf) |
| `STM32F4DISCOVERY`   | `STM32F407VG` | [User Manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/70/fe/4a/3f/e7/e1/4f/7d/DM00039084.pdf/files/DM00039084.pdf/jcr:content/translations/en.DM00039084.pdf) \| [MCU Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf) |
| `STM32F411DISCOVERY` | `STM32F411VE` | [User Manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/e9/d2/00/5e/15/46/44/0e/DM00148985.pdf/files/DM00148985.pdf/jcr:content/translations/en.DM00148985.pdf) \| [MCU Datasheet](https://www.st.com/resource/en/datasheet/stm32f411re.pdf) |

Hardware specifications, peripherals, pinouts and all other data can be found in above User Manuals. Rest of this document is instead going to focus on setup of the board for Betaflight and example assemblies/wirings.

_Note: Not all pin-outs seem to be listed in the User Manual, double-checking in MCU Datasheet is recommended._

### Pins

| Function       | F3   | F4   | F411 |
| ---            | ---  | ---  | ---  |
| PPM Receiver   | PB8  | PB9  | PB8  |
| Motor 1        | PA8  | PB1  | PD12 |
| Motor 2        | PC6  | PB0  | PB1  |
| Motor 3        | PC7  | PA2  | PB0  |
| Motor 4        | PC8  | PA3  | PA2  |
| Motor 5        | /    | PA10 | PA3  |
| Motor 6        | /    | PA8  | PA10 |
| Beeper         | PD12 | /    | PA8  |
| USART1 TX      | PA9  | PB6  | PA15 |
| USART1 RX      | PA10 | PB7  | PA10 |
| USART2 TX      | PD5  | PA2  | PA2  |
| USART2 RX      | PD6  | PA3  | PA3  |
| USART3 TX      | PB10 | PB10 | /    |
| USART3 RX      | PB11 | PB11 | /    |
| USART4 TX      | PC10 | PA0  | /    |
| USART4 RX      | PC11 | PA1  | /    |
| USART5 TX      | PC12 | /    | /    |
| USART5 RX      | PD2  | /    | /    |
| USART6 TX      | /    | PC6  | PC6  |
| USART6 RX      | /    | PC7  | PC7  |
| HCSR04 Trigger | PB0  | /    | /    |
| HCSR04 Echo    | PB1  | /    | /    |

_Note: `/` means not supported or configured._


## Setup

### Flash

- Connect board using `ST-LINK` labelled port (usually Mini-B USB connector)

**Linux & Mac:**
- Install [stlink](https://github.com/texane/stlink)
- `$ st-flash --format ihex write program.hex`

**Windows:**
- Install and use [STM32 ST-LINK utility](https://www.st.com/en/development-tools/stsw-link004.html)

### Connect Configurator

- Connect board using `USER` labelled port (usually Micro-AB or secondary Mini-B USB connector)


## Example Assembly

As an example of full assembly, [feriCopterV1](https://github.com/Nailim/feriCopterV1) is completely custom built drone as part of a student project. Includes simple 3D printed frame, parts list, assembly and pin connection instructions, providing a great entry-point to anyone who wants to assemble their own little drone cheaply and learn more about how everything works.

It is based on `STM32F3DISCOVERY` target, required changes in Configurator are noted below. Also refer to [Target Pins](#pins) table for other Discovery boards. Additional non-pin changes are noted below.

**Configurator:**

| Setting  | Value |
| ---      | ---   |
| Mixer    | Quad X 1234 |
| Receiver | PPM RX input |

### F411 Changes

- Inversed Front/Back (Front is Back and Back is Front).
- Frame requires minor incision to fit different jumper location.
