# RP2350 (PICO) Platform

Betaflight platform support for the Raspberry Pi RP2350 microcontroller
(Raspberry Pi Pico 2 and compatible boards).

Two targets are provided:

| Target    | MCU      | GPIO count | Notes                          |
|-----------|----------|------------|--------------------------------|
| `RP2350A` | RP2350A  | 30 (PA0–PA29) | SD card SPI blackbox backend |
| `RP2350B` | RP2350B  | 48 (PA0–PA47) | QSPI flash blackbox backend  |

---

## Board `config.h` reference

Each supported flight controller board has a `config.h` under
`src/config/configs/<BOARD_NAME>/`. The sections below document the
defines required for each feature area.

### ExpressLRS (ELRS) receiver via SPI

ELRS is supported using an SX1280 (2.4 GHz) radio connected over SPI.
The RP2350 has two SPI buses (`SPI0` / `SPI1`); either can be used.

#### Required defines

```c
// --- SPI bus pins (whichever bus the radio is wired to) ---
#define SPI0_SCK_PIN        PA<n>   // SPI clock
#define SPI0_SDI_PIN        PA<n>   // MISO (SX1280 → MCU)
#define SPI0_SDO_PIN        PA<n>   // MOSI (MCU → SX1280)

// --- Radio control pins ---
#define RX_SPI_INSTANCE             SPI0    // or SPI1
#define RX_SPI_CS                   PA<n>   // SX1280 chip-select (active-low)
#define RX_SPI_EXTI                 PA<n>   // SX1280 DIO1 – packet-ready interrupt
#define RX_EXPRESSLRS_SPI_BUSY_PIN  PA<n>   // SX1280 BUSY output
#define RX_EXPRESSLRS_SPI_RESET_PIN PA<n>   // SX1280 nRESET (active-low)

// --- Protocol defaults ---
#define RX_SPI_DEFAULT_PROTOCOL     RX_SPI_EXPRESSLRS
#define DEFAULT_RX_FEATURE          FEATURE_RX_SPI
```

#### Optional defines

```c
// Bind button GPIO (active-low by convention).
#define RX_SPI_BIND         PA<n>

// LED driven by the ELRS receiver driver (e.g. status LED on the radio module).
#define RX_SPI_LED          PA<n>
#define RX_SPI_LED_INVERTED         // add if the LED is active-low
```

#### RP2350-specific notes

- **`RX_EXPRESSLRS_TIMER_INSTANCE` is not used on RP2350.**  The ELRS
  tick/tock timer is implemented in `expresslrs_timer_pico.c` using the
  pico-sdk `hardware_alarm` API rather than a hardware timer peripheral.
  Do not copy this define from STM32 board configs.

- `USE_RX_SPI`, `USE_RX_EXPRESSLRS`, and `USE_RX_SX1280` are enabled by
  default via `common_pre.h`.  They do not need to be redefined in
  `config.h` unless you are selectively disabling one of them with
  `#undef`.

- SX127X (900 MHz) is not currently supported; `USE_RX_SX127X` is
  `#undef`'d in `target.h`.

#### Minimal example

```c
// SPI bus 0 wired to the radio module
#define SPI0_SCK_PIN                PA2
#define SPI0_SDI_PIN                PA4
#define SPI0_SDO_PIN                PA3

// Radio control
#define RX_SPI_INSTANCE             SPI0
#define RX_SPI_CS                   PA5
#define RX_SPI_EXTI                 PA6
#define RX_EXPRESSLRS_SPI_BUSY_PIN  PA7
#define RX_EXPRESSLRS_SPI_RESET_PIN PA8

// Protocol
#define RX_SPI_DEFAULT_PROTOCOL     RX_SPI_EXPRESSLRS
#define DEFAULT_RX_FEATURE          FEATURE_RX_SPI
```
