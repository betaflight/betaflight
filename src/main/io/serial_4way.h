/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/

#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_SERIAL_4WAY_SK_BOOTLOADER

#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3

typedef struct {
    GPIO_TypeDef* gpio;
    uint16_t pinpos;
    uint16_t pin;
    gpio_config_t gpio_config_INPUT;
    gpio_config_t gpio_config_OUTPUT;
} escHardware_t;

extern uint8_t selected_esc;

bool isEscHi(uint8_t selEsc);
bool isEscLo(uint8_t selEsc);
void setEscHi(uint8_t selEsc);
void setEscLo(uint8_t selEsc);
void setEscInput(uint8_t selEsc);
void setEscOutput(uint8_t selEsc);

#define ESC_IS_HI isEscHi(selected_esc)
#define ESC_IS_LO isEscLo(selected_esc)
#define ESC_SET_HI setEscHi(selected_esc)
#define ESC_SET_LO setEscLo(selected_esc)
#define ESC_INPUT setEscInput(selected_esc)
#define ESC_OUTPUT setEscOutput(selected_esc)

typedef struct ioMem_s {
    uint8_t D_NUM_BYTES;
    uint8_t D_FLASH_ADDR_H;
    uint8_t D_FLASH_ADDR_L;
    uint8_t *D_PTR_I;
} ioMem_t;

extern ioMem_t ioMem;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
} uint8_32_u;

//extern uint8_32_u DeviceInfo;

bool isMcuConnected(void);
uint8_t Initialize4WayInterface(void);
void Process4WayInterface(mspPort_t *mspPort, bufWriter_t *bufwriter);
void DeInitialize4WayInterface(void);
