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

typedef struct {
    GPIO_TypeDef* gpio;
    uint16_t pinpos;
    uint16_t pin;
    gpio_config_t gpio_config_INPUT;
    gpio_config_t gpio_config_OUTPUT;
} escHardware_t;

extern uint8_t escSelected;

bool isEscHi(uint8_t selEsc);
bool isEscLo(uint8_t selEsc);
void setEscHi(uint8_t selEsc);
void setEscLo(uint8_t selEsc);
void setEscInput(uint8_t selEsc);
void setEscOutput(uint8_t selEsc);

#define ESC_IS_HI isEscHi(escSelected)
#define ESC_IS_LO isEscLo(escSelected)
#define ESC_SET_HI setEscHi(escSelected)
#define ESC_SET_LO setEscLo(escSelected)
#define ESC_INPUT setEscInput(escSelected)
#define ESC_OUTPUT setEscOutput(escSelected)

typedef struct ioMem_s {
    uint16_t len;
    uint16_t addr;
    uint8_t *data;
} ioMem_t;
