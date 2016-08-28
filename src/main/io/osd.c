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
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"
#include "version.h"
#include "scheduler/scheduler.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"
#include "common/typeconversion.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/sdcard.h"

#include "rx/rx.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/osd.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build_config.h"
#include "debug.h"

#ifdef OSD

#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"

#ifdef USE_RTC6705
#include "drivers/vtx_soft_spi_rtc6705.h"
#endif

#include "common/printf.h"

#define MICROSECONDS_IN_A_SECOND (1000 * 1000)

#define OSD_UPDATE_FREQUENCY (MICROSECONDS_IN_A_SECOND / 5)
#define OSD_LINE_LENGTH 30

#define STICKMIN 10
#define STICKMAX 90

/** Artificial Horizon limits **/
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AHISIDEBARWIDTHPOSITION 7
#define AHISIDEBARHEIGHTPOSITION 3

static uint32_t next_osd_update_at = 0;
static uint32_t armed_seconds      = 0;
static uint32_t armed_at           = 0;
static uint8_t armed               = 0;

static uint8_t current_page        = 0;
static uint8_t sticks[]            = {0,0,0,0};

static uint8_t cursor_row          = 255;
static uint8_t cursor_col          = 0;
static uint8_t in_menu             = 0;
static uint8_t activating_menu     = 0;

static char string_buffer[30];

extern uint16_t rssi;

enum {
    MENU_VALUE_DECREASE = 0,
    MENU_VALUE_INCREASE,
};

#ifdef USE_RTC6705

static const char *vtx_bands[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

void update_vtx_band(int value_change_direction, uint8_t col) {
    UNUSED(col);
    if (value_change_direction) {
        if (current_vtx_channel < (BANDS_NUMBER * CHANNELS_PER_BAND - CHANNELS_PER_BAND))
            current_vtx_channel += CHANNELS_PER_BAND;
    } else {
        if (current_vtx_channel >= CHANNELS_PER_BAND)
            current_vtx_channel -= CHANNELS_PER_BAND;
    }
}

void print_vtx_band(uint16_t pos, uint8_t col) {
    UNUSED(col);
    sprintf(string_buffer,  "%s", vtx_bands[current_vtx_channel / CHANNELS_PER_BAND]);
    max7456_write_string(string_buffer, pos);
}

void update_vtx_channel(int value_change_direction, uint8_t col) {
    UNUSED(col);
    if (value_change_direction) {
        if ((current_vtx_channel % CHANNELS_PER_BAND) < (CHANNELS_PER_BAND - 1))
            current_vtx_channel++;
    } else {
        if ((current_vtx_channel % CHANNELS_PER_BAND) > 0)
            current_vtx_channel--;
    }
}

void print_vtx_channel(uint16_t pos, uint8_t col) {
    UNUSED(col);
    sprintf(string_buffer,  "%d", current_vtx_channel % CHANNELS_PER_BAND + 1);
    max7456_write_string(string_buffer, pos);
}

void print_vtx_freq(uint16_t pos, uint8_t col) {
    UNUSED(col);
    sprintf(string_buffer,  "%d M", vtx_freq[current_vtx_channel]);
    max7456_write_string(string_buffer, pos);
}

void update_vtx_power(int value_change_direction, uint8_t col) {
    UNUSED(col);
    if (value_change_direction) {
        masterConfig.vtx_power = 0;
    } else {
        masterConfig.vtx_power = 1;
    }
    rtc6705_soft_spi_set_rf_power(masterConfig.vtx_power);
}

void print_vtx_power(uint16_t pos, uint8_t col) {
    UNUSED(col);
    sprintf(string_buffer, "%s", masterConfig.vtx_power ? "25MW" : "200MW");
    max7456_write_string(string_buffer, pos);
}

#endif

void print_pid(uint16_t pos, uint8_t col, int pid_term) {
    switch(col) {
        case 0:
            sprintf(string_buffer, "%d", currentProfile->pidProfile.P8[pid_term]);
            break;
        case 1:
            sprintf(string_buffer, "%d", currentProfile->pidProfile.I8[pid_term]);
            break;
        case 2:
            sprintf(string_buffer, "%d", currentProfile->pidProfile.D8[pid_term]);
            break;
        default:
            return;
    }
    max7456_write_string(string_buffer, pos);
}

void print_roll_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, ROLL);
}

void print_pitch_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, PITCH);
}

void print_yaw_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, YAW);
}

void print_roll_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_ROLL]);
        max7456_write_string(string_buffer, pos);
    }
}

void print_pitch_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_PITCH]);
        max7456_write_string(string_buffer, pos);
    }
}

void print_yaw_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_YAW]);
        max7456_write_string(string_buffer, pos);
    }
}

void print_tpa_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        sprintf(string_buffer, "%d", currentControlRateProfile->dynThrPID);
        max7456_write_string(string_buffer, pos);
    }
}

void print_tpa_brkpt(uint16_t pos, uint8_t col) {
    if (col == 0) {
        sprintf(string_buffer, "%d", currentControlRateProfile->tpa_breakpoint);
        max7456_write_string(string_buffer, pos);
    }
}

void update_int_pid(int value_change_direction, uint8_t col, int pid_term) {
    void* ptr;

    switch(col) {
        case 0:
            ptr = &currentProfile->pidProfile.P8[pid_term];
            break;
        case 1:
            ptr = &currentProfile->pidProfile.I8[pid_term];
            break;
        case 2:
            ptr = &currentProfile->pidProfile.D8[pid_term];
            break;
        default:
            return;
    }

    if (value_change_direction) {
        if (*(uint8_t*)ptr < 200)
            *(uint8_t*)ptr += 1;
    } else {
        if (*(uint8_t*)ptr > 0)
            *(uint8_t*)ptr -= 1;
    }
}

void update_roll_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, ROLL);
}

void update_pitch_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, PITCH);
}

void update_yaw_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, YAW);
}

void update_roll_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_ROLL] < CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX)
            currentControlRateProfile->rates[FD_ROLL]++;
    } else {
        if (currentControlRateProfile->rates[FD_ROLL] > 0)
            currentControlRateProfile->rates[FD_ROLL]--;
    }
}

void update_pitch_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_PITCH] < CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX)
            currentControlRateProfile->rates[FD_PITCH]++;
    } else {
        if (currentControlRateProfile->rates[FD_PITCH] > 0)
            currentControlRateProfile->rates[FD_PITCH]--;
    }
}

void update_yaw_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_YAW] < CONTROL_RATE_CONFIG_YAW_RATE_MAX)
            currentControlRateProfile->rates[FD_YAW]++;
    } else {
        if (currentControlRateProfile->rates[FD_YAW] > 0)
            currentControlRateProfile->rates[FD_YAW]--;
    }
}

void update_tpa_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->dynThrPID < CONTROL_RATE_CONFIG_TPA_MAX)
            currentControlRateProfile->dynThrPID++;
    } else {
        if (currentControlRateProfile->dynThrPID > 0)
            currentControlRateProfile->dynThrPID--;
    }
}

void update_tpa_brkpt(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->tpa_breakpoint < PWM_RANGE_MAX)
            currentControlRateProfile->tpa_breakpoint++;
    } else {
        if (currentControlRateProfile->tpa_breakpoint > PWM_RANGE_MIN)
            currentControlRateProfile->tpa_breakpoint--;
    }
}

void print_average_system_load(uint16_t pos, uint8_t col) {
    UNUSED(col);

    sprintf(string_buffer, "%d", averageSystemLoadPercent);
    max7456_write_string(string_buffer, pos);
}

void print_batt_voltage(uint16_t pos, uint8_t col) {
    UNUSED(col);

    sprintf(string_buffer, "%d.%1d", vbat / 10, vbat % 10);
    max7456_write_string(string_buffer, pos);
}

osd_page_t menu_pages[] = {
    {
        .title = "STATUS",
        .cols_number = 1,
        .rows_number = 2,
        .cols = {
            {
                .title = NULL,
                .x_pos = 15
            }
        },
        .rows = {
            {
                .title  = "AVG LOAD",
                .update = NULL,
                .print  = print_average_system_load
            },
            {
                .title  = "BATT",
                .update = NULL,
                .print  = print_batt_voltage
            },
        }
    },
#ifdef USE_RTC6705
    {
        .title       = "VTX SETTINGS",
        .cols_number = 1,
        .rows_number = 4,
        .cols = {
            {
                .title = NULL,
                .x_pos = 15
            }
        },
        .rows = {
            {
                .title  = "BAND",
                .update = update_vtx_band,
                .print  = print_vtx_band
            },
            {
                .title  = "CHANNEL",
                .update = update_vtx_channel,
                .print  = print_vtx_channel
            },
            {
                .title  = "FREQUENCY",
                .update = NULL,
                .print  = print_vtx_freq
            },
            {
                .title  = "POWER",
                .update = update_vtx_power,
                .print  = print_vtx_power
            }
        }
    },
#endif
    {
        .title       = "PID SETTINGS",
        .cols_number = 3,
        .rows_number = 8,
        .cols = {
            {
                .title = "P",
                .x_pos = 13
            },
            {
                .title = "I",
                .x_pos = 19
            },
            {
                .title = "D",
                .x_pos = 25
            }
        },
        .rows = {
            {
                .title  = "ROLL",
                .update = update_roll_pid,
                .print  = print_roll_pid
            },
            {
                .title  = "PITCH",
                .update = update_pitch_pid,
                .print  = print_pitch_pid
            },
            {
                .title  = "YAW",
                .update = update_yaw_pid,
                .print  = print_yaw_pid
            },
            {
                .title  = "ROLL RATE",
                .update = update_roll_rate,
                .print  = print_roll_rate
            },
            {
                .title  = "PITCH RATE",
                .update = update_pitch_rate,
                .print  = print_pitch_rate
            },
            {
                .title  = "YAW RATE",
                .update = update_yaw_rate,
                .print  = print_yaw_rate
            },
            {
                .title  = "TPA RATE",
                .update = update_tpa_rate,
                .print  = print_tpa_rate
            },
            {
                .title  = "TPA BRKPT",
                .update = update_tpa_brkpt,
                .print  = print_tpa_brkpt
            },
        }
    },
};

void show_menu(void) {
    uint8_t line = 1;
    uint16_t pos;
    osd_col_t *col;
    osd_row_t *row;
    int16_t cursor_x = 0;
    int16_t cursor_y = 0;

    if (activating_menu) {
        if (sticks[YAW] < 60 && sticks[YAW] > 40 && sticks[PITCH] > 40 && sticks[PITCH] < 60 && sticks[ROLL] > 40 && sticks[ROLL] < 60)
            activating_menu = false;
        else
            return;
    }

    if (sticks[YAW] > STICKMAX && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMIN && sticks[PITCH] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            switch(cursor_col) {
                case 0:
                    in_menu = false;
                    break;
                case 1:
                    in_menu = false;
#ifdef USE_RTC6705
                    if (masterConfig.vtx_channel != current_vtx_channel) {
                        masterConfig.vtx_channel = current_vtx_channel;
                        rtc6705_soft_spi_set_channel(vtx_freq[current_vtx_channel]);
                    }
#endif
                    writeEEPROM();
                    break;
                case 2:
                    if (current_page < (sizeof(menu_pages) / sizeof(osd_page_t) - 1))
                        current_page++;
                    else
                        current_page = 0;
            }
        } else {
            if (menu_pages[current_page].rows[cursor_row].update)
                menu_pages[current_page].rows[cursor_row].update(MENU_VALUE_INCREASE, cursor_col);
        }
    }

    if (sticks[YAW] < STICKMIN && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMIN && sticks[PITCH] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            if (cursor_col == 2 && current_page > 0) {
                current_page--;
            }
        } else {
            if (menu_pages[current_page].rows[cursor_row].update)
                menu_pages[current_page].rows[cursor_row].update(MENU_VALUE_DECREASE, cursor_col);
        }
    }

    if (sticks[PITCH] > STICKMAX && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            cursor_row = menu_pages[current_page].rows_number - 1;
            cursor_col = 0;
        } else {
            if (cursor_row > 0)
                cursor_row--;
        }
    }
    if (sticks[PITCH] < STICKMIN && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row < (menu_pages[current_page].rows_number - 1))
            cursor_row++;
        else
            cursor_row = 255;
    }
    if (sticks[ROLL] > STICKMAX && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            if (cursor_col < 2)
                cursor_col++;
        } else {
            if (cursor_col < (menu_pages[current_page].cols_number - 1))
                cursor_col++;
        }
    }
    if (sticks[ROLL] < STICKMIN && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_col > 0)
            cursor_col--;
    }

    if (cursor_row > MAX_MENU_ROWS) {
        cursor_row = 255;
        cursor_y = -1;
        switch(cursor_col) {
            case 0:
                cursor_x = 0;
                break;
            case 1:
                cursor_x = 9;
                break;
            case 2:
                cursor_x = 23;
                break;
            default:
                cursor_x = 0;
        }
    }

    sprintf(string_buffer, "EXIT     SAVE+EXIT     PAGE");
    max7456_write_string(string_buffer, -29);

    pos = (OSD_LINE_LENGTH - strlen(menu_pages[current_page].title)) / 2 + line * OSD_LINE_LENGTH;
    sprintf(string_buffer, "%s", menu_pages[current_page].title);
    max7456_write_string(string_buffer, pos);

    line += 2;

    for (int i = 0; i < menu_pages[current_page].cols_number; i++){
        col = &menu_pages[current_page].cols[i];
        if (cursor_col == i && cursor_row < MAX_MENU_ROWS)
            cursor_x = col->x_pos - 1;

        if (col->title) {
            sprintf(string_buffer, "%s", col->title);
            max7456_write_string(string_buffer, line * OSD_LINE_LENGTH + col->x_pos);
        }
    }

    line++;
    for (int i = 0; i < menu_pages[current_page].rows_number; i++) {
        row = &menu_pages[current_page].rows[i];
        if (cursor_row == i)
            cursor_y = line;

        sprintf(string_buffer, "%s", row->title);
        max7456_write_string(string_buffer, line * OSD_LINE_LENGTH + 1);
        for (int j = 0; j < menu_pages[current_page].cols_number; j++) {
            col = &menu_pages[current_page].cols[j];
            row->print(line * OSD_LINE_LENGTH + col->x_pos, j);
        }
        line++;
    }

    max7456_write_string(">", cursor_x + cursor_y * OSD_LINE_LENGTH);
}

// Write the artifical horizon to the screen buffer
void osdDrawArtificialHorizon(int rollAngle, int pitchAngle, uint8_t show_sidebars) {
    uint16_t position = 194;
    MAX7456_CHAR_TYPE *screenBuffer = max7456_get_screen_buffer();

    if (pitchAngle > AHIPITCHMAX)
        pitchAngle = AHIPITCHMAX;
    if (pitchAngle < -AHIPITCHMAX)
        pitchAngle = -AHIPITCHMAX;
    if (rollAngle > AHIROLLMAX)
        rollAngle = AHIROLLMAX;
    if (rollAngle < -AHIROLLMAX)
        rollAngle = -AHIROLLMAX;

    for (uint8_t X = 0; X <= 8; X++) {
        if (X == 4)
            X = 5;
        int Y = (rollAngle * (4 - X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if (Y >= 0 && Y <= 81) {
            uint16_t pos = position - 7 + LINE * (Y / 9) + 3 - 4 * LINE + X;
            screenBuffer[pos] = MAX7456_CHAR(SYM_AH_BAR9_0 + (Y % 9));
        }
    }
    screenBuffer[position - 1] = MAX7456_CHAR(SYM_AH_CENTER_LINE);
    screenBuffer[position + 1] = MAX7456_CHAR(SYM_AH_CENTER_LINE_RIGHT);
    screenBuffer[position]     = MAX7456_CHAR(SYM_AH_CENTER);

    if (show_sidebars) {
        // Draw AH sides
        int8_t hudwidth  = AHISIDEBARWIDTHPOSITION;
        int8_t hudheight = AHISIDEBARHEIGHTPOSITION;
        for (int8_t X = -hudheight; X <= hudheight; X++) {
            screenBuffer[position - hudwidth + (X * LINE)] = MAX7456_CHAR(SYM_AH_DECORATION);
            screenBuffer[position + hudwidth + (X * LINE)] = MAX7456_CHAR(SYM_AH_DECORATION);
        }
        // AH level indicators
        screenBuffer[position-hudwidth+1] =  MAX7456_CHAR(SYM_AH_LEFT);
        screenBuffer[position+hudwidth-1] =  MAX7456_CHAR(SYM_AH_RIGHT);
    }
}

void updateOsd(void)
{
    static uint8_t blink = 0;
    static uint8_t arming = 0;
    uint32_t seconds;
    char line[30];
    uint32_t now = micros();

    bool updateNow = (int32_t)(now - next_osd_update_at) >= 0L;
    if (!updateNow) {
        return;
    }
    next_osd_update_at = now + OSD_UPDATE_FREQUENCY;
    blink++;

    if (ARMING_FLAG(ARMED)) {
        if (!armed) {
            armed = true;
            armed_at = now;
            in_menu = false;
            arming = 5;
        }
    } else {
        if (armed) {
            armed = false;
            armed_seconds += ((now - armed_at) / 1000000);
        }
        for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
            sticks[channelIndex] = (constrain(rcData[channelIndex], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        }
        if (!in_menu && sticks[YAW] > STICKMAX && sticks[THROTTLE] > STICKMIN && sticks[THROTTLE] < STICKMAX && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMAX) {
            in_menu = true;
            cursor_row = 255;
            cursor_col = 2;
            activating_menu = true;
        }
    }
    if (in_menu) {
        show_menu();
    } else {
        if (batteryWarningVoltage > vbat && (blink & 1) && masterConfig.osdProfile.item_pos[OSD_VOLTAGE_WARNING] != -1) {
            max7456_write_string("LOW VOLTAGE", masterConfig.osdProfile.item_pos[OSD_VOLTAGE_WARNING]);
        }
        if (arming && (blink & 1) && masterConfig.osdProfile.item_pos[OSD_ARMED] != -1) {
            max7456_write_string("ARMED", masterConfig.osdProfile.item_pos[OSD_ARMED]);
            arming--;
        }
        if (!armed && masterConfig.osdProfile.item_pos[OSD_DISARMED] != -1) {
            max7456_write_string("DISARMED", masterConfig.osdProfile.item_pos[OSD_DISARMED]);
        }

        if (masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE] != -1) {
            line[0] = SYM_VOLT;
            sprintf(line+1, "%d.%1d", vbat / 10, vbat % 10);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]);
        }

        if (masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW] != -1) {
            line[0] = SYM_AMP;
            sprintf(line+1, "%d.%02d", amperage / 100, amperage % 100);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]);
        }

        if (masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN] != -1) {
            line[0] = SYM_MAH;
            sprintf(line+1, "%d", mAhDrawn);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]);
        }

        if (masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME] != -1) {
            for (uint8_t i = 0; i < MAX_NAME_LENGTH; i++) {
                line[i] = toupper((unsigned char)masterConfig.name[i]);
                if (masterConfig.name[i] == 0)
                    break;
            }
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]);
        }

        if (masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE] != -1) {
            line[0] = SYM_RSSI;
            sprintf(line+1, "%d", rssi / 10);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]);
        }
        if (masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS] != -1) {
            line[0] = SYM_THR;
            line[1] = SYM_THR1;
            sprintf(line+2, "%3d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]);
        }
        if (masterConfig.osdProfile.item_pos[OSD_TIMER] != -1) {
            if (armed) {
                seconds = armed_seconds + ((now-armed_at) / 1000000);
                line[0] = SYM_FLY_M;
                sprintf(line+1, " %02d:%02d", seconds / 60, seconds % 60);
            } else {
                line[0] = SYM_ON_M;
                seconds = now  / 1000000;
                sprintf(line+1, " %02d:%02d", seconds / 60, seconds % 60);
            }
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_TIMER]);
        }
        if (masterConfig.osdProfile.item_pos[OSD_CPU_LOAD] != -1) {
            print_average_system_load(masterConfig.osdProfile.item_pos[OSD_CPU_LOAD], 0);
        }
        if (masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON] != -1) {
            osdDrawArtificialHorizon(attitude.values.roll, attitude.values.pitch, masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS] != -1);
        }
    }
    max7456_draw_screen();
}

void osdInit(void)
{
    uint16_t x;
    MAX7456_CHAR_TYPE* screen_buffer = max7456_get_screen_buffer();

    max7456_init(masterConfig.osdProfile.video_system);

    // display logo and help
    x =  160;
    for (int i = 1; i < 5; i++) {
        for (int j = 3; j < 27; j++)
            if (x != 255)
                screen_buffer[(i * LINE + j)] = MAX7456_CHAR(x++);
    }
    sprintf(string_buffer, "BF VERSION: %s", FC_VERSION_STRING);
    max7456_write_string(string_buffer, LINE06 + 5);
    max7456_write_string("MENU: THRT MID", LINE07 + 7);
    max7456_write_string("YAW RIGHT", LINE08 + 13);
    max7456_write_string("PITCH UP", LINE09 + 13);
    max7456_draw_screen();
}

void resetOsdConfig(osd_profile *osdProfile)
{
    osdProfile->video_system = AUTO;
    osdProfile->item_pos[OSD_MAIN_BATT_VOLTAGE]  = -29;
    osdProfile->item_pos[OSD_RSSI_VALUE]         = -59;
    osdProfile->item_pos[OSD_TIMER]              = -39;
    osdProfile->item_pos[OSD_THROTTLE_POS]       = -9;
    osdProfile->item_pos[OSD_CPU_LOAD]           = 26;
    osdProfile->item_pos[OSD_VTX_CHANNEL]        = 1;
    osdProfile->item_pos[OSD_VOLTAGE_WARNING]    = -80;
    osdProfile->item_pos[OSD_ARMED]              = -107;
    osdProfile->item_pos[OSD_DISARMED]           = -109;
    osdProfile->item_pos[OSD_ARTIFICIAL_HORIZON] = -1;
    osdProfile->item_pos[OSD_HORIZON_SIDEBARS]   = -1;
    osdProfile->item_pos[OSD_CURRENT_DRAW]       = -23;
    osdProfile->item_pos[OSD_MAH_DRAWN]          = -18;
    osdProfile->item_pos[OSD_CRAFT_NAME]         = 1;
}
#endif
