/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _TRANSLATIONS_H_
#define _TRANSLATIONS_H_

#if defined(USE_OSD_HD)                         // HD screen
  #define TR2(x, y) y
#elif defined(USE_OSD) || defined(USE_OSD_SD)   // PAL or NTCS screen
  #define TR2(x, y) x
#else
  #define TR2(x, y) x
#endif

#include "locales/untranslated.h"

#if defined(TRANSLATIONS_CZ)
  #include "locales/cz.h"
#elif defined(TRANSLATIONS_DA)
  #include "locales/da.h"
#elif defined(TRANSLATIONS_ES)
  #include "locales/es.h"
#elif defined(TRANSLATIONS_FR)
  #include "locales/fr.h"
#elif defined(TRANSLATIONS_NL)
  #include "locales/nl.h"
#else
  #include "locales/en.h"
#endif

// extern const char* const STR_OFFON[];
// extern const char STR_ALT[]=TR_ALT;

extern const char STR_VISUAL_BEEP[];

extern const char STR_OSD_ARMED[];
extern const char STR_OSD_STATS[];

extern const char STR_OSDW_CRASH_FLIP_WARNING[];
extern const char STR_OSDW_BEACON_ON[];
extern const char STR_OSDW_ARM_IN[];
extern const char STR_OSDW_FAIL_SAFE[];
extern const char STR_OSDW_CRASH_FLIP[];
extern const char STR_OSDW_LAUNCH_ANGEL[];
extern const char STR_OSDW_LAUNCH[];
extern const char STR_OSDW_RSSI_LOW[];
extern const char STR_OSDW_RSSI_DBM[];
extern const char STR_OSDW_RSNR_LOW[];
extern const char STR_OSDW_LINK_QUA[];
extern const char STR_OSDW_LAND_NOW[];
extern const char STR_OSDW_RESCUE_NA[];
extern const char STR_OSDW_RESCUE_OFF[];
extern const char STR_OSDW_HEADFREE[];
extern const char STR_OSDW_CORE_TEMP[];
extern const char STR_OSDW_LOW_BATT[];
extern const char STR_OSDW_RCSMOOTHING[];
extern const char STR_OSDW_OVER_CAP[];
extern const char STR_OSDW_BATT_CONTINUE[];
extern const char STR_OSDW_BATT_BELOW_FULL[];

extern const char STR_OSDE_DISARMED[];
extern const char STR_OSDE_UP[];
extern const char STR_OSDE_DOWN[];
extern const char STR_OSDE_GPS_WEST[];
extern const char STR_OSDE_GPS_EAST[];
extern const char STR_OSDE_GPS_SOUTH[];
extern const char STR_OSDE_GPS_NORTH[];
extern const char STR_OSDE_PILOT_NAME[];
extern const char STR_OSDE_RATE[];
extern const char STR_OSDE_PID[];
extern const char STR_OSDE_OID[];
extern const char STR_OSDE_ESC_SENSOR[];
extern const char STR_ODSE_ESC_TELEMETRY[];

extern const char STR_ODSE_FLYMODE_FAILSAFE[];
extern const char STR_ODSE_FLYMODE_RESCUE[];
extern const char STR_ODSE_FLYMODE_HEAD[];
extern const char STR_ODSE_FLYMODE_ANGL[];
extern const char STR_ODSE_FLYMODE_HOR[];
extern const char STR_ODSE_FLYMODE_ATRN[];
extern const char STR_ODSE_FLYMODE_AIR[];
extern const char STR_ODSE_FLYMODE_ACRO[];

extern const char STR_ODSE_ELEMENT_PIT[];
extern const char STR_ODSE_ELEMENT_ROL[];
extern const char STR_ODSE_ELEMENT_YAW[];

extern const char STR_OSDE_READY[];

extern const char STR_CLI_ERROR_INVALID_NAME[];
extern const char STR_CLI_ERROR_MESSAGE[];

extern const char STR_CLI_DSHOT_READ[];
extern const char STR_CLI_DSHOT_INVALID_PKT[];
extern const char STR_CLI_DSHOT_DIR_CHANGE[];
extern const char STR_CLI_DSHOT_HEAD1[];
extern const char STR_CLI_DSHOT_LINE1[];
extern const char STR_CLI_DSHOT_HEAD2[];
extern const char STR_CLI_DSHOT_LINE2[];
extern const char STR_CLI_DSHOT_NO_TELEM[];
extern const char STR_CLI_STORAGE_NOTFOUND[];
extern const char STR_CLI_NO_MATCH[];
extern const char STR_CLI_ERROR_FOUND[];
extern const char STR_CLI_FIX_ERROR[];

extern const char STR_CLI_STATUS_MCU[];
extern const char STR_CLI_STATUS_VREF[];
extern const char STR_CLI_STATUS_STACK_SIZE[];
extern const char STR_CLI_STATUS_STACK_USED[];
extern const char STR_CLI_STATUS_CONFIGURATION[];
extern const char STR_CLI_STATUS_DEVICES[];
extern const char STR_CLI_STATUS_SPI[];
extern const char STR_CLI_STATUS_I2C[];
extern const char STR_CLI_STATUS_I2C_ERRORS[];
extern const char STR_CLI_STATUS_GYRO_DETECT[];
extern const char STR_CLI_STATUS_GYRO[];
extern const char STR_CLI_STATUS_GYRO_LOCKED[];
extern const char STR_CLI_STATUS_GYRO_DMA[];
extern const char STR_CLI_STATUS_GYRO_SHARED[];
extern const char STR_CLI_STATUS_OSD[];
extern const char STR_CLI_STATUS_BUILD_KEY[];
extern const char STR_CLI_STATUS_SYSTEM_UPTIME[];
extern const char STR_CLI_STATUS_TIME_CURRENT[];
extern const char STR_CLI_STATUS_CPU_INFO[];
extern const char STR_CLI_STATUS_BATTERY[];
extern const char STR_CLI_STATUS_FLASH[];

extern const char STR_CLI_TASK_LATE_LIST[];
extern const char STR_CLI_TASK_LIST[];
extern const char STR_CLI_TASK_MINIMAL[];
extern const char STR_CLI_TASK_RX_CHECK[];
extern const char STR_CLI_TASK_TOTAL[];
extern const char STR_CLI_TASK_SCHEDULER[];

extern const char STR_CLI_VERSION_HAS_CONFIG[];
extern const char STR_CLI_VERSION_NO_CONFIG[];
extern const char STR_CLI_VERSION_INFO[];

extern const char STR_CLI_NONE[];
extern const char STR_CLI_TIMER_FORMAT[];
extern const char STR_CLI_TIMER_EMPTY[];
extern const char STR_CLI_TIMERS[];
extern const char STR_CLI_TIMERS_ACTIVE[];
extern const char STR_CLI_TIMERS_SHORT[];
extern const char STR_CLI_TIMERS_FREE[];

extern const char STR_CLI_STATUS_ARM_DISABLE[];

extern const char STR_CLI_DSHOT_TEL_INFO[];
extern const char STR_CLI_MSC_TIMEZONE[];
extern const char STR_CLI_MSC_RESTART[];
extern const char STR_CLI_MSC_REBOOT[];
extern const char STR_CLI_PROCESS[];
extern const char STR_CLI_ENTER_LONG[];
extern const char STR_CLI_ENTER_SHORT[];

#endif // _TRANSLATIONS_H_

