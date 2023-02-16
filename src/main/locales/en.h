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

#define TR_THOUSAND 	            "."

// OSD
#define TR_OSD_ARMED                "ARMED"
#define TR_OSD_STATS                "--- STATS ---"

// OSD warnings
#define TR_OSDW_CRASH_FLIP_WARNING  "> CRASH FLIP <"
#define TR_OSDW_BEACON_ON           " BEACON ON"
#define TR_OSDW_ARM_IN              "ARM IN %d.%d"
#define TR_OSDW_FAIL_SAFE           "FAIL SAFE"
#define TR_OSDW_CRASH_FLIP          "CRASH FLIP SWITCH"
#define TR_OSDW_LAUNCH_ANGEL        "LAUNCH %d"
#define TR_OSDW_LAUNCH              "LAUNCH"
#define TR_OSDW_RSSI_LOW            "RSSI LOW"
#define TR_OSDW_RSSI_DBM            "RSSI DBM"
#define TR_OSDW_RSNR_LOW            "RSNR LOW"
#define TR_OSDW_LINK_QUA            "LINK QUALITY"
#define TR_OSDW_LAND_NOW            " LAND NOW"
#define TR_OSDW_RESCUE_NA           "RESCUE N/A"
#define TR_OSDW_RESCUE_OFF          "RESCUE OFF"
#define TR_OSDW_HEADFREE            "HEADFREE"
#define TR_OSDW_CORE_TEMP           "CORE %c: %3d%c"
#define TR_OSDW_LOW_BATT            "LOW BATTERY"
#define TR_OSDW_RCSMOOTHING         "RCSMOOTHING"
#define TR_OSDW_OVER_CAP            "OVER CAP"
#define TR_OSDW_BATT_CONTINUE       "BATTERY CONTINUE"
#define TR_OSDW_BATT_BELOW_FULL     "BATT < FULL"

// OSD elements
#define TR_OSDE_DISARMED            "DISARMED"
#define TR_OSDE_UP                  "U"
#define TR_OSDE_DOWN                "D"
#define TR_OSDE_GPS_WEST            "W"
#define TR_OSDE_GPS_EAST            "E"
#define TR_OSDE_GPS_SOUTH           "S"
#define TR_OSDE_GPS_NORTH           "N"
#define TR_OSDE_PILOT_NAME          "PILOT_NAME"
#define TR_OSDE_RATE                "RATE_%u"
#define TR_OSDE_PID                 "PID_%u"
#define TR_OSDE_OID                 "OSD_%u"
#define TR_OSDE_ESC_SENSOR          "E%c%3d%c"
#define TR_ODSE_ESC_TELEMETRY       "E%c"

#define TR_ODSE_FLYMODE_FAILSAFE    "!FS!"   
#define TR_ODSE_FLYMODE_RESCUE      "RESC"
#define TR_ODSE_FLYMODE_HEAD        "HEAD"
#define TR_ODSE_FLYMODE_ANGL        "ANGL"
#define TR_ODSE_FLYMODE_HOR         "HOR "
#define TR_ODSE_FLYMODE_ATRN        "ATRN"
#define TR_ODSE_FLYMODE_AIR         "AIR "
#define TR_ODSE_FLYMODE_ACRO        "ACRO"

#define TR_ODSE_ELEMENT_PIT         "PIT"
#define TR_ODSE_ELEMENT_ROL         "ROL"
#define TR_ODSE_ELEMENT_YAW         "YAW"

#define TR_ODSE_ANTIGRAVITY         "AG"

#define TR_OSDE_READY               "READY"

// CLI
#define TR_CLI_ERROR_INVALID_NAME   "INVALID NAME: %s"
#define TR_CLI_ERROR_MESSAGE        "%s CANNOT BE CHANGED. CURRENT VALUE: '%s'"

#define TR_CLI_DSHOT_READ           "Dshot reads: %u"
#define TR_CLI_DSHOT_INVALID_PKT    "Dshot invalid pkts: %u"
#define TR_CLI_DSHOT_DIR_CHANGE     "Dshot directionChange cycles: %u, micros: %u"
#define TR_CLI_DSHOT_HEAD1          "Motor    Type   eRPM    RPM     Hz Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define TR_CLI_DSHOT_LINE1          "=====  ====== ====== ====== ====== ======= ====== ====== ====== ====== ====== ====== ======"
#define TR_CLI_DSHOT_HEAD2          "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define TR_CLI_DSHOT_LINE2          "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======"
#define TR_CLI_DSHOT_NO_TELEM       "Dshot telemetry not enabled"
#define TR_CLI_STORAGE_NOTFOUND     "Storage not present or failed to initialize!"
#define TR_CLI_NO_MATCH             "NO MATCHES FOR '%s'"
#define TR_CLI_ERROR_FOUND          "ERRORS WERE DETECTED - PLEASE REVIEW BEFORE CONTINUING"
#define TR_CLI_FIX_ERROR            "PLEASE FIX ERRORS THEN 'SAVE'"

#define TR_CLI_STATUS_MCU           "MCU %s Clock=%dMHz"
#define TR_CLI_STATUS_VREF          ", Vref=%d.%2dV, Core temp=%ddegC"
#define TR_CLI_STATUS_STACK_SIZE    "Stack size: %d, Stack address: 0x%x"
#define TR_CLI_STATUS_STACK_USED    ", Stack used: %d"
#define TR_CLI_STATUS_CONFIGURATION "Configuration: %s, size: %d, max available: %d"
#define TR_CLI_STATUS_DEVICES       "Devices detected:"
#define TR_CLI_STATUS_SPI           " SPI:%d"
#define TR_CLI_STATUS_I2C           " I2C:%d"
#define TR_CLI_STATUS_I2C_ERRORS    "I2C Errors: %d"
#define TR_CLI_STATUS_GYRO_DETECT   "Gyros detected:"
#define TR_CLI_STATUS_GYRO          " gyro %d"
#define TR_CLI_STATUS_GYRO_LOCKED   " locked"
#define TR_CLI_STATUS_GYRO_DMA      " dma"
#define TR_CLI_STATUS_GYRO_SHARED   " shared"
#define TR_CLI_STATUS_OSD           "OSD: %s (%u x %u)"
#define TR_CLI_STATUS_BUILD_KEY     "BUILD KEY: %s"
#define TR_CLI_STATUS_SYSTEM_UPTIME "System Uptime: %d seconds"
#define TR_CLI_STATUS_TIME_CURRENT  ", Current Time: %s"
#define TR_CLI_STATUS_CPU_INFO      "CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d"
#define TR_CLI_STATUS_BATTERY       "Voltage: %d * 0.01V (%dS battery - %s)"
#define TR_CLI_STATUS_FLASH         "FLASH: JEDEC ID=0x%08x %uM"

#define TR_CLI_TASK_LATE_LIST       "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us"
#define TR_CLI_TASK_LIST            "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms"
#define TR_CLI_TASK_MINIMAL         "Task list"
#define TR_CLI_TASK_RX_CHECK        "RX Check Function %19d %7d %25d"
#define TR_CLI_TASK_TOTAL           "Total (excluding SERIAL) %33d.%1d%%"
#define TR_CLI_TASK_SCHEDULER       "Scheduler start cycles %d guard cycles %d"

#define TR_CLI_VERSION_HAS_CONFIG   "config: YES"
#define TR_CLI_VERSION_NO_CONFIG    "NO CONFIG FOUND"
#define TR_CLI_VERSION_INFO         "# board: manufacturer_id: %s, board_name: %s"

#define TR_CLI_NONE                 "NONE"
#define TR_CLI_TIMER_FORMAT         "timer %c%02d AF%d"
#define TR_CLI_TIMER_EMPTY          "timer %c%02d NONE"
#define TR_CLI_TIMERS               "Timers:"
#define TR_CLI_TIMERS_ACTIVE        "Currently active Timers:"
#define TR_CLI_TIMERS_SHORT         "TIM%d:"
#define TR_CLI_TIMERS_FREE          " FREE"

#define TR_CLI_STATUS_ARM_DISABLE   "Arming disable flags:"

#define TR_CLI_DSHOT_TEL_INFO       " NO DATA"
#define TR_CLI_MSC_TIMEZONE         "INVALID TIMEZONE OFFSET"
#define TR_CLI_MSC_RESTART          "Restarting in mass storage mode"
#define TR_CLI_MSC_REBOOT           "\r\nRebooting"
#define TR_CLI_PROCESS              "UNKNOWN COMMAND, TRY 'HELP'"
#define TR_CLI_ENTER_LONG           "\r\nEntering CLI Mode, type 'exit' to return, or 'help'"
#define TR_CLI_ENTER_SHORT          "\r\nCLI"
