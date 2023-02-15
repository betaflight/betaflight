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

// DK translations author: HThuren <thuren.henrik@gmail.com>

#define TR_THOUSAND 	            "."

// OSD
#define TR_OSD_ARMED                TR2("* AKTIV *", "*** AKTIVERET ***")
#define TR_OSD_STATS                TR2("- STATISTIK -", "--- STATISTIK ---")

// OSD warnings
#define TR_OSDW_CRASH_FLIP_WARNING  "> CRASH FLIP <"
#define TR_OSDW_BEACON_ON           " BEACON ON"
#define TR_OSDW_ARM_IN              "ARM IN %d.%d"
#define TR_OSDW_FAIL_SAFE           "FEJLSIKRING"
#define TR_OSDW_CRASH_FLIP          "CRASH FLIP SWITCH"
#define TR_OSDW_LAUNCH_ANGEL        "LAUNCH %d"
#define TR_OSDW_LAUNCH              "LAUNCH"
#define TR_OSDW_RSSI_LOW            "RSSI LOW"
#define TR_OSDW_RSSI_DBM            "RSSI DBM"
#define TR_OSDW_RSNR_LOW            "RSNR LOW"
#define TR_OSDW_LINK_QUA            "LINK QUALITY"
#define TR_OSDW_LAND_NOW            " LAND NU"
#define TR_OSDW_RESCUE_NA           TR2("FEJLSIKRING N/A", "FEJLSIKRING IKKE MULIG")
#define TR_OSDW_RESCUE_OFF          "FEJLSIKRING FRA"
#define TR_OSDW_HEADFREE            "HEADFREE"
#define TR_OSDW_CORE_TEMP           "CORE %c: %3d%c"
#define TR_OSDW_LOW_BATT            "BATTERI LAVT"
#define TR_OSDW_RCSMOOTHING         "RCSMOOTHING"
#define TR_OSDW_OVER_CAP            "OVER CAP"
#define TR_OSDW_BATT_CONTINUE       "BATTERI FORTSÆT"
#define TR_OSDW_BATT_BELOW_FULL     "BATTERI < FULDT"

// OSD elements
#define TR_OSDE_DISARMED            TR2("EJ AKTIV", "*** DEAKTIVERET ***")
#define TR_OSDE_UP                  "O"
#define TR_OSDE_DOWN                "N"
#define TR_OSDE_GPS_WEST            "V"
#define TR_OSDE_GPS_EAST            "Ø"
#define TR_OSDE_GPS_SOUTH           "S"
#define TR_OSDE_GPS_NORTH           "N"
#define TR_OSDE_PILOT_NAME          "PILOT_NAVN"
#define TR_OSDE_RATE                TR2("HAST_%u", "HASTIGHED_%u")
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

#define TR_ODSE_ELEMENT_PIT         "HØJ"
#define TR_ODSE_ELEMENT_ROL         "KRÆ"
#define TR_ODSE_ELEMENT_YAW         "SID"

#define TR_OSDE_READY               "KLAR"

// CLI
#define TR_CLI_ERROR_INVALID_NAME   "UGYLDIGT NAVN: %s"
#define TR_CLI_ERROR_MESSAGE        "%s KAN IKKE AENDRES. AKTUEL VAERDI: '%s'"

#define TR_CLI_DSHOT_READ           "Dshot laest: %u"
#define TR_CLI_DSHOT_INVALID_PKT    "Dshot ugyldig pakker: %u"
#define TR_CLI_DSHOT_DIR_CHANGE     "Dshot retning  Skift  cyklus: %u, mikro:  %u"
#define TR_CLI_DSHOT_HEAD1          "Motor    Type   eRPM    RPM     Hz Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define TR_CLI_DSHOT_LINE1          "=====  ====== ====== ====== ====== ======= ====== ====== ====== ====== ====== ====== ======"
#define TR_CLI_DSHOT_HEAD2          "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define TR_CLI_DSHOT_LINE2          "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======"
#define TR_CLI_DSHOT_NO_TELEM       "Dshot telemetri ikke aktiv"
#define TR_CLI_STORAGE_NOTFOUND     "Masselager findes ikke eller kan ikke aktiveres!"
#define TR_CLI_NO_MATCH             "INGEN MATCH FOR '%s'"
#define TR_CLI_ERROR_FOUND          "FEJL ER FUNDET - UNDERSOEG INDEN DU FORTSAETTER"
#define TR_CLI_FIX_ERROR            "RET FEJL, HEREFTER 'SAVE'"

#define TR_CLI_STATUS_MCU           "MCU %s klokke=%dMHz"
#define TR_CLI_STATUS_VREF          ", Vref=%d.%2dV, kerne temp=%d C"
#define TR_CLI_STATUS_STACK_SIZE    "Stak stoerrelse: %d, Stak adresse: 0x%x"
#define TR_CLI_STATUS_STACK_USED    ", Stak brugt: %d"
#define TR_CLI_STATUS_CONFIGURATION "Konfiguration: %s, stoerrelse: %d, max anvendeligt: %d"
#define TR_CLI_STATUS_DEVICES       "Enheder opdaget:"
#define TR_CLI_STATUS_SPI           " SPI:%d"
#define TR_CLI_STATUS_I2C           " I2C:%d"
#define TR_CLI_STATUS_I2C_ERRORS    "I2C fejl: %d"
#define TR_CLI_STATUS_GYRO_DETECT   "Gyro opdaget:"
#define TR_CLI_STATUS_GYRO          " gyro %d"
#define TR_CLI_STATUS_GYRO_LOCKED   " laast"
#define TR_CLI_STATUS_GYRO_DMA      " dma"
#define TR_CLI_STATUS_GYRO_SHARED   " delt"
#define TR_CLI_STATUS_OSD           "OSD: %s (%u x %u)"
#define TR_CLI_STATUS_BUILD_KEY     "BUILD KEY: %s"
#define TR_CLI_STATUS_SYSTEM_UPTIME "System oppetid: %d sek."
#define TR_CLI_STATUS_TIME_CURRENT  ", aktuel tid: %s"
#define TR_CLI_STATUS_CPU_INFO      "CPU:%d%%, cyklus tid: %d, GYRO hast.: %d, RX hast.: %d, System hast.: %d"
#define TR_CLI_STATUS_BATTERY       "Spaending: %d * 0.01V (%dS batteri - %s)"
#define TR_CLI_STATUS_FLASH         "FLASH: JEDEC ID=0x%08x %uM"

#define TR_CLI_TASK_LATE_LIST       "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us"
#define TR_CLI_TASK_LIST            "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms"
#define TR_CLI_TASK_MINIMAL         "Task list"
#define TR_CLI_TASK_RX_CHECK        "RX Check Function %19d %7d %25d"
#define TR_CLI_TASK_TOTAL           "Total (excluding SERIAL) %33d.%1d%%"
#define TR_CLI_TASK_SCHEDULER       "Scheduler start cycles %d guard cycles %d"

#define TR_CLI_VERSION_HAS_CONFIG   "Konfig: JA"
#define TR_CLI_VERSION_NO_CONFIG    TR2("INGEN KONFIG", "INGEN KONFIGURATION FUNDET")
#define TR_CLI_VERSION_INFO         "# board: manufacturer_id: %s, board_name: %s"

#define TR_CLI_NONE                 "INGEN"
#define TR_CLI_TIMER_FORMAT         "timer %c%02d AF%d"
#define TR_CLI_TIMER_EMPTY          "timer %c%02d INGEN"
#define TR_CLI_TIMERS               "Timers:"
#define TR_CLI_TIMERS_ACTIVE        "Aktiv tidtagning:"
#define TR_CLI_TIMERS_SHORT         "TIM%d:"
#define TR_CLI_TIMERS_FREE          " FRI"

#define TR_CLI_STATUS_ARM_DISABLE   "Aktiver ikke pga:"

#define TR_CLI_DSHOT_TEL_INFO       " INGEN DATA"
#define TR_CLI_MSC_TIMEZONE         "UGYLDIG TIDSZONE OFFSET"
#define TR_CLI_MSC_RESTART          "Skifter til masselager tilstand"
#define TR_CLI_MSC_REBOOT           "\r\nGenstarter"
#define TR_CLI_PROCESS              "UKENDT KOMMANDO, PROEV 'HELP'"
#define TR_CLI_ENTER_LONG           "\r\nAabnet i CLI tilstand, tast 'exit' for at forlade, eller 'help' for at faa hjaelp"
#define TR_CLI_ENTER_SHORT          "\r\nCLI"

