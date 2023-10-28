/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// DK translations author: HThuren <thuren.henrik@gmail.com>

#define LOCALE                       "da"

#define STR_COMMA	                 ","
#define STR_PERIOD	                 "."
#define STR_THOUSAND 	             "."
#define STR_SECONDS                  "sekunder"

#define STR_VISUAL_BEEP              "  * * * *"
#define STR_MSP_API_NAME             "MSP API:"

// CMS
#define CMS_STARTUP_HELP_TEXT1      "MENU:GAS MIDT"
#define CMS_STARTUP_HELP_TEXT2      " + SIDEROR VENSTRE"
#define CMS_STARTUP_HELP_TEXT3      " + HOEJDE OP"

// OSD
#define STR_OSD_ARMED                TR2("* AKTIV *", "*** AKTIVERET ***")
#define STR_OSD_STATS                TR2("- STATISTIK -", "--- STATISTIK ---")

#define STR_OSD_TIM_SOURCE_1         TR2("ON TIME  ", "ON TIME  ")
#define STR_OSD_TIM_SOURCE_2         TR2("TOTAL AKT", "TOTAL AKTIVERET")
#define STR_OSD_TIM_SOURCE_3         TR2("SIDST AKT", "SIDST AKTIVERET")
#define STR_OSD_TIM_SOURCE_4         TR2("TIL/AKT  ", "TIL/AKTIVERET  ")

#define STR_OSD_STAT_MAX_ALTITUDE    TR2("MAX HOEJDE",   "MAXIMAL HOEJDE")
#define STR_OSD_STAT_MAX_SPEED       TR2("MAX FART",     "MAXIMAL FART")
#define STR_OSD_STAT_MAX_DISTANCE    TR2("MAX DISTANCE", "MAXIMAL DISTANCE")
#define STR_OSD_STAT_FLIGHT_DISTANCE "FLYVE DISTANCE"
#define STR_OSD_STAT_MIN_BATTERY_AVG "MIN GNM CELL"
#define STR_OSD_STAT_MIN_BATTERY     "MIN BATTERI"
#define STR_OSD_STAT_END_BATTERY_AVG "END GNM CELL"
#define STR_OSD_STAT_END_BATTERY     "END BATTERI"
#define STR_OSD_STAT_BATTERY_AVG     "GNM BATT CELL"
#define STR_OSD_STAT_BATTERY         "BATTERI"
#define STR_OSD_STAT_MIN_RSSI        "MIN RSSI"
#define STR_OSD_STAT_MAX_CURRENT     TR2("MAX STROEM", "MAXIMAL STROEM")
#define STR_OSD_STAT_USED_MAH        "BRUGT MAH"
#define STR_OSD_STAT_WATT_HOURS_DRAWN "BRUGT WATT TIMER"
#define STR_OSD_STAT_BLACKBOX        "BLACKBOX"
#define STR_OSD_STAT_BLACKBOX_NUMBER "BB LOG NUM"
#define STR_OSD_STAT_MAX_G_FORCE     TR2("MAX G-KRAFT",  "MAXIMAL G-KRAFT")
#define STR_OSD_STAT_MAX_ESC_TEMP    TR2("MAX ESC TEMP", "MAXIMAL ESC TEMP")
#define STR_OSD_STAT_MAX_ESC_RPM     TR2("MAX ESC RPM",  "MAXIMAL ESV RPM")
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"
#define STR_OSD_STAT_MAX_FFT         "SPIDS FFT"
#define STR_OSD_STAT_MAX_FFT_THRT    "GAS<20%"
#define STR_OSD_STAT_MIN_RSSI_DBM    "MIN RSSI DBM"
#define STR_OSD_STAT_MIN_RSNR        "MIN RSNR"
#define STR_USE_PERSISTENT_STATS     "TOTALE FLYVNINGER"
#define STR_OSD_STAT_TOTAL_TIME      "TOTALE FLYVE TID"
#define STR_OSD_STAT_TOTAL_DIST      "TOTAL DISTANCE"

// OSD warnings
#define STR_OSDW_CRASH_FLIP_WARNING  "> CRASH FLIP <"
#define STR_OSDW_BEACON_ON           " BEACON TIL"
#define STR_OSDW_ARM_IN              "ARM IN"
#define STR_OSDW_FAIL_SAFE           "FEJLSIKRING"
#define STR_OSDW_CRASH_FLIP          "CRASH FLIP SWITCH"
#define STR_OSDW_LAUNCH              "LAUNCH"
#define STR_OSDW_RSSI_LOW            "RSSI LAV"
#define STR_OSDW_RSSI_DBM            "RSSI DBM"
#define STR_OSDW_RSNR_LOW            "RSNR LAV"
#define STR_OSDW_LINK_QUA            "LINK KVALITET"
#define STR_OSDW_LAND_NOW            " LAND NU"
#define STR_OSDW_RESCUE_NA           TR2("FEJLSIKRING N/A", "FEJLSIKRING IKKE MULIG")
#define STR_OSDW_RESCUE_OFF          "FEJLSIKRING FRA"
#define STR_OSDW_HEADFREE            "HEADFREE"
#define STR_OSDW_CORE                "CORE"
#define STR_OSDW_LOW_BATT            "BATTERI LAVT"
#define STR_OSDW_RCSMOOTHING         "RCSMOOTHING"
#define STR_OSDW_OVER_CAP            "OVER CAP"
#define STR_OSDW_BATT_CONTINUE       "BATTERI FORTSÆT"
#define STR_OSDW_BATT_BELOW_FULL     "BATTERI < FULDT"

// OSD elements
#define STR_OSDE_DISARMED            TR2("EJ AKTIV", "*** DEAKTIVERET ***")
#define STR_OSDE_UP                  "O"
#define STR_OSDE_DOWN                "N"
#define STR_OSDE_GPS_WEST            "V"
#define STR_OSDE_GPS_EAST            "Ø"
#define STR_OSDE_GPS_SOUTH           "S"
#define STR_OSDE_GPS_NORTH           "N"
#define STR_OSDE_PILOT_NAME          "PILOT_NAVN"
#define STR_OSDE_RATE                TR2("HAST_", "HASTIGHED_")
#define STR_OSDE_PID                 "PID_"
#define STR_OSDE_OID                 "OID_"

#define STR_ODSE_FLYMODE_FAILSAFE    "!FS!"   
#define STR_ODSE_FLYMODE_RESCUE      "RESC"
#define STR_ODSE_FLYMODE_HEAD        "HEAD"
#define STR_ODSE_FLYMODE_ANGL        "ANGL"
#define STR_ODSE_FLYMODE_HOR         "HOR "
#define STR_ODSE_FLYMODE_ATRN        "ATRN"
#define STR_ODSE_FLYMODE_AIR         "AIR "
#define STR_ODSE_FLYMODE_ACRO        "ACRO"

#define STR_ODSE_ELEMENT_PIT         "HØJ"
#define STR_ODSE_ELEMENT_ROL         "KRÆ"
#define STR_ODSE_ELEMENT_YAW         "SID"

#define STR_ODSE_ANTIGRAVITY         "AT"        // Anti Tyndekraft

#define STR_OSDE_READY               "KLAR"

// CLI
#define STR_LOCALE_SETUP             "Sprog:"

#define STR_CLI_ERROR_INVALID_NAME   "UGYLDIGT NAVN:"
#define STR_CLI_ERROR_MESSAGE        "KAN IKKE AENDRES. AKTUEL VAERDI:"
#define STR_CLI_PARSINGFAIL          "PARSING FEJLET"
#define STR_CLI_INVALIDRESSOURCE     "UGYLDIGT RESOURCE NAVN:"
#define STR_CLI_ARG_INVALIDCOUNT     "UGYLDIGT ANTAL ARGUMENTER"
#define STR_CLI_ARG_NOTBETWEEN       "IKKE MELLEM"
#define STR_CLI_ARG_AND              "OG"
#define STR_CLI_ARG_OUTOFRANGE       "ARGUMENT UDENFOR INTERVAL"

#define STR_CLI_DSHOT_READ           "Dshot laest:"
#define STR_CLI_DSHOT_INVALID_PKT    "Dshot ugyldig pakker:"
#define STR_CLI_DSHOT_DIR_CHANGE     "Dshot retning  Skift  cyklus:"
#define STR_CLI_DSHOT_DIR_CHANGE_MIC ", mikro:"
#define STR_CLI_DSHOT_HEAD1          "Motor    Type   eRPM    RPM     Hz    Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define STR_CLI_DSHOT_LINE1          "=====  ====== ====== ====== ====== ========== ====== ====== ====== ====== ====== ====== ======"
#define STR_CLI_DSHOT_HEAD2          "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define STR_CLI_DSHOT_LINE2          "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======"
#define STR_CLI_DSHOT_NO_TELEM       "Dshot telemetri ikke aktiv"

#define STR_CLI_STORAGE_NOTFOUND     "Masselager findes ikke eller kan ikke aktiveres!"
#define STR_CLI_NO_MATCH             "INGEN MATCH FOR:"
#define STR_CLI_ERROR_FOUND          "FEJL ER FUNDET - UNDERSOEG INDEN DU FORTSAETTER"
#define STR_CLI_FIX_ERROR            "RET FEJL, INDEN GEM MED 'SAVE'"
#define STR_CLI_STATUS_MCU           "MCU:"
#define STR_CLI_STATUS_CLOCK         "klokke="
#define STR_CLI_STATUS_VREF          "Vref="
#define STR_CLI_STATUS_CORETEMP      "kerne temp="
#define STR_CLI_STATUS_STACK_SIZE    "Stak stoerrelse:"
#define STR_CLI_STATUS_STACK_ADDR    "Stak adresse:"
#define STR_CLI_STATUS_STACK_USED    "Stak brugt:"
#define STR_CLI_STATUS_CONFIG        "Konfiguration:"
#define STR_CLI_STATUS_CONFIG_SIZE   "stoerrelse:"
#define STR_CLI_STATUS_CONFIG_AVAIL  "max anvendeligt:"
#define STR_CLI_STATUS_DEVICES       "Enheder opdaget:"
#define STR_CLI_STATUS_SPI           " SPI:"
#define STR_CLI_STATUS_I2C           " I2C:"
#define STR_CLI_STATUS_I2C_ERRORS    "I2C fejl:"

#define STR_CLI_STATUS_GYRO_DETECT   "Gyro opdaget:"
#define STR_CLI_STATUS_GYRO          " gyro"
#define STR_CLI_STATUS_GYRO_LOCKED   " laast"
#define STR_CLI_STATUS_GYRO_DMA      " dma"
#define STR_CLI_STATUS_GYRO_SHARED   " delt"

#define STR_CLI_STATUS_OSD           "OSD:"
#define STR_CLI_STATUS_BUILD_KEY     "RELEASE BYG ID:"
#define STR_CLI_STATUS_SYSTEM_UPTIME "System oppetid:"
#define STR_CLI_STATUS_TIME_CURRENT  ", system tid:"

#define STR_CLI_STATUS_CPU           "CPU:"
#define STR_CLI_STATUS_CPU_CYCLE     "cyklus tid:"
#define STR_CLI_STATUS_CPU_GYRO      "GYRO hast.:"
#define STR_CLI_STATUS_CPU_RX        "RX hast.:"
#define STR_CLI_STATUS_CPU_SYSTEM    "System hast.:"

#define STR_CLI_STATUS_BAT_VOLTAGE   "Spaending:"
#define STR_CLI_STATUS_BAT_TYPE      "batteri"

#define STR_CLI_STATUS_FLASH         "FLASH: JEDEC ID="

#define STR_CLI_TASK_LATE_LIST       "Task liste            rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us"
#define STR_CLI_TASK_LIST            "Task liste            rate/hz  max/us  avg/us maxload avgload  total/ms"
#define STR_CLI_TASK_MINIMAL         "Task liste"
#define STR_CLI_TASK_RX_CHECK        "RX check funktion"
#define STR_CLI_TASK_TOTAL           "Total (eksklusiv SERIAL)"
#define STR_CLI_TASK_SCHEDULER_START "Scheduler start cykler"
#define STR_CLI_TASK_SCHEDULER_GAURD "guard cykler"

#define STR_CLI_VERSION_HAS_CONFIG   "Konfig: JA"
#define STR_CLI_VERSION_NO_CONFIG    TR2("INGEN KONFIG", "INGEN KONFIGURATION FUNDET")
#define STR_CLI_VERSION_INFO_MANUF   "# FC: manufacturer_id:"
#define STR_CLI_VERSION_INFO_BOARD   "board_name:"

#define STR_CLI_NONE                 "INGEN"
#define STR_CLI_TIMER                "timer"
#define STR_CLI_TIMERS               "Timers:"
#define STR_CLI_TIMERS_ACTIVE        "Aktiv tidtagning:"
#define STR_CLI_TIMERS_FREE          " FRI"

#define STR_CLI_STATUS_ARM_DISABLE   "Aktiver IKKE flag:"

#define STR_CLI_DSHOT_TEL_INFO       " INGEN DATA"
#define STR_CLI_MSC_TIMEZONE         "UGYLDIG TIDSZONE OFFSET"
#define STR_CLI_MSC_RESTART          "Skifter til masselager tilstand"
#define STR_CLI_MSC_REBOOT           "\r\nGenstarter"
#define STR_CLI_PROCESS              "UKENDT KOMMANDO, TAST 'HELP'"
#define STR_CLI_ENTER_LONG           "\r\nCLI tilstand, tast 'exit' for at forlade, eller 'help' for vejledning"
#define STR_CLI_ENTER_SHORT          "\r\nCLI"

#define STR_CLI_SDCARD_NAME          "SD kort: "
#define STR_CLI_SDCARD_NOCONFIG      "Ikke konfigureret"
#define STR_CLI_SDCARD_NONE          "Ikke indsat"
#define STR_CLI_SDCARD_NOSTART       "Startop fejlet"
#define STR_CLI_SDCARD_MANUFAC       "Producent"
#define STR_CLI_SDCARD_FILESYSTEM    "\r\nFilsystem: "
#define STR_CLI_SDCARD_READY         "Klart"
#define STR_CLI_SDCARD_INIT          "Initialiserer"
#define STR_CLI_SDCARD_FATAL         "Fejlet helt"
#define STR_CLI_SDCARD_NOFATMBR      " - ingen FAT MBR partitioner"
#define STR_CLI_SDCARD_BADFAT        " - fejlagtig FAT blok"

#define STR_CLI_FLASH_SEC            "Flash sektorer="
#define STR_CLI_FLASH_SEC_SIZE       "sektorStoerrelse="
#define STR_CLI_FLASH_SEC_PAGE       "siderPerSektor="
#define STR_CLI_FLASH_PAGE_SIZE      "sideStoerrelse="
#define STR_CLI_FLASH_TOTAL_SIZE     "totalStoerrelse="
#define STR_CLI_FLASH_JEDEC          "JEDEC ID="

#define STR_CLI_FLASH_PART           "Partitioner:"
#define STR_CLI_FLASH_SIZE           "FlashFS stoerrelse="
#define STR_CLI_FLASH_SIZE_USED      "brugtStoerrelse="
#define STR_CLI_FLASH_ER_LONG        "Sletter, vent venligst ... "
#define STR_CLI_FLASH_ER_SHORT       "Sletter,"
#define STR_CLI_FLASH_ER_DONE        "Faerdig."
#define STR_CLI_FLASH_VR             "Kontrollerer"
#define STR_CLI_FLASH_VR_OK          "Succes"
#define STR_CLI_FLASH_VR_FAIL        "Fejlet"
#define STR_CLI_FLASH_WROTE          "Skrevet"
#define STR_CLI_FLASH_READ           "Laest"
#define STR_CLI_FLASH_BYTES_AT       "bytes ved:"

#define STR_CLI_ENABLED              "Aktiveret"
#define STR_CLI_DISABLED             "Deaktiveret"
#define STR_CLI_AVAILABLE            "Mulige:"
#define STR_CLI_UNAVAILABLE          "Ikke mulig"
#define STR_CLI_NOSUPPORT            "Ikke understoettet."
