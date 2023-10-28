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

<<<<<<< HEAD
#define LOCALE                        "en"                                  	// Max length:  2; Current language in short form: en, da, es, fr, nl ...;
#define STR_LOCALE_SETUP              "Language:"                           	// Max length: 10; trans lation of: Language:;
#define STR_COMMA                     ","                                   	// Max length:  1; translation of: Comma string ,;
#define STR_PERIOD                    "."                                   	// Max length:  1; translation of: Period string .;
#define STR_THOUSAND                  "."                                   	// Max length:  1; translation of: Thousand delimiter .;
#define STR_SECONDS                   "seconds"                             	// Max length:  8; translation of: seconds;
#define STR_VISUAL_BEEP               "  * * * *"                           	// Max length: 10; translation of:   * * * *;
#define CMS_STARTUP_HELP_TEXT1        "MENU:THR MID"                        	// Max length: 20; translation of: MENU:THR MID;
#define CMS_STARTUP_HELP_TEXT2        "+ YAW LEFT  "                        	// Max length: 20; translation of: + YAW LEFT  ;
#define CMS_STARTUP_HELP_TEXT3        "+ PITCH UP  "                        	// Max length: 20; translation of: + PITCH UP  ;
#define STR_OSD_ARMED                 TR2("ARMED", "*** ARMED ***")         	// Max length: 10; translation of: ARMED; HD> Max length: 20; HD translation of: *** ARMED ***
#define STR_OSD_STATS                 TR2("--- STATS ---", "--- STATISTICS ---") 	// Max length: 15; translation of: --- STATS ---; HD> Max length: 20; HD translation of: --- STATISTICS ---
#define STR_OSD_TIM_SOURCE_1          TR2("ON TIME  ", "ON TIME    ")       	// Max length:  9; translation of: ON TIME  ; HD> Max length: 11; HD translation of: ON TIME  
#define STR_OSD_TIM_SOURCE_2          TR2("TOTAL ARM", "TOTAL ARMED")       	// Max length:  9; translation of: TOTAL ARM; HD> Max length: 11; HD translation of: TOTAL ARMED
#define STR_OSD_TIM_SOURCE_3          TR2("LAST ARM ", "LAST ARMED ")       	// Max length:  9; translation of: LAST ARM ; HD> Max length: 11; HD translation of: LAST ARMED 
#define STR_OSD_TIM_SOURCE_4          TR2("ON/ARM   ", "ON/ARMED   ")       	// Max length:  9; translation of: ON/ARM   ; HD> Max length: 11; HD translation of: ON/ARMED   
#define STR_OSD_STAT_MAX_ALTITUDE     TR2("MAX ALTITUDE", "MAXIMAL ALTITUDE") 	// Max length: 12; translation of: MAX ALTITUDE; HD> Max length: 16; HD translation of: MAXIMAL ALTITUDE
#define STR_OSD_STAT_MAX_SPEED        TR2("MAX SPEED   ", "MAXIMAL SPEED   ") 	// Max length: 12; translation of: MAX SPEED   ; HD> Max length: 16; HD translation of: MAXIMAL SPEED   
#define STR_OSD_STAT_MAX_DISTANCE     TR2("MAX DISTANCE", "MAXIMAL DISTANCE") 	// Max length: 12; translation of: MAX DISTANCE; HD> Max length: 16; HD translation of: MAXIMAL DISTANCE
#define STR_OSD_STAT_FLIGHT_DISTANCE  "FLIGHT DISTANCE"                     	// Max length: 15; translation of: FLIGHT DISTANCE;
#define STR_OSD_STAT_MIN_BATTERY_AVG  "MIN AVG CELL"                        	// Max length: 15; translation of: MIN AVG CELL;
#define STR_OSD_STAT_MIN_BATTERY      "MIN BATTERY"                         	// Max length: 15; translation of: MIN BATTERY;
#define STR_OSD_STAT_END_BATTERY_AVG  "END AVG CELL"                        	// Max length: 15; translation of: END AVG CELL;
#define STR_OSD_STAT_END_BATTERY      "END BATTERY"                         	// Max length: 15; translation of: END BATTERY;
#define STR_OSD_STAT_BATTERY_AVG      "AVG BATT CELL"                       	// Max length: 15; translation of: AVG BATT CELL;
#define STR_OSD_STAT_BATTERY          "BATTERY"                             	// Max length: 15; translation of: BATTERY;
#define STR_OSD_STAT_MIN_RSSI         "MIN RSSI"                            	// Max length: 15; translation of: MIN RSSI;
#define STR_OSD_STAT_MAX_CURRENT      TR2("MAX CURRENT", "MAXIMAL CURRENT") 	// Max length: 15; translation of: MAX CURRENT; HD> Max length: 15; HD translation of: MAXIMAL CURRENT
#define STR_OSD_STAT_USED_MAH         "USED MAH"                            	// Max length: 15; translation of: USED MAH;
#define STR_OSD_STAT_WATT_HOURS_DRAWN "USED WATT HOURS"                     	// Max length: 15; translation of: USED WATT HOURS;
#define STR_OSD_STAT_BLACKBOX         "BLACKBOX"                            	// Max length: 15; translation of: BLACKBOX;
#define STR_OSD_STAT_BLACKBOX_NUMBER  TR2("BB LOG NUM", "BLACKBOX LOG NUMBER") 	// Max length: 15; translation of: BB LOG NUM; HD> Max length: 20; HD translation of: BLACKBOX LOG NUMBER
#define STR_OSD_STAT_MAX_G_FORCE      TR2("MAX G-FORCE", "MAXIMAL G-FORCE") 	// Max length: 15; translation of: MAX G-FORCE; HD> Max length: 15; HD translation of: MAXIMAL G-FORCE
#define STR_OSD_STAT_MAX_ESC_TEMP     TR2("MAX ESC TEMP", "MAXIMAL ESC TEMP") 	// Max length: 15; translation of: MAX ESC TEMP; HD> Max length: 16; HD translation of: MAXIMAL ESC TEMP
#define STR_OSD_STAT_MAX_ESC_RPM      TR2("MAX ESC RPM", "MAXIMAL ESC RPM") 	// Max length: 15; translation of: MAX ESC RPM; HD> Max length: 15; HD translation of: MAXIMAL ESC RPM
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"                            	// Max length: 15; translation of: MIN LINK;
#define STR_OSD_STAT_MAX_FFT          "PEAK FFT"                            	// Max length: 15; translation of: PEAK FFT;
#define STR_OSD_STAT_MAX_FFT_THRT     "THRT<20%"                            	// Max length: 15; translation of: THRT<20%;
#define STR_OSD_STAT_MIN_RSSI_DBM     "MIN RSSI DBM"                        	// Max length: 15; translation of: MIN RSSI DBM;
#define STR_OSD_STAT_MIN_RSNR         "MIN RSNR"                            	// Max length: 15; translation of: MIN RSNR;
#define STR_USE_PERSISTENT_STATS      "TOTAL FLIGHTS"                       	// Max length: 20; translation of: TOTAL FLIGHTS;
#define STR_OSD_STAT_TOTAL_TIME       "TOTAL FLIGHT TIME"                   	// Max length: 20; translation of: TOTAL FLIGHT TIME;
#define STR_OSD_STAT_TOTAL_DIST       "TOTAL DISTANCE"                      	// Max length: 20; translation of: TOTAL DISTANCE;
#define STR_OSDW_CRASH_FLIP_WARNING   "> CRASH FLIP <"                      	// Max length: 15; translation of: > CRASH FLIP <;
#define STR_OSDW_CRASH_FLIP_SWITCH    "CRASH FLIP SWITCH"                   	// Max length: 20; translation of: CRASH FLIP SWITCH;
#define STR_OSDW_BEACON_ON            " BEACON ON"                          	// Max length: 15; translation of:  BEACON ON;
#define STR_OSDW_ARM_IN               TR2("ARM IN", "ARMED IN")             	// Max length: 15; translation of: ARM IN; HD> Max length: 15; HD translation of: ARMED IN
#define STR_OSDW_FAIL_SAFE            "FAIL SAFE"                           	// Max length: 15; translation of: FAIL SAFE;
#define STR_OSDW_LAUNCH               "LAUNCH"                              	// Max length: 15; translation of: LAUNCH;
#define STR_OSDW_RSSI_LOW             "RSSI LOW"                            	// Max length: 15; translation of: RSSI LOW;
#define STR_OSDW_RSSI_DBM             "RSSI DBM"                            	// Max length: 15; translation of: RSSI DBM;
#define STR_OSDW_RSNR_LOW             "RSNR LOW"                            	// Max length: 15; translation of: RSNR LOW;
#define STR_OSDW_LINK_QUALITY         "LINK QUALITY"                        	// Max length: 15; translation of: LINK QUALITY;
#define STR_OSDW_LAND_NOW             " LAND NOW"                           	// Max length: 15; translation of:  LAND NOW;
#define STR_OSDW_RESCUE_NA            TR2("RESCUE N/A", "RESCUE NOT AVAILABLE") 	// Max length: 15; translation of: RESCUE N/A; HD> Max length: 20; HD translation of: RESCUE NOT AVAILABLE
#define STR_OSDW_RESCUE_OFF           "RESCUE OFF"                          	// Max length: 15; translation of: RESCUE OFF;
#define STR_OSDW_HEADFREE             "HEADFREE"                            	// Max length: 15; translation of: HEADFREE;
#define STR_OSDW_CORE                 "CORE"                                	// Max length: 15; translation of: CORE;
#define STR_OSDW_LOW_BATT             "LOW BATTERY"                         	// Max length: 15; translation of: LOW BATTERY;
#define STR_OSDW_RCSMOOTHING          "RCSMOOTHING"                         	// Max length: 15; translation of: RCSMOOTHING;
#define STR_OSDW_OVER_CAP             "OVER CAP"                            	// Max length: 15; translation of: OVER CAP;
#define STR_OSDW_BATT_CONTINUE        "BATTERY CONTINUE"                    	// Max length: 20; translation of: BATTERY CONTINUE;
#define STR_OSDW_BATT_BELOW_FULL      "BATT < FULL"                         	// Max length: 15; translation of: BATT < FULL;
#define STR_OSDE_DISARMED             TR2("DISARMED", "*** DISARMED ***")   	// Max length: 15; translation of: DISARMED; HD> Max length: 20; HD translation of: *** DISARMED ***
#define STR_OSDE_UP                   "U"                                   	// Max length:  1; translation of: U;
#define STR_OSDE_DOWN                 "D"                                   	// Max length:  1; translation of: D;
#define STR_OSDE_GPS_DIRECTION        "NSEW"                                	// Max length:  4; translation of: NSEW;
#define STR_OSDE_PILOT_NAME           "PILOT NAME"                          	// Max length: 10; translation of: PILOT NAME;
#define STR_OSDE_RATE                 TR2("RATE_", "RATE_")                 	// Max length:  5; translation of: RATE_; HD> Max length: 15; HD translation of: RATE_
#define STR_OSDE_PID                  "PID_"                                	// Max length:  5; translation of: PID_;
#define STR_OSDE_OID                  "OID_"                                	// Max length:  5; translation of: OID_;
#define STR_ODSE_FLYMODE_FAILSAFE     "!FS!"                                	// Max length:  5; translation of: !FS!;
#define STR_ODSE_FLYMODE_RESCUE       "RESC"                                	// Max length:  5; translation of: RESC;
#define STR_ODSE_FLYMODE_HEAD         "HEAD"                                	// Max length:  5; translation of: HEAD;
#define STR_ODSE_FLYMODE_ANGL         "ANGL"                                	// Max length:  5; translation of: ANGL;
#define STR_ODSE_FLYMODE_HOR          "HOR "                                	// Max length:  5; translation of: HOR ;
#define STR_ODSE_FLYMODE_ATRN         "ATRN"                                	// Max length:  5; translation of: ATRN;
#define STR_ODSE_FLYMODE_AIR          "AIR "                                	// Max length:  5; translation of: AIR ;
#define STR_ODSE_FLYMODE_ACRO         "ACRO"                                	// Max length:  5; translation of: ACRO;
#define STR_ODSE_ELEMENT_PITCH        "PIT"                                 	// Max length:  4; translation of: PIT;
#define STR_ODSE_ELEMENT_ROLL         "ROL"                                 	// Max length:  4; translation of: ROL;
#define STR_ODSE_ELEMENT_YAW          "YAW"                                 	// Max length:  4; translation of: YAW;
#define STR_ODSE_ANTIGRAVITY          TR2("AG", "ANTI G")                   	// Max length:  2; translation of: AG; HD> Max length:  6; HD translation of: ANTI G
#define STR_OSDE_READY                "READY"                               	// Max length:  5; translation of: READY;
#define STR_CLI_ERROR_INVALID_NAME    "INVALID NAME:"                       	// Max length: 35; translation of: INVALID NAME;
#define STR_CLI_ERROR_MESSAGE         "CANNOT BE CHANGED. CURRENT VALUE:"   	// Max length: 35; translation of: CANNOT BE CHANGED. CURRENT VALUE:;
=======
#define LOCALE                       "en"

#define STR_COMMA	                 ","
#define STR_PERIOD	                 "."
#define STR_THOUSAND 	             "."
#define STR_SECONDS                  "seconds"

#define STR_VISUAL_BEEP              "  * * * *"
#define STR_MSP_API_NAME             "MSP API:"

// CMS
#define CMS_STARTUP_HELP_TEXT1       "MENU:THR MID"
#define CMS_STARTUP_HELP_TEXT2       "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3       "+ PITCH UP"

// OSD
#define STR_OSD_ARMED                "ARMED"
#define STR_OSD_STATS                "--- STATS ---"

#define STR_OSD_TIM_SOURCE_1         "ON TIME  "
#define STR_OSD_TIM_SOURCE_2         "TOTAL ARM"
#define STR_OSD_TIM_SOURCE_3         "LAST ARM "
#define STR_OSD_TIM_SOURCE_4         "ON/ARM   "

#define STR_OSD_STAT_MAX_ALTITUDE    "MAX ALTITUDE"
#define STR_OSD_STAT_MAX_SPEED       "MAX SPEED"
#define STR_OSD_STAT_MAX_DISTANCE    "MAX DISTANCE"
#define STR_OSD_STAT_FLIGHT_DISTANCE "FLIGHT DISTANCE"
#define STR_OSD_STAT_MIN_BATTERY_AVG "MIN AVG CELL"
#define STR_OSD_STAT_MIN_BATTERY     "MIN BATTERY"
#define STR_OSD_STAT_END_BATTERY_AVG "END AVG CELL"
#define STR_OSD_STAT_END_BATTERY     "END BATTERY"
#define STR_OSD_STAT_BATTERY_AVG     "AVG BATT CELL"
#define STR_OSD_STAT_BATTERY         "BATTERY"
#define STR_OSD_STAT_MIN_RSSI        "MIN RSSI"
#define STR_OSD_STAT_MAX_CURRENT     "MAX CURRENT"
#define STR_OSD_STAT_USED_MAH        "USED MAH"
#define STR_OSD_STAT_WATT_HOURS_DRAWN "USED WATT HOURS"
#define STR_OSD_STAT_BLACKBOX        "BLACKBOX"
#define STR_OSD_STAT_BLACKBOX_NUMBER "BB LOG NUM"
#define STR_OSD_STAT_MAX_G_FORCE     "MAX G-FORCE"
#define STR_OSD_STAT_MAX_ESC_TEMP    "MAX ESC TEMP"
#define STR_OSD_STAT_MAX_ESC_RPM     "MAX ESC RPM"
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"
#define STR_OSD_STAT_MAX_FFT         "PEAK FFT"
#define STR_OSD_STAT_MAX_FFT_THRT    "THRT<20%"
#define STR_OSD_STAT_MIN_RSSI_DBM    "MIN RSSI DBM"
#define STR_OSD_STAT_MIN_RSNR        "MIN RSNR"
#define STR_USE_PERSISTENT_STATS     "TOTAL FLIGHTS"
#define STR_OSD_STAT_TOTAL_TIME      "TOTAL FLIGHT TIME"
#define STR_OSD_STAT_TOTAL_DIST      "TOTAL DISTANCE"

// OSD warnings
#define STR_OSDW_CRASH_FLIP_WARNING  "> CRASH FLIP <"
#define STR_OSDW_BEACON_ON           " BEACON ON"
#define STR_OSDW_ARM_IN              "ARM IN"
#define STR_OSDW_FAIL_SAFE           "FAIL SAFE"
#define STR_OSDW_CRASH_FLIP          "CRASH FLIP SWITCH"
#define STR_OSDW_LAUNCH              "LAUNCH"
#define STR_OSDW_RSSI_LOW            "RSSI LOW"
#define STR_OSDW_RSSI_DBM            "RSSI DBM"
#define STR_OSDW_RSNR_LOW            "RSNR LOW"
#define STR_OSDW_LINK_QUA            "LINK QUALITY"
#define STR_OSDW_LAND_NOW            " LAND NOW"
#define STR_OSDW_RESCUE_NA           "RESCUE N/A"
#define STR_OSDW_RESCUE_OFF          "RESCUE OFF"
#define STR_OSDW_HEADFREE            "HEADFREE"
#define STR_OSDW_CORE                "CORE"
#define STR_OSDW_LOW_BATT            "LOW BATTERY"
#define STR_OSDW_RCSMOOTHING         "RCSMOOTHING"
#define STR_OSDW_OVER_CAP            "OVER CAP"
#define STR_OSDW_BATT_CONTINUE       "BATTERY CONTINUE"
#define STR_OSDW_BATT_BELOW_FULL     "BATT < FULL"

// OSD elements
#define STR_OSDE_DISARMED            "DISARMED"
#define STR_OSDE_UP                  "U"
#define STR_OSDE_DOWN                "D"
#define STR_OSDE_GPS_WEST            "W"
#define STR_OSDE_GPS_EAST            "E"
#define STR_OSDE_GPS_SOUTH           "S"
#define STR_OSDE_GPS_NORTH           "N"
#define STR_OSDE_PILOT_NAME          "PILOT_NAME"
#define STR_OSDE_RATE                "RATE_"
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

#define STR_ODSE_ELEMENT_PIT         "PIT"
#define STR_ODSE_ELEMENT_ROL         "ROL"
#define STR_ODSE_ELEMENT_YAW         "YAW"

#define STR_ODSE_ANTIGRAVITY         "AG"

#define STR_OSDE_READY               "READY"

// CLI
#define STR_LOCALE_SETUP             "Language:"

#define STR_CLI_ERROR_INVALID_NAME   "INVALID NAME:"
#define STR_CLI_ERROR_MESSAGE        "CANNOT BE CHANGED. CURRENT VALUE:"
#define STR_CLI_PARSINGFAIL          "PARSING FAILED"
#define STR_CLI_INVALIDRESSOURCE     "INVALID RESOURCE NAME:"
#define STR_CLI_ARG_INVALIDCOUNT     "INVALID ARGUMENT COUNT"
#define STR_CLI_ARG_NOTBETWEEN       "NOT BETWEEN"
#define STR_CLI_ARG_AND              "AND"
#define STR_CLI_ARG_OUTOFRANGE       "ARGUMENT OUT OF RANGE"

#define STR_CLI_DSHOT_READ           "Dshot reads:"
#define STR_CLI_DSHOT_INVALID_PKT    "Dshot invalid pkts:"
#define STR_CLI_DSHOT_DIR_CHANGE     "Dshot directionChange cycles:"
#define STR_CLI_DSHOT_DIR_CHANGE_MIC ", micros:"
#define STR_CLI_DSHOT_HEAD1          "Motor    Type   eRPM    RPM     Hz Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define STR_CLI_DSHOT_LINE1          "=====  ====== ====== ====== ====== ======= ====== ====== ====== ====== ====== ====== ======"
#define STR_CLI_DSHOT_HEAD2          "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3"
#define STR_CLI_DSHOT_LINE2          "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======"
#define STR_CLI_DSHOT_NO_TELEM       "Dshot telemetry not enabled"

#define STR_CLI_STORAGE_NOTFOUND     "Storage not present or failed to initialize!"
#define STR_CLI_NO_MATCH             "NO MATCHES FOR:"
#define STR_CLI_ERROR_FOUND          "ERRORS WERE DETECTED - PLEASE REVIEW BEFORE CONTINUING"
#define STR_CLI_FIX_ERROR            "PLEASE FIX ERRORS BEFORE 'SAVE'"
#define STR_CLI_STATUS_MCU           "MCU:"
#define STR_CLI_STATUS_CLOCK         "Clock="
#define STR_CLI_STATUS_VREF          "Vref="
#define STR_CLI_STATUS_CORETEMP      "Core temp="
#define STR_CLI_STATUS_STACK_SIZE    "Stack size:"
#define STR_CLI_STATUS_STACK_ADDR    "Stack address:"
#define STR_CLI_STATUS_STACK_USED    "Stack used:"
#define STR_CLI_STATUS_CONFIG        "Configuration:"
#define STR_CLI_STATUS_CONFIG_SIZE   "size:"
#define STR_CLI_STATUS_CONFIG_AVAIL  "max available:"
#define STR_CLI_STATUS_DEVICES       "Devices detected:"
#define STR_CLI_STATUS_SPI           " SPI:"
#define STR_CLI_STATUS_I2C           " I2C:"
#define STR_CLI_STATUS_I2C_ERRORS    "I2C Errors:"

#define STR_CLI_STATUS_GYRO_DETECT   "Gyros detected:"
#define STR_CLI_STATUS_GYRO          " gyro"
#define STR_CLI_STATUS_GYRO_LOCKED   " locked"
#define STR_CLI_STATUS_GYRO_DMA      " dma"
#define STR_CLI_STATUS_GYRO_SHARED   " shared"

#define STR_CLI_STATUS_OSD           "OSD:"
#define STR_CLI_STATUS_BUILD_KEY     "BUILD KEY:"
#define STR_CLI_STATUS_SYSTEM_UPTIME "System Uptime:"
#define STR_CLI_STATUS_TIME_CURRENT  ", Current Time:"

#define STR_CLI_STATUS_CPU           "CPU:"
#define STR_CLI_STATUS_CPU_CYCLE     "cycle time:"
#define STR_CLI_STATUS_CPU_GYRO      "GYRO rate:"
#define STR_CLI_STATUS_CPU_RX        "RX rate:"
#define STR_CLI_STATUS_CPU_SYSTEM    "System rate:"

#define STR_CLI_STATUS_BAT_VOLTAGE   "Voltage:"
#define STR_CLI_STATUS_BAT_TYPE      "battery"

#define STR_CLI_STATUS_FLASH         "FLASH: JEDEC ID="

#define STR_CLI_TASK_LATE_LIST       "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us"
#define STR_CLI_TASK_LIST            "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms"
#define STR_CLI_TASK_MINIMAL         "Task list"
#define STR_CLI_TASK_RX_CHECK        "RX Check Function"
#define STR_CLI_TASK_TOTAL           "Total (excluding SERIAL)"
#define STR_CLI_TASK_SCHEDULER_START "Scheduler start cycles"
#define STR_CLI_TASK_SCHEDULER_GAURD "guard cycles"

#define STR_CLI_VERSION_HAS_CONFIG   "config: YES"
#define STR_CLI_VERSION_NO_CONFIG    "NO CONFIG FOUND"
#define STR_CLI_VERSION_INFO_MANUF   "# board: manufacturer_id:"
#define STR_CLI_VERSION_INFO_BOARD   "board_name:"

#define STR_CLI_NONE                 "NONE"
#define STR_CLI_TIMER                "timer"
#define STR_CLI_TIMERS               "Timers:"
#define STR_CLI_TIMERS_ACTIVE        "Currently active Timers:"
#define STR_CLI_TIMERS_FREE          " FREE"

#define STR_CLI_STATUS_ARM_DISABLE   "Arming disable flags:"

#define STR_CLI_DSHOT_TEL_INFO       " NO DATA"
#define STR_CLI_MSC_TIMEZONE         "INVALID TIMEZONE OFFSET"
#define STR_CLI_MSC_RESTART          "Restarting in mass storage mode"
#define STR_CLI_MSC_REBOOT           "\r\nRebooting"
#define STR_CLI_PROCESS              "UNKNOWN COMMAND, TRY 'HELP'"
#define STR_CLI_ENTER_LONG           "\r\nEntering CLI Mode, type 'exit' to return, or 'help'"
#define STR_CLI_ENTER_SHORT          "\r\nCLI"

#define STR_CLI_SDCARD_NAME          "SD card: "
#define STR_CLI_SDCARD_NOCONFIG      "Not configured"
#define STR_CLI_SDCARD_NONE          "None inserted"
#define STR_CLI_SDCARD_NOSTART       "Startup failed"
#define STR_CLI_SDCARD_MANUFAC       "Manufacturer"
#define STR_CLI_SDCARD_FILESYSTEM    "\r\nFilsystem: "
#define STR_CLI_SDCARD_READY         "Ready"
#define STR_CLI_SDCARD_INIT          "Initializing"
#define STR_CLI_SDCARD_FATAL         "Fatal"
#define STR_CLI_SDCARD_NOFATMBR      " - no FAT MBR partitions"
#define STR_CLI_SDCARD_BADFAT        " - bad FAT header"

#define STR_CLI_FLASH_SEC            "Flash sectors="
#define STR_CLI_FLASH_SEC_SIZE       "sectorSize="
#define STR_CLI_FLASH_SEC_PAGE       "pagesPerSector="
#define STR_CLI_FLASH_PAGE_SIZE      "pageSize="
#define STR_CLI_FLASH_TOTAL_SIZE     "totalSize="
#define STR_CLI_FLASH_JEDEC          "JEDEC ID="

#define STR_CLI_FLASH_PART           "Partitions:"
#define STR_CLI_FLASH_SIZE           "FlashFS size="
#define STR_CLI_FLASH_SIZE_USED      "usedSize="
#define STR_CLI_FLASH_ER_LONG        "Erasing, please wait ... "
#define STR_CLI_FLASH_ER_SHORT       "Erasing,"
#define STR_CLI_FLASH_ER_DONE        "Done."
#define STR_CLI_FLASH_VR             "Verifying"
#define STR_CLI_FLASH_VR_OK          "Success"
#define STR_CLI_FLASH_VR_FAIL        "Failed"
#define STR_CLI_FLASH_WROTE          "Wrote"
#define STR_CLI_FLASH_READ           "Reading"
#define STR_CLI_FLASH_BYTES_AT       "bytes at:"

#define STR_CLI_ENABLED              "Enabled"
#define STR_CLI_DISABLED             "Disabled"
#define STR_CLI_AVAILABLE            "Available:"
#define STR_CLI_UNAVAILABLE          "unavailable"
#define STR_CLI_NOSUPPORT            "Not supported."
>>>>>>> cd1cda2fb (Report locale in cli command - status)
