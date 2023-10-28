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


#define LOCALE                        "en"                                  	// Max length:  2; Current language in short form: en, da, es, fr, nl ...
#define STR_COMMA                     ","                                   	// Max length:  1; Comma string
#define STR_PERIOD                    "."                                   	// Max length:  1; Period string
#define STR_THOUSAND                  "."                                   	// Max length:  1; Thousand delimiter
#define STR_SECONDS                   "seconds"                             	// Max length:  8; Seconds string
#define STR_VISUAL_BEEP               "  * * * *"                           	// Max length: 10; Visual beep
#define STR_MSP_API_NAME              "MSP API:"                            	// Max length:  8; MSP API prompt
#define CMS_STARTUP_HELP_TEXT1        "MENU:THR MID"                        	// Max length: 20; CMS menu line 1
#define CMS_STARTUP_HELP_TEXT2        "+ YAW LEFT  "                        	// Max length: 20; CMS menu line 2
#define CMS_STARTUP_HELP_TEXT3        "+ PITCH UP  "                        	// Max length: 20; CMS menu line 3
#define STR_OSD_ARMED                 TR2("ARMED", "*** ARMED ***")         	// Max length: 10; ARMED notification; HD> Max length: 20; HD text for ARMED notification
#define STR_OSD_STATS                 TR2("--- STATS ---", "--- STATISTICS ---") 	// Max length: 15; STATISTICS; HD> Max length: 20; HD text for STATISTICS
#define STR_OSD_TIM_SOURCE_1          TR2("ON TIME  ", "ON TIME    ")       	// Max length:  9; TIME SOURCE 1; HD> Max length: 11; HD text for TIME SOURCE 1
#define STR_OSD_TIM_SOURCE_2          TR2("TOTAL ARM", "TOTAL ARMED")       	// Max length:  9; TIME SOURCE 2; HD> Max length: 11; HD text for TIME SOURCE 2
#define STR_OSD_TIM_SOURCE_3          TR2("LAST ARM ", "LAST ARMED ")       	// Max length:  9; TIME SOURCE 3; HD> Max length: 11; HD text for TIME SOURCE 3
#define STR_OSD_TIM_SOURCE_4          TR2("ON/ARM   ", "ON/ARMED   ")       	// Max length:  9; TIME SOURCE 4; HD> Max length: 11; HD text for TIME SOURCE 4
#define STR_OSD_STAT_MAX_ALTITUDE     TR2("MAX ALTITUDE", "MAXIMAL ALTITUDE") 	// Max length: 12; MAXIMAL ALTITUDE; HD> Max length: 16; HD text for MAXIMAL ALTITUDE
#define STR_OSD_STAT_MAX_SPEED        TR2("MAX SPEED   ", "MAXIMAL SPEED   ") 	// Max length: 12; MAXIMAL SPEED; HD> Max length: 16; HD text for MAXIMAL SPEED
#define STR_OSD_STAT_MAX_DISTANCE     TR2("MAX DISTANCE", "MAXIMAL DISTANCE") 	// Max length: 12; MAXIMAL DISTANCE; HD> Max length: 16; HD text for MAXIMAL DISTANCE
#define STR_OSD_STAT_FLIGHT_DISTANCE  "FLIGHT DISTANCE"                     	// Max length: 15; FLIGHT DISTANCE
#define STR_OSD_STAT_MIN_BATTERY_AVG  "MIN AVG CELL"                        	// Max length: 15; description
#define STR_OSD_STAT_MIN_BATTERY      "MIN BATTERY"                         	// Max length: 15; description
#define STR_OSD_STAT_END_BATTERY_AVG  "END AVG CELL"                        	// Max length: 15; description
#define STR_OSD_STAT_END_BATTERY      "END BATTERY"                         	// Max length: 15; description
#define STR_OSD_STAT_BATTERY_AVG      "AVG BATT CELL"                       	// Max length: 15; description
#define STR_OSD_STAT_BATTERY          "BATTERY"                             	// Max length: 15; BATTERY
#define STR_OSD_STAT_MIN_RSSI         "MIN RSSI"                            	// Max length: 15; description
#define STR_OSD_STAT_MAX_CURRENT      TR2("MAX CURRENT", "MAXIMAL CURRENT") 	// Max length: 15; MAXIMAL CURRENT; HD> Max length: 15; HD text for MAXIMAL CURRENT
#define STR_OSD_STAT_USED_MAH         "USED MAH"                            	// Max length: 15; description
#define STR_OSD_STAT_WATT_HOURS_DRAWN "USED WATT HOURS"                     	// Max length: 15; description
#define STR_OSD_STAT_BLACKBOX         "BLACKBOX"                            	// Max length: 15; description
#define STR_OSD_STAT_BLACKBOX_NUMBER  TR2("BB LOG NUM", "BLACKBOX LOG NUMBER") 	// Max length: 15; BLACKBOX LOG NUMBER; HD> Max length: 20; HD text for BLACKBOX LOG NUMBER
#define STR_OSD_STAT_MAX_G_FORCE      TR2("MAX G-FORCE", "MAXIMAL G-FORCE") 	// Max length: 15; MAXIMAL G-FORCE; HD> Max length: 15; HD text for MAXIMAL G-FORCE
#define STR_OSD_STAT_MAX_ESC_TEMP     TR2("MAX ESC TEMP", "MAXIMAL ESC TEMP") 	// Max length: 15; MAXIMAL ESC TEMP; HD> Max length: 16; HD text for MAXIMAL ESC TEMP
#define STR_OSD_STAT_MAX_ESC_RPM      TR2("MAX ESC RPM", "MAXIMAL ESC RPM") 	// Max length: 15; MAXIMAL ESC RPM; HD> Max length: 15; HD text for MAXIMAL ESC RPM
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"                            	// Max length: 15; description
#define STR_OSD_STAT_MAX_FFT          "PEAK FFT"                            	// Max length: 15; description
#define STR_OSD_STAT_MAX_FFT_THRT     "THRT<20%"                            	// Max length: 15; description
#define STR_OSD_STAT_MIN_RSSI_DBM     "MIN RSSI DBM"                        	// Max length: 15; description
#define STR_OSD_STAT_MIN_RSNR         "MIN RSNR"                            	// Max length: 15; description
#define STR_USE_PERSISTENT_STATS      "TOTAL FLIGHTS"                       	// Max length: 20; COUNT TOTAL FLIGHTS
#define STR_OSD_STAT_TOTAL_TIME       "TOTAL FLIGHT TIME"                   	// Max length: 20; COUNT TOTAL FLIGHT TIME
#define STR_OSD_STAT_TOTAL_DIST       "TOTAL DISTANCE"                      	// Max length: 20; COUNT TOTAL DISTANCE
#define STR_OSDW_CRASH_FLIP_WARNING   "> CRASH FLIP <"                      	// Max length: 15; description
#define STR_OSDW_BEACON_ON            " BEACON ON"                          	// Max length: 15; description
#define STR_OSDW_ARM_IN               TR2("ARM IN", "ARMED IN")             	// Max length: 15; TIME ARMED IN; HD> Max length: 15; HD text for TIME ARMED IN
#define STR_OSDW_FAIL_SAFE            "FAIL SAFE"                           	// Max length: 15; description
#define STR_OSDW_CRASH_FLIP           "CRASH FLIP SWITCH"                   	// Max length: 20; description
#define STR_OSDW_LAUNCH               "LAUNCH"                              	// Max length: 15; description
#define STR_OSDW_RSSI_LOW             "RSSI LOW"                            	// Max length: 15; description
#define STR_OSDW_RSSI_DBM             "RSSI DBM"                            	// Max length: 15; description
#define STR_OSDW_RSNR_LOW             "RSNR LOW"                            	// Max length: 15; description
#define STR_OSDW_LINK_QUA             "LINK QUALITY"                        	// Max length: 15; description
#define STR_OSDW_LAND_NOW             " LAND NOW"                           	// Max length: 15; description
#define STR_OSDW_RESCUE_NA            TR2("RESCUE N/A", "RESCUE NOT AVAILABLE") 	// Max length: 15; RESCUE NOT AVAILABLE; HD> Max length: 20; HD text for RESCUE NOT AVAILABLE
#define STR_OSDW_RESCUE_OFF           "RESCUE OFF"                          	// Max length: 15; description
#define STR_OSDW_HEADFREE             "HEADFREE"                            	// Max length: 15; description
#define STR_OSDW_CORE                 "CORE"                                	// Max length: 15; description
#define STR_OSDW_LOW_BATT             "LOW BATTERY"                         	// Max length: 15; description
#define STR_OSDW_RCSMOOTHING          "RCSMOOTHING"                         	// Max length: 15; description
#define STR_OSDW_OVER_CAP             "OVER CAP"                            	// Max length: 15; description
#define STR_OSDW_BATT_CONTINUE        "BATTERY CONTINUE"                    	// Max length: 20; description
#define STR_OSDW_BATT_BELOW_FULL      "BATT < FULL"                         	// Max length: 15; description
#define STR_OSDE_DISARMED             TR2("DISARMED", "*** DISARMED ***")   	// Max length: 15; DISARMED; HD> Max length: 20; HD text for DISARMED
#define STR_OSDE_UP                   "U"                                   	// Max length:  1; Up
#define STR_OSDE_DOWN                 "D"                                   	// Max length:  1; Down
#define STR_OSDE_GPS_WEST             "W"                                   	// Max length:  1; West
#define STR_OSDE_GPS_EAST             "E"                                   	// Max length:  1; East
#define STR_OSDE_GPS_SOUTH            "S"                                   	// Max length:  1; South
#define STR_OSDE_GPS_NORTH            "N"                                   	// Max length:  1; North
#define STR_OSDE_PILOT_NAME           "PILOT NAME"                          	// Max length: 10; PILOT NAME
#define STR_OSDE_RATE                 TR2("RATE_", "RATE_")                 	// Max length:  5; description; HD> Max length: 15; HD text for RATE_
#define STR_OSDE_PID                  "PID_"                                	// Max length:  5; description
#define STR_OSDE_OID                  "OID_"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_FAILSAFE     "!FS!"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_RESCUE       "RESC"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_HEAD         "HEAD"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_ANGL         "ANGL"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_HOR          "HOR "                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_ATRN         "ATRN"                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_AIR          "AIR "                                	// Max length:  5; description
#define STR_ODSE_FLYMODE_ACRO         "ACRO"                                	// Max length:  5; description
#define STR_ODSE_ELEMENT_PIT          "PIT"                                 	// Max length:  3; Pitch
#define STR_ODSE_ELEMENT_ROL          "ROL"                                 	// Max length:  3; Roll
#define STR_ODSE_ELEMENT_YAW          "YAW"                                 	// Max length:  3; Yaw
#define STR_ODSE_ANTIGRAVITY          "AG"                                  	// Max length:  2; Anti Gravity
#define STR_OSDE_READY                "READY"                               	// Max length:  5; Ready
#define STR_LOCALE_SETUP              "Language:"                           	// Max length: 10; Language
#define STR_CLI_ERROR_INVALID_NAME    "INVALID NAME:"                       	// Max length: 35; ERROR_INVALID_NAME
#define STR_CLI_ERROR_MESSAGE         "CANNOT BE CHANGED. CURRENT VALUE:"   	// Max length: 35; description
#define STR_CLI_PARSINGFAIL           "PARSING FAILED"                      	// Max length: 35; PARSING FAILED
#define STR_CLI_INVALIDRESSOURCE      "INVALID RESOURCE NAME:"              	// Max length: 35; INVALID RESOURCE NAME
#define STR_CLI_ARG_INVALIDCOUNT      "INVALID ARGUMENT COUNT"              	// Max length: 35; INVALID ARGUMENT COUNT
#define STR_CLI_ARG_NOTBETWEEN        "NOT BETWEEN"                         	// Max length: 35; NOT BETWEEN
#define STR_CLI_ARG_AND               "AND"                                 	// Max length:  3; and
#define STR_CLI_ARG_OUTOFRANGE        "ARGUMENT OUT OF RANGE"               	// Max length: 35; ARGUMENT OUT OF RANGE
#define STR_CLI_DSHOT_READ            "Dshot reads:"                        	// Max length: 15; description
#define STR_CLI_DSHOT_INVALID_PKT     "Dshot invalid pkts:"                 	// Max length: 20; description
#define STR_CLI_DSHOT_DIR_CHANGE      "Dshot directionChange cycles:"       	// Max length: 29; description
#define STR_CLI_DSHOT_DIR_CHANGE_MIC  ", micros:"                           	// Max length: 10; description
#define STR_CLI_DSHOT_HEAD1           "Motor    Type   eRPM    RPM     Hz Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3" 	// Max length:115; description
#define STR_CLI_DSHOT_LINE1           "=====  ====== ====== ====== ====== ======= ====== ====== ====== ====== ====== ====== ======" 	// Max length:115; description
#define STR_CLI_DSHOT_HEAD2           "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3" 	// Max length:115; description
#define STR_CLI_DSHOT_LINE2           "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======" 	// Max length:115; description
#define STR_CLI_DSHOT_NO_TELEM        "Dshot telemetry not enabled"         	// Max length: 30; description
#define STR_CLI_STORAGE_NOTFOUND      "Storage not present or failed to initialize!" 	// Max length: 50; description
#define STR_CLI_NO_MATCH              "NO MATCHES FOR:"                     	// Max length: 15; NO MATCHES
#define STR_CLI_ERROR_FOUND           "ERRORS WERE DETECTED - PLEASE REVIEW BEFORE CONTINUING" 	// Max length: 55; ERRORS WERE DETECTED
#define STR_CLI_FIX_ERROR             "PLEASE FIX ERRORS BEFORE 'SAVE'"     	// Max length: 32; FIX ERRORS
#define STR_CLI_STATUS_MCU            "MCU:"                                	// Max length:  6; description
#define STR_CLI_STATUS_CLOCK          "Clock="                              	// Max length:  6; description
#define STR_CLI_STATUS_VREF           "Vref="                               	// Max length:  6; description
#define STR_CLI_STATUS_CORETEMP       "Core temp="                          	// Max length: 15; description
#define STR_CLI_STATUS_STACK_SIZE     "Stack size:"                         	// Max length: 15; description
#define STR_CLI_STATUS_STACK_ADDR     "Stack address:"                      	// Max length: 15; description
#define STR_CLI_STATUS_STACK_USED     "Stack used:"                         	// Max length: 15; description
#define STR_CLI_STATUS_CONFIG         "Configuration:"                      	// Max length: 15; description
#define STR_CLI_STATUS_CONFIG_SIZE    "size:"                               	// Max length:  6; description
#define STR_CLI_STATUS_CONFIG_AVAIL   "max available:"                      	// Max length: 15; description
#define STR_CLI_STATUS_DEVICES        "Devices detected:"                   	// Max length: 20; no of devices
#define STR_CLI_STATUS_SPI            " SPI:"                               	// Max length:  5; description
#define STR_CLI_STATUS_I2C            " I2C:"                               	// Max length:  5; I2C
#define STR_CLI_STATUS_I2C_ERRORS     "I2C Errors:"                         	// Max length: 15; I2C Errors
#define STR_CLI_STATUS_GYRO_DETECT    "Gyros detected:"                     	// Max length: 15; description
#define STR_CLI_STATUS_GYRO           " gyro"                               	// Max length:  8; description
#define STR_CLI_STATUS_GYRO_LOCKED    " locked"                             	// Max length:  8; description
#define STR_CLI_STATUS_GYRO_DMA       " dma"                                	// Max length:  8; description
#define STR_CLI_STATUS_GYRO_SHARED    " shared"                             	// Max length:  8; description
#define STR_CLI_STATUS_OSD            "OSD:"                                	// Max length:  8; description
#define STR_CLI_STATUS_BUILD_KEY      "BUILD KEY:"                          	// Max length: 10; BUILD KEY ID
#define STR_CLI_STATUS_SYSTEM_UPTIME  "System Uptime:"                      	// Max length: 15; description
#define STR_CLI_STATUS_TIME_CURRENT   ", Current Time:"                     	// Max length: 15; description
#define STR_CLI_STATUS_CPU            "CPU:"                                	// Max length:  4; description
#define STR_CLI_STATUS_CPU_CYCLE      "cycle time:"                         	// Max length: 15; description
#define STR_CLI_STATUS_CPU_GYRO       "GYRO rate:"                          	// Max length: 15; description
#define STR_CLI_STATUS_CPU_RX         "RX rate:"                            	// Max length: 10; description
#define STR_CLI_STATUS_CPU_SYSTEM     "System rate:"                        	// Max length: 12; description
#define STR_CLI_STATUS_BAT_VOLTAGE    "Voltage:"                            	// Max length:  8; description
#define STR_CLI_STATUS_BAT_TYPE       "battery"                             	// Max length:  7; description
#define STR_CLI_STATUS_FLASH          "FLASH: JEDEC ID="                    	// Max length: 16; description
#define STR_CLI_TASK_LATE_LIST        "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us" 	// Max length:115; description
#define STR_CLI_TASK_LIST             "Task list             rate/hz  max/us  avg/us maxload avgload  total/ms" 	// Max length:100; description
#define STR_CLI_TASK_MINIMAL          "Task list"                           	// Max length: 10; description
#define STR_CLI_TASK_RX_CHECK         "RX Check Function"                   	// Max length: 20; description
#define STR_CLI_TASK_TOTAL            "Total (excluding SERIAL)"            	// Max length: 25; description
#define STR_CLI_TASK_SCHEDULER_START  "Scheduler start cycles"              	// Max length: 25; description
#define STR_CLI_TASK_SCHEDULER_GAURD  "guard cycles"                        	// Max length: 12; description
#define STR_CLI_VERSION_HAS_CONFIG    "config: YES"                         	// Max length: 15; description
#define STR_CLI_VERSION_NO_CONFIG     TR2("NO CONFIG FOUND", "NO CONFIGURATION FOUND") 	// Max length: 15; description; HD> Max length: 30; HD text for description
#define STR_CLI_VERSION_INFO_MANUF    "# board: manufacturer_id:"           	// Max length: 25; description
#define STR_CLI_VERSION_INFO_BOARD    "board_name:"                         	// Max length: 11; description
#define STR_CLI_NONE                  "NONE"                                	// Max length:  6; description
#define STR_CLI_TIMER                 "timer"                               	// Max length:  6; description
#define STR_CLI_TIMERS                "Timers:"                             	// Max length:  8; description
#define STR_CLI_TIMERS_ACTIVE         "Currently active Timers:"            	// Max length: 30; description
#define STR_CLI_TIMERS_FREE           " FREE"                               	// Max length:  6; description
#define STR_CLI_STATUS_ARM_DISABLE    "Arming disable flags:"               	// Max length: 25; Arming disable flags
#define STR_CLI_DSHOT_TEL_INFO        " NO DATA"                            	// Max length: 10; description
#define STR_CLI_MSC_TIMEZONE          "INVALID TIMEZONE OFFSET"             	// Max length: 25; description
#define STR_CLI_MSC_RESTART           "Restarting in mass storage mode"     	// Max length: 35; description
#define STR_CLI_MSC_REBOOT            "\r\nRebooting"                       	// Max length: 15; rebooting
#define STR_CLI_PROCESS               "UNKNOWN COMMAND, TRY 'HELP'"         	// Max length: 30; UNKNOWN COMMAND
#define STR_CLI_ENTER_LONG            "\r\nEntering CLI Mode, type 'exit' to return, or 'help'" 	// Max length: 75; Enter CLI mode
#define STR_CLI_ENTER_SHORT           "\r\nCLI"                             	// Max length: 10; description
#define STR_CLI_SDCARD_NAME           "SD card: "                           	// Max length: 10; description
#define STR_CLI_SDCARD_NOCONFIG       "Not configured"                      	// Max length: 20; description
#define STR_CLI_SDCARD_NONE           "None inserted"                       	// Max length: 20; description
#define STR_CLI_SDCARD_NOSTART        "Startup failed"                      	// Max length: 20; description
#define STR_CLI_SDCARD_MANUFAC        "Manufacturer"                        	// Max length: 15; description
#define STR_CLI_SDCARD_FILESYSTEM     "\r\nFilsystem: "                     	// Max length: 15; description
#define STR_CLI_SDCARD_READY          "Ready"                               	// Max length:  5; description
#define STR_CLI_SDCARD_INIT           "Initializing"                        	// Max length: 15; description
#define STR_CLI_SDCARD_FATAL          "Fatal"                               	// Max length: 15; description
#define STR_CLI_SDCARD_NOFATMBR       " - no FAT MBR partitions"            	// Max length: 30; description
#define STR_CLI_SDCARD_BADFAT         " - bad FAT header"                   	// Max length: 30; description
#define STR_CLI_FLASH_SEC             "Flash sectors="                      	// Max length: 30; description
#define STR_CLI_FLASH_SEC_SIZE        "sectorSize="                         	// Max length: 11; description
#define STR_CLI_FLASH_SEC_PAGE        "pagesPerSector="                     	// Max length: 15; description
#define STR_CLI_FLASH_PAGE_SIZE       "pageSize="                           	// Max length: 10; description
#define STR_CLI_FLASH_TOTAL_SIZE      "totalSize="                          	// Max length: 10; description
#define STR_CLI_FLASH_JEDEC           "JEDEC ID="                           	// Max length: 10; description
#define STR_CLI_FLASH_PART            "Partitions:"                         	// Max length: 15; description
#define STR_CLI_FLASH_SIZE            "FlashFS size="                       	// Max length: 15; description
#define STR_CLI_FLASH_SIZE_USED       "usedSize="                           	// Max length: 15; description
#define STR_CLI_FLASH_ER_LONG         "Erasing, please wait ... "           	// Max length: 30; description
#define STR_CLI_FLASH_ER_SHORT        "Erasing,"                            	// Max length: 15; description
#define STR_CLI_FLASH_ER_DONE         "Done."                               	// Max length: 10; description
#define STR_CLI_FLASH_VR              "Verifying"                           	// Max length: 15; description
#define STR_CLI_FLASH_VR_OK           "Success"                             	// Max length: 15; description
#define STR_CLI_FLASH_VR_FAIL         "Failed"                              	// Max length: 15; description
#define STR_CLI_FLASH_WROTE           "Wrote"                               	// Max length: 10; description
#define STR_CLI_FLASH_READ            "Reading"                             	// Max length: 10; description
#define STR_CLI_FLASH_BYTES_AT        "bytes at:"                           	// Max length: 10; description
#define STR_CLI_ENABLED               "Enabled"                             	// Max length: 15; description
#define STR_CLI_DISABLED              "Disabled"                            	// Max length: 15; description
#define STR_CLI_AVAILABLE             "Available:"                          	// Max length: 15; description
#define STR_CLI_UNAVAILABLE           "unavailable"                         	// Max length: 15; description
>>>>>>> 442ec423d (last corrections)
