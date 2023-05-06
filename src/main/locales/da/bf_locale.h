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

#define LOCALE                        "da"                                  	// Max length:  2; description
#define STR_COMMA                     ","                                   	// Max length:  1; description
#define next                          "another"                             	// Max length: 10; Translation version
#define STR_PERIOD                    "."                                   	// Max length:  1; description
#define STR_THOUSAND                  "."                                   	// Max length:  1; description
#define STR_SECONDS                   "sekunder"                            	// Max length:  8; description
#define STR_VISUAL_BEEP               " * * * *"                            	// Max length:  8; description
#define STR_MSP_API_NAME              "MSP API:"                            	// Max length:  8; description
#define CMS_STARTUP_HELP_TEXT1        "MENU:GAS MIDT"                       	// Max length: 13; description
#define CMS_STARTUP_HELP_TEXT2        " + SIDEROR VENSTRE"                  	// Max length: 18; description
#define CMS_STARTUP_HELP_TEXT3        " + HOEJDE OP"                        	// Max length: 12; description
#define STR_OSD_ARMED                 "TR2(* AKTIV *, *** AKTIVERET ***)"   	// Max length: 35; description
#define STR_OSD_STATS                 "TR2(- STATISTIK -, --- STATISTIK ---)" 	// Max length: 39; description
#define STR_OSD_TIM_SOURCE_1          "TR2(ON TIME , ON TIME )"             	// Max length: 25; description
#define STR_OSD_TIM_SOURCE_2          "TR2(TOTAL AKT, TOTAL AKTIVERET)"     	// Max length: 33; description
#define STR_OSD_TIM_SOURCE_3          "TR2(SIDST AKT, SIDST AKTIVERET)"     	// Max length: 33; description
#define STR_OSD_TIM_SOURCE_4          "TR2(TIL/AKT , TIL/AKTIVERET )"       	// Max length: 31; description
#define STR_OSD_STAT_MAX_ALTITUDE     "TR2(MAX HOEJDE, MAXIMAL HOEJDE)"     	// Max length: 33; description
#define STR_OSD_STAT_MAX_SPEED        "TR2(MAX FART, MAXIMAL FART)"         	// Max length: 29; description
#define STR_OSD_STAT_MAX_DISTANCE     "TR2(MAX DISTANCE, MAXIMAL DISTANCE)" 	// Max length: 37; description
#define STR_OSD_STAT_FLIGHT_DISTANCE  "FLYVE DISTANCE"                      	// Max length: 14; description
#define STR_OSD_STAT_MIN_BATTERY_AVG  "MIN GNM CELL"                        	// Max length: 12; description
#define STR_OSD_STAT_MIN_BATTERY      "MIN BATTERI"                         	// Max length: 11; description
#define STR_OSD_STAT_END_BATTERY_AVG  "END GNM CELL"                        	// Max length: 12; description
#define STR_OSD_STAT_END_BATTERY      "END BATTERI"                         	// Max length: 11; description
#define STR_OSD_STAT_BATTERY_AVG      "GNM BATT CELL"                       	// Max length: 13; description
#define STR_OSD_STAT_BATTERY          "BATTERI"                             	// Max length:  7; description
#define STR_OSD_STAT_MIN_RSSI         "MIN RSSI"                            	// Max length:  8; description
#define STR_OSD_STAT_MAX_CURRENT      "TR2(MAX STROEM, MAXIMAL STROEM)"     	// Max length: 33; description
#define STR_OSD_STAT_USED_MAH         "BRUGT MAH"                           	// Max length:  9; description
#define STR_OSD_STAT_WATT_HOURS_DRAWN "BRUGT WATT TIMER"                    	// Max length: 16; description
#define STR_OSD_STAT_BLACKBOX         "BLACKBOX"                            	// Max length:  8; description
#define STR_OSD_STAT_BLACKBOX_NUMBER  "BB LOG NUM"                          	// Max length: 10; description
#define STR_OSD_STAT_MAX_G_FORCE      "TR2(MAX G-KRAFT, MAXIMAL G-KRAFT)"   	// Max length: 35; description
#define STR_OSD_STAT_MAX_ESC_TEMP     "TR2(MAX ESC TEMP, MAXIMAL ESC TEMP)" 	// Max length: 37; description
#define STR_OSD_STAT_MAX_ESC_RPM      "TR2(MAX ESC RPM, MAXIMAL ESV RPM)"   	// Max length: 35; description
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"                            	// Max length:  8; description
#define STR_OSD_STAT_MAX_FFT          "SPIDS FFT"                           	// Max length:  9; description
#define STR_OSD_STAT_MAX_FFT_THRT     "GAS<20%"                             	// Max length:  7; description
#define STR_OSD_STAT_MIN_RSSI_DBM     "MIN RSSI DBM"                        	// Max length: 12; description
#define STR_OSD_STAT_MIN_RSNR         "MIN RSNR"                            	// Max length:  8; description
#define STR_USE_PERSISTENT_STATS      "TOTALE FLYVNINGER"                   	// Max length: 17; description
#define STR_OSD_STAT_TOTAL_TIME       "TOTALE FLYVE TID"                    	// Max length: 16; description
#define STR_OSD_STAT_TOTAL_DIST       "TOTAL DISTANCE"                      	// Max length: 14; description
#define STR_OSDW_CRASH_FLIP_WARNING   "> CRASH FLIP <"                      	// Max length: 14; description
#define STR_OSDW_BEACON_ON            " BEACON TIL"                         	// Max length: 11; description
#define STR_OSDW_ARM_IN               "ARM IN"                              	// Max length:  6; description
#define STR_OSDW_FAIL_SAFE            "FEJLSIKRING"                         	// Max length: 11; description
#define STR_OSDW_CRASH_FLIP           "CRASH FLIP SWITCH"                   	// Max length: 17; description
#define STR_OSDW_LAUNCH               "LAUNCH"                              	// Max length:  6; description
#define STR_OSDW_RSSI_LOW             "RSSI LAV"                            	// Max length:  8; description
#define STR_OSDW_RSSI_DBM             "RSSI DBM"                            	// Max length:  8; description
#define STR_OSDW_RSNR_LOW             "RSNR LAV"                            	// Max length:  8; description
#define STR_OSDW_LINK_QUA             "LINK KVALITET"                       	// Max length: 13; description
#define STR_OSDW_LAND_NOW             " LAND NU"                            	// Max length:  8; description
#define STR_OSDW_RESCUE_NA            "TR2(FEJLSIKRING N/A, FEJLSIKRING IKKE MULIG)" 	// Max length: 46; description
#define STR_OSDW_RESCUE_OFF           "FEJLSIKRING FRA"                     	// Max length: 15; description
#define STR_OSDW_HEADFREE             "HEADFREE"                            	// Max length:  8; description
#define STR_OSDW_CORE                 "CORE"                                	// Max length:  4; description
#define STR_OSDW_LOW_BATT             "BATTERI LAVT"                        	// Max length: 12; description
#define STR_OSDW_RCSMOOTHING          "RCSMOOTHING"                         	// Max length: 11; description
#define STR_OSDW_OVER_CAP             "OVER CAP"                            	// Max length:  8; description
#define STR_OSDW_BATT_CONTINUE        "BATTERI FORTSAET"                    	// Max length: 16; description
#define STR_OSDW_BATT_BELOW_FULL      "BATTERI < FULDT"                     	// Max length: 15; description
#define STR_OSDE_DISARMED             "TR2(EJ AKTIV, *** DEAKTIVERET ***)"  	// Max length: 36; description
#define STR_OSDE_UP                   "O"                                   	// Max length:  1; description
#define STR_OSDE_DOWN                 "N"                                   	// Max length:  1; description
#define STR_OSDE_GPS_WEST             "V"                                   	// Max length:  1; description
#define STR_OSDE_GPS_EAST             "E"                                   	// Max length:  2; description
#define STR_OSDE_GPS_SOUTH            "S"                                   	// Max length:  1; description
#define STR_OSDE_GPS_NORTH            "N"                                   	// Max length:  1; description
#define STR_OSDE_PILOT_NAME           "PILOT_NAVN"                          	// Max length: 10; description
#define STR_OSDE_RATE                 "TR2(HAST_, HASTIGHED_)"              	// Max length: 24; description
#define STR_OSDE_PID                  "PID_"                                	// Max length:  4; description
#define STR_OSDE_OID                  "OID_"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_FAILSAFE     "!FS!"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_RESCUE       "RESC"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_HEAD         "HEAD"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_ANGL         "ANGL"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_HOR          "HOR "                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_ATRN         "ATRN"                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_AIR          "AIR "                                	// Max length:  4; description
#define STR_ODSE_FLYMODE_ACRO         "ACRO"                                	// Max length:  4; description
#define STR_ODSE_ELEMENT_PIT          "HOEJ"                                	// Max length:  4; description
#define STR_ODSE_ELEMENT_ROL          "KRAE"                                	// Max length:  4; description
#define STR_ODSE_ELEMENT_YAW          "SID"                                 	// Max length:  3; description
#define STR_ODSE_ANTIGRAVITY          "AT // Anti Tyndekraft"               	// Max length: 21; description
#define STR_OSDE_READY                "KLAR"                                	// Max length:  4; description
#define STR_LOCALE_SETUP              "Sprog:"                              	// Max length:  6; description
#define STR_CLI_ERROR_INVALID_NAME    "UGYLDIGT NAVN:"                      	// Max length: 14; description
#define STR_CLI_ERROR_MESSAGE         "KAN IKKE AENDRES. AKTUEL VAERDI:"    	// Max length: 32; description
#define STR_CLI_PARSINGFAIL           "PARSING FEJLET"                      	// Max length: 14; description
#define STR_CLI_INVALIDRESSOURCE      "UGYLDIGT RESOURCE NAVN:"             	// Max length: 23; description
#define STR_CLI_ARG_INVALIDCOUNT      "UGYLDIGT ANTAL ARGUMENTER"           	// Max length: 25; description
#define STR_CLI_ARG_NOTBETWEEN        "IKKE MELLEM"                         	// Max length: 11; description
#define STR_CLI_ARG_AND               "OG"                                  	// Max length:  2; description
#define STR_CLI_ARG_OUTOFRANGE        "ARGUMENT UDENFOR INTERVAL"           	// Max length: 25; description
#define STR_CLI_DSHOT_READ            "Dshot laest:"                        	// Max length: 12; description
#define STR_CLI_DSHOT_INVALID_PKT     "Dshot ugyldig pakker:"               	// Max length: 21; description
#define STR_CLI_DSHOT_DIR_CHANGE      "Dshot retning Skift cyklus:"         	// Max length: 27; description
#define STR_CLI_DSHOT_DIR_CHANGE_MIC  ", mikro:"                            	// Max length:  8; description
#define STR_CLI_DSHOT_HEAD1           "Motor    Type   eRPM    RPM     Hz    Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3" 	// Max length:115; description
#define STR_CLI_DSHOT_LINE1           "=====  ====== ====== ====== ====== ========== ====== ====== ====== ====== ====== ====== ======" 	// Max length:115; description
#define STR_CLI_DSHOT_HEAD2           "Motor    Type   eRPM    RPM     Hz   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3" 	// Max length:115; description
#define STR_CLI_DSHOT_LINE2           "=====  ====== ====== ====== ====== ====== ====== ====== ====== ====== ====== ======" 	// Max length:115; description
#define STR_CLI_DSHOT_NO_TELEM        "Dshot telemetri ikke aktiv"          	// Max length: 26; description
#define STR_CLI_STORAGE_NOTFOUND      "Masselager findes ikke eller kan ikke aktiveres!" 	// Max length: 48; description
#define STR_CLI_NO_MATCH              "INGEN MATCH FOR:"                    	// Max length: 16; description
#define STR_CLI_ERROR_FOUND           "FEJL ER FUNDET - UNDERSOEG INDEN DU FORTSAETTER" 	// Max length: 47; description
#define STR_CLI_FIX_ERROR             "RET FEJL, INDEN GEM MED 'SAVE'"      	// Max length: 30; description
#define STR_CLI_STATUS_MCU            "MCU:"                                	// Max length:  4; description
#define STR_CLI_STATUS_CLOCK          "klokke="                             	// Max length:  7; description
#define STR_CLI_STATUS_VREF           "Vref="                               	// Max length:  5; description
#define STR_CLI_STATUS_CORETEMP       "kerne temp="                         	// Max length: 11; description
#define STR_CLI_STATUS_STACK_SIZE     "Stak stoerrelse:"                    	// Max length: 16; description
#define STR_CLI_STATUS_STACK_ADDR     "Stak adresse:"                       	// Max length: 13; description
#define STR_CLI_STATUS_STACK_USED     "Stak brugt:"                         	// Max length: 11; description
#define STR_CLI_STATUS_CONFIG         "Konfiguration:"                      	// Max length: 14; description
#define STR_CLI_STATUS_CONFIG_SIZE    "stoerrelse:"                         	// Max length: 11; description
#define STR_CLI_STATUS_CONFIG_AVAIL   "max anvendeligt:"                    	// Max length: 16; description
#define STR_CLI_STATUS_DEVICES        "Enheder opdaget:"                    	// Max length: 16; description
#define STR_CLI_STATUS_SPI            " SPI:"                               	// Max length:  5; description
#define STR_CLI_STATUS_I2C            " I2C:"                               	// Max length:  5; description
#define STR_CLI_STATUS_I2C_ERRORS     "I2C fejl:"                           	// Max length:  9; description
#define STR_CLI_STATUS_GYRO_DETECT    "Gyro opdaget:"                       	// Max length: 13; description
#define STR_CLI_STATUS_GYRO           " gyro"                               	// Max length:  5; description
#define STR_CLI_STATUS_GYRO_LOCKED    " laast"                              	// Max length:  6; description
#define STR_CLI_STATUS_GYRO_DMA       " dma"                                	// Max length:  4; description
#define STR_CLI_STATUS_GYRO_SHARED    " delt"                               	// Max length:  5; description
#define STR_CLI_STATUS_OSD            "OSD:"                                	// Max length:  4; description
#define STR_CLI_STATUS_BUILD_KEY      "RELEASE BYG ID:"                     	// Max length: 15; description
#define STR_CLI_STATUS_SYSTEM_UPTIME  "System oppetid:"                     	// Max length: 15; description
#define STR_CLI_STATUS_TIME_CURRENT   ", system tid:"                       	// Max length: 13; description
#define STR_CLI_STATUS_CPU            "CPU:"                                	// Max length:  4; description
#define STR_CLI_STATUS_CPU_CYCLE      "cyklus tid:"                         	// Max length: 11; description
#define STR_CLI_STATUS_CPU_GYRO       "GYRO hast.:"                         	// Max length: 11; description
#define STR_CLI_STATUS_CPU_RX         "RX hast.:"                           	// Max length:  9; description
#define STR_CLI_STATUS_CPU_SYSTEM     "System hast.:"                       	// Max length: 13; description
#define STR_CLI_STATUS_BAT_VOLTAGE    "Spaending:"                          	// Max length: 10; description
#define STR_CLI_STATUS_BAT_TYPE       "batteri"                             	// Max length:  7; description
#define STR_CLI_STATUS_FLASH          "FLASH: JEDEC ID="                    	// Max length: 16; description
#define STR_CLI_TASK_LATE_LIST        "Task liste            rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us" 	// Max length:115; description
#define STR_CLI_TASK_LIST             "Task liste            rate/hz  max/us  avg/us maxload avgload  total/ms" 	// Max length:100; description
#define STR_CLI_TASK_MINIMAL          "Task liste"                          	// Max length: 10; description
#define STR_CLI_TASK_RX_CHECK         "RX check funktion"                   	// Max length: 17; description
#define STR_CLI_TASK_TOTAL            "Total (eksklusiv SERIEL)"            	// Max length: 24; description
#define STR_CLI_TASK_SCHEDULER_START  "Scheduler start cykler"              	// Max length: 22; description
#define STR_CLI_TASK_SCHEDULER_GAURD  "guard cykler"                        	// Max length: 12; description
#define STR_CLI_VERSION_HAS_CONFIG    "Konfig: JA"                          	// Max length: 10; description
#define STR_CLI_VERSION_NO_CONFIG     "TR2(INGEN KONFIG, INGEN KONFIGURATION FUNDET)" 	// Max length: 47; description
#define STR_CLI_VERSION_INFO_MANUF    "# FC: manufacturer_id:"              	// Max length: 22; description
#define STR_CLI_VERSION_INFO_BOARD    "board_name:"                         	// Max length: 11; description
#define STR_CLI_NONE                  "INGEN"                               	// Max length:  5; description
#define STR_CLI_TIMER                 "timer"                               	// Max length:  5; description
#define STR_CLI_TIMERS                "Timers:"                             	// Max length:  7; description
#define STR_CLI_TIMERS_ACTIVE         "Aktiv tidtagning:"                   	// Max length: 17; description
#define STR_CLI_TIMERS_FREE           " FRI"                                	// Max length:  4; description
#define STR_CLI_STATUS_ARM_DISABLE    "Aktiver IKKE flag:"                  	// Max length: 18; description
#define STR_CLI_DSHOT_TEL_INFO        " INGEN DATA"                         	// Max length: 11; description
#define STR_CLI_MSC_TIMEZONE          "UGYLDIG TIDSZONE OFFSET"             	// Max length: 23; description
#define STR_CLI_MSC_RESTART           "Skifter til masselager tilstand"     	// Max length: 31; description
#define STR_CLI_MSC_REBOOT            "\r\nGenstarter"                      	// Max length: 14; description
#define STR_CLI_PROCESS               "UKENDT KOMMANDO, TAST 'HELP'"        	// Max length: 28; description
#define STR_CLI_ENTER_LONG            "\r\nCLI tilstand, tast 'exit' for at forlade, eller 'help' for vejledning" 	// Max length: 73; description
#define STR_CLI_ENTER_SHORT           "\r\nCLI"                             	// Max length:  7; description
#define STR_CLI_SDCARD_NAME           "SD kort: "                           	// Max length:  9; description
#define STR_CLI_SDCARD_NOCONFIG       "Ikke konfigureret"                   	// Max length: 17; description
#define STR_CLI_SDCARD_NONE           "Ikke indsat"                         	// Max length: 11; description
#define STR_CLI_SDCARD_NOSTART        "Startop fejlet"                      	// Max length: 14; description
#define STR_CLI_SDCARD_MANUFAC        "Producent"                           	// Max length:  9; description
#define STR_CLI_SDCARD_FILESYSTEM     "\r\nFilsystem: "                     	// Max length: 15; description
#define STR_CLI_SDCARD_READY          "Klart"                               	// Max length:  5; description
#define STR_CLI_SDCARD_INIT           "Initialiserer"                       	// Max length: 13; description
#define STR_CLI_SDCARD_FATAL          "Fejlet helt"                         	// Max length: 11; description
#define STR_CLI_SDCARD_NOFATMBR       " - ingen FAT MBR partitioner"        	// Max length: 28; description
#define STR_CLI_SDCARD_BADFAT         " - fejlagtig FAT blok"               	// Max length: 21; description
#define STR_CLI_FLASH_SEC             "Flash sektorer="                     	// Max length: 15; description
#define STR_CLI_FLASH_SEC_SIZE        "sektorStoerrelse="                   	// Max length: 17; description
#define STR_CLI_FLASH_SEC_PAGE        "siderPerSektor="                     	// Max length: 15; description
#define STR_CLI_FLASH_PAGE_SIZE       "sideStoerrelse="                     	// Max length: 15; description
#define STR_CLI_FLASH_TOTAL_SIZE      "totalStoerrelse="                    	// Max length: 16; description
#define STR_CLI_FLASH_JEDEC           "JEDEC ID="                           	// Max length:  9; description
#define STR_CLI_FLASH_PART            "Partitioner:"                        	// Max length: 12; description
#define STR_CLI_FLASH_SIZE            "FlashFS stoerrelse="                 	// Max length: 19; description
#define STR_CLI_FLASH_SIZE_USED       "brugtStoerrelse="                    	// Max length: 16; description
#define STR_CLI_FLASH_ER_LONG         "Sletter, vent venligst ... "         	// Max length: 27; description
#define STR_CLI_FLASH_ER_SHORT        "Sletter,"                            	// Max length:  8; description
#define STR_CLI_FLASH_ER_DONE         "Faerdig."                            	// Max length:  8; description
#define STR_CLI_FLASH_VR              "Kontrollerer"                        	// Max length: 12; description
#define STR_CLI_FLASH_VR_OK           "Succes"                              	// Max length:  6; description
#define STR_CLI_FLASH_VR_FAIL         "Fejlet"                              	// Max length:  6; description
#define STR_CLI_FLASH_WROTE           "Skrevet"                             	// Max length:  7; description
#define STR_CLI_FLASH_READ            "Laest"                               	// Max length:  5; description
#define STR_CLI_FLASH_BYTES_AT        "bytes ved:"                          	// Max length: 10; description
#define STR_CLI_ENABLED               "Aktiveret"                           	// Max length:  9; description
#define STR_CLI_DISABLED              "Deaktiveret"                         	// Max length: 11; description
#define STR_CLI_AVAILABLE             "Mulige:"                             	// Max length:  7; description
#define STR_CLI_UNAVAILABLE           "Ikke mulig"                          	// Max length: 10; description
