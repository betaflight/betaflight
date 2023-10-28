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

#include "drivers/osd.h"

#define LOCALE                        "da"                                  	// Max length:  2; Current language in short form: en, da, es, fr, nl ...;
#define STR_LOCALE_SETUP              "Sprog:"                              	// Max length: 10; translation of: Language:;
#define STR_COMMA                     ","                                   	// Max length:  1; translation of: Comma string ,;
#define STR_PERIOD                    "."                                   	// Max length:  1; translation of: Period string .;
#define STR_THOUSAND                  "."                                   	// Max length:  1; translation of: Thousand delimiter .;
#define STR_SECONDS                   "sekunder"                            	// Max length:  8; translation of: seconds;
#define STR_VISUAL_BEEP               "  * * * *"                           	// Max length: 10; translation of:   * * * *;
#define CMS_STARTUP_HELP_TEXT1        "MENU:GAS MIDT"                       	// Max length: 20; translation of: MENU:THR MID;
#define CMS_STARTUP_HELP_TEXT2        " + SIDEROR VENSTRE"                  	// Max length: 20; translation of: + YAW LEFT  ;
#define CMS_STARTUP_HELP_TEXT3        " + HOEJDE OP"                        	// Max length: 20; translation of: + PITCH UP  ;
//#define STR_OSD_ARMED                 (TR_TEST ? "AKTIVERET" : "*** AKTIVERET ***") 	// Max length: 10; translation of: ARMED; HD> Max length: 20; HD translation of: *** ARMED ***
const char* STR_OSD_ARMED[] = { "AKTIVERET", "*** AKTIVERET ***" }; 	// Max length: 10; translation of: ARMED; HD> Max length: 20; HD translation of: *** ARMED ***
#define STR_OSD_STATS                 (TR_TEST ? "- STATISTIK -" : "--- STATISTIK ---") 	// Max length: 39; description; HD> Max length: 39; description
#define STR_OSD_TIM_SOURCE_1          (TR_TEST ? "ON TIME " : "ON TIME ")   	// Max length: 25; description; HD> Max length: 25; description
#define STR_OSD_TIM_SOURCE_2          (TR_TEST ? "TOTAL AKT" : "TOTAL AKTIVERET") 	// Max length: 33; description; HD> Max length: 33; description
#define STR_OSD_TIM_SOURCE_3          (TR_TEST ? "SIDST AKT" : "SIDST AKTIVERET") 	// Max length: 33; description; HD> Max length: 33; description
#define STR_OSD_TIM_SOURCE_4          (TR_TEST ? "TIL/AKT " : "TIL/AKTIVERET ") 	// Max length: 31; description; HD> Max length: 31; description
#define STR_OSD_STAT_MAX_ALTITUDE     (TR_TEST ? "MAX HOEJDE" : "MAXIMAL HOEJDE") 	// Max length: 33; description; HD> Max length: 33; description
#define STR_OSD_STAT_MAX_SPEED        (TR_TEST ? "MAX FART" : "MAXIMAL FART") 	// Max length: 29; description; HD> Max length: 29; description
#define STR_OSD_STAT_MAX_DISTANCE     (TR_TEST ? "MAX DISTANCE" : "MAXIMAL DISTANCE") 	// Max length: 37; description; HD> Max length: 37; description
//#define STR_OSD_STAT_FLIGHT_DISTANCE  "FLYVE DISTANCE"                      	// Max length: 14; description;
const char* STR_OSD_STAT_FLIGHT_DISTANCE[] = { "FLYVE DISTANCE" };                      	// Max length: 14; description;
#define STR_OSD_STAT_MIN_BATTERY_AVG  "MIN GNM CELL"                        	// Max length: 12; description;
#define STR_OSD_STAT_MIN_BATTERY      "MIN BATTERI"                         	// Max length: 11; description;
#define STR_OSD_STAT_END_BATTERY_AVG  "END GNM CELL"                        	// Max length: 12; description;
#define STR_OSD_STAT_END_BATTERY      "END BATTERI"                         	// Max length: 11; description;
#define STR_OSD_STAT_BATTERY_AVG      "GNM BATT CELL"                       	// Max length: 13; description;
#define STR_OSD_STAT_BATTERY          "BATTERI"                             	// Max length:  7; description;
#define STR_OSD_STAT_MIN_RSSI         "MIN RSSI"                            	// Max length:  8; description;
#define STR_OSD_STAT_MAX_CURRENT      (TR_TEST ? "MAX STROEM" : "MAXIMAL STROEM") 	// Max length: 33; description; HD> Max length: 33; description
#define STR_OSD_STAT_USED_MAH         "BRUGT MAH"                           	// Max length:  9; description;
#define STR_OSD_STAT_WATT_HOURS_DRAWN "BRUGT WATT TIMER"                    	// Max length: 16; description;
#define STR_OSD_STAT_BLACKBOX         "BLACKBOX"                            	// Max length:  8; description;
#define STR_OSD_STAT_BLACKBOX_NUMBER  "BB LOG NUM"                          	// Max length: 10; description;
#define STR_OSD_STAT_MAX_G_FORCE      (TR_TEST ? "MAX G-KRAFT" : "MAXIMAL G-KRAFT") 	// Max length: 35; description; HD> Max length: 35; description
#define STR_OSD_STAT_MAX_ESC_TEMP     (TR_TEST ? "MAX ESC TEMP" : "MAXIMAL ESC TEMP") 	// Max length: 37; description; HD> Max length: 37; description
#define STR_OSD_STAT_MAX_ESC_RPM      (TR_TEST ? "MAX ESC RPM" : "MAXIMAL ESV RPM") 	// Max length: 35; description; HD> Max length: 35; description
#define STR_OSD_STAT_MIN_LINK_QUALITY "MIN LINK"                            	// Max length:  8; description;
#define STR_OSD_STAT_MAX_FFT          "SPIDS FFT"                           	// Max length:  9; description;
#define STR_OSD_STAT_MAX_FFT_THRT     "GAS<20%"                             	// Max length:  7; description;
#define STR_OSD_STAT_MIN_RSSI_DBM     "MIN RSSI DBM"                        	// Max length: 12; description;
#define STR_OSD_STAT_MIN_RSNR         "MIN RSNR"                            	// Max length:  8; description;
#define STR_USE_PERSISTENT_STATS      "TOTALE FLYVNINGER"                   	// Max length: 17; description;
#define STR_OSD_STAT_TOTAL_TIME       "TOTALE FLYVE TID"                    	// Max length: 16; description;
#define STR_OSD_STAT_TOTAL_DIST       "TOTAL DISTANCE"                      	// Max length: 14; description;
#define STR_OSDW_CRASH_FLIP_WARNING   "> CRASH FLIP <"                      	// Max length: 14; description;
#define STR_OSDW_CRASH_FLIP_SWITCH    "CRASH FLIP SWITCH"                   	// Max length: 17; description;
#define STR_OSDW_BEACON_ON            " BEACON TIL"                         	// Max length: 11; description;
#define STR_OSDW_ARM_IN               "ARM IN"                              	// Max length:  6; description;
#define STR_OSDW_FAIL_SAFE            "FEJLSIKRING"                         	// Max length: 11; description;
#define STR_OSDW_LAUNCH               "LAUNCH"                              	// Max length:  6; description;
#define STR_OSDW_RSSI_LOW             "RSSI LAV"                            	// Max length:  8; description;
#define STR_OSDW_RSSI_DBM             "RSSI DBM"                            	// Max length:  8; description;
#define STR_OSDW_RSNR_LOW             "RSNR LAV"                            	// Max length:  8; description;
#define STR_OSDW_LINK_QUALITY         "LINK KVALITET"                       	// Max length: 13; description;
#define STR_OSDW_LAND_NOW             " LAND NU"                            	// Max length:  8; description;
#define STR_OSDW_RESCUE_NA            (TR_TEST ? "FEJLSIKRING N/A" : "FEJLSIKRING IKKE MULIG") 	// Max length: 46; description; HD> Max length: 46; description
#define STR_OSDW_RESCUE_OFF           "FEJLSIKRING FRA"                     	// Max length: 15; description;
#define STR_OSDW_HEADFREE             "HEADFREE"                            	// Max length:  8; description;
#define STR_OSDW_CORE                 "CORE"                                	// Max length:  4; description;
#define STR_OSDW_LOW_BATT             "BATTERI LAVT"                        	// Max length: 12; description;
#define STR_OSDW_RCSMOOTHING          "RCSMOOTHING"                         	// Max length: 11; description;
#define STR_OSDW_OVER_CAP             "OVER CAP"                            	// Max length:  8; description;
#define STR_OSDW_BATT_CONTINUE        "BATTERI FORTSAET"                    	// Max length: 16; description;
#define STR_OSDW_BATT_BELOW_FULL      "BATTERI < FULDT"                     	// Max length: 15; description;
#define STR_OSDE_DISARMED             (TR_TEST ? "EJ AKTIV)" : "*** DEAKTIVERET ***") 	// Max length: 36; description; HD> Max length: 36; description
#define STR_OSDE_UP                   "O"                                   	// Max length:  1; description;
#define STR_OSDE_DOWN                 "N"                                   	// Max length:  1; description;
#define STR_OSDE_GPS_DIRECTION        "NSEV"                                	// Max length:  4; translation of: NSEW;
#define STR_OSDE_PILOT_NAME           "PILOT_NAVN"                          	// Max length: 10; description;
#define STR_OSDE_RATE                 (TR_TEST ? "HAST_" : "HASTIGHED_")    	// Max length: 24; description; HD> Max length: 24; description
#define STR_OSDE_PID                  "PID_"                                	// Max length:  4; description;
#define STR_OSDE_OID                  "OID_"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_FAILSAFE     "!FS!"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_RESCUE       "RESC"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_HEAD         "HEAD"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_ANGL         "ANGL"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_HOR          "HOR "                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_ATRN         "ATRN"                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_AIR          "AIR "                                	// Max length:  4; description;
#define STR_ODSE_FLYMODE_ACRO         "ACRO"                                	// Max length:  4; description;
#define STR_ODSE_ELEMENT_PITCH        "HOEJ"                                	// Max length:  4; description;
#define STR_ODSE_ELEMENT_ROLL         "KRAE"                                	// Max length:  4; description;
#define STR_ODSE_ELEMENT_YAW          "SID"                                 	// Max length:  3; description;
#define STR_ODSE_ANTIGRAVITY          (TR_TEST ? "AT" : "ANTI G")           	// Max length:  2; description; HD> Max length:  6; HD translation of: ANTI G
#define STR_OSDE_READY                "KLAR"                                	// Max length:  4; description;
#define STR_CLI_ERROR_INVALID_NAME    "INVALID NAME:"                       	// Max length: 35; translation of: INVALID NAME;
#define STR_CLI_ERROR_MESSAGE         "CANNOT BE CHANGED. CURRENT VALUE:"   	// Max length: 35; translation of: CANNOT BE CHANGED. CURRENT VALUE:;
