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
