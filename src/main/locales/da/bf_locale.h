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

/*
	NOTICE!
	NOTICE: this header file for LOCALE 'da' is generated from da/bf_locale.xml
	Changes to translations (i.e., da/bf_locale.h) must be made in da/bf_locale.xml

	To enable TR2 (HD extended text), write TXT then TXT_HD and build with USE_HD_EXTENDED=1
	To build use 'make <target> LOCALE=da'
*/

#pragma once

#define LOCALE                             "da"                                       	// Current language in short form: en, da, es, fr, nl ...
#define STR_LANGUAGE                       "Sprog:"                                   	// translation of: Language:
#define STR_COMMA                          ","                                        	// translation of: Comma string ,
#define STR_PERIOD                         "."                                        	// translation of: Period string .
#define STR_THOUSAND                       "."                                        	// translation of: Thousand delimiter .
#define STR_SECONDS                        "sekund"                                   	// translation of: seconds
#define STR_CMS_STARTUP_MENU_TEXT1         "MENU:GAS MID"                             	// translation of: MENU:THR MID
#define STR_CMS_STARTUP_MENU_TEXT2         "+ SIDE VENST"                             	// translation of: + YAW LEFT  
#define STR_CMS_STARTUP_MENU_TEXT3         "+ HOEJDE OP"                              	// translation of: + PITCH UP  
#define STR_OSD_ARMED                      TR2("AKTIV", "** AKTIV **")                	// translation of: ARMED; HD translation of: *** ARMED ***
#define STR_OSDE_DISARMED                  TR2("EJ AKTIV", "** DE-AKTIV **")          	// translation of: DISARMED; HD translation of: *** DISARMED ***
#define STR_OSD_STATS                      TR2("- STATISTIK -", "--- STATISTIK ---")  	// translation of: --- STATS ---; HD translation of: --- STATISTICS ---
#define STR_OSD_TIMER_ON_TIME              TR2("ON TIME ", "ON TIME ")                	// translation of: ON TIME  ; HD translation of: ON TIME  
#define STR_OSD_TIMER_TOTAL_ARM            TR2("TOTAL AKT", "TOTAL AKTIV")            	// translation of: TOTAL ARM; HD translation of: TOTAL ARMED
#define STR_OSD_TIMER_LAST_ARM             TR2("SIDST AKT", "SIDST AKTIV")            	// translation of: LAST ARM ; HD translation of: LAST ARMED 
#define STR_OSD_TIMER_ON_ARM               TR2("TIL/AKT ", "TIL/AKTIV  ")             	// translation of: ON/ARM   ; HD translation of: ON/ARMED   
#define STR_OSD_STAT_MAX_ALTITUDE          TR2("MAX HOEJDE", "MAXIMAL HOEJDE")        	// translation of: MAX ALTITUDE; HD translation of: MAXIMAL ALTITUDE
#define STR_OSD_STAT_MAX_SPEED             TR2("MAX FART    ", "MAXIMAL FART")        	// translation of: MAX SPEED   ; HD translation of: MAXIMAL SPEED   
#define STR_OSD_STAT_MAX_DISTANCE          TR2("MAX DISTANCE", "MAXIMAL DISTANCE")    	// translation of: MAX DISTANCE; HD translation of: MAXIMAL DISTANCE
#define STR_OSD_STAT_FLIGHT_DISTANCE       "FLYVE DISTANCE"                           	// translation of: FLIGHT DISTANCE
#define STR_OSD_STAT_MIN_BATTERY_AVG       "MIN GNM CELL"                             	// translation of: MIN AVG CELL
#define STR_OSD_STAT_MIN_BATTERY           "MIN BATTERI"                              	// translation of: MIN BATTERY
#define STR_OSD_STAT_END_BATTERY_AVG       "END GNM CELL"                             	// translation of: END AVG CELL
#define STR_OSD_STAT_END_BATTERY           "END BATTERI"                              	// translation of: END BATTERY
#define STR_OSD_STAT_BATTERY_AVG           "GNM BATT CELL"                            	// translation of: AVG BATT CELL
#define STR_OSD_STAT_BATTERY               "BATTERI"                                  	// translation of: BATTERY
#define STR_OSD_STAT_MIN_RSSI              "MIN RSSI"                                 	// translation of: MIN RSSI
#define STR_OSD_STAT_MAX_CURRENT           TR2("MAX STROEM", "MAXIMAL STROEM")        	// translation of: MAX CURRENT; HD translation of: MAXIMAL CURRENT
#define STR_OSD_STAT_USED_MAH              "BRUGT MAH"                                	// translation of: USED MAH
#define STR_OSD_STAT_WATT_HOURS_DRAWN      "BRUGT WATT TIME"                          	// translation of: USED WATT HOURS
#define STR_OSD_STAT_BLACKBOX              "BLACKBOX"                                 	// translation of: BLACKBOX
#define STR_OSD_STAT_BLACKBOX_NUMBER       TR2("BB LOG NUM", "BLACKBOX LOG #")        	// translation of: BB LOG NUM; HD translation of: BLACKBOX LOG NUMBER
#define STR_OSD_STAT_MAX_G_FORCE           TR2("MAX G-KRAFT", "MAXIMAL G-KRAFT")      	// translation of: MAX G-FORCE; HD translation of: MAXIMAL G-FORCE
#define STR_OSD_STAT_MAX_ESC_TEMP          TR2("MAX ESC TEMP", "MAXIMAL ESC TEMP")    	// translation of: MAX ESC TEMP; HD translation of: MAXIMAL ESC TEMP
#define STR_OSD_STAT_MAX_ESC_RPM           TR2("MAX ESC RPM", "MAXIMAL ESC RPM")      	// translation of: MAX ESC RPM; HD translation of: MAXIMAL ESC RPM
#define STR_OSD_STAT_MIN_LINK_QUALITY      "MIN LINK"                                 	// translation of: MIN LINK
#define STR_OSD_STAT_MAX_FFT               "TOP MAX"                                  	// translation of: PEAK FFT
#define STR_OSD_STAT_MAX_FFT_THRT          "GAS<20%"                                  	// translation of: THRT<20%
#define STR_OSD_STAT_MIN_RSSI_DBM          "MIN RSSI DBM"                             	// translation of: MIN RSSI DBM
#define STR_OSD_STAT_MIN_RSNR              "MIN RSNR"                                 	// translation of: MIN RSNR
#define STR_USE_PERSISTENT_STATS           "TOTAL FLYVNING"                           	// translation of: TOTAL FLIGHTS
#define STR_OSD_STAT_TOTAL_TIME            "TOTAL FLYVE TID"                          	// translation of: TOTAL FLIGHT TIME
#define STR_OSD_STAT_TOTAL_DIST            "TOTAL DISTANCE"                           	// translation of: TOTAL DISTANCE
#define STR_OSDW_CRASH_FLIP_WARNING        ">CRASH FLIP<"                             	// translation of: >CRASH FLIP<
#define STR_OSDW_CRASH_FLIP_SWITCH         "CRASH FLIP +"                             	// translation of: CRASH FLIP SWITCH
#define STR_OSDW_BEACON_ON                 " BEACON +"                                	// translation of:  BEACON ON
#define STR_OSDW_ARM_IN                    TR2("AKTIV", "AKTIV I")                    	// translation of: ARM IN; HD translation of: ARMED IN
#define STR_OSDW_FAIL_SAFE                 "FEJLSIKR"                                 	// translation of: FAIL SAFE
#define STR_OSDW_LAUNCH                    "LAUNCH"                                   	// translation of: LAUNCH
#define STR_OSDW_RSSI_LOW                  "RSSI LAV"                                 	// translation of: RSSI LOW
#define STR_OSDW_RSSI_DBM                  "RSSI DBM"                                 	// translation of: RSSI DBM
#define STR_OSDW_RSNR_LOW                  "RSNR LAV"                                 	// translation of: RSNR LOW
#define STR_OSDW_LINK_QUALITY              "LINK KVALI"                               	// translation of: LINK QUALITY
#define STR_OSDW_LAND_NOW                  " LAND NU"                                 	// translation of:  LAND NOW
#define STR_OSDW_CPU_OVERLOAD              "CPU BELASTET"                             	// translation of: CPU OVERLOAD
#define STR_OSDW_RESCUE_NA                 TR2("FEJLSIK NA", "FEJLSIKR IKKE MULIG")   	// translation of: RESCUE N/A; HD translation of: RESCUE NOT AVAILABLE
#define STR_OSDW_RESCUE_OFF                "FEJLSIKR FRA"                             	// translation of: RESCUE OFF
#define STR_OSDW_POSHOLD_FAIL              "POSHOLD FEJL"                             	// translation of: POSHOLD FAIL
#define STR_OSDW_HEADFREE                  "HEADFREE"                                 	// translation of: HEADFREE
#define STR_OSDW_CORE                      "CORE"                                     	// translation of: CORE
#define STR_OSDW_LOW_BATT                  "BATTERI LAV"                              	// translation of: LOW BATTERY
#define STR_OSDW_OVER_CAP                  "OVER CAP"                                 	// translation of: OVER CAP
#define STR_OSDW_BATT_CONTINUE             "BATT FORTSAET"                            	// translation of: BATTERY CONTINUE
#define STR_OSDW_BATT_BELOW_FULL           "BATT < FULD"                              	// translation of: BATT < FULL
#define STR_OSDW_VISUAL_BEEP               "  * * * *"                                	// translation of:   * * * *
#define STR_OSDW_CHIRP_EXC_FINISHED        "CHIRP EXC SLUT"                           	// translation of: CHIRP EXC FINISHED
#define STR_OSDE_UP                        "O"                                        	// translation of: U
#define STR_OSDE_DOWN                      "N"                                        	// translation of: D
#define STR_OSDE_GPS_DIRECTION             "NSEV"                                     	// translation of EXACTLY 4 chars in order: N S E W
#define STR_OSDE_PILOT_NAME                "PILOT_NAVN"                               	// translation of: PILOT NAME
#define STR_OSDE_RATE                      TR2("HAST_", "HASTIGHED_")                 	// translation of: RATE_, notice %u follow; HD translation of: RATE_, notice %u follow
#define STR_OSDE_PID                       "PID_"                                     	// translation of: PID_, notice %u follow
#define STR_OSDE_OID                       "OID_"                                     	// translation of: OID_, notice %u follow
#define STR_OSDE_ANTIGRAVITY               TR2("AG", "ANTI G")                        	// translation of: AG; HD translation of: ANTI G
#define STR_OSDE_READY                     TR2("KLAR", "KLAR")                        	// translation of: READY; translation of: READY
#define STR_OSDE_FLYMODE_FAILSAFE          "!FS!"                                     	// translation of: !FS!
#define STR_OSDE_FLYMODE_RESCUE            "RESC"                                     	// translation of: RESC
#define STR_OSDE_FLYMODE_HEAD              "HEAD"                                     	// translation of: HEAD
#define STR_OSDE_FLYMODE_PASS              "PASS"                                     	// translation of: PASS
#define STR_OSDE_FLYMODE_POSH              "POSH"                                     	// translation of: POSH
#define STR_OSDE_FLYMODE_ALTH              "ALTH"                                     	// translation of: ALTH
#define STR_OSDE_FLYMODE_ANGL              "ANGL"                                     	// translation of: ANGL
#define STR_OSDE_FLYMODE_HOR               "HOR "                                     	// translation of: HOR 
#define STR_OSDE_FLYMODE_ATRN              "ATRN"                                     	// translation of: ATRN
#define STR_OSDE_FLYMODE_CHIR              "CHIR"                                     	// translation of: CHIR
#define STR_OSDE_FLYMODE_AIR               "AIR "                                     	// translation of: AIR 
#define STR_OSDE_FLYMODE_ACRO              "ACRO"                                     	// translation of: ACRO
#define STR_OSDE_ELEMENT_PITCH             "PIT"                                      	// translation of: PIT
#define STR_OSDE_ELEMENT_ROLL              "ROL"                                      	// translation of: ROL
#define STR_OSDE_ELEMENT_YAW               "YAW"                                      	// translation of: YAW
#define STR_OSDE_RPM_LIMIT_ON              "RPM MAX TIL"                              	// translation of: RPM LIMIT ON
#define STR_OSDE_RPM_LIMIT_OFF             "RPM MAX FRA"                              	// translation of: RPM LIMIT OFF
#define STR_OSDE_POLES                     "POLER"                                    	// translation of: POLES
#define STR_OSDE_THR_LIMIT                 "GAS MAX"                                  	// translation of: THR LIMIT
#define STR_OSDE_MOTOR_LIMIT               "MOTOR MAX"                                	// translation of: MOTOR LIMIT
