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
	Changes to locale (i.e., da/bf_locale.h) must be made in da/bf_locale.xml

	To enable TR2 (HD extended text), write TXT then TXT_HD and build with USE_HD_EXTENDED=1
	To build use 'make <target> LOCALE=da'
*/

#pragma once

#define LOCALE                             "da"                                       	// Current language in short form: en, da, es, fr, nl ...
#define STR_LANGUAGE                       "Sprog:"                                   	// localization of: Language:
#define STR_COMMA                          ","                                        	// localization of: Comma string ','
#define STR_PERIOD                         "."                                        	// localization of: Period string '.'
#define STR_THOUSAND                       "."                                        	// localization of: Thousand delimiter ','
#define STR_SECONDS                        "sekund"                                   	// localization of: 'seconds'
#define STR_CMS_STARTUP_MENU_TEXT1         "MENU:GAS MID"                             	// localization of: 'MENU:THR MID'
#define STR_CMS_STARTUP_MENU_TEXT2         "+ SIDE VENST"                             	// localization of: '+ YAW LEFT  '
#define STR_CMS_STARTUP_MENU_TEXT3         "+ HOEJDE OP"                              	// localization of: '+ PITCH UP  '
#define STR_OSD_ARMED                      TR2("AKTIV", "** AKTIV **")                	// localization of: 'ARMED'; HD localization of: '*** ARMED ***'
#define STR_OSDE_DISARMED                  TR2("EJ AKTIV", "** DE-AKTIV **")          	// localization of: 'DISARMED'; HD localization of: '*** DISARMED ***'
#define STR_OSD_STATS                      TR2("- STATISTIK -", "--- STATISTIK ---")  	// localization of: '--- STATS ---'; HD localization of: '--- STATISTICS ---'
#define STR_OSD_TIMER_ON_TIME              TR2("ON TIME ", "ON TIME ")                	// localization of: 'ON TIME  '; HD localization of: 'ON TIME  '
#define STR_OSD_TIMER_TOTAL_ARM            TR2("TOTAL AKT", "TOTAL AKTIV")            	// localization of: 'TOTAL ARM'; HD localization of: 'TOTAL ARMED'
#define STR_OSD_TIMER_LAST_ARM             TR2("SIDST AKT", "SIDST AKTIV")            	// localization of: 'LAST ARM '; HD localization of: 'LAST ARMED '
#define STR_OSD_TIMER_ON_ARM               TR2("TIL/AKT ", "TIL/AKTIV  ")             	// localization of: 'ON/ARM   '; HD localization of: 'ON/ARMED   '
#define STR_OSD_STAT_MAX_ALTITUDE          TR2("MAX HOEJDE", "MAXIMAL HOEJDE")        	// localization of: 'MAX ALTITUDE'; HD localization of: 'MAXIMAL ALTITUDE'
#define STR_OSD_STAT_MAX_SPEED             TR2("MAX FART    ", "MAXIMAL FART")        	// localization of: 'MAX SPEED   '; HD localization of: 'MAXIMAL SPEED   '
#define STR_OSD_STAT_MAX_DISTANCE          TR2("MAX DISTANCE", "MAXIMAL DISTANCE")    	// localization of: 'MAX DISTANCE'; HD localization of: 'MAXIMAL DISTANCE'
#define STR_OSD_STAT_FLIGHT_DISTANCE       "FLYVE DISTANCE"                           	// localization of: 'FLIGHT DISTANCE'
#define STR_OSD_STAT_MIN_BATTERY_AVG       "MIN GNM CELL"                             	// localization of: 'MIN AVG CELL'
#define STR_OSD_STAT_MIN_BATTERY           "MIN BATTERI"                              	// localization of: 'MIN BATTERY'
#define STR_OSD_STAT_END_BATTERY_AVG       "END GNM CELL"                             	// localization of: 'END AVG CELL'
#define STR_OSD_STAT_END_BATTERY           "END BATTERI"                              	// localization of: 'END BATTERY'
#define STR_OSD_STAT_BATTERY_AVG           "GNM BATT CELL"                            	// localization of: 'AVG BATT CELL'
#define STR_OSD_STAT_BATTERY               "BATTERI"                                  	// localization of: 'BATTERY'
#define STR_OSD_STAT_MIN_RSSI              "MIN RSSI"                                 	// localization of: 'MIN RSSI'
#define STR_OSD_STAT_MAX_CURRENT           TR2("MAX STROEM", "MAXIMAL STROEM")        	// localization of: 'MAX CURRENT'; HD localization of: 'MAXIMAL CURRENT'
#define STR_OSD_STAT_USED_MAH              "BRUGT MAH"                                	// localization of: 'USED MAH'
#define STR_OSD_STAT_WATT_HOURS_DRAWN      "BRUGT WATT TIME"                          	// localization of: 'USED WATT HOURS'
#define STR_OSD_STAT_BLACKBOX              "BLACKBOX"                                 	// localization of: 'BLACKBOX'
#define STR_OSD_STAT_BLACKBOX_NUMBER       TR2("BB LOG NUM", "BLACKBOX LOG #")        	// localization of: 'BB LOG NUM'; HD localization of: 'BLACKBOX LOG NUMBER'
#define STR_OSD_STAT_MAX_G_FORCE           TR2("MAX G-KRAFT", "MAXIMAL G-KRAFT")      	// localization of: 'MAX G-FORCE'; HD localization of: 'MAXIMAL G-FORCE'
#define STR_OSD_STAT_MAX_ESC_TEMP          TR2("MAX ESC TEMP", "MAXIMAL ESC TEMP")    	// localization of: 'MAX ESC TEMP'; HD localization of: 'MAXIMAL ESC TEMP'
#define STR_OSD_STAT_MAX_ESC_RPM           TR2("MAX ESC RPM", "MAXIMAL ESC RPM")      	// localization of: 'MAX ESC RPM'; HD localization of: 'MAXIMAL ESC RPM'
#define STR_OSD_STAT_MIN_LINK_QUALITY      "MIN LINK"                                 	// localization of: 'MIN LINK'
#define STR_OSD_STAT_MAX_FFT               "TOP MAX"                                  	// localization of: 'PEAK FFT'
#define STR_OSD_STAT_MAX_FFT_THRT          "GAS<20%"                                  	// localization of: 'THRT<20%'
#define STR_OSD_STAT_MIN_RSSI_DBM          "MIN RSSI DBM"                             	// localization of: 'MIN RSSI DBM'
#define STR_OSD_STAT_MIN_RSNR              "MIN RSNR"                                 	// localization of: 'MIN RSNR'
#define STR_USE_PERSISTENT_STATS           "TOTAL FLYVNING"                           	// localization of: 'TOTAL FLIGHTS'
#define STR_OSD_STAT_TOTAL_TIME            "TOTAL FLYVE TID"                          	// localization of: 'TOTAL FLIGHT TIME'
#define STR_OSD_STAT_TOTAL_DIST            "TOTAL DISTANCE"                           	// localization of: 'TOTAL DISTANCE'
#define STR_OSDW_CRASH_FLIP_WARNING        ">CRASH FLIP<"                             	// localization of: '>CRASH FLIP<'
#define STR_OSDW_CRASH_FLIP_SWITCH         "CRASH FLIP +"                             	// localization of: 'CRASH FLIP SWITCH'
#define STR_OSDW_BEACON_ON                 " BEACON +"                                	// localization of: ' BEACON ON'
#define STR_OSDW_ARM_IN                    TR2("AKTIV", "AKTIV I")                    	// localization of: 'ARM IN'; HD localization of: 'ARMED IN'
#define STR_OSDW_FAIL_SAFE                 "FEJLSIKR"                                 	// localization of: 'FAIL SAFE'
#define STR_OSDW_LAUNCH                    "LAUNCH"                                   	// localization of: 'LAUNCH'
#define STR_OSDW_RSSI_LOW                  "RSSI LAV"                                 	// localization of: 'RSSI LOW'
#define STR_OSDW_RSSI_DBM                  "RSSI DBM"                                 	// localization of: 'RSSI DBM'
#define STR_OSDW_RSNR_LOW                  "RSNR LAV"                                 	// localization of: 'RSNR LOW'
#define STR_OSDW_LINK_QUALITY              "LINK KVALI"                               	// localization of: 'LINK QUALITY'
#define STR_OSDW_LAND_NOW                  " LAND NU"                                 	// localization of: ' LAND NOW'
#define STR_OSDW_CPU_OVERLOAD              "CPU BELASTET"                             	// localization of: 'CPU OVERLOAD'
#define STR_OSDW_RESCUE_NA                 TR2("FEJLSIK NA", "FEJLSIKR IKKE MULIG")   	// localization of: 'RESCUE N/A'; HD localization of: 'RESCUE NOT AVAILABLE'
#define STR_OSDW_RESCUE_OFF                "FEJLSIKR FRA"                             	// localization of: 'RESCUE OFF'
#define STR_OSDW_POSHOLD_FAIL              "POSHOLD FEJL"                             	// localization of: 'POSHOLD FAIL'
#define STR_OSDW_HEADFREE                  "HEADFREE"                                 	// localization of: 'HEADFREE'
#define STR_OSDW_CORE                      "KERNE"                                    	// localization of: 'CORE'
#define STR_OSDW_LOW_BATT                  "BATTERI LAV"                              	// localization of: 'LOW BATTERY'
#define STR_OSDW_OVER_CAP                  "OVER CAP"                                 	// localization of: 'OVER CAP'
#define STR_OSDW_BATT_CONTINUE             "BATT FORTSAET"                            	// localization of: 'BATTERY CONTINUE'
#define STR_OSDW_BATT_BELOW_FULL           "BATT < FULD"                              	// localization of: 'BATT < FULL'
#define STR_OSDW_VISUAL_BEEP               "  * * * *"                                	// localization of: '  * * * *'
#define STR_OSDW_CHIRP_EXC_FINISHED        "CHIRP EXC SLUT"                           	// localization of: 'CHIRP EXC FINISHED'
#define STR_OSDE_UP                        "O"                                        	// localization of: 'U'
#define STR_OSDE_DOWN                      "N"                                        	// localization of: 'D'
#define STR_OSDE_GPS_DIRECTION             "NSEV"                                     	// localization of: EXACTLY 4 chars in order: N S E W
#define STR_OSDE_PILOT_NAME                "PILOT_NAVN"                               	// localization of: 'PILOT NAME'
#define STR_OSDE_RATE                      TR2("HAST_", "HASTIGHED_")                 	// localization of: 'RATE_', notice %u follow; HD localization of: 'RATE_', notice %u follow
#define STR_OSDE_PID                       "PID_"                                     	// localization of: 'PID_', notice %u follow
#define STR_OSDE_OID                       "OID_"                                     	// localization of: 'OID_', notice %u follow
#define STR_OSDE_ANTIGRAVITY               TR2("AG", "ANTI G")                        	// localization of: 'AG'; HD localization of: 'ANTI G'
#define STR_OSDE_READY                     TR2("KLAR", "KLAR")                        	// localization of: 'READY'; localization of: 'READY'
#define STR_OSDE_FLYMODE_FAILSAFE          "!FS!"                                     	// localization of: '!FS!'
#define STR_OSDE_FLYMODE_RESCUE            "RESC"                                     	// localization of: 'RESC'
#define STR_OSDE_FLYMODE_HEAD              "HEAD"                                     	// localization of: 'HEAD'
#define STR_OSDE_FLYMODE_PASS              "PASS"                                     	// localization of: 'PASS'
#define STR_OSDE_FLYMODE_POSH              "POSH"                                     	// localization of: 'POSH'
#define STR_OSDE_FLYMODE_ALTH              "ALTH"                                     	// localization of: 'ALTH'
#define STR_OSDE_FLYMODE_ANGL              "ANGL"                                     	// localization of: 'ANGL'
#define STR_OSDE_FLYMODE_HOR               "HOR "                                     	// localization of: 'HOR '
#define STR_OSDE_FLYMODE_ATRN              "ATRN"                                     	// localization of: 'ATRN'
#define STR_OSDE_FLYMODE_CHIR              "CHIR"                                     	// localization of: 'CHIR'
#define STR_OSDE_FLYMODE_AIR               "AIR "                                     	// localization of: 'AIR '
#define STR_OSDE_FLYMODE_ACRO              "ACRO"                                     	// localization of: 'ACRO'
#define STR_OSDE_ELEMENT_PITCH             "PIT"                                      	// localization of: 'PIT'
#define STR_OSDE_ELEMENT_ROLL              "ROL"                                      	// localization of: 'ROL'
#define STR_OSDE_ELEMENT_YAW               "YAW"                                      	// localization of: 'YAW'
#define STR_OSDE_RPM_LIMIT_ON              "RPM MAX TIL"                              	// localization of: 'RPM LIMIT ON'
#define STR_OSDE_RPM_LIMIT_OFF             "RPM MAX FRA"                              	// localization of: 'RPM LIMIT OFF'
#define STR_OSDE_POLES                     "POLER"                                    	// localization of: 'POLES'
#define STR_OSDE_THR_LIMIT                 "GAS MAX"                                  	// localization of: 'THR LIMIT'
#define STR_OSDE_MOTOR_LIMIT               "MOTOR MAX"                                	// localization of: 'MOTOR LIMIT'
