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
	NOTICE !
	NOTICE, this header file is generated from en/bf_locale.xml
	Changes to translation ie. en/bf_locale.h must be made in en/bf_locale.xml

	This file provides fallback defines for strings not yet translated in a locale, allowing targets to compile.
*/

#pragma once

#ifndef LOCALE
   #define LOCALE                             "en"
#endif

#ifndef STR_LANGUAGE
   #define STR_LANGUAGE                       "Language:"
#endif

#ifndef STR_COMMA
   #define STR_COMMA                          ","
#endif

#ifndef STR_PERIOD
   #define STR_PERIOD                         "."
#endif

#ifndef STR_THOUSAND
   #define STR_THOUSAND                       "."
#endif

#ifndef STR_SECONDS
   #define STR_SECONDS                        "seconds"
#endif

#ifndef STR_CMS_STARTUP_MENU_TEXT1
   #define STR_CMS_STARTUP_MENU_TEXT1         "MENU:THR MID"
#endif

#ifndef STR_CMS_STARTUP_MENU_TEXT2
   #define STR_CMS_STARTUP_MENU_TEXT2         "+ YAW LEFT  "
#endif

#ifndef STR_CMS_STARTUP_MENU_TEXT3
   #define STR_CMS_STARTUP_MENU_TEXT3         "+ PITCH UP  "
#endif

#ifndef STR_OSD_ARMED
   #define STR_OSD_ARMED                      "ARMED"
#endif

#ifndef STR_OSDE_DISARMED
   #define STR_OSDE_DISARMED                  "DISARMED"
#endif

#ifndef STR_OSD_STATS
   #define STR_OSD_STATS                      "--- STATS ---"
#endif

#ifndef STR_OSD_TIMER_ON_TIME
   #define STR_OSD_TIMER_ON_TIME              "ON TIME  "
#endif

#ifndef STR_OSD_TIMER_TOTAL_ARM
   #define STR_OSD_TIMER_TOTAL_ARM            "TOTAL ARM"
#endif

#ifndef STR_OSD_TIMER_LAST_ARM
   #define STR_OSD_TIMER_LAST_ARM             "LAST ARM "
#endif

#ifndef STR_OSD_TIMER_ON_ARM
   #define STR_OSD_TIMER_ON_ARM               "ON/ARM   "
#endif

#ifndef STR_OSD_STAT_MAX_ALTITUDE
   #define STR_OSD_STAT_MAX_ALTITUDE          "MAX ALTITUDE"
#endif

#ifndef STR_OSD_STAT_MAX_SPEED
   #define STR_OSD_STAT_MAX_SPEED             "MAX SPEED   "
#endif

#ifndef STR_OSD_STAT_MAX_DISTANCE
   #define STR_OSD_STAT_MAX_DISTANCE          "MAX DISTANCE"
#endif

#ifndef STR_OSD_STAT_FLIGHT_DISTANCE
   #define STR_OSD_STAT_FLIGHT_DISTANCE       "FLIGHT DISTANCE"
#endif

#ifndef STR_OSD_STAT_MIN_BATTERY_AVG
   #define STR_OSD_STAT_MIN_BATTERY_AVG       "MIN AVG CELL"
#endif

#ifndef STR_OSD_STAT_MIN_BATTERY
   #define STR_OSD_STAT_MIN_BATTERY           "MIN BATTERY"
#endif

#ifndef STR_OSD_STAT_END_BATTERY_AVG
   #define STR_OSD_STAT_END_BATTERY_AVG       "END AVG CELL"
#endif

#ifndef STR_OSD_STAT_END_BATTERY
   #define STR_OSD_STAT_END_BATTERY           "END BATTERY"
#endif

#ifndef STR_OSD_STAT_BATTERY_AVG
   #define STR_OSD_STAT_BATTERY_AVG           "AVG BATT CELL"
#endif

#ifndef STR_OSD_STAT_BATTERY
   #define STR_OSD_STAT_BATTERY               "BATTERY"
#endif

#ifndef STR_OSD_STAT_MIN_RSSI
   #define STR_OSD_STAT_MIN_RSSI              "MIN RSSI"
#endif

#ifndef STR_OSD_STAT_MAX_CURRENT
   #define STR_OSD_STAT_MAX_CURRENT           "MAX CURRENT"
#endif

#ifndef STR_OSD_STAT_USED_MAH
   #define STR_OSD_STAT_USED_MAH              "USED MAH"
#endif

#ifndef STR_OSD_STAT_WATT_HOURS_DRAWN
   #define STR_OSD_STAT_WATT_HOURS_DRAWN      "USED WATT HOURS"
#endif

#ifndef STR_OSD_STAT_BLACKBOX
   #define STR_OSD_STAT_BLACKBOX              "BLACKBOX"
#endif

#ifndef STR_OSD_STAT_BLACKBOX_NUMBER
   #define STR_OSD_STAT_BLACKBOX_NUMBER       "BB LOG NUM"
#endif

#ifndef STR_OSD_STAT_MAX_G_FORCE
   #define STR_OSD_STAT_MAX_G_FORCE           "MAX G-FORCE"
#endif

#ifndef STR_OSD_STAT_MAX_ESC_TEMP
   #define STR_OSD_STAT_MAX_ESC_TEMP          "MAX ESC TEMP"
#endif

#ifndef STR_OSD_STAT_MAX_ESC_RPM
   #define STR_OSD_STAT_MAX_ESC_RPM           "MAX ESC RPM"
#endif

#ifndef STR_OSD_STAT_MIN_LINK_QUALITY
   #define STR_OSD_STAT_MIN_LINK_QUALITY      "MIN LINK"
#endif

#ifndef STR_OSD_STAT_MAX_FFT
   #define STR_OSD_STAT_MAX_FFT               "PEAK FFT"
#endif

#ifndef STR_OSD_STAT_MAX_FFT_THRT
   #define STR_OSD_STAT_MAX_FFT_THRT          "THRT<20%"
#endif

#ifndef STR_OSD_STAT_MIN_RSSI_DBM
   #define STR_OSD_STAT_MIN_RSSI_DBM          "MIN RSSI DBM"
#endif

#ifndef STR_OSD_STAT_MIN_RSNR
   #define STR_OSD_STAT_MIN_RSNR              "MIN RSNR"
#endif

#ifndef STR_USE_PERSISTENT_STATS
   #define STR_USE_PERSISTENT_STATS           "TOTAL FLIGHTS"
#endif

#ifndef STR_OSD_STAT_TOTAL_TIME
   #define STR_OSD_STAT_TOTAL_TIME            "TOTAL FLIGHT TIME"
#endif

#ifndef STR_OSD_STAT_TOTAL_DIST
   #define STR_OSD_STAT_TOTAL_DIST            "TOTAL DISTANCE"
#endif

#ifndef STR_OSDW_CRASH_FLIP_WARNING
   #define STR_OSDW_CRASH_FLIP_WARNING        ">CRASH FLIP<"
#endif

#ifndef STR_OSDW_CRASH_FLIP_SWITCH
   #define STR_OSDW_CRASH_FLIP_SWITCH         "CRASH FLIP SW"
#endif

#ifndef STR_OSDW_BEACON_ON
   #define STR_OSDW_BEACON_ON                 " BEACON ON"
#endif

#ifndef STR_OSDW_ARM_IN
   #define STR_OSDW_ARM_IN                    "ARM IN"
#endif

#ifndef STR_OSDW_FAIL_SAFE
   #define STR_OSDW_FAIL_SAFE                 "FAIL SAFE"
#endif

#ifndef STR_OSDW_LAUNCH
   #define STR_OSDW_LAUNCH                    "LAUNCH"
#endif

#ifndef STR_OSDW_RSSI_LOW
   #define STR_OSDW_RSSI_LOW                  "RSSI LOW"
#endif

#ifndef STR_OSDW_RSSI_DBM
   #define STR_OSDW_RSSI_DBM                  "RSSI DBM"
#endif

#ifndef STR_OSDW_RSNR_LOW
   #define STR_OSDW_RSNR_LOW                  "RSNR LOW"
#endif

#ifndef STR_OSDW_LINK_QUALITY
   #define STR_OSDW_LINK_QUALITY              "LINK QUALITY"
#endif

#ifndef STR_OSDW_LAND_NOW
   #define STR_OSDW_LAND_NOW                  " LAND NOW"
#endif

#ifndef STR_OSDW_CPU_OVERLOAD
   #define STR_OSDW_CPU_OVERLOAD              "CPU OVERLOAD"
#endif

#ifndef STR_OSDW_RESCUE_NA
   #define STR_OSDW_RESCUE_NA                 "RESCUE N/A"
#endif

#ifndef STR_OSDW_RESCUE_OFF
   #define STR_OSDW_RESCUE_OFF                "RESCUE OFF"
#endif

#ifndef STR_OSDW_POSHOLD_FAIL
   #define STR_OSDW_POSHOLD_FAIL              "POSHOLD FAIL"
#endif

#ifndef STR_OSDW_HEADFREE
   #define STR_OSDW_HEADFREE                  "HEADFREE"
#endif

#ifndef STR_OSDW_CORE
   #define STR_OSDW_CORE                      "CORE"
#endif

#ifndef STR_OSDW_LOW_BATT
   #define STR_OSDW_LOW_BATT                  "LOW BATTERY"
#endif

#ifndef STR_OSDW_OVER_CAP
   #define STR_OSDW_OVER_CAP                  "OVER CAP"
#endif

#ifndef STR_OSDW_BATT_CONTINUE
   #define STR_OSDW_BATT_CONTINUE             "BATTERY CONTINUE"
#endif

#ifndef STR_OSDW_BATT_BELOW_FULL
   #define STR_OSDW_BATT_BELOW_FULL           "BATT < FULL"
#endif

#ifndef STR_OSDW_VISUAL_BEEP
   #define STR_OSDW_VISUAL_BEEP               "  * * * *"
#endif

#ifndef STR_OSDW_CHIRP_EXC_FINISHED
   #define STR_OSDW_CHIRP_EXC_FINISHED        "CHIRP EXC FINISHED"
#endif

#ifndef STR_OSDE_UP
   #define STR_OSDE_UP                        "U"
#endif

#ifndef STR_OSDE_DOWN
   #define STR_OSDE_DOWN                      "D"
#endif

#ifndef STR_OSDE_GPS_DIRECTION
   #define STR_OSDE_GPS_DIRECTION             "NSEW"
#endif

#ifndef STR_OSDE_PILOT_NAME
   #define STR_OSDE_PILOT_NAME                "PILOT NAME"
#endif

#ifndef STR_OSDE_RATE
   #define STR_OSDE_RATE                      "RATE_"
#endif

#ifndef STR_OSDE_PID
   #define STR_OSDE_PID                       "PID_"
#endif

#ifndef STR_OSDE_OID
   #define STR_OSDE_OID                       "OID_"
#endif

#ifndef STR_OSDE_ANTIGRAVITY
   #define STR_OSDE_ANTIGRAVITY               "AG"
#endif

#ifndef STR_OSDE_READY
   #define STR_OSDE_READY                     "READY"
#endif

#ifndef STR_OSDE_FLYMODE_FAILSAFE
   #define STR_OSDE_FLYMODE_FAILSAFE          "!FS!"
#endif

#ifndef STR_OSDE_FLYMODE_RESCUE
   #define STR_OSDE_FLYMODE_RESCUE            "RESC"
#endif

#ifndef STR_OSDE_FLYMODE_HEAD
   #define STR_OSDE_FLYMODE_HEAD              "HEAD"
#endif

#ifndef STR_OSDE_FLYMODE_PASS
   #define STR_OSDE_FLYMODE_PASS              "PASS"
#endif

#ifndef STR_OSDE_FLYMODE_POSH
   #define STR_OSDE_FLYMODE_POSH              "POSH"
#endif

#ifndef STR_OSDE_FLYMODE_ALTH
   #define STR_OSDE_FLYMODE_ALTH              "ALTH"
#endif

#ifndef STR_OSDE_FLYMODE_ANGL
   #define STR_OSDE_FLYMODE_ANGL              "ANGL"
#endif

#ifndef STR_OSDE_FLYMODE_HOR
   #define STR_OSDE_FLYMODE_HOR               "HOR "
#endif

#ifndef STR_OSDE_FLYMODE_ATRN
   #define STR_OSDE_FLYMODE_ATRN              "ATRN"
#endif

#ifndef STR_OSDE_FLYMODE_CHIR
   #define STR_OSDE_FLYMODE_CHIR              "CHIR"
#endif

#ifndef STR_OSDE_FLYMODE_AIR
   #define STR_OSDE_FLYMODE_AIR               "AIR "
#endif

#ifndef STR_OSDE_FLYMODE_ACRO
   #define STR_OSDE_FLYMODE_ACRO              "ACRO"
#endif

#ifndef STR_OSDE_ELEMENT_PITCH
   #define STR_OSDE_ELEMENT_PITCH             "PIT"
#endif

#ifndef STR_OSDE_ELEMENT_ROLL
   #define STR_OSDE_ELEMENT_ROLL              "ROL"
#endif

#ifndef STR_OSDE_ELEMENT_YAW
   #define STR_OSDE_ELEMENT_YAW               "YAW"
#endif

#ifndef STR_OSDE_RPM_LIMIT_ON
   #define STR_OSDE_RPM_LIMIT_ON              "RPM LIMIT ON"
#endif

#ifndef STR_OSDE_RPM_LIMIT_OFF
   #define STR_OSDE_RPM_LIMIT_OFF             "RPM LIMIT OFF"
#endif

#ifndef STR_OSDE_POLES
   #define STR_OSDE_POLES                     "POLES"
#endif

#ifndef STR_OSDE_THR_LIMIT
   #define STR_OSDE_THR_LIMIT                 "THR LIMIT"
#endif

#ifndef STR_OSDE_MOTOR_LIMIT
   #define STR_OSDE_MOTOR_LIMIT               "MOTOR LIMIT"
#endif

