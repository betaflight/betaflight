/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

void taskSystem(void);
void taskMainPidLoopCheck(void);
void taskUpdateAccelerometer(void);
void taskUpdateAttitude(void);
bool taskUpdateRxCheck(uint32_t currentDeltaTime);
void taskUpdateRxMain(void);
void taskHandleSerial(void);
void taskUpdateBattery(void);
void taskUpdateBeeper(void);
void taskProcessGPS(void);
void taskUpdateCompass(void);
void taskUpdateBaro(void);
void taskUpdateSonar(void);
void taskCalculateAltitude(void);
void taskUpdateDisplay(void);
void taskTelemetry(void);
void taskLedStrip(void);
void taskTransponder(void);
#ifdef OSD
void taskUpdateOsd(void);
#endif
#ifdef USE_BST
void taskBstReadWrite(void);
void taskBstMasterProcess(void);
#endif

