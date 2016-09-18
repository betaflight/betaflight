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

void taskSystem(uint32_t currentTime);
void taskMainPidLoopCheck(uint32_t currentTime);
void taskUpdateAccelerometer(uint32_t currentTime);
void taskUpdateAttitude(uint32_t currentTime);
bool taskUpdateRxCheck(uint32_t currentTime, uint32_t currentDeltaTime);
void taskUpdateRxMain(uint32_t currentTime);
void taskHandleSerial(uint32_t currentTime);
void taskUpdateBattery(uint32_t currentTime);
void taskUpdateBeeper(uint32_t currentTime);
void taskProcessGPS(uint32_t currentTime);
void taskUpdateCompass(uint32_t currentTime);
void taskUpdateBaro(uint32_t currentTime);
void taskUpdateSonar(uint32_t currentTime);
void taskCalculateAltitude(uint32_t currentTime);
void taskUpdateDisplay(uint32_t currentTime);
void taskTelemetry(uint32_t currentTime);
void taskLedStrip(uint32_t currentTime);
void taskTransponder(uint32_t currentTime);
#ifdef OSD
void taskUpdateOsd(uint32_t currentTime);
#endif
#ifdef USE_BST
void taskBstReadWrite(uint32_t currentTime);
void taskBstMasterProcess(uint32_t currentTime);
#endif

