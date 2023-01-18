/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND      100
#define MILLIS_PER_SECOND           1000
#define PERIOD_OF_1_SECONDS            1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS            3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS          30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE        10     // millis
#define PERIOD_RXDATA_RECOVERY       200    // millis

typedef struct failsafeConfig_s {
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint16_t failsafe_throttle_low_delay;   // Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint8_t failsafe_switch_mode;           // failsafe switch action is 0: Stage 1, 1: Disarms instantly, 2: Stage 2
    uint8_t failsafe_procedure;             // selected full failsafe procedure is 0: auto-landing, 1: Drop it
    uint16_t failsafe_recovery_delay;       // Time (in 0.1sec) of valid rx data (min 200ms PERIOD_RXDATA_RECOVERY) to allow recovering from failsafe procedure
    uint8_t failsafe_stick_threshold;       // Stick deflection percentage to exit GPS Rescue procedure
} failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);

typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED,
    FAILSAFE_RX_LOSS_MONITORING,
    FAILSAFE_RX_LOSS_RECOVERED,
    FAILSAFE_GPS_RESCUE
} failsafePhase_e;

typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,
    FAILSAFE_RXLINK_UP
} failsafeRxLinkState_e;

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT,
#ifdef USE_GPS_RESCUE
    FAILSAFE_PROCEDURE_GPS_RESCUE,
#endif
    FAILSAFE_PROCEDURE_COUNT   // must be last
} failsafeProcedure_e;

extern const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT];

typedef enum {
    FAILSAFE_SWITCH_MODE_STAGE1 = 0,
    FAILSAFE_SWITCH_MODE_KILL,
    FAILSAFE_SWITCH_MODE_STAGE2
} failsafeSwitchMode_e;

typedef struct failsafeState_s {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t rxDataRecoveryPeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    uint32_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    uint32_t landingShouldBeFinishedAt;
    uint32_t receivingRxDataPeriod;         // period for the required period of valid rxData
    uint32_t receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;
    bool failsafeSwitchWasOn;
} failsafeState_t;

void failsafeInit(void);
void failsafeReset(void);

void failsafeStartMonitoring(void);
void failsafeUpdateState(void);
void failsafeCheckDataFailurePeriod(void);

failsafePhase_e failsafePhase(void);
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
bool failsafeIsReceivingRxData(void);
void failsafeOnRxSuspend(uint32_t suspendPeriod);
void failsafeOnRxResume(void);
void failsafeOnValidDataReceived(void);
void failsafeOnValidDataFailed(void);
