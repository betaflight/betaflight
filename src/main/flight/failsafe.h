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
#include "common/time.h"
#include "config/parameter_group.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND         100
#define MILLIS_PER_SECOND              1000
#define PERIOD_OF_1_SECONDS               1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS               3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS             30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE           200    // millis
#define PERIOD_RXDATA_RECOVERY          200    // millis


typedef struct failsafeConfig_s {
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint16_t failsafe_throttle_low_delay;   // Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure" (TENTH_SECOND)
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_recovery_delay;        // Time from RC link recovery to failsafe abort. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint8_t failsafe_procedure;             // selected full failsafe procedure is 0: auto-landing, 1: Drop it, 2: Return To Home (RTH)

    int16_t failsafe_fw_roll_angle;         // Settings to be applies during "LAND" procedure on a fixed-wing
    int16_t failsafe_fw_pitch_angle;
    int16_t failsafe_fw_yaw_rate;

    uint16_t failsafe_stick_motion_threshold;
} failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);

typedef enum {
    /* Failsafe mode is not active. All other
     * phases indicate that the failsafe flight
     * mode is active.
     */
    FAILSAFE_IDLE = 0,
    /* In this phase, the connection from the receiver
     * has been confirmed as lost and it will either
     * transition into FAILSAFE_RX_LOSS_RECOVERED if the
     * RX link is recovered inmmediately or one of the
     * recovery phases otherwise (as configured via
     * failsafe_procedure) or into FAILSAFE_RX_LOSS_IDLE
     * if failsafe_procedure is NONE.
     */
    FAILSAFE_RX_LOSS_DETECTED,
    /* This phase will just do nothing else than wait
     * until the RX connection is re-established and the
     * sticks are moved more than the failsafe_stick_threshold
     * settings and then transition to FAILSAFE_RX_LOSS_RECOVERED.
     * Note that this phase is only used when
     * failsafe_procedure = NONE.
     */
    FAILSAFE_RX_LOSS_IDLE,
#if defined(NAV)
    /* Failsafe is executing RTH. This phase is the first one
     * enabled when failsafe_procedure = RTH if an RTH is
     * deemed possible (RTH might not be activated if e.g.
     * a HOME position was not recorded or some required
     * sensors are not working at the moment). If RTH can't
     * be started, this phase will transition to FAILSAFE_LANDING.
     */
    FAILSAFE_RETURN_TO_HOME,
#endif
    /* Failsafe mode is performing a simplified landing procedure.
     * This is done by setting throttle and roll/pitch/yaw controls
     * to a pre-configured values that will allow aircraft
     * to reach ground in somewhat safe "controlled crash" way.
     * This is the first recovery phase enabled when
     * failsafe_procedure = SET-THR. Once timeout expires or if a
     * "controlled crash" can't be executed, this phase will
     * transition to FAILSAFE_LANDED.
     */
    FAILSAFE_LANDING,
    /* Failsafe has either detected that the model has landed and disabled
     * the motors or either decided to drop the model because it couldn't
     * perform an emergency landing. It will disarm, prevent re-arming
     * and transition into FAILSAFE_RX_LOSS_MONITORING immediately. This is
     * the first recovery phase enabled when failsafe_procedure = DROP.
     */
    FAILSAFE_LANDED,
    /* This phase will wait until the RX connection is
     * working for some time and if and only if switch arming
     * is used and the switch is in the unarmed position
     * will allow rearming again.
     */
    FAILSAFE_RX_LOSS_MONITORING,
    /* This phase indicates that the RX link has been re-established and
     * it will immediately transition out of failsafe mode (phase will
     * transition to FAILSAFE_IDLE.)
     */
    FAILSAFE_RX_LOSS_RECOVERED
} failsafePhase_e;

typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,
    FAILSAFE_RXLINK_UP
} failsafeRxLinkState_e;

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT,
    FAILSAFE_PROCEDURE_RTH,
    FAILSAFE_PROCEDURE_NONE
} failsafeProcedure_e;

typedef enum {
    RTH_IDLE = 0,               // RTH is waiting
    RTH_IN_PROGRESS,            // RTH is active
    RTH_HAS_LANDED              // RTH is active and has landed.
} rthState_e;

typedef struct failsafeState_s {
    int16_t events;
    bool monitoring;
    bool active;
    bool controlling;
    timeMs_t rxDataFailurePeriod;
    timeMs_t rxDataRecoveryPeriod;
    timeMs_t validRxDataReceivedAt;
    timeMs_t validRxDataFailedAt;
    timeMs_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    timeMs_t landingShouldBeFinishedAt;
    timeMs_t receivingRxDataPeriod;         // period for the required period of valid rxData
    timeMs_t receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;
    int16_t lastGoodRcCommand[4];
} failsafeState_t;

void failsafeInit(void);
void failsafeReset(void);

void failsafeStartMonitoring(void);
void failsafeUpdateState(void);

failsafePhase_e failsafePhase();
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
bool failsafeIsReceivingRxData(void);
void failsafeOnRxSuspend(uint32_t suspendPeriod);
void failsafeOnRxResume(void);
bool failsafeMayRequireNavigationMode(void);
void failsafeApplyControlInput(void);
bool failsafeRequiresAngleMode(void);
bool failsafeRequiresMotorStop(void);
bool failsafeShouldApplyControlInput(void);
void failsafeUpdateRcCommandValues(void);

void failsafeOnValidDataReceived(void);
void failsafeOnValidDataFailed(void);
