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

#ifdef SRC_MAIN_SCHEDULER_C_
#ifdef UNIT_TEST

cfTask_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;
uint16_t unittest_scheduler_waitingTasks;
bool unittest_outsideRealtimeGuardInterval;

#define GET_SCHEDULER_LOCALS() \
    { \
    unittest_scheduler_selectedTask = selectedTask; \
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority; \
    unittest_scheduler_waitingTasks = waitingTasks; \
    unittest_outsideRealtimeGuardInterval = outsideRealtimeGuardInterval; \
    }

#else

#define GET_SCHEDULER_LOCALS() {}

#endif
#endif


#ifdef SRC_MAIN_FLIGHT_PID_C_
#ifdef UNIT_TEST

float unittest_pidLuxFloat_lastErrorForDelta[3];
float unittest_pidLuxFloat_delta1[3];
float unittest_pidLuxFloat_delta2[3];
float unittest_pidLuxFloat_pterm[3];
float unittest_pidLuxFloat_iterm[3];
float unittest_pidLuxFloat_dterm[3];

#define SET_PID_LUX_FLOAT_LOCALS(axis) \
    { \
        lastErrorForDelta[axis] = unittest_pidLuxFloat_lastErrorForDelta[axis]; \
        delta1[axis] = unittest_pidLuxFloat_delta1[axis]; \
        delta2[axis] = unittest_pidLuxFloat_delta2[axis]; \
    }

#define GET_PID_LUX_FLOAT_LOCALS(axis) \
    { \
        unittest_pidLuxFloat_lastErrorForDelta[axis] = lastErrorForDelta[axis]; \
        unittest_pidLuxFloat_delta1[axis] = delta1[axis]; \
        unittest_pidLuxFloat_delta2[axis] = delta2[axis]; \
        unittest_pidLuxFloat_pterm[axis] = pterm; \
        unittest_pidLuxFloat_iterm[axis] = iterm; \
        unittest_pidLuxFloat_dterm[axis] = dterm; \
    }

int32_t unittest_pidMultiWiiRewrite_lastErrorForDelta[3];
int32_t unittest_pidMultiWiiRewrite_pterm[3];
int32_t unittest_pidMultiWiiRewrite_iterm[3];
int32_t unittest_pidMultiWiiRewrite_dterm[3];

#define SET_PID_MULTI_WII_REWRITE_LOCALS(axis) \
    { \
    lastErrorForDelta[axis] = unittest_pidMultiWiiRewrite_lastErrorForDelta[axis]; \
    }

#define GET_PID_MULTI_WII_REWRITE_LOCALS(axis) \
    { \
        unittest_pidMultiWiiRewrite_lastErrorForDelta[axis] = lastErrorForDelta[axis]; \
        unittest_pidMultiWiiRewrite_pterm[axis] = pterm; \
        unittest_pidMultiWiiRewrite_iterm[axis] = iterm; \
        unittest_pidMultiWiiRewrite_dterm[axis] = dterm; \
    }

#else

#define SET_PID_LUX_FLOAT_LOCALS(axis) {}
#define GET_PID_LUX_FLOAT_LOCALS(axis) {}
#define SET_PID_MULTI_WII_REWRITE_LOCALS(axis) {}
#define GET_PID_MULTI_WII_REWRITE_LOCALS(axis) {}

#endif // UNIT_TEST
#endif // SRC_MAIN_FLIGHT_PID_C_
