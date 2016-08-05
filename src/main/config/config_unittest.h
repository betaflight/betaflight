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

#ifdef SRC_MAIN_SCHEDULER_C_
#ifdef UNIT_TEST

cfTask_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;
uint16_t unittest_scheduler_waitingTasks;
uint32_t unittest_scheduler_timeToNextRealtimeTask;
bool unittest_outsideRealtimeGuardInterval;

#define GET_SCHEDULER_LOCALS() \
    { \
    unittest_scheduler_selectedTask = selectedTask; \
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority; \
    unittest_scheduler_waitingTasks = waitingTasks; \
    unittest_scheduler_timeToNextRealtimeTask = timeToNextRealtimeTask; \
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
float unittest_pidLuxFloat_PTerm[3];
float unittest_pidLuxFloat_ITerm[3];
float unittest_pidLuxFloat_DTerm[3];

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
        unittest_pidLuxFloat_PTerm[axis] = PTerm; \
        unittest_pidLuxFloat_ITerm[axis] = ITerm; \
        unittest_pidLuxFloat_DTerm[axis] = DTerm; \
    }

int32_t unittest_pidMultiWiiRewrite_lastErrorForDelta[3];
int32_t unittest_pidMultiWiiRewrite_PTerm[3];
int32_t unittest_pidMultiWiiRewrite_ITerm[3];
int32_t unittest_pidMultiWiiRewrite_DTerm[3];

#define SET_PID_MULTI_WII_REWRITE_LOCALS(axis) \
    { \
    lastErrorForDelta[axis] = unittest_pidMultiWiiRewrite_lastErrorForDelta[axis]; \
    }

#define GET_PID_MULTI_WII_REWRITE_LOCALS(axis) \
    { \
        unittest_pidMultiWiiRewrite_lastErrorForDelta[axis] = lastErrorForDelta[axis]; \
        unittest_pidMultiWiiRewrite_PTerm[axis] = PTerm; \
        unittest_pidMultiWiiRewrite_ITerm[axis] = ITerm; \
        unittest_pidMultiWiiRewrite_DTerm[axis] = DTerm; \
    }

#else

#define SET_PID_LUX_FLOAT_LOCALS(axis) {}
#define GET_PID_LUX_FLOAT_LOCALS(axis) {}
#define SET_PID_MULTI_WII_REWRITE_LOCALS(axis) {}
#define GET_PID_MULTI_WII_REWRITE_LOCALS(axis) {}

#endif // UNIT_TEST
#endif // SRC_MAIN_FLIGHT_PID_C_

