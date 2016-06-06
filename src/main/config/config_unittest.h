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


#ifdef SRC_MAIN_FLIGHT_PID_LUXFLOAT_C_
#ifdef UNIT_TEST

float unittest_pidLuxFloatCore_lastRateForDelta[3];
float unittest_pidLuxFloatCore_PTerm[3];
float unittest_pidLuxFloatCore_ITerm[3];
float unittest_pidLuxFloatCore_DTerm[3];

#define SET_PID_LUX_FLOAT_CORE_LOCALS(axis) \
    { \
        lastRateForDelta[axis] = unittest_pidLuxFloatCore_lastRateForDelta[axis]; \
    }

#define GET_PID_LUX_FLOAT_CORE_LOCALS(axis) \
    { \
        unittest_pidLuxFloatCore_lastRateForDelta[axis] = lastRateForDelta[axis]; \
        unittest_pidLuxFloatCore_PTerm[axis] = PTerm; \
        unittest_pidLuxFloatCore_ITerm[axis] = ITerm; \
        unittest_pidLuxFloatCore_DTerm[axis] = DTerm; \
    }

#else

#define SET_PID_LUX_FLOAT_CORE_LOCALS(axis) {}
#define GET_PID_LUX_FLOAT_CORE_LOCALS(axis) {}

#endif // UNIT_TEST
#endif // SRC_MAIN_FLIGHT_PID_LUXFLOAT_C_


#ifdef SRC_MAIN_FLIGHT_PID_MWREWRITE_C_
#ifdef UNIT_TEST

int32_t unittest_pidMultiWiiRewriteCore_lastRateForDelta[3];
int32_t unittest_pidMultiWiiRewriteCore_PTerm[3];
int32_t unittest_pidMultiWiiRewriteCore_ITerm[3];
int32_t unittest_pidMultiWiiRewriteCore_DTerm[3];

#define SET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis) \
    { \
        lastRateForDelta[axis] = unittest_pidMultiWiiRewriteCore_lastRateForDelta[axis]; \
    }

#define GET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis) \
    { \
        unittest_pidMultiWiiRewriteCore_lastRateForDelta[axis] = lastRateForDelta[axis]; \
        unittest_pidMultiWiiRewriteCore_PTerm[axis] = PTerm; \
        unittest_pidMultiWiiRewriteCore_ITerm[axis] = ITerm; \
        unittest_pidMultiWiiRewriteCore_DTerm[axis] = DTerm; \
    }

#else

#define SET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis) {}
#define GET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis) {}

#endif // UNIT_TEST
#endif // SRC_MAIN_FLIGHT_PID_MWREWRITE_C_

