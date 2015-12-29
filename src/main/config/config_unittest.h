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


#ifdef SRC_MAIN_FLIGHT_PID_C_
#ifdef UNIT_TEST

float unittest_pidLuxFloatAxis_lastError[3];
float unittest_pidLuxFloatAxis_PTerm[3];
float unittest_pidLuxFloatAxis_ITerm[3];
float unittest_pidLuxFloatAxis_DTerm[3];
float unittest_pidLuxFloatAxis_previousDelta[3][8];

#define SET_PID_LUX_FLOAT_AXIS_LOCALS(axis) \
    { \
        lastError[axis] = unittest_pidLuxFloatAxis_lastError[axis]; \
        {for (int ii = 0; ii < 8; ++ii) previousDelta[axis][ii] = unittest_pidLuxFloatAxis_previousDelta[axis][ii];} \
    }

#define GET_PID_LUX_FLOAT_AXIS_LOCALS(axis) \
    { \
        unittest_pidLuxFloatAxis_lastError[axis] = lastError[axis]; \
        unittest_pidLuxFloatAxis_PTerm[axis] = PTerm; \
        unittest_pidLuxFloatAxis_ITerm[axis] = ITerm; \
        unittest_pidLuxFloatAxis_DTerm[axis] = DTerm; \
        {for (int ii = 0; ii < 8; ++ii) unittest_pidLuxFloatAxis_previousDelta[axis][ii] = previousDelta[axis][ii];} \
    }

int32_t unittest_pidRewriteAxis_lastError[3];
int32_t unittest_pidRewriteAxis_PTerm[3];
int32_t unittest_pidRewriteAxis_ITerm[3];
int32_t unittest_pidRewriteAxis_DTerm[3];
int32_t unittest_pidRewriteAxis_previousDelta[3][8];

#define SET_PID_REWRITE_AXIS_LOCALS(axis) \
    { \
        lastError[axis] = unittest_pidRewriteAxis_lastError[axis]; \
        {for (int ii = 0; ii < 8; ++ii) previousDelta[axis][ii] = unittest_pidRewriteAxis_previousDelta[axis][ii];} \
    }

#define GET_PID_REWRITE_AXIS_LOCALS(axis) \
    { \
        unittest_pidRewriteAxis_lastError[axis] = lastError[axis]; \
        unittest_pidRewriteAxis_PTerm[axis] = PTerm; \
        unittest_pidRewriteAxis_ITerm[axis] = ITerm; \
        unittest_pidRewriteAxis_DTerm[axis] = DTerm; \
        {for (int ii = 0; ii < 8; ++ii) unittest_pidRewriteAxis_previousDelta[axis][ii] = previousDelta[axis][ii];} \
    }

#else

#define SET_PID_LUX_FLOAT_AXIS_LOCALS(axis) {}
#define GET_PID_LUX_FLOAT_AXIS_LOCALS(axis) {}
#define SET_PID_REWRITE_AXIS_LOCALS(axis) {}
#define GET_PID_REWRITE_AXIS_LOCALS(axis) {}

#endif // UNIT_TEST
#endif // SRC_MAIN_FLIGHT_PID_C_

