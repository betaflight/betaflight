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

//No prediction:
#define FLIGHT_LOG_FIELD_PREDICTOR_0              0

//Predict that the field is the same as last frame:
#define FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS       1

//Predict that the slope between this field and the previous item is the same as that between the past two history items:
#define FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE  2

//Predict that this field is the same as the average of the last two history items:
#define FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2      3

//Predict that this field is minthrottle
#define FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE    4

//Predict that this field is the same as motor 0
#define FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0        5

//This field always increments
#define FLIGHT_LOG_FIELD_PREDICTOR_INC            6

//Predict this GPS co-ordinate is the GPS home co-ordinate (or no prediction if that coordinate is not set)
#define FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD     7


#define FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       0
#define FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     1
#define FLIGHT_LOG_FIELD_ENCODING_U8              2
#define FLIGHT_LOG_FIELD_ENCODING_U16             3
#define FLIGHT_LOG_FIELD_ENCODING_U32             4
#define FLIGHT_LOG_FIELD_ENCODING_S8              5
#define FLIGHT_LOG_FIELD_ENCODING_S16             6
#define FLIGHT_LOG_FIELD_ENCODING_S32             7
#define FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       8
#define FLIGHT_LOG_FIELD_ENCODING_NULL            9
