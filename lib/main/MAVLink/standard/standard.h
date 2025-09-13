/** @file
 *  @brief MAVLink comm protocol generated from standard.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_STANDARD_H
#define MAVLINK_STANDARD_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_STANDARD.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_STANDARD_XML_HASH -197706228828233048

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_STANDARD

// ENUM DEFINITIONS


/** @brief Enum used to indicate true or false (also: success or failure, enabled or disabled, active or inactive). */
#ifndef HAVE_ENUM_MAV_BOOL
#define HAVE_ENUM_MAV_BOOL
typedef enum MAV_BOOL
{
   MAV_BOOL_FALSE=0, /* False. | */
   MAV_BOOL_TRUE=1, /* True. | */
   MAV_BOOL_ENUM_END=2, /*  | */
} MAV_BOOL;
#endif

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
#ifndef HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
#define HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
typedef enum MAV_PROTOCOL_CAPABILITY
{
   MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT=1, /* Autopilot supports the MISSION_ITEM float message type.
          Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_INT instead.
         | */
   MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT=2, /* Autopilot supports the new param float message type. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_INT=4, /* Autopilot supports MISSION_ITEM_INT scaled integer message type.
          Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).
         | */
   MAV_PROTOCOL_CAPABILITY_COMMAND_INT=8, /* Autopilot supports COMMAND_INT scaled integer message type. | */
   MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE=16, /* Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST should be set if the parameter protocol is supported.
         | */
   MAV_PROTOCOL_CAPABILITY_FTP=32, /* Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html. | */
   MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET=64, /* Autopilot supports commanding attitude offboard. | */
   MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED=128, /* Autopilot supports commanding position and velocity targets in local NED frame. | */
   MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT=256, /* Autopilot supports commanding position and velocity targets in global scaled integers. | */
   MAV_PROTOCOL_CAPABILITY_TERRAIN=512, /* Autopilot supports terrain protocol / data handling. | */
   MAV_PROTOCOL_CAPABILITY_RESERVED3=1024, /* Reserved for future use. | */
   MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION=2048, /* Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination). | */
   MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION=4096, /* Autopilot supports onboard compass calibration. | */
   MAV_PROTOCOL_CAPABILITY_MAVLINK2=8192, /* Autopilot supports MAVLink version 2. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_FENCE=16384, /* Autopilot supports mission fence protocol. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_RALLY=32768, /* Autopilot supports mission rally point protocol. | */
   MAV_PROTOCOL_CAPABILITY_RESERVED2=65536, /* Reserved for future use. | */
   MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST=131072, /* Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if the parameter protocol is supported.
         | */
   MAV_PROTOCOL_CAPABILITY_COMPONENT_IMPLEMENTS_GIMBAL_MANAGER=262144, /* This component implements/is a gimbal manager. This means the GIMBAL_MANAGER_INFORMATION, and other messages can be requested.
         | */
   MAV_PROTOCOL_CAPABILITY_COMPONENT_ACCEPTS_GCS_CONTROL=524288, /* Component supports locking control to a particular GCS independent of its system (via MAV_CMD_REQUEST_OPERATOR_CONTROL). | */
   MAV_PROTOCOL_CAPABILITY_GRIPPER=1048576, /* Autopilot has a connected gripper. MAVLink Grippers would set MAV_TYPE_GRIPPER instead. | */
   MAV_PROTOCOL_CAPABILITY_ENUM_END=1048577, /*  | */
} MAV_PROTOCOL_CAPABILITY;
#endif

/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
#ifndef HAVE_ENUM_FIRMWARE_VERSION_TYPE
#define HAVE_ENUM_FIRMWARE_VERSION_TYPE
typedef enum FIRMWARE_VERSION_TYPE
{
   FIRMWARE_VERSION_TYPE_DEV=0, /* development release | */
   FIRMWARE_VERSION_TYPE_ALPHA=64, /* alpha release | */
   FIRMWARE_VERSION_TYPE_BETA=128, /* beta release | */
   FIRMWARE_VERSION_TYPE_RC=192, /* release candidate | */
   FIRMWARE_VERSION_TYPE_OFFICIAL=255, /* official stable release | */
   FIRMWARE_VERSION_TYPE_ENUM_END=256, /*  | */
} FIRMWARE_VERSION_TYPE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_autopilot_version.h"

// base include
#include "../minimal/minimal.h"


#if MAVLINK_STANDARD_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION, MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION}
# define MAVLINK_MESSAGE_NAMES {{ "AUTOPILOT_VERSION", 148 }, { "HEARTBEAT", 0 }, { "PROTOCOL_VERSION", 300 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_STANDARD_H
