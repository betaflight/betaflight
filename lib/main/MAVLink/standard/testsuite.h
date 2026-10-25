/** @file
 *    @brief MAVLink comm protocol testsuite generated from standard.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef STANDARD_TESTSUITE_H
#define STANDARD_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_minimal(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_standard(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_minimal(system_id, component_id, last_msg);
    mavlink_test_standard(system_id, component_id, last_msg);
}
#endif

#include "../minimal/testsuite.h"


static void mavlink_test_global_position_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_POSITION_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_position_int_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587
    };
    mavlink_global_position_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.hdg = packet_in.hdg;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GLOBAL_POSITION_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) != NULL);
#endif
}

static void mavlink_test_autopilot_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTOPILOT_VERSION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_autopilot_version_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296,963498504,963498712,963498920,18899,19003,{ 113, 114, 115, 116, 117, 118, 119, 120 },{ 137, 138, 139, 140, 141, 142, 143, 144 },{ 161, 162, 163, 164, 165, 166, 167, 168 },{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };
    mavlink_autopilot_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.capabilities = packet_in.capabilities;
        packet1.uid = packet_in.uid;
        packet1.flight_sw_version = packet_in.flight_sw_version;
        packet1.middleware_sw_version = packet_in.middleware_sw_version;
        packet1.os_sw_version = packet_in.os_sw_version;
        packet1.board_version = packet_in.board_version;
        packet1.vendor_id = packet_in.vendor_id;
        packet1.product_id = packet_in.product_id;
        
        mav_array_memcpy(packet1.flight_custom_version, packet_in.flight_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.middleware_custom_version, packet_in.middleware_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.os_custom_version, packet_in.os_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.uid2, packet_in.uid2, sizeof(uint8_t)*18);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_pack(system_id, component_id, &msg , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_autopilot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_send(MAVLINK_COMM_1 , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTOPILOT_VERSION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTOPILOT_VERSION) != NULL);
#endif
}

static void mavlink_test_standard(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_global_position_int(system_id, component_id, last_msg);
    mavlink_test_autopilot_version(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // STANDARD_TESTSUITE_H
