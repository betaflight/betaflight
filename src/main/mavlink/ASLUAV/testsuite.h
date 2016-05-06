/** @file
 *	@brief MAVLink comm protocol testsuite generated from ASLUAV.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ASLUAV_TESTSUITE_H
#define ASLUAV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ASLUAV(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_ASLUAV(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_sens_power(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sens_power_t packet_in = {
		17.0,45.0,73.0,101.0
    };
	mavlink_sens_power_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.adc121_vspb_volt = packet_in.adc121_vspb_volt;
        	packet1.adc121_cspb_amp = packet_in.adc121_cspb_amp;
        	packet1.adc121_cs1_amp = packet_in.adc121_cs1_amp;
        	packet1.adc121_cs2_amp = packet_in.adc121_cs2_amp;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_power_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_power_pack(system_id, component_id, &msg , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
	mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_power_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
	mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sens_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_power_send(MAVLINK_COMM_1 , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
	mavlink_msg_sens_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sens_mppt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sens_mppt_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,18899,19003,19107,247,58,125
    };
	mavlink_sens_mppt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mppt_timestamp = packet_in.mppt_timestamp;
        	packet1.mppt1_volt = packet_in.mppt1_volt;
        	packet1.mppt1_amp = packet_in.mppt1_amp;
        	packet1.mppt2_volt = packet_in.mppt2_volt;
        	packet1.mppt2_amp = packet_in.mppt2_amp;
        	packet1.mppt3_volt = packet_in.mppt3_volt;
        	packet1.mppt3_amp = packet_in.mppt3_amp;
        	packet1.mppt1_pwm = packet_in.mppt1_pwm;
        	packet1.mppt2_pwm = packet_in.mppt2_pwm;
        	packet1.mppt3_pwm = packet_in.mppt3_pwm;
        	packet1.mppt1_status = packet_in.mppt1_status;
        	packet1.mppt2_status = packet_in.mppt2_status;
        	packet1.mppt3_status = packet_in.mppt3_status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_mppt_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_mppt_pack(system_id, component_id, &msg , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_mppt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sens_mppt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_mppt_send(MAVLINK_COMM_1 , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_sens_mppt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aslctrl_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_aslctrl_data_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,37,104
    };
	mavlink_aslctrl_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.h = packet_in.h;
        	packet1.hRef = packet_in.hRef;
        	packet1.hRef_t = packet_in.hRef_t;
        	packet1.PitchAngle = packet_in.PitchAngle;
        	packet1.PitchAngleRef = packet_in.PitchAngleRef;
        	packet1.q = packet_in.q;
        	packet1.qRef = packet_in.qRef;
        	packet1.uElev = packet_in.uElev;
        	packet1.uThrot = packet_in.uThrot;
        	packet1.uThrot2 = packet_in.uThrot2;
        	packet1.aZ = packet_in.aZ;
        	packet1.AirspeedRef = packet_in.AirspeedRef;
        	packet1.YawAngle = packet_in.YawAngle;
        	packet1.YawAngleRef = packet_in.YawAngleRef;
        	packet1.RollAngle = packet_in.RollAngle;
        	packet1.RollAngleRef = packet_in.RollAngleRef;
        	packet1.p = packet_in.p;
        	packet1.pRef = packet_in.pRef;
        	packet1.r = packet_in.r;
        	packet1.rRef = packet_in.rRef;
        	packet1.uAil = packet_in.uAil;
        	packet1.uRud = packet_in.uRud;
        	packet1.aslctrl_mode = packet_in.aslctrl_mode;
        	packet1.SpoilersEngaged = packet_in.SpoilersEngaged;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_pack(system_id, component_id, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aslctrl_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_aslctrl_debug_t packet_in = {
		963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113,180
    };
	mavlink_aslctrl_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.i32_1 = packet_in.i32_1;
        	packet1.f_1 = packet_in.f_1;
        	packet1.f_2 = packet_in.f_2;
        	packet1.f_3 = packet_in.f_3;
        	packet1.f_4 = packet_in.f_4;
        	packet1.f_5 = packet_in.f_5;
        	packet1.f_6 = packet_in.f_6;
        	packet1.f_7 = packet_in.f_7;
        	packet1.f_8 = packet_in.f_8;
        	packet1.i8_1 = packet_in.i8_1;
        	packet1.i8_2 = packet_in.i8_2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_debug_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_debug_pack(system_id, component_id, &msg , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
	mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
	mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_aslctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_debug_send(MAVLINK_COMM_1 , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
	mavlink_msg_aslctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_asluav_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_asluav_status_t packet_in = {
		17.0,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158 }
    };
	mavlink_asluav_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Motor_rpm = packet_in.Motor_rpm;
        	packet1.LED_status = packet_in.LED_status;
        	packet1.SATCOM_status = packet_in.SATCOM_status;
        
        	mav_array_memcpy(packet1.Servo_status, packet_in.Servo_status, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asluav_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asluav_status_pack(system_id, component_id, &msg , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
	mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asluav_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
	mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_asluav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asluav_status_send(MAVLINK_COMM_1 , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
	mavlink_msg_asluav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ekf_ext(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ekf_ext_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0
    };
	mavlink_ekf_ext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.Windspeed = packet_in.Windspeed;
        	packet1.WindDir = packet_in.WindDir;
        	packet1.WindZ = packet_in.WindZ;
        	packet1.Airspeed = packet_in.Airspeed;
        	packet1.beta = packet_in.beta;
        	packet1.alpha = packet_in.alpha;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ekf_ext_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ekf_ext_pack(system_id, component_id, &msg , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
	mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ekf_ext_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
	mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ekf_ext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ekf_ext_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
	mavlink_msg_ekf_ext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_asl_obctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_asl_obctrl_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,101
    };
	mavlink_asl_obctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.uElev = packet_in.uElev;
        	packet1.uThrot = packet_in.uThrot;
        	packet1.uThrot2 = packet_in.uThrot2;
        	packet1.uAilL = packet_in.uAilL;
        	packet1.uAilR = packet_in.uAilR;
        	packet1.uRud = packet_in.uRud;
        	packet1.obctrl_status = packet_in.obctrl_status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asl_obctrl_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asl_obctrl_pack(system_id, component_id, &msg , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
	mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asl_obctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
	mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_asl_obctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_asl_obctrl_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
	mavlink_msg_asl_obctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sens_atmos(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sens_atmos_t packet_in = {
		17.0,45.0
    };
	mavlink_sens_atmos_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.TempAmbient = packet_in.TempAmbient;
        	packet1.Humidity = packet_in.Humidity;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_atmos_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_atmos_pack(system_id, component_id, &msg , packet1.TempAmbient , packet1.Humidity );
	mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_atmos_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.TempAmbient , packet1.Humidity );
	mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sens_atmos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_atmos_send(MAVLINK_COMM_1 , packet1.TempAmbient , packet1.Humidity );
	mavlink_msg_sens_atmos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sens_batmon(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sens_batmon_t packet_in = {
		17235,17339,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379
    };
	mavlink_sens_batmon_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.temperature = packet_in.temperature;
        	packet1.voltage = packet_in.voltage;
        	packet1.current = packet_in.current;
        	packet1.batterystatus = packet_in.batterystatus;
        	packet1.serialnumber = packet_in.serialnumber;
        	packet1.hostfetcontrol = packet_in.hostfetcontrol;
        	packet1.cellvoltage1 = packet_in.cellvoltage1;
        	packet1.cellvoltage2 = packet_in.cellvoltage2;
        	packet1.cellvoltage3 = packet_in.cellvoltage3;
        	packet1.cellvoltage4 = packet_in.cellvoltage4;
        	packet1.cellvoltage5 = packet_in.cellvoltage5;
        	packet1.cellvoltage6 = packet_in.cellvoltage6;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_batmon_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_batmon_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.batterystatus , packet1.serialnumber , packet1.hostfetcontrol , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
	mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_batmon_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.batterystatus , packet1.serialnumber , packet1.hostfetcontrol , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
	mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sens_batmon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sens_batmon_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.batterystatus , packet1.serialnumber , packet1.hostfetcontrol , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
	mavlink_msg_sens_batmon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ASLUAV(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_sens_power(system_id, component_id, last_msg);
	mavlink_test_sens_mppt(system_id, component_id, last_msg);
	mavlink_test_aslctrl_data(system_id, component_id, last_msg);
	mavlink_test_aslctrl_debug(system_id, component_id, last_msg);
	mavlink_test_asluav_status(system_id, component_id, last_msg);
	mavlink_test_ekf_ext(system_id, component_id, last_msg);
	mavlink_test_asl_obctrl(system_id, component_id, last_msg);
	mavlink_test_sens_atmos(system_id, component_id, last_msg);
	mavlink_test_sens_batmon(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ASLUAV_TESTSUITE_H
