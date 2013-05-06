/*
 * aq_mavlink_gnd.cpp
 *
 *  Created on: Dec 23, 2012
 *      Author: Max
 */

#include <string.h>
#include "logDump_mavlink.h"
#include "logDump.h"
#include "mavlink.h"

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

void mavlinkInit(void) {
	mavlinkData.wpCount = 0;
	mavlinkData.wpCurrent = mavlinkData.wpCount + 1;

	mavlink_system.sysid = 42;
	mavlink_system.compid = MAV_COMP_ID_ALL;
	mavlink_system.type = MAV_TYPE_QUADROTOR;

	mavlinkData.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	mavlinkData.nav_mode = MAV_STATE_STANDBY;
	mavlinkData.status = MAV_STATE_ACTIVE;
	mavlinkData.idlePercent = 999;
	mavlinkData.packetDrops = 0;
}

void mavlinkWpReached(uint16_t seqId) {
	//mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0, seqId);
}

void mavlinkWpAnnounceCurrent(uint16_t seqId) {
	//mavlink_msg_mission_current_send(MAVLINK_COMM_0, seqId);
}

void mavlogWritePacket(mavlink_message_t *msg, uint64_t ts) {
	uint8_t buf[mavBufLen];

//	printf("ts: %llu; len: %d; seq: %d; sysid: %d; compid: %d; magic: %d; msgid: %d; payload: %llu; chk: %lu\n",
//			ts, msg->len, msg->seq, msg->sysid, msg->compid, msg->magic, msg->msgid, msg->payload64, msg->checksum);
	memcpy(buf, (void*)&ts, sizeof(uint64_t));
	//memcpy(buf+sizeof(uint64_t), (const mavlink_message_t *)msg, MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
	int buflen = mavlink_msg_to_send_buffer(buf+sizeof(uint64_t), msg);
	fwrite(buf, sizeof(uint8_t), sizeof(buf), outFP);
	fflush(outFP);
	memset(msg->payload64, 0x0, sizeof(msg->payload64));
}

void mavlinkDo(loggerRecord_t *l) {
	// StatusType result;
	mavlink_message_t msg;
	uint16_t msgLen;
	uint64_t ts = ((uint64_t)towStartTime*1000 + (uint64_t)logDumpGetValue(l, GPS_UTC_TIME)) * 1000;
	double micros = logDumpGetValue(l, MICROS);

	//fprintf(stderr, "logDump: TIMESTAMP: %llu; START: %u; TOW: %lld\n", ts, towStartTime, (uint64_t)logDumpGetValue(l, GPS_UTC_TIME));

	mavlinkData.status = MAV_STATE_ACTIVE;

	double modeChan = logDumpGetValue(l, RADIO_CHANNEL6);
	if (modeChan < 250) 		// manual
		mavlinkData.mode = MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	else if (modeChan > 250) 	// mission
		mavlinkData.mode = MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_GUIDED_ENABLED;
	else 						// pos/alt hold
		mavlinkData.mode = MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	// heartbeat
	msgLen = mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type,
			MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, mavlinkData.mode,
			mavlinkData.nav_mode, mavlinkData.status);

	mavlogWritePacket(&msg, ts);
/*
	// status
	double vIn = logDumpGetValue(l, VIN);
	msgLen = mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, 0, 0,
			1000 - mavlinkData.idlePercent, vIn * 1000, -1,
			(vIn - 9.8f) / 12.6f * 1000, 0,
			mavlinkData.packetDrops, 0, 0, 0, 0);

	mavlogWritePacket(&msg, ts);


	// raw acc, rate, mag sensors
	mavlink_msg_scaled_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros,
			logDumpGetValue(l, ACCX) * 1000.0f, logDumpGetValue(l, ACCY) * 1000.0f, logDumpGetValue(l, ACCZ) * 1000.0f,
			logDumpGetValue(l, RATEX) * 1000.0f, logDumpGetValue(l, RATEY) * 1000.0f, logDumpGetValue(l, RATEZ) * 1000.0f,
			logDumpGetValue(l, MAGX) * 1000.0f, logDumpGetValue(l, MAGY) * 1000.0f, logDumpGetValue(l, MAGZ) * 1000.0f);

	mavlogWritePacket(&msg, ts);

	// pressure and temp
	mavlink_msg_scaled_pressure_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros,
			logDumpGetValue(l, PRESSURE1) * 0.01f, 0.0f, logDumpGetValue(l, TEMP1) * 100);

	mavlogWritePacket(&msg, ts);


	// position
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros,
			3, logDumpGetValue(l, LAT) * 1e7, logDumpGetValue(l, LON) * 1e7,
			logDumpGetValue(l, GPS_ALT) * 1e3, logDumpGetValue(l, AUX_RATEY), logDumpGetValue(l, AUX_RATEZ),
			logDumpGetValue(l, GPS_H_SPEED), 65535, 255);
		//	    mavlink_msg_gps_raw_send(MAVLINK_COMM_0, micros, 3, gpsData.lat, gpsData.lon, gpsData.height, gpsData.hDOP, gpsData.vDOP, gpsData.speed, gpsData.heading);

	mavlogWritePacket(&msg, ts);

	// rc channels
	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros, 0,
			logDumpGetValue(l, RADIO_CHANNEL1) + 1024, logDumpGetValue(l, RADIO_CHANNEL2) + 1024, logDumpGetValue(l, RADIO_CHANNEL3) + 1024,
			logDumpGetValue(l, RADIO_CHANNEL4) + 1024, logDumpGetValue(l, RADIO_CHANNEL5) + 1024, logDumpGetValue(l, RADIO_CHANNEL6) + 1024,
			logDumpGetValue(l, RADIO_CHANNEL7) + 1024, logDumpGetValue(l, RADIO_CHANNEL8) + 1024, logDumpGetValue(l, RADIO_QUALITY));

	mavlogWritePacket(&msg, ts);

	// motor outputs
	mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros, 0,
			logDumpGetValue(l, MOTOR1), logDumpGetValue(l, MOTOR2),
			logDumpGetValue(l, MOTOR2), logDumpGetValue(l, MOTOR4),
			logDumpGetValue(l, MOTOR5), logDumpGetValue(l, MOTOR6),
			logDumpGetValue(l, MOTOR7), logDumpGetValue(l, MOTOR8));

	mavlogWritePacket(&msg, ts);

	mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg, micros, 1,
			logDumpGetValue(l, MOTOR9), logDumpGetValue(l, MOTOR10),
			logDumpGetValue(l, MOTOR11), logDumpGetValue(l, MOTOR12),
			logDumpGetValue(l, MOTOR13), logDumpGetValue(l, MOTOR14),
			0, 0);

	mavlogWritePacket(&msg, ts);
	*/
	/*
	// rc channels
	else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL]
			|| mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS])
			&& mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS]
					< micros) {
		mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, micros, 0,
				RADIO_THROT + 1024, RADIO_ROLL + 1024, RADIO_PITCH + 1024,
				RADIO_RUDD + 1024, RADIO_GEAR + 1024, RADIO_FLAPS + 1024,
				RADIO_AUX2 + 1024, RADIO_AUX3 + 1024, RADIO_QUALITY);
		mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, micros, 0,
				(RADIO_THROT - 750) * 13, RADIO_ROLL * 13, RADIO_PITCH * 13,
				RADIO_RUDD * 13, RADIO_GEAR * 13, RADIO_FLAPS * 13,
				RADIO_AUX2 * 13, RADIO_AUX3 * 13, RADIO_QUALITY);
		mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] = micros
				+ mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS];
	}
	// raw controller
	else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL]
			|| mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER])
			&& mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER]
					< micros) {
		mavlink_msg_attitude_send(MAVLINK_COMM_0, micros,
				AQ_ROLL * DEG_TO_RAD, AQ_PITCH * DEG_TO_RAD,
				AQ_YAW * DEG_TO_RAD,
				-(IMU_RATEX - UKF_GYO_BIAS_X) * DEG_TO_RAD,
				(IMU_RATEY - UKF_GYO_BIAS_Y) * DEG_TO_RAD,
				(IMU_RATEZ - UKF_GYO_BIAS_Z) * DEG_TO_RAD);
		mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, micros, 0,
				motorsData.value[0], motorsData.value[1],
				motorsData.value[2], motorsData.value[3],
				motorsData.value[4], motorsData.value[5],
				motorsData.value[6], motorsData.value[7]);
		mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] =
				micros
						+ mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER];
	}

	// send pending notices
	msg = CoAcceptQueueMail(mavlinkData.notices, &result);
	if (result == E_OK)
		mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *) msg);

	// list all parameters
	if (mavlinkData.currentParam < mavlinkData.numParams
			&& mavlinkData.nextParam < micros) {
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
				configParameterStrings[mavlinkData.currentParam],
				p[mavlinkData.currentParam], MAVLINK_TYPE_FLOAT,
				mavlinkData.numParams, mavlinkData.currentParam);
		mavlinkData.currentParam++;
		mavlinkData.nextParam = micros + MAVLINK_PARAM_INTERVAL;
	} else if (mavlinkData.wpCurrent < mavlinkData.wpCount
			&& mavlinkData.wpNext < micros) {
		mavlink_msg_mission_request_send(MAVLINK_COMM_0,
				mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId,
				mavlinkData.wpCurrent);
		mavlinkData.wpNext = micros + MAVLINK_WP_TIMEOUT;
	} else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,
				mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
		mavlinkData.wpCurrent++;
	}
	*/
}
