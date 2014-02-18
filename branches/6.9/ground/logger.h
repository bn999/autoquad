/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _logger_h
#define _logger_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

enum {
	LOG_LASTUPDATE = 0,
	LOG_VOLTAGE0,
	LOG_VOLTAGE1,
	LOG_VOLTAGE2,
	LOG_VOLTAGE3,
	LOG_VOLTAGE4,
	LOG_VOLTAGE5,
	LOG_VOLTAGE6,
	LOG_VOLTAGE7,
	LOG_VOLTAGE8,
	LOG_VOLTAGE9,
	LOG_VOLTAGE10,
	LOG_VOLTAGE11,
	LOG_VOLTAGE12,
	LOG_VOLTAGE13,
	LOG_VOLTAGE14,
	LOG_IMU_RATEX,
	LOG_IMU_RATEY,
	LOG_IMU_RATEZ,
	LOG_IMU_ACCX,
	LOG_IMU_ACCY,
	LOG_IMU_ACCZ,
	LOG_IMU_MAGX,
	LOG_IMU_MAGY,
	LOG_IMU_MAGZ,
	LOG_GPS_PDOP,
	LOG_GPS_HDOP,
	LOG_GPS_VDOP,
	LOG_GPS_TDOP,
	LOG_GPS_NDOP,
	LOG_GPS_EDOP,
	LOG_GPS_ITOW,
	LOG_GPS_POS_UPDATE,
	LOG_GPS_LAT,
	LOG_GPS_LON,
	LOG_GPS_HEIGHT,
	LOG_GPS_HACC,
	LOG_GPS_VACC,
	LOG_GPS_VEL_UPDATE,
	LOG_GPS_VELN,
	LOG_GPS_VELE,
	LOG_GPS_VELD,
	LOG_GPS_SACC,
	LOG_ADC_PRESSURE1,
	LOG_ADC_PRESSURE2,
	LOG_ADC_TEMP0,
	LOG_ADC_TEMP1,
	LOG_ADC_TEMP2,
	LOG_ADC_VIN,
	LOG_ADC_MAG_SIGN,
	LOG_UKF_Q1,
	LOG_UKF_Q2,
	LOG_UKF_Q3,
	LOG_UKF_Q4,
	LOG_UKF_POSN,
	LOG_UKF_POSE,
	LOG_UKF_POSD,
	LOG_UKF_PRES_ALT,
	LOG_UKF_ALT,
	LOG_UKF_VELN,
	LOG_UKF_VELE,
	LOG_UKF_VELD,
	LOG_MOT_MOTOR0,
	LOG_MOT_MOTOR1,
	LOG_MOT_MOTOR2,
	LOG_MOT_MOTOR3,
	LOG_MOT_MOTOR4,
	LOG_MOT_MOTOR5,
	LOG_MOT_MOTOR6,
	LOG_MOT_MOTOR7,
	LOG_MOT_MOTOR8,
	LOG_MOT_MOTOR9,
	LOG_MOT_MOTOR10,
	LOG_MOT_MOTOR11,
	LOG_MOT_MOTOR12,
	LOG_MOT_MOTOR13,
	LOG_MOT_THROTTLE,
	LOG_MOT_PITCH,
	LOG_MOT_ROLL,
	LOG_MOT_YAW,
	LOG_RADIO_QUALITY,
	LOG_RADIO_CHANNEL0,
	LOG_RADIO_CHANNEL1,
	LOG_RADIO_CHANNEL2,
	LOG_RADIO_CHANNEL3,
	LOG_RADIO_CHANNEL4,
	LOG_RADIO_CHANNEL5,
	LOG_RADIO_CHANNEL6,
	LOG_RADIO_CHANNEL7,
	LOG_RADIO_CHANNEL8,
	LOG_RADIO_CHANNEL9,
	LOG_RADIO_CHANNEL10,
	LOG_RADIO_CHANNEL11,
	LOG_RADIO_CHANNEL12,
	LOG_RADIO_CHANNEL13,
	LOG_RADIO_CHANNEL14,
	LOG_RADIO_CHANNEL15,
	LOG_RADIO_CHANNEL16,
	LOG_RADIO_CHANNEL17,
	LOG_RADIO_ERRORS,
	LOG_GMBL_TRIGGER,
	LOG_ACC_BIAS_X,
	LOG_ACC_BIAS_Y,
	LOG_ACC_BIAS_Z,
	LOG_NUM_IDS
};

enum {
	LOG_TYPE_DOUBLE = 0,
	LOG_TYPE_FLOAT,
	LOG_TYPE_U32,
	LOG_TYPE_S32,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U8,
	LOG_TYPE_S8
};

typedef struct {
	unsigned char fieldId;
	unsigned char fieldType;
} loggerFields_t;

#define LOG_VOLT_RATEX		0
#define LOG_VOLT_RATEY		1
#define LOG_VOLT_RATEZ		2
#define LOG_VOLT_MAGX		3
#define LOG_VOLT_MAGY		4
#define LOG_VOLT_MAGZ		5
#define LOG_VOLT_TEMP1		6
#define LOG_VOLT_VIN		7
#define LOG_VOLT_ACCX		8
#define LOG_VOLT_ACCY		9
#define LOG_VOLT_ACCZ		10
#define LOG_VOLT_PRES1		11
#define LOG_VOLT_PRES2		12
#define LOG_VOLT_TEMP2		13
#define LOG_VOLT_TEMP3		14

// legacy
#define LOG_VOLT_TEMP		LOG_VOLT_TEMP1
#define LOG_VOLT_PRES		LOG_VOLT_PRES1

typedef struct {
	unsigned int lastUpdate;
	float voltages[15];
	float rate[3];			// IMU_RATEX, Y, Z
	float acc[3];			// IMU_ACCX, Y, Z
	float mag[3];			// IMU_MAGX, Y, Z
	float magSign;			// ADC_MAG_SIGN
	float pressure[2];		// ADC_PRESSURE1, 2
	float temp[1];			// ADC_TEMP0
	float vIn;				// ADC_VIN
	float quat[4];			// UKF_Q1, 2, 3, 4
	float ukfAlt;			// UKF_ALT
	float pressAlt;			// UKF_PRES_ALT
	float pos[3];			// UKF_POSN, E, D
	float vel[3];			// UKF_VELN, E, D
	float accBias[3];		// ACC_BIAS_X, Y, Z

	unsigned int gpsPosUpdate;
	unsigned int gpsVelUpdate;
	double lat, lon;		// GPS_LAT, LON
	float gpsAlt;			// GPS_HEIGHT
	float gpsPosAcc;		// GPS_HACC
	float gpsVel[3];		// GPS_VELN, E, D
	float gpsVelAcc;		// GPS_SACC
	float gpsVAcc;			// GPS_VACC
	unsigned int gpsItow;		// GPS_ITOW
	float gpsDops[6]; 		// GPS_(P)DOP,H,V,T,N,E

	short int motors[14];	// MOT_MOTOR0 - 13
	short int throttle;		// MOT_THROTTLE
	float motPRY[3];		// MOT_PITCH, ROLL, YAW

	float radioQuality;
	short int radioChannels[18];
	unsigned short radioErrors;

	unsigned short gimbalTrig;
	
	char ckA, ckB;
} __attribute__((packed)) loggerRecord_t;

extern int loggerReadEntry(FILE *fp, loggerRecord_t *r);
extern int loggerReadLog(const char *fname, loggerRecord_t **l);
extern void loggerFree(loggerRecord_t *l);

#ifdef __cplusplus
}
#endif

#endif
