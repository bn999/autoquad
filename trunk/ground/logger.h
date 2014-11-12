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

    Copyright © 2011-2014  Bill Nesbitt
*/

#ifndef _logger_h
#define _logger_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

enum log_fields {
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
    LOG_CURRENT_PDB,
    LOG_CURRENT_EXT,
    LOG_VIN_PDB,
    LOG_UKF_ALT_VEL,
	LOG_NUM_IDS
};

static const char *loggerFieldLabels[] = {
	"LASTUPDATE",
	"VOLTAGE0",
	"VOLTAGE1",
	"VOLTAGE2",
	"VOLTAGE3",
	"VOLTAGE4",
	"VOLTAGE5",
	"VOLTAGE6",
	"VOLTAGE7",
	"VOLTAGE8",
	"VOLTAGE9",
	"VOLTAGE10",
	"VOLTAGE11",
	"VOLTAGE12",
	"VOLTAGE13",
	"VOLTAGE14",
	"IMU_RATEX",
	"IMU_RATEY",
	"IMU_RATEZ",
	"IMU_ACCX",
	"IMU_ACCY",
	"IMU_ACCZ",
	"IMU_MAGX",
	"IMU_MAGY",
	"IMU_MAGZ",
	"GPS_PDOP",
	"GPS_HDOP",
	"GPS_VDOP",
	"GPS_TDOP",
	"GPS_NDOP",
	"GPS_EDOP",
	"GPS_ITOW",
	"GPS_POS_UPDATE",
	"GPS_LAT",
	"GPS_LON",
	"GPS_HEIGHT",
	"GPS_HACC",
	"GPS_VACC",
	"GPS_VEL_UPDATE",
	"GPS_VELN",
	"GPS_VELE",
	"GPS_VELD",
	"GPS_SACC",
	"ADC_PRESSURE1",
	"ADC_PRESSURE2",
	"ADC_TEMP0",
	"ADC_TEMP1",
	"ADC_TEMP2",
	"ADC_VIN",
	"ADC_MAG_SIGN",
	"UKF_Q1",
	"UKF_Q2",
	"UKF_Q3",
	"UKF_Q4",
	"UKF_POSN",
	"UKF_POSE",
	"UKF_POSD",
	"UKF_PRES_ALT",
	"UKF_ALT (m)",
	"UKF_VELN (m/s)",
	"UKF_VELE (m/s)",
	"UKF_VELD (m/s)",
	"MOT_MOTOR0",
	"MOT_MOTOR1",
	"MOT_MOTOR2",
	"MOT_MOTOR3",
	"MOT_MOTOR4",
	"MOT_MOTOR5",
	"MOT_MOTOR6",
	"MOT_MOTOR7",
	"MOT_MOTOR8",
	"MOT_MOTOR9",
	"MOT_MOTOR10",
	"MOT_MOTOR11",
	"MOT_MOTOR12",
	"MOT_MOTOR13",
	"MOT_THROTTLE",
	"MOT_PITCH",
	"MOT_ROLL",
	"MOT_YAW",
	"RADIO_QUALITY",
	"RADIO_CHANNEL0",
	"RADIO_CHANNEL1",
	"RADIO_CHANNEL2",
	"RADIO_CHANNEL3",
	"RADIO_CHANNEL4",
	"RADIO_CHANNEL5",
	"RADIO_CHANNEL6",
	"RADIO_CHANNEL7",
	"RADIO_CHANNEL8",
	"RADIO_CHANNEL9",
	"RADIO_CHANNEL10",
	"RADIO_CHANNEL11",
	"RADIO_CHANNEL12",
	"RADIO_CHANNEL13",
	"RADIO_CHANNEL14",
	"RADIO_CHANNEL15",
	"RADIO_CHANNEL16",
	"RADIO_CHANNEL17",
	"RADIO_ERRORS",
	"GMBL_TRIGGER",
	"ACC_BIAS_X",
	"ACC_BIAS_Y",
	"ACC_BIAS_Z",
	"CURRENT_PDB",
	"CURRENT_EXT",
	"VIN_PDB",
	"UKF_ALT_VEL",
	0 // terminate
};

enum log_field_types {
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

#define LOG_NUM_VOLTAGES	15
#define LOG_NUM_MOTORS		14
#define LOG_NUM_RADIO_CHAN	18

// legacy
#define LOG_VOLT_TEMP		LOG_VOLT_TEMP1
#define LOG_VOLT_PRES		LOG_VOLT_PRES1

typedef struct {
	float quat[4];									// LOG_UKF_Qn
	float voltages[LOG_NUM_VOLTAGES];				// LOG_VOLTAGEn
	short int motors[LOG_NUM_MOTORS];				// LOG_MOT_MOTORn
	short int radioChannels[LOG_NUM_RADIO_CHAN];	// LOG_RADIO_CHANNELn
	
	double data[LOG_NUM_IDS];						// all data values for record
	char ckA, ckB;									// checksums

} __attribute__((packed)) loggerRecord_t;

extern int loggerReadEntry(FILE *fp, loggerRecord_t *r);
extern int loggerReadLog(const char *fname, loggerRecord_t **l);
extern void loggerFree(loggerRecord_t *l);

#ifdef __cplusplus
}
#endif

#endif
