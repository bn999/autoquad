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

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "logger.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

loggerFields_t *loggerFields;
int loggerNumFields;
int loggerPacketSize;

void loggerChecksumError(const char *s) {
	fprintf(stderr, "logger: checksum error in '%s' packet\n", s);
}

void loggerDecodePacket(char *buf, loggerRecord_t *r) {
	char *ptr;
	int i;

	for (i = 0; i < loggerNumFields; i++) {
		switch (loggerFields[i].fieldId) {
			case LOG_LASTUPDATE:
				r->lastUpdate = *(unsigned int *)buf;
				break;
			case LOG_VOLTAGE0:
				r->voltages[0] = *(float *)buf;
				break;
			case LOG_VOLTAGE1:
				r->voltages[1] = *(float *)buf;
				break;
			case LOG_VOLTAGE2:
				r->voltages[2] = *(float *)buf;
				break;
			case LOG_VOLTAGE3:
				r->voltages[3] = *(float *)buf;
				break;
			case LOG_VOLTAGE4:
				r->voltages[4] = *(float *)buf;
				break;
			case LOG_VOLTAGE5:
				r->voltages[5] = *(float *)buf;
				break;
			case LOG_VOLTAGE6:
				r->voltages[6] = *(float *)buf;
				break;
			case LOG_VOLTAGE7:
				r->voltages[7] = *(float *)buf;
				break;
			case LOG_VOLTAGE8:
				r->voltages[8] = *(float *)buf;
				break;
			case LOG_VOLTAGE9:
				r->voltages[9] = *(float *)buf;
				break;
			case LOG_VOLTAGE10:
				r->voltages[10] = *(float *)buf;
				break;
			case LOG_VOLTAGE11:
				r->voltages[11] = *(float *)buf;
				break;
			case LOG_VOLTAGE12:
				r->voltages[12] = *(float *)buf;
				break;
			case LOG_VOLTAGE13:
				r->voltages[13] = *(float *)buf;
				break;
			case LOG_VOLTAGE14:
				r->voltages[14] = *(float *)buf;
				break;
			case LOG_IMU_RATEX:
				r->rate[0] = *(float *)buf;
				break;
			case LOG_IMU_RATEY:
				r->rate[1] = *(float *)buf;
				break;
			case LOG_IMU_RATEZ:
				r->rate[2] = *(float *)buf;
				break;
			case LOG_IMU_ACCX:
				r->acc[0] = *(float *)buf;
				break;
			case LOG_IMU_ACCY:
				r->acc[1] = *(float *)buf;
				break;
			case LOG_IMU_ACCZ:
				r->acc[2] = *(float *)buf;
				break;
			case LOG_IMU_MAGX:
				r->mag[0] = *(float *)buf;
				break;
			case LOG_IMU_MAGY:
				r->mag[1] = *(float *)buf;
				break;
			case LOG_IMU_MAGZ:
				r->mag[2] = *(float *)buf;
				break;
			case LOG_GPS_PDOP:
				r->rateAux[0] = *(float *)buf;
				break;
			case LOG_GPS_HDOP:
				r->rateAux[1] = *(float *)buf;
				break;
			case LOG_GPS_VDOP:
				r->rateAux[2] = *(float *)buf;
				break;
			case LOG_GPS_TDOP:
				r->magAux[0] = *(float *)buf;
				break;
			case LOG_GPS_NDOP:
				r->magAux[1] = *(float *)buf;
				break;
			case LOG_GPS_EDOP:
				r->magAux[2] = *(float *)buf;
				break;
			case LOG_GPS_ITOW:
				r->gpsItow = *(unsigned int *)buf;
				break;
			case LOG_GPS_POS_UPDATE:
				r->gpsPosUpdate = *(unsigned int *)buf;
				break;
			case LOG_GPS_LAT:
				r->lat = *(double *)buf;
				break;
			case LOG_GPS_LON:
				r->lon = *(double *)buf;
				break;
			case LOG_GPS_HEIGHT:
				r->gpsAlt = *(float *)buf;
				break;
			case LOG_GPS_HACC:
				r->gpsPosAcc = *(float *)buf;
				break;
			case LOG_GPS_VACC:
				r->gpsVAcc = *(float *)buf;
				break;
			case LOG_GPS_VEL_UPDATE:
				r->gpsVelUpdate = *(unsigned int *)buf;
				break;
			case LOG_GPS_VELN:
				r->gpsVel[0] = *(float *)buf;
				break;
			case LOG_GPS_VELE:
				r->gpsVel[1] = *(float *)buf;
				break;
			case LOG_GPS_VELD:
				r->gpsVel[2] = *(float *)buf;
				break;
			case LOG_GPS_SACC:
				r->gpsVelAcc = *(float *)buf;
				break;
			case LOG_ADC_PRESSURE1:
				r->pressure[0] = *(float *)buf;
				break;
			case LOG_ADC_PRESSURE2:
				r->pressure[1] = *(float *)buf;
				break;
			case LOG_ADC_TEMP0:
				r->temp[0] = *(float *)buf;
				break;
			case LOG_ADC_VIN:
				r->vIn = *(float *)buf;
				break;
			case LOG_ADC_MAG_SIGN:
				r->magSign = (float)*(char *)buf;
				break;
			case LOG_UKF_Q1:
				r->quat[0] = *(float *)buf;
				break;
			case LOG_UKF_Q2:
				r->quat[1] = *(float *)buf;
				break;
			case LOG_UKF_Q3:
				r->quat[2] = *(float *)buf;
				break;
			case LOG_UKF_Q4:
				r->quat[3] = *(float *)buf;
				break;
			case LOG_UKF_POSN:
				r->pos[0] = *(float *)buf;
				break;
			case LOG_UKF_POSE:
				r->pos[1] = *(float *)buf;
				break;
			case LOG_UKF_POSD:
				r->pos[2] = *(float *)buf;
				break;
			case LOG_UKF_PRES_ALT:
				r->pressAlt = *(float *)buf;
				break;
			case LOG_UKF_ALT:
				r->ukfAlt = *(float *)buf;
				break;
			case LOG_UKF_VELN:
				r->vel[0] = *(float *)buf;
				break;
			case LOG_UKF_VELE:
				r->vel[1] = *(float *)buf;
				break;
			case LOG_UKF_VELD:
				r->vel[2] = *(float *)buf;
				break;
			case LOG_MOT_MOTOR0:
				r->motors[0] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR1:
				r->motors[1] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR2:
				r->motors[2] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR3:
				r->motors[3] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR4:
				r->motors[4] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR5:
				r->motors[5] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR6:
				r->motors[6] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR7:
				r->motors[7] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR8:
				r->motors[8] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR9:
				r->motors[9] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR10:
				r->motors[10] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR11:
				r->motors[11] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR12:
				r->motors[12] = *(short int *)buf;
				break;
			case LOG_MOT_MOTOR13:
				r->motors[13] = *(short int *)buf;
				break;
			case LOG_MOT_THROTTLE:
				r->throttle = (short int)*(float *)buf;
				break;
			case LOG_MOT_PITCH:
				r->accAux[0] = *(float *)buf;
				break;
			case LOG_MOT_ROLL:
				r->accAux[1] = *(float *)buf;
				break;
			case LOG_MOT_YAW:
				r->accAux[2] = *(float *)buf;
				break;
			case LOG_RADIO_QUALITY:
				r->radioQuality = *(float *)buf;
				break;
			case LOG_RADIO_CHANNEL0:
				r->radioChannels[0] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL1:
				r->radioChannels[1] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL2:
				r->radioChannels[2] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL3:
				r->radioChannels[3] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL4:
				r->radioChannels[4] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL5:
				r->radioChannels[5] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL6:
				r->radioChannels[6] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL7:
				r->radioChannels[7] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL8:
				r->radioChannels[8] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL9:
				r->radioChannels[9] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL10:
				r->radioChannels[10] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL11:
				r->radioChannels[11] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL12:
				r->radioChannels[12] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL13:
				r->radioChannels[13] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL14:
				r->radioChannels[14] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL15:
				r->radioChannels[15] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL16:
				r->radioChannels[16] = *(short int *)buf;
				break;
			case LOG_RADIO_CHANNEL17:
				r->radioChannels[17] = *(short int *)buf;
				break;
			case LOG_RADIO_ERRORS:
				r->radioErrors = *(unsigned int *)buf;
				break;
		}

		switch (loggerFields[i].fieldType) {
			case LOG_TYPE_DOUBLE:
				buf += 8;
				break;
			case LOG_TYPE_FLOAT:
			case LOG_TYPE_U32:
			case LOG_TYPE_S32:
				buf += 4;
				break;
			case LOG_TYPE_U16:
			case LOG_TYPE_S16:
				buf += 2;
				break;
			case LOG_TYPE_U8:
			case LOG_TYPE_S8:
				buf += 1;
				break;
		}
	}
}

int loggerReadEntryM(FILE *fp, loggerRecord_t *r) {
	char buf[1024];
	unsigned char ckA, ckB;
	int i;

	if (loggerPacketSize > 0 && fread(buf, loggerPacketSize, 1, fp) == 1) {
		// calc checksum
		ckA = ckB = 0;
		for (i = 0; i < loggerPacketSize; i++) {
			ckA += buf[i];
			ckB += ckA;
		}

		if (fgetc(fp) == ckA && fgetc(fp) == ckB) {
			loggerDecodePacket(buf, r);

			return 1;
		}

		loggerChecksumError("M");
	}

	return 0;
}

int loggerReadEntryH(FILE *fp) {
	char buf[1024];
	unsigned char ckA, ckB;
	int numFields;
	int i;

	numFields = fgetc(fp);

	if (fread(buf, numFields * sizeof(loggerFields_t), 1, fp) == 1) {
		// calc checksum
		ckA = ckB = numFields;
		for (i = 0; i < numFields * sizeof(loggerFields_t); i++) {
			ckA += buf[i];
			ckB += ckA;
		}

		if (fgetc(fp) == ckA && fgetc(fp) == ckB) {
			loggerFields = (loggerFields_t *)realloc(loggerFields, numFields * sizeof(loggerFields_t));
			memcpy(loggerFields, buf, numFields * sizeof(loggerFields_t));
			loggerNumFields = numFields;

			loggerPacketSize = 0;
			for (i = 0; i < numFields; i++) {
				switch (loggerFields[i].fieldType) {
					case LOG_TYPE_DOUBLE:
						loggerPacketSize += 8;
						break;
					case LOG_TYPE_FLOAT:
					case LOG_TYPE_U32:
					case LOG_TYPE_S32:
						loggerPacketSize += 4;
						break;
					case LOG_TYPE_U16:
					case LOG_TYPE_S16:
						loggerPacketSize += 2;
						break;
					case LOG_TYPE_U8:
					case LOG_TYPE_S8:
						loggerPacketSize += 1;
						break;
				}
			}

			return 1;
		}
		else {
			loggerChecksumError("H");
		}
	}

	return 0;
}

int loggerReadEntryL(FILE *fp, loggerRecord_t *r) {
	char *buf = (char *)r;
	char ckA, ckB;
	int i;

	if (fread(buf, sizeof(loggerRecord_t), 1, fp) == 1) {
		// calc checksum
		ckA = ckB = 0;
		for (i = 0; i < sizeof(loggerRecord_t) - 2; i++) {
			ckA += buf[i];
			ckB += ckA;
		}

		if (r->ckA == ckA && r->ckB == ckB) {
			return 1;
		}
		else {
			loggerChecksumError("L");
			return 0;
		}
	}
	return 0;
}

int loggerReadEntry(FILE *fp, loggerRecord_t *r) {
	int c = 0;

	loggerTop:

	if (c != EOF) {
		if ((c = fgetc(fp)) != 'A')
			goto loggerTop;
		if ((c = fgetc(fp)) != 'q')
			goto loggerTop;

		c = fgetc(fp);

		if (c == 'L') {
			if (loggerReadEntryL(fp, r) == 0)
				goto loggerTop;
			else
				return 1;
		}
		else if (c == 'H') {
			loggerReadEntryH(fp);
			goto loggerTop;
		}
		else if (c == 'M') {
			if (loggerReadEntryM(fp, r) == 0)
				goto loggerTop;
			else
				return 1;
		}
		else {
//			fprintf(stderr, "logger: Unknown record type '%d'\n", c);
			goto loggerTop;
		}

	}

	return EOF;
}

// allocates memory and reads an entire log
int loggerReadLog(const char *fname, loggerRecord_t **l) {
	loggerRecord_t buf;
	FILE *fp;
	int n = 0;
	int i;

	*l = NULL;

#if defined (__WIN32__)
	fp = fopen(fname, "rb");
#else
	fp = fopen(fname, "r");
#endif
	if (fp == NULL) {
		fprintf(stderr, "logger: cannot open log file '%s'\n", fname);
	}
	else {
		loggerPacketSize = 0;
		loggerNumFields = 0;

		// force header read
		loggerReadEntry(fp, &buf);
		rewind(fp);

		while (loggerReadEntry(fp, &buf) != EOF)
			n++;

		*l = (loggerRecord_t *)calloc(n, sizeof(loggerRecord_t));

		rewind(fp);

		for (i = 0; i < n; i++)
			loggerReadEntry(fp, &(*l)[i]);

		fclose(fp);
	}

	return n;
}

int loggerRecordSize(void) {
	if (loggerPacketSize)
		return loggerPacketSize + 2 + 3;
	else
		return (sizeof(loggerRecord_t));
}

void loggerFree(loggerRecord_t *l) {
	if (l) {
		free(l);
		l = NULL;
	}

	if (loggerFields) {
		free(loggerFields);
		loggerPacketSize = 0;
		loggerNumFields = 0;
	}
}
