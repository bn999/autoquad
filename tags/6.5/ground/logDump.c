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
#include "plplot/plplot.h"
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <math.h>

#define P0                  	101325.0	// standard static pressure at sea level
#define ADC_REF_VOLTAGE		3.3f
#define ADC_TEMP_OFFSET		1.25f		// volts (IDG500 PTATS)
#define ADC_TEMP_SLOPE		(1.0f / 0.004f)	// deg C / volts

#define ADC_TEMP_A		+167.5358f
#define ADC_TEMP_B		+6.6871f
#define ADC_TEMP_C		+0.0347f
#define ADC_TEMP_R2		100000.0f	// ohms
#define ADC_KELVIN_TO_CELCIUS	-273.0f

enum fields {
	MICROS = 0,
	VOLTAGE1,
	VOLTAGE2,
	VOLTAGE3,
	VOLTAGE4,
	VOLTAGE5,
	VOLTAGE6,
	VOLTAGE7,
	VOLTAGE8,
	VOLTAGE9,
	VOLTAGE10,
	VOLTAGE11,
	VOLTAGE12,
	VOLTAGE13,
	VOLTAGE14,
	VOLTAGE15,
	RATEX,
	RATEY,
	RATEZ,
	ACCX,
	ACCY,
	ACCZ,
	MAGX,
	MAGY,
	MAGZ,
	PRESSURE1,
	PRESSURE2,
	TEMP1,
	TEMP2,
	TEMP3,
	TEMP4,
	AUX_RATEX,
	AUX_RATEY,
	AUX_RATEZ,
	AUX_ACCX,
	AUX_ACCY,
	AUX_ACCZ,
	AUX_MAGX,
	AUX_MAGY,
	AUX_MAGZ,
	VIN,
	GPS_POS_MICROS,
	LAT,
	LON,
	GPS_ALT,
	GPS_POS_ACC,
	GPS_VEL_MICROS,
	GPS_VELN,
	GPS_VELE,
	GPS_VELD,
	GPS_VEL_ACC,
	POSN,
	POSE,
	POSD,
	VELN,
	VELE,
	VELD,
	QUAT0,
	QUAT1,
	QUAT2,
	QUAT3,
	MOTOR1,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR9,
	MOTOR10,
	MOTOR11,
	MOTOR12,
	MOTOR13,
	MOTOR14,
	THROTTLE,
	EXTRA1,
	EXTRA2,
	EXTRA3,
	EXTRA4,
	NUM_FIELDS
};

int dumpOrder[NUM_FIELDS];
int dumpNum;

float dumpMin[NUM_FIELDS];
float dumpMax[NUM_FIELDS];

int dumpPlot;
float scaleMin, scaleMax;
PLFLT *xVals;
PLFLT *yVals;

loggerRecord_t logEntry;

void attitudeExtractEulerQuat(float *q, float *yaw, float *pitch, float *roll) {
        float q0, q1, q2, q3;

        q0 = q[1];
        q1 = q[2];
        q2 = q[3];
        q3 = q[0];

        *yaw = atan2((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
        *pitch = asin(-2.0f * (q0 * q2 - q1 * q3));
        *roll = atan((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));
}

float presToAlt(float pressure) {
	return (1.0 - pow(pressure / P0, 0.19)) * (1.0 / 22.558e-6);
}

float adcIDGVoltsToTemp(float volts) {
	return (25.0f + (volts - ADC_TEMP_OFFSET) * ADC_TEMP_SLOPE);
}

// TODO: optimize
float adcT1VoltsToTemp(float volts) {
	float r1;
	float ln;
	float kelvin;

	r1 = (ADC_TEMP_R2 * ADC_REF_VOLTAGE) / volts - ADC_TEMP_R2;
	ln = log(r1);
	kelvin = ADC_TEMP_A + ADC_TEMP_B * ln + ADC_TEMP_C * (ln*ln*ln);

	return (kelvin + ADC_KELVIN_TO_CELCIUS);
}


void usage(void) {
	fprintf(stderr, "usage: logDump [--help] [--plot] [--scale-min] [--scale-max] [ --micros --voltages --rates --accs --mags --pressures --temps\n");
	fprintf(stderr, "		--aux-rates --aux-accs --aux-mags --vin\n");
	fprintf(stderr, "		--gps-pos-micros --lat --lon --gps-alt --gps-pos-acc --gps-vel-micros --gps-vels --gps-vel-acc --quat\n");
	fprintf(stderr, "		--poss --vels --motors --throttle --extras ] <log file>\n");
}

void logDumpOpts(int argc, char **argv) {
        int ch;
	static int longOpt;

	enum longOptions {
		O_MICROS,
		O_VOLTAGES,
		O_RATES,
		O_ACCS,
		O_MAGS,
		O_PRESSURES,
		O_TEMPS,
		O_AUX_RATES,
		O_AUX_ACCS,
		O_AUX_MAGS,
		O_VIN,
		O_GPS_POS_MICROS,
		O_LAT,
		O_LON,
		O_GPS_ALT,
		O_GPS_POS_ACC,
		O_GPS_VEL_MICROS,
		O_GPS_VELS,
		O_GPS_VEL_ACC,
		O_POSS,
		O_VELS,
		O_QUAT,
		O_MOTORS,
		O_THROTTLE,
		O_EXTRAS,
	};

        /* options descriptor */
        static struct option longopts[] = {
                {"help",		no_argument, 		NULL,		'h'},
                {"plot",		optional_argument,	NULL,		'p'},
                {"scale-min",		required_argument,	NULL,		'n'},
                {"scale-max",		required_argument,	&longOpt,	'x'},
                {"micros",		no_argument,		&longOpt,	O_MICROS},
                {"voltages",		no_argument,		&longOpt,	O_VOLTAGES},
                {"rates",		no_argument,		&longOpt,	O_RATES},
                {"accs",		no_argument,		&longOpt,	O_ACCS},
                {"mags",		no_argument,		&longOpt,	O_MAGS},
                {"pressures",		no_argument,		&longOpt,	O_PRESSURES},
                {"temps",		no_argument,		&longOpt,	O_TEMPS},
                {"aux-rates",		no_argument,		&longOpt,	O_AUX_RATES},
                {"aux-accs",		no_argument,		&longOpt,	O_AUX_ACCS},
                {"aux-mags",		no_argument,		&longOpt,	O_AUX_MAGS},
                {"vin",			no_argument,		&longOpt,	O_VIN},
                {"gps-pos-micros",	no_argument,		&longOpt,	O_GPS_POS_MICROS},
                {"lat",			no_argument,		&longOpt,	O_LAT},
                {"lon",			no_argument,		&longOpt,	O_LON},
                {"gps-alt",		no_argument,		&longOpt,	O_GPS_ALT},
                {"gps-pos-acc",		no_argument,		&longOpt,	O_GPS_POS_ACC},
                {"gps-vel-micros",	no_argument,		&longOpt,	O_GPS_VEL_MICROS},
                {"gps-vels",		no_argument,		&longOpt,	O_GPS_VELS},
                {"gps-vel-acc",		no_argument,		&longOpt,	O_GPS_VEL_ACC},
                {"poss",		no_argument,		&longOpt,	O_POSS},
                {"vels",		no_argument,		&longOpt,	O_VELS},
                {"quat",		no_argument,		&longOpt,	O_QUAT},
                {"motors",		no_argument,		&longOpt,	O_MOTORS},
                {"throttle",		no_argument,		&longOpt,	O_THROTTLE},
                {"extras",		no_argument,		&longOpt,	O_EXTRAS},
                {NULL,          	0,                      NULL,		0}
	};

	scaleMin = scaleMax = nan("");
        while ((ch = getopt_long(argc, argv, "hpn:x:", longopts, NULL)) != -1)
                switch (ch) {
		case 'h':
			usage();
			exit(0);
			break;
		case 'p':
			dumpPlot++;
			break;
		case 'n':
			scaleMin = atof(optarg);
			break;
		case 'x':
			scaleMax = atof(optarg);
			break;
		case 0:
			switch (longOpt) {
			case O_MICROS:
				dumpOrder[dumpNum++] = MICROS;
				break;
			case O_VOLTAGES:
				dumpOrder[dumpNum++] = VOLTAGE1;
				dumpOrder[dumpNum++] = VOLTAGE2;
				dumpOrder[dumpNum++] = VOLTAGE3;
				dumpOrder[dumpNum++] = VOLTAGE4;
				dumpOrder[dumpNum++] = VOLTAGE5;
				dumpOrder[dumpNum++] = VOLTAGE6;
				dumpOrder[dumpNum++] = VOLTAGE7;
				dumpOrder[dumpNum++] = VOLTAGE8;
				dumpOrder[dumpNum++] = VOLTAGE9;
				dumpOrder[dumpNum++] = VOLTAGE10;
				dumpOrder[dumpNum++] = VOLTAGE11;
				dumpOrder[dumpNum++] = VOLTAGE12;
				dumpOrder[dumpNum++] = VOLTAGE13;
				dumpOrder[dumpNum++] = VOLTAGE14;
				dumpOrder[dumpNum++] = VOLTAGE15;
				break;
			case O_RATES:
				dumpOrder[dumpNum++] = RATEX;
				dumpOrder[dumpNum++] = RATEY;
				dumpOrder[dumpNum++] = RATEZ;
				break;
			case O_ACCS:
				dumpOrder[dumpNum++] = ACCX;
				dumpOrder[dumpNum++] = ACCY;
				dumpOrder[dumpNum++] = ACCZ;
				break;
			case O_MAGS:
				dumpOrder[dumpNum++] = MAGX;
				dumpOrder[dumpNum++] = MAGY;
				dumpOrder[dumpNum++] = MAGZ;
				break;
			case O_PRESSURES:
				dumpOrder[dumpNum++] = PRESSURE1;
				dumpOrder[dumpNum++] = PRESSURE2;
				break;
			case O_TEMPS:
				dumpOrder[dumpNum++] = TEMP1;
				dumpOrder[dumpNum++] = TEMP2;
				dumpOrder[dumpNum++] = TEMP3;
				dumpOrder[dumpNum++] = TEMP4;
				break;
			case O_AUX_RATES:
				dumpOrder[dumpNum++] = AUX_RATEX;
				dumpOrder[dumpNum++] = AUX_RATEY;
				dumpOrder[dumpNum++] = AUX_RATEZ;
				break;
			case O_AUX_ACCS:
				dumpOrder[dumpNum++] = AUX_ACCX;
				dumpOrder[dumpNum++] = AUX_ACCY;
				dumpOrder[dumpNum++] = AUX_ACCZ;
				break;
			case O_AUX_MAGS:
				dumpOrder[dumpNum++] = AUX_MAGX;
				dumpOrder[dumpNum++] = AUX_MAGY;
				dumpOrder[dumpNum++] = AUX_MAGZ;
				break;
			case O_VIN:
				dumpOrder[dumpNum++] = VIN;
				break;
			case O_GPS_POS_MICROS:
				dumpOrder[dumpNum++] = GPS_POS_MICROS;
				break;
			case O_LAT:
				dumpOrder[dumpNum++] = LAT;
				break;
			case O_LON:
				dumpOrder[dumpNum++] = LON;
				break;
			case O_GPS_ALT:
				dumpOrder[dumpNum++] = GPS_ALT;
				break;
			case O_GPS_POS_ACC:
				dumpOrder[dumpNum++] = GPS_POS_ACC;
				break;
			case O_GPS_VEL_MICROS:
				dumpOrder[dumpNum++] = GPS_VEL_MICROS;
				break;
			case O_GPS_VELS:
				dumpOrder[dumpNum++] = GPS_VELN;
				dumpOrder[dumpNum++] = GPS_VELE;
				dumpOrder[dumpNum++] = GPS_VELD;
				break;
			case O_GPS_VEL_ACC:
				dumpOrder[dumpNum++] = GPS_VEL_ACC;
				break;
			case O_POSS:
				dumpOrder[dumpNum++] = POSN;
				dumpOrder[dumpNum++] = POSE;
				dumpOrder[dumpNum++] = POSD;
				break;
			case O_VELS:
				dumpOrder[dumpNum++] = VELN;
				dumpOrder[dumpNum++] = VELE;
				dumpOrder[dumpNum++] = VELD;
				break;
			case O_QUAT:
				dumpOrder[dumpNum++] = QUAT0;
				dumpOrder[dumpNum++] = QUAT1;
				dumpOrder[dumpNum++] = QUAT2;
				dumpOrder[dumpNum++] = QUAT3;
				break;
			case O_MOTORS:
				dumpOrder[dumpNum++] = MOTOR1;
				dumpOrder[dumpNum++] = MOTOR2;
				dumpOrder[dumpNum++] = MOTOR3;
				dumpOrder[dumpNum++] = MOTOR4;
				dumpOrder[dumpNum++] = MOTOR5;
				dumpOrder[dumpNum++] = MOTOR6;
				dumpOrder[dumpNum++] = MOTOR7;
				dumpOrder[dumpNum++] = MOTOR8;
				dumpOrder[dumpNum++] = MOTOR9;
				dumpOrder[dumpNum++] = MOTOR10;
				dumpOrder[dumpNum++] = MOTOR11;
				dumpOrder[dumpNum++] = MOTOR12;
				dumpOrder[dumpNum++] = MOTOR13;
				dumpOrder[dumpNum++] = MOTOR14;
				break;
			case O_THROTTLE:
				dumpOrder[dumpNum++] = THROTTLE;
				break;
			case O_EXTRAS:
				dumpOrder[dumpNum++] = EXTRA1;
				dumpOrder[dumpNum++] = EXTRA2;
				dumpOrder[dumpNum++] = EXTRA3;
				dumpOrder[dumpNum++] = EXTRA4;
				break;
			}
			break;
                default:
//			fprintf(stderr, "logDump: logDumpOpts: error\n");
			break;
                }
}

double logDumpGetValue(loggerRecord_t *l, int field) {
	double val;

	switch (field) {
	case MICROS:
		val =  l->lastUpdate;
		break;
	case VOLTAGE1:
		val = l->voltages[0];
		break;
	case VOLTAGE2:
		val = l->voltages[1];
		break;
	case VOLTAGE3:
		val = l->voltages[2];
		break;
	case VOLTAGE4:
		val = l->voltages[3];
		break;
	case VOLTAGE5:
		val = l->voltages[4];
		break;
	case VOLTAGE6:
		val = l->voltages[5];
		break;
	case VOLTAGE7:
		val = l->voltages[6];
		break;
	case VOLTAGE8:
		val = l->voltages[7];
		break;
	case VOLTAGE9:
		val = l->voltages[8];
		break;
	case VOLTAGE10:
		val = l->voltages[9];
		break;
	case VOLTAGE11:
		val = l->voltages[10];
		break;
	case VOLTAGE12:
		val = l->voltages[11];
		break;
	case VOLTAGE13:
		val = l->voltages[12];
		break;
	case VOLTAGE14:
		val = l->voltages[13];
		break;
	case VOLTAGE15:
		val = l->voltages[14];
		break;
	case RATEX:
		val = l->rate[0];
		break;
	case RATEY:
		val = l->rate[1];
		break;
	case RATEZ:
		val = l->rate[2];
		break;
	case ACCX:
		val = l->acc[0];
		break;
	case ACCY:
		val = l->acc[1];
		break;
	case ACCZ:
		val = l->acc[2];
		break;
	case MAGX:
		val = l->mag[0];
		break;
	case MAGY:
		val = l->mag[1];
		break;
	case MAGZ:
		val = l->mag[2];
		break;
	case PRESSURE1:
		val = l->pressure[0];
		break;
	case PRESSURE2:
		val = l->pressure[1];
		break;
	case TEMP1:
		val = l->temp[0];
		break;
	case TEMP2:
		val = l->temp[1];
		break;
	case TEMP3:
		val = l->temp[2];
		break;
	case TEMP4:
		val = l->temp[3];
		break;
	case AUX_RATEX:
		val = l->rateAux[0];
		break;
	case AUX_RATEY:
		val = l->rateAux[1];
		break;
	case AUX_RATEZ:
		val = l->rateAux[2];
		break;
	case AUX_ACCX:
		val = l->accAux[0];
		break;
	case AUX_ACCY:
		val = l->accAux[1];
		break;
	case AUX_ACCZ:
		val = l->accAux[2];
		break;
	case AUX_MAGX:
		val = l->magAux[0];
		break;
	case AUX_MAGY:
		val = l->magAux[1];
		break;
	case AUX_MAGZ:
		val = l->magAux[2];
		break;
	case VIN:
		val = l->vIn;
		break;
	case GPS_POS_MICROS:
		val = l->gpsPosUpdate;
		break;
	case LAT:
		val = l->lat;
		break;
	case LON:
		val = l->lon;
		break;
	case GPS_ALT:
		val = l->gpsAlt;
		break;
	case GPS_POS_ACC:
		val = l->gpsPosAcc;
		break;
	case GPS_VEL_MICROS:
		val = l->gpsVelUpdate;
		break;
	case GPS_VELN:
		val = l->gpsVel[0];
		break;
	case GPS_VELE:
		val = l->gpsVel[1];
		break;
	case GPS_VELD:
		val = l->gpsVel[2];
		break;
	case GPS_VEL_ACC:
		val = l->gpsVelAcc;
		break;
	case POSN:
		val = l->pos[0];
		break;
	case POSE:
		val = l->pos[1];
		break;
	case POSD:
		val = l->pos[2];
		break;
	case VELN:
		val = l->vel[0];
		break;
	case VELE:
		val = l->vel[1];
		break;
	case QUAT0:
		val = l->quat[0];
		break;
	case QUAT1:
		val = l->quat[1];
		break;
	case QUAT2:
		val = l->quat[2];
		break;
	case QUAT3:
		val = l->quat[3];
		break;
	case VELD:
		val = l->vel[2];
		break;
	case MOTOR1:
		val = l->motors[0];
		break;
	case MOTOR2:
		val = l->motors[1];
		break;
	case MOTOR3:
		val = l->motors[2];
		break;
	case MOTOR4:
		val = l->motors[3];
		break;
	case MOTOR5:
		val = l->motors[4];
		break;
	case MOTOR6:
		val = l->motors[5];
		break;
	case MOTOR7:
		val = l->motors[6];
		break;
	case MOTOR8:
		val = l->motors[7];
		break;
	case MOTOR9:
		val = l->motors[8];
		break;
	case MOTOR11:
		val = l->motors[9];
		break;
	case MOTOR12:
		val = l->motors[10];
		break;
	case MOTOR13:
		val = l->motors[11];
		break;
	case MOTOR14:
		val = l->motors[12];
		break;
	case THROTTLE:
		val = l->throttle;
		break;
	case EXTRA1:
		val = l->extra[0];
		break;
	case EXTRA2:
		val = l->extra[1];
		break;
	case EXTRA3:
		val = l->extra[2];
		break;
	case EXTRA4:
		val = l->extra[3];
		break;
	}

	return val;
}

void logDumpStats(loggerRecord_t *l) {
	int i, j;
	double val;

	for (i = 0; i < dumpNum; i++) {
		val = logDumpGetValue(l, dumpOrder[i]);
		if (val > dumpMax[i])
			dumpMax[i] = val;
		if (val < dumpMin[i])
			dumpMin[i] = val;
	}
}

void logDumpText(loggerRecord_t *l) {
	int i, j;

	for (i = 0; i < dumpNum; i++)
		printf("%.15G ", logDumpGetValue(l, dumpOrder[i]));

	if (dumpNum)
		printf("\n");
}


void logDumpPlotInit(int n) {
	float min = +9999999.99;
	float max = -9999999.99;
	int i;

	xVals = (PLFLT *)calloc(n, sizeof(PLFLT));
	yVals = (PLFLT *)calloc(n, sizeof(PLFLT));

	for (i = 0; i < dumpNum; i++) {
		if (dumpMin[i] < min)
			min = dumpMin[i];
		if (dumpMax[i] > max)
			max = dumpMax[i];
	}
	if (!isnan(scaleMin))
		min = scaleMin;
	if (!isnan(scaleMax))
		max = scaleMax;

        plinit();
        plcol0(15);
        plenv(0, n, min, max, 0, 0);

        for (i = 0; i < n; i++)
                xVals[i] = i;
}

int main(int argc, char **argv) {
        FILE *lf;
	int i;
	int count;

	plparseopts(&argc, (const char**)argv, PL_PARSE_SKIP);
        logDumpOpts(argc, argv);
        argc -= optind;
        argv += optind;

	if (argc < 1) {
		fprintf(stderr, "logDump: need log file argument\n");
		usage();
		exit(1);
	}

	lf = fopen(argv[0], "r");

	if (lf) {
		for (i = 0; i < NUM_FIELDS; i++) {
			dumpMin[i] = +9999999.99;
			dumpMax[i] = -9999999.99;
		}

		if (dumpPlot) {
			count = 0;

			// force header read
			loggerReadEntry(lf, &logEntry);
			rewind(lf);

			while (loggerReadEntry(lf, &logEntry) != EOF) {
				logDumpStats(&logEntry);
				count++;
			}

			logDumpPlotInit(count);
			
			for (i = 0; i < dumpNum; i++) {
				rewind(lf);
				count = 0;
				while (loggerReadEntry(lf, &logEntry) != EOF)
					yVals[count++] = logDumpGetValue(&logEntry, dumpOrder[i]);

				plcol0(2+i);
				plline(count, xVals, yVals);
			}

			plend();
		}
		else {
			count = 0;
			while (loggerReadEntry(lf, &logEntry) != EOF) {
				logDumpText(&logEntry);
				count++;
			}
		}

		fprintf(stderr, "logDump: %d records X %lu bytes = %4.1f MB\n", count, sizeof(logEntry), (float)count*sizeof(logEntry)/1024/1000);
		fprintf(stderr, "logDump: %d mins %d seconds @ 200Hz\n", count/200/60, count/200 % 60);
	}
	else {
		fprintf(stderr, "logDump: cannot open logfile\n");
	}
}
