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

#include "logDump.h"
#ifdef USE_MAVLINK
	#include "logDump_mavlink.h"
#endif
#include "plotter.h"
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>

// include export formatting templates (gpx/kml)
#include "logDump_templates.h"

bool dumpGpsTrack;
bool dumpTrigger;
bool usrSpecOutFreq;
bool exportMAV;
int dumpNum;
int dumpOrder[NUM_FIELDS];
const char *dumpHeaders[NUM_FIELDS];
int gpxWptCnt;
int gpxTrkCnt;
unsigned camTrigActivatedTime;
unsigned camTrigLastActive;
unsigned camTrigCnt;
double lastGpsFixTime;
double homeLat, homeLon;
double *dumpYMin, *dumpYMax;
double *dumpXMin, *dumpXMax;
char *trackDateStr;
char *gpxWaypoints;

filespec_t logfilespec;
loggerRecord_t logEntry;
time_t towStartTime;
FILE *outFP;

static const char *blnk = "";

void usage(void) {
	char outTxt[8000] = "\n\
Usage: logDump [options] [values] [plot options] logfile [ > outfile.ext ]\n\n\
Options Summary (see below for shorthand option names):\n\n\
	[--exp-format (csv|tab|gpx|kml)] [--col-headers] [--plot]\n\
	[--out-freq HZ] [--range-min num] [--range-max num]\n\
	[ --gps-track\n\
		[--gps-wpoints (include|only)]\n\
		[--alt-source (press|ukf)] [--alt-offset num]\n\
		[--track-min-hacc num] [--track-min-vacc num]\n\
	]\n\
	[--localtime] [--log-date DDMMYY]\n\
	[--trig-chan num] [--trig-val num] [--trig-only] [--trig-delay num]\n\
\n\
Option Details:\n\
\n\
 --exp-format (-e) type\n\
	Defines the export format. One of: txt, csv, tab, gpx, or kml.\n\
	(KML and GPX only work with the --gps-track option).\n\
\n\
 --col-headers (-c)\n\
	Include column headings row in the export.\n\
\n\
 --plot (-p)\n\
	Plot the data instead of exporting it (ignores -e & -c options).\n\
	(See plotting options, below. Use -h to get details.)\n\
\n\
 --out-freq (-f) number\n\
	Frequency of log dump output in whole Hz. Valid values are\n\
	1 to 200; default is 200 except when --gps-track is used,\n\
	in which case it is 5.\n\
\n\
 --range-min (-m) number\n\
	Start export at this record number (default is 1).\n\
\n\
 --range-max (-M) number\n\
	End export at this record number (zero means all records until end).\n\
\n\
 --gps-track (-g)\n\
	Dumps a GPS track log with date & time, lat, lon, altitude, and\n\
	horizontal velocity columns (uses CSV format w/headings by default).\n\
\n\
 --localtime (-l)\n\
	Output local time instead of UTC/GMT time (when used\n\
	with --gps-track or --gps-time).\n\
\n\
 --log-date (-d) DDMMYY\n\
	Use specified UTC date as actual date of log\n\
	instead of the log file modification date.\n\
	Used only with the --gps-track or --gps-time options.\n\
\n\
Options for use with --gps-track:\n\
\n\
 --gps-wpoints (-w)\n\
	Waypoint handling; one of: (i)nclude to export track\n\
	AND waypoints, or (o)nly to export the track as waypoints; \n\
	this option only applies when --exp-format is GPX or KML; \n\
	default behavior is to export only a track unless --trig-chan\n\
	is also used. NOTE: generating waypoints produces bigger\n\
	files and may take more time/memory to generate!\n\
\n\
 --alt-source (-A)\n\
	Use alternate altitude source instead of GPS;\n\
	One of: (p)ress to use un-adjusted pressure sensor altitude;\n\
		(u)kf to use derived altitude (UKF_POSD);\n\
\n\
 --alt-offset (-O)\n\
	Meters to add to altitude to get true MSL;\n\
	can be negative (decimal; default is 0).\n\
\n\
 --gps-min-hacc (-a)\n\
	Min. GPS horizontal accuracy for track log output\n\
	in meters (decimal; default is 2).\n\
\n\
 --gps-min-vacc (-v)\n\
	Min. GPS vertical accuracy for track log output\n\
	in meters (decimal; default is 2).\n\
\n\
Trigger log export options:\n\
\n\
 --trig-chan  (-t)\n\
	Radio channel used as a \"trigger\" (eg. for camera);\n\
	if specified, a column named \"trigger\" will be included\n\
	in the export with an incrementing value when trigger is active;\n\
	if GPX/KML output format is used, then each triggered track\n\
	point is also saved as a special waypoint.\n\
\n\
 --trig-val (-r)\n\
	Radio channel \"triggered\" value (output value at which\n\
	trigger is considered active); can be positive, negative, \n\
	or zero; if zero, then channel value must be within +/- 100;\n\
	(default is 250).\n\
\n\
 --trig-delay (-i)\n\
	Typical time delay (in ms) between trigger activation\n\
	and actual shutter opening (eg. focus delay). Using this option\n\
	will attempt to pinpoint the actual time/location of a photo.\n\
	Only useful in conjunction with --trig-chan or --gmbl-trig.\n\
\n\
 --trig-only (-y)\n\
	Only export log records where the trigger is active.\n\
\n\
Values (at least one is required unless --gps-track is used):\n\
\n\
 --all        Export all available values. MUST be last option before file name.\n\
 --micros     Microseconds since AQ boot.\n\
 --voltages   Voltage readings 0-14.\n\
 --rates      IMU RATE X, Y, & Z.\n\
 --accs       IMU ACC X, Y, Z, & overall ACC Magnitude.\n\
 --acc-bias   ACC_BIAS X, Y, & Z.\n\
 --mags       IMU MAG X, Y, Z, & overall MAG Magnitude.\n\
 --mag-sign   ADC_MAG_SIGN.\n\
 --pressures  Pressure 1, pressure 2.\n\
 --pres-alt   UKF_ALT and UKF_PRES_ALT.\n\
 --temps      Board temperature\n\
 --vin        Battery voltage.\n\
 --curr       Battery current.\n\
 --quat       Quaternions.\n\
 --poss       UKF positions N, E, & D.\n\
 --vels       UKF velocity N, E, D, and ALT.\n\
 --motors     Motor port outputs 0-13.\n\
 --throttle   Motor throttle.\n\
 --mot-pry    Motor pitch, roll, & yaw.\n\
 --attitude   Heading, roll, and pitch in degrees. IMU-derived and ACC-only derived.\n\
 --gps-time   Timestamp (based on log date + logged GPS time of week).\n\
 --gps-pos    GPS Latitude and Longitude.\n\
 --gps-alt    GPS altitude in meters.\n\
 --gps-vels   Velocities N, E, D, and calculated Ground Speed.\n\
 --gps-acc    GPS horizontal, vertical, and velocity accuracies in meters.\n\
 --gps-dops   PDOP, HDOP, VDOP, TDOP, NDOP, EDOP.\n\
 --gps-tow    GPS Time of Week.\n\
 --gps-updt   Timestamps of GPS position and velocity updates.\n\
 --radio-8    Radio channel inputs 0-7.\n\
 --radio-gt8  Radio channel inputs 8-17.\n\
 --radio-qual Radio quality rating and errors count.\n\
 --gmbl-trig  Trigger active state and activation count.\n\
";

	fprintf(stderr, "%s", outTxt);
	plotterUsage();
	fprintf(stderr, "\n\
Examples:\n\
 Dump all IMU_RATE values to the screen with space delimiter:\n\
    logDump --rates AQL-001.LOG\n\n\
 Export radio values to a CSV file with column headings:\n\
    logDump -e csv -c --radio-chan-8 AQL.LOG >output.csv\n\n\
 Export motor values and actual local time at 100Hz to text file with column headings:\n\
    logDump -f 100 -c -l --gps-time --motors AQL.LOG >output.txt\n\n\
 Export a GPS track log in GPX format:\n\
    logDump -g -e gpx AQL.LOG >gpslog.gpx\n\n\
 Plot some IMU values with 3 values per graph, to an SVG image file of 1200x1024px:\n\
    logDump --plot --accs --mags -vpg 3 -dev svg -o imu.svg -geo 1200x1024 AQL.LOG\n\
");
}

void logDumpOpts(int argc, char **argv) {
	int ch, i;
	static int longOpt;

	enum longOptions {
		O_ALL,
		O_MICROS,
		O_VOLTAGES,
		O_RATES,
		O_ACCS,
		O_MAGS,
		O_MAG_SIGN,
		O_PRESSURES,
		O_PRES_ALT,
		O_TEMPS,
		O_VIN,
		O_CURR,
		O_GPS_UPDATE_MICROS,
		O_LAT_LON,
		O_GPS_ALT,
		O_GPS_VELS,
		O_GPS_UTC_TIME,
		O_GPS_ACC,
		O_GPS_DOPS,
		O_GPS_ITOW,
		O_POSS,
		O_VELS,
		O_QUAT,
		O_MOTORS,
		O_THROTTLE,
		O_PRY,
		O_RADIO_QUALITY,
		O_RADIO_CHAN_8,
		O_RADIO_CHAN_GT8,
		O_ATTITUDE,
		O_ACC_BIAS,
		O_GMBL_TRIG
	};

	/* options descriptor */
	static struct option longopts[] = {
		{"help",			no_argument, 		NULL,		'h'},
		{"plot",			optional_argument,	NULL,		'p'},
		{"gps-track",		no_argument,		NULL,		'g'},
		{"out-freq",		required_argument,	NULL,		'f'},
		{"track-min-hacc",	required_argument,	NULL,		'a'},
		{"track-min-vacc",	required_argument,	NULL,		'v'},
		{"track-date",		required_argument,	NULL,		'd'},
		{"trig-chan",		optional_argument,	NULL,		't'},
		{"trig",			optional_argument,	NULL,		't'}, // legacy
		{"trig-val",		required_argument,	NULL,		'r'},
		{"out-trig-only",	no_argument,		NULL,		'y'}, // legacy
		{"trig-only",		no_argument,		NULL,		'y'},
		{"trig-delay",		required_argument,	NULL,		'i'},
		{"localtime",		no_argument,		NULL,		'l'},
		{"col-headers",		no_argument,		NULL,		'c'},
		{"exp-format",		required_argument,	NULL,		'e'},
		{"gps-wpoints",		required_argument,	NULL,		'w'},
		{"alt-source",		required_argument,	NULL,		'A'},
		{"alt-offset",		required_argument,	NULL,		'O'},
		{"range-min",		required_argument,	NULL,		'm'},
		{"range-max",		required_argument,	NULL,		'M'},
		{"all",				no_argument,		&longOpt,	O_ALL},
		{"micros",			no_argument,		&longOpt,	O_MICROS},
		{"voltages",		no_argument,		&longOpt,	O_VOLTAGES},
		{"rates",			no_argument,		&longOpt,	O_RATES},
		{"accs",			no_argument,		&longOpt,	O_ACCS},
		{"mags",			no_argument,		&longOpt,	O_MAGS},
		{"mag-sign",		no_argument,		&longOpt,	O_MAG_SIGN},
		{"pressures",		no_argument,		&longOpt,	O_PRESSURES},
		{"pres-alt",		no_argument,		&longOpt,	O_PRES_ALT},
		{"temps",			no_argument,		&longOpt,	O_TEMPS},
		{"vin",				no_argument,		&longOpt,	O_VIN},
		{"curr",			no_argument,		&longOpt,	O_CURR},
		{"gps-pos",			no_argument,		&longOpt,	O_LAT_LON},
		{"gps-lat-lon",		no_argument,		&longOpt,	O_LAT_LON}, // legacy
		{"gps-updt",		no_argument,		&longOpt,	O_GPS_UPDATE_MICROS},
		{"gps-pos-micros",	no_argument,		&longOpt,	O_GPS_UPDATE_MICROS}, // legacy
		{"gps-vel-micros",	no_argument,		&longOpt,	O_GPS_UPDATE_MICROS}, // legacy
		{"gps-alt",			no_argument,		&longOpt,	O_GPS_ALT},
		{"gps-vels",		no_argument,		&longOpt,	O_GPS_VELS},
		{"gps-acc",			no_argument,		&longOpt,	O_GPS_ACC},
		{"gps-pos-acc",		no_argument,		&longOpt,	O_GPS_ACC}, // legacy
		{"gps-alt-acc",		no_argument,		&longOpt,	O_GPS_ACC}, // legacy
		{"gps-vel-acc",		no_argument,		&longOpt,	O_GPS_ACC}, // legacy
		{"gps-time",		no_argument,		&longOpt,	O_GPS_UTC_TIME},
		{"gps-dops",		no_argument,		&longOpt,	O_GPS_DOPS},
		{"gps-tow",			no_argument,		&longOpt,	O_GPS_ITOW},
		{"poss",			no_argument,		&longOpt,	O_POSS},
		{"vels",			no_argument,		&longOpt,	O_VELS},
		{"quat",			no_argument,		&longOpt,	O_QUAT},
		{"motors",			no_argument,		&longOpt,	O_MOTORS},
		{"throttle",		no_argument,		&longOpt,	O_THROTTLE},
		{"mot-pry",			no_argument,		&longOpt,	O_PRY},
		{"out-pry",			no_argument,		&longOpt,	O_PRY}, // legacy
		{"radio-quality",	no_argument,		&longOpt,	O_RADIO_QUALITY},
		{"radio-8",			no_argument,		&longOpt,	O_RADIO_CHAN_8},
		{"radio-chan-8",	no_argument,		&longOpt,	O_RADIO_CHAN_8}, // legacy
		{"radio-gt8",		no_argument,		&longOpt,	O_RADIO_CHAN_GT8},
		{"radio-chan-gt8",	no_argument,		&longOpt,	O_RADIO_CHAN_GT8}, // legacy
		{"attitude",		no_argument,		&longOpt,	O_ATTITUDE},
		{"acc-bias",		no_argument,		&longOpt,	O_ACC_BIAS},
		{"gmbl-trig",		no_argument,		&longOpt,	O_GMBL_TRIG},
		{NULL,				0,					NULL,		0}
	};

	while ((ch = getopt_long(argc, argv, "hpglcyf:a:v:d:t::r:i:e:w:A:O:m:M:", longopts, NULL)) != -1) {
		switch (ch) {
			case 'h':
				usage();
				exit(0);
				break;
			case 'p':
				dumpPlot = true;
				break;
			case 'g':
				dumpGpsTrack = true;
				outputRealDate = true;
				includeHeaders = true;
				valueSep = ',';
				dumpOrder[dumpNum++] = FLD_GPS_UTC_TIME;
				dumpOrder[dumpNum++] = LOG_GPS_LAT;
				dumpOrder[dumpNum++] = LOG_GPS_LON;
				dumpOrder[dumpNum++] = LOG_GPS_HEIGHT;
				dumpOrder[dumpNum++] = FLD_GPS_H_SPEED;
				dumpOrder[dumpNum++] = LOG_UKF_VELD;
				dumpOrder[dumpNum++] = FLD_YAW;
				dumpOrder[dumpNum++] = FLD_ROLL;
				dumpOrder[dumpNum++] = FLD_PITCH;
				//dumpOrder[dumpNum++] = BRG_TO_HOME;
				//dumpOrder[dumpNum++] = CAM_TRIGGER;
				break;
			case 'f':
				if (atof(optarg)) {
					outputFreq = atoi(optarg);
					gpsTrackFreq = outputFreq;
					usrSpecOutFreq = true;
				}
				break;
			case 'a':
				gpsTrackMinHAcc = atof(optarg);
				break;
			case 'v':
				gpsTrackMinVAcc = atof(optarg);
				break;
			case 'd':
				trackDateStr = strdup(optarg);
				break;
			case 'l':
				utcToLocal = true;
				break;
			case 't':
				if (optarg)
					camTrigChannel = atoi(optarg);
				dumpTrigger = true;
				dumpOrder[dumpNum++] = FLD_CAM_TRIGGER;
				break;
			case 'r':
				camTrigValue = atoi(optarg);
				break;
			case 'y':
				dumpTriggeredOnly = true;
				break;
			case 'i':
				camTrigDelay = atoi(optarg) * 1000;
				break;
			case 'c':
				includeHeaders = true;
				break;
			case 'e':
				if (strcmp(optarg, "csv") == 0)
					valueSep = ',';
				else if (strcmp(optarg, "tab") == 0)
					valueSep = '	';
				else if (strcmp(optarg, "gpx") == 0)
					exportGPX = true;
				else if (strcmp(optarg, "kml") == 0)
					exportKML = true;
#ifdef USE_MAVLINK
				else if (strcmp(optarg, "mav") == 0)
					exportMAV = true;
#endif
				break;
			case 'w':
				if (!strcmp(optarg, "o") || !strcmp(optarg, "only"))
					gpsTrackAsWpts = true;
				else if (!strcmp(optarg, "i") || !strcmp(optarg, "include"))
					gpsTrackInclWpts = true;
				break;
			case 'A':
				if (!strcmp(optarg, "p") || !strcmp(optarg, "press"))
					gpsTrackUsePresAlt = true;
				else if (!strcmp(optarg, "u") || !strcmp(optarg, "ukf"))
					gpsTrackUseUkfAlt = true;
				break;
			case 'O':
				gpsTrackAltOffset = atof(optarg);
				break;
			case 'm':
				dumpRangeMin = strtoul(optarg, 0, 0);
				break;
			case 'M':
				dumpRangeMax = strtoul(optarg, 0, 0);
				break;
			case 0:
				switch (longOpt) {
					case O_ALL:
						dumpNum = 0;
						for (i=0; i < NUM_FIELDS; i++) {
							if (i != LOG_NUM_IDS)
								dumpOrder[dumpNum++] = i;
						}
						return;  // prevent dumpOrder overflow
					case O_MICROS:
						dumpOrder[dumpNum++] = LOG_LASTUPDATE;
						break;
					case O_VOLTAGES:
						for (i=0; i < LOG_NUM_VOLTAGES; i++)
							dumpOrder[dumpNum++] = LOG_VOLTAGE0 + i;
						break;
					case O_RATES:
						dumpOrder[dumpNum++] = LOG_IMU_RATEX;
						dumpOrder[dumpNum++] = LOG_IMU_RATEY;
						dumpOrder[dumpNum++] = LOG_IMU_RATEZ;
						break;
					case O_ACCS:
						dumpOrder[dumpNum++] = LOG_IMU_ACCX;
						dumpOrder[dumpNum++] = LOG_IMU_ACCY;
						dumpOrder[dumpNum++] = LOG_IMU_ACCZ;
						dumpOrder[dumpNum++] = FLD_ACC_MAGNITUDE;
						break;
					case O_MAGS:
						dumpOrder[dumpNum++] = LOG_IMU_MAGX;
						dumpOrder[dumpNum++] = LOG_IMU_MAGY;
						dumpOrder[dumpNum++] = LOG_IMU_MAGZ;
						dumpOrder[dumpNum++] = FLD_MAG_MAGNITUDE;
						break;
					case O_MAG_SIGN:
						dumpOrder[dumpNum++] = LOG_ADC_MAG_SIGN;
						break;
					case O_PRESSURES:
						dumpOrder[dumpNum++] = LOG_ADC_PRESSURE1;
						dumpOrder[dumpNum++] = LOG_ADC_PRESSURE2;
						break;
					case O_PRES_ALT:
						dumpOrder[dumpNum++] = LOG_UKF_ALT;
						dumpOrder[dumpNum++] = LOG_UKF_PRES_ALT;
						break;
					case O_TEMPS:
						dumpOrder[dumpNum++] = LOG_ADC_TEMP0;
						break;
					case O_VIN:
						dumpOrder[dumpNum++] = LOG_ADC_VIN;
						dumpOrder[dumpNum++] = LOG_VIN_PDB;
						break;
					case O_CURR:
						dumpOrder[dumpNum++] = LOG_CURRENT_PDB;
						dumpOrder[dumpNum++] = LOG_CURRENT_EXT;
						break;
					case O_GPS_UPDATE_MICROS:
						dumpOrder[dumpNum++] = LOG_GPS_POS_UPDATE;
						dumpOrder[dumpNum++] = LOG_GPS_VEL_UPDATE;
						break;
					case O_LAT_LON:
						dumpOrder[dumpNum++] = LOG_GPS_LAT;
						dumpOrder[dumpNum++] = LOG_GPS_LON;
						break;
					case O_GPS_ALT:
						dumpOrder[dumpNum++] = LOG_GPS_HEIGHT;
						break;
					case O_GPS_VELS:
						dumpOrder[dumpNum++] = LOG_GPS_VELN;
						dumpOrder[dumpNum++] = LOG_GPS_VELE;
						dumpOrder[dumpNum++] = LOG_GPS_VELD;
						dumpOrder[dumpNum++] = FLD_GPS_H_SPEED;
						break;
					case O_GPS_UTC_TIME:
						outputRealDate++;
						dumpOrder[dumpNum++] = FLD_GPS_UTC_TIME;
						break;
					case O_GPS_ACC:
						dumpOrder[dumpNum++] = LOG_GPS_HACC;
						dumpOrder[dumpNum++] = LOG_GPS_VACC;
						dumpOrder[dumpNum++] = LOG_GPS_SACC;
						break;
					case O_GPS_DOPS:
						dumpOrder[dumpNum++] = LOG_GPS_PDOP;
						dumpOrder[dumpNum++] = LOG_GPS_HDOP;
						dumpOrder[dumpNum++] = LOG_GPS_VDOP;
						dumpOrder[dumpNum++] = LOG_GPS_TDOP;
						dumpOrder[dumpNum++] = LOG_GPS_NDOP;
						dumpOrder[dumpNum++] = LOG_GPS_EDOP;
						break;
					case O_GPS_ITOW:
						dumpOrder[dumpNum++] = LOG_GPS_ITOW;
						break;
					case O_POSS:
						dumpOrder[dumpNum++] = LOG_UKF_POSN;
						dumpOrder[dumpNum++] = LOG_UKF_POSE;
						dumpOrder[dumpNum++] = LOG_UKF_POSD;
						break;
					case O_VELS:
						dumpOrder[dumpNum++] = LOG_UKF_VELN;
						dumpOrder[dumpNum++] = LOG_UKF_VELE;
						dumpOrder[dumpNum++] = LOG_UKF_VELD;
						dumpOrder[dumpNum++] = LOG_UKF_ALT_VEL;
						break;
					case O_QUAT:
						dumpOrder[dumpNum++] = LOG_UKF_Q1;
						dumpOrder[dumpNum++] = LOG_UKF_Q2;
						dumpOrder[dumpNum++] = LOG_UKF_Q3;
						dumpOrder[dumpNum++] = LOG_UKF_Q4;
						break;
					case O_MOTORS:
						for (i=0; i < LOG_NUM_MOTORS; i++)
							dumpOrder[dumpNum++] = LOG_MOT_MOTOR0 + i;
						break;
					case O_THROTTLE:
						dumpOrder[dumpNum++] = LOG_MOT_THROTTLE;
						break;
					case O_PRY:
						dumpOrder[dumpNum++] = LOG_MOT_PITCH;
						dumpOrder[dumpNum++] = LOG_MOT_ROLL;
						dumpOrder[dumpNum++] = LOG_MOT_YAW;
						break;
					case O_RADIO_QUALITY:
						dumpOrder[dumpNum++] = LOG_RADIO_QUALITY;
						dumpOrder[dumpNum++] = LOG_RADIO_ERRORS;
						break;
					case O_RADIO_CHAN_8:
						for (i=0; i < 8; i++)
							dumpOrder[dumpNum++] = LOG_RADIO_CHANNEL0 + i;
						break;
					case O_RADIO_CHAN_GT8:
						for (i=8; i < LOG_NUM_RADIO_CHAN; i++)
							dumpOrder[dumpNum++] = LOG_RADIO_CHANNEL0 + i;
						break;
					case O_ATTITUDE:
						dumpOrder[dumpNum++] = FLD_ROLL;
						dumpOrder[dumpNum++] = FLD_ACC_ROLL;
						dumpOrder[dumpNum++] = FLD_PITCH;
						dumpOrder[dumpNum++] = FLD_ACC_PITCH;
						dumpOrder[dumpNum++] = FLD_YAW;
						break;
					case O_ACC_BIAS:
						dumpOrder[dumpNum++] = LOG_ACC_BIAS_X;
						dumpOrder[dumpNum++] = LOG_ACC_BIAS_Y;
						dumpOrder[dumpNum++] = LOG_ACC_BIAS_Z;
						break;
					case O_GMBL_TRIG:
						dumpTrigger = true;
						dumpOrder[dumpNum++] = LOG_GMBL_TRIGGER;
						break;
				} // longopt switch
				break;
			default:
				// fprintf(stderr, "logDump: logDumpOpts: error\n");
				break;
		} // getopt value switch
	} // while getopt has value loop
}

void attitudeExtractEulerQuat(float *q, double *yaw, double *pitch, double *roll) {
	float q0, q1, q2, q3;

	q0 = q[1];
	q1 = q[2];
	q2 = q[3];
	q3 = q[0];

	*yaw = atan2((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
	*pitch = asin(-2.0f * (q0 * q2 - q1 * q3));
	*roll = atan((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));
}

//float presToAlt(float pressure) {
//	return (1.0 - pow(pressure / P0, 0.19)) * (1.0 / 22.558e-6);
//}

//float adcIDGVoltsToTemp(float volts) {
//	return (25.0f + (volts - ADC_TEMP_OFFSET) * ADC_TEMP_SLOPE);
//}

// TODO: optimize
//float adcT1VoltsToTemp(float volts) {
//	float r1;
//	float ln;
//	float kelvin;
//
//	r1 = (ADC_TEMP_R2 * ADC_REF_VOLTAGE) / volts - ADC_TEMP_R2;
//	ln = log(r1);
//	kelvin = ADC_TEMP_A + ADC_TEMP_B * ln + ADC_TEMP_C * (ln*ln*ln);
//
//	return (kelvin + ADC_KELVIN_TO_CELCIUS);
//}

// get filename from full path; return path, file, and ext separately
filespec_t extractFileName(char *fp) {
	// filespec_t = {path, name, ext}
	filespec_t r = {(char *)blnk ,(char *)blnk ,(char *)blnk };

	if (strchr(fp, '\\') || strchr(fp, '/')) {
		r.name = fp + strlen(fp);
		r.path = fp;
		for (; r.name > fp; r.name--) {
			if ((*r.name == '\\') || (*r.name == '/')) {
				r.path[strlen(r.path)-strlen(r.name)] = 0;
				r.name++;
				break;
			}
		}
	} else {
		r.name = fp;
	}
	if (strchr(r.name, '.')) {
		r.ext = r.name + strlen(r.name);
		for (; r.ext > fp; r.ext--) {
			if (*r.ext == '.') {
				r.name[strlen(r.name)-strlen(r.ext)] = 0;
				r.ext++;
				break;
			}
		}
	}

	return r;
}

// return the UTC offset in seconds
int getUTCOffset(void) {
	time_t now = time(NULL);

	struct tm ltime = *localtime(&now);
	struct tm utime = *gmtime(&now);
	int utcOffset = difftime(mktime(&ltime), mktime(&utime));

	return utcOffset;
}

// format iTOW to full ISO8601 date-time
void formatIsoTime(char *s, double v) {
	char timeStr[31], buff[10];
	time_t timeVal;
	int utcOffset = getUTCOffset();
	int utcOffsetHrs = utcOffset / 3600;
	int utcOffsetMin = (utcOffset % 3600) / 60;

	timeVal = towStartTime + (v/1000);
	if (utcToLocal)
		timeVal += utcOffset;
	strftime(timeStr, 31, "%Y-%m-%dT%H:%M:%S", localtime(&timeVal));
	sprintf(buff, ".%.3d", (int)v % 1000);
	strcat(timeStr, buff);
	if (utcToLocal) {
		sprintf(buff, "%.2d:%.2d", utcOffsetHrs, utcOffsetMin);
		strcat(timeStr, buff);
	} else {
		strcat(timeStr, "Z");
	}

	sprintf(s, "%s", timeStr);
}

// input lat/lon in degrees, returns bearing in radians
double navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 *= (double)DEG_TO_RAD;
    lat2 *= (double)DEG_TO_RAD;
    lon1 *= (double)DEG_TO_RAD;
    lon2 *= (double)DEG_TO_RAD;
    double ret = atan2(sin(lat1) * (lon2 - lon1), lat2 - lat1);

	if (!isfinite(ret))
		ret = 0.0f;

	return ret;
}

double logDumpGetValue(loggerRecord_t *l, int field) {
	double val = nan("");
	double rpy[3];

	switch (field) {
		// calculated values:
		case FLD_GPS_H_SPEED:
			val = (fabs(l->data[LOG_GPS_VELN]) + fabs(l->data[LOG_GPS_VELE])); // * 3600 / 1000; // km/h
			break;
		case FLD_GPS_UTC_TIME:
			val = l->data[LOG_GPS_ITOW];
			break;
		case FLD_CAM_TRIGGER:
		case LOG_GMBL_TRIGGER:
			// is trigger active?
			if ( l->data[LOG_GMBL_TRIGGER] || (
					camTrigChannel > 0 && camTrigChannel < 19 && (
						(camTrigValue < 0 && l->radioChannels[camTrigChannel-1] < camTrigValue) ||
						(camTrigValue > 0 && l->radioChannels[camTrigChannel-1] > camTrigValue) ||
						(camTrigValue == 0 && l->radioChannels[camTrigChannel-1] > -TRIG_ZERO_BUFFER && l->radioChannels[camTrigChannel-1] < TRIG_ZERO_BUFFER) )
					)) {
				// first time this trigger is activated
				if (!camTrigActivatedTime)
					camTrigActivatedTime = l->data[LOG_LASTUPDATE];

				// use count in LOG_GMBL_TRIGGER if available
				val = (l->data[LOG_GMBL_TRIGGER]) ? l->data[LOG_GMBL_TRIGGER] : ++camTrigCnt;

				// if a shutter delay is configured, we only want one positive return value per trigger activation, after the specified delay time
				if (camTrigDelay && (val == camTrigLastActive || l->data[LOG_LASTUPDATE] < camTrigActivatedTime + camTrigDelay))
					val = 0;
			}
			// trigger is not active
			else {
				val = 0;
				camTrigActivatedTime = 0;
			}
			break;
		case FLD_ROLL:
			attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
			val = rpy[0] * -1.0 * RAD_TO_DEG;
			break;
		case FLD_PITCH:
			attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
			val = rpy[1] * -1.0 * RAD_TO_DEG;
			break;
		case FLD_YAW:
			attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
			val = rpy[2] * RAD_TO_DEG;
			if (val < 0) val = 360 + val;
			break;
	    case FLD_ACC_PITCH :
	    	val = atan2(l->data[LOG_IMU_ACCX], -l->data[LOG_IMU_ACCZ]) * -1.0 * RAD_TO_DEG;
	        break;
	    case FLD_ACC_ROLL :
	    	val = atan2(-l->data[LOG_IMU_ACCY], -l->data[LOG_IMU_ACCZ]) * -1.0 * RAD_TO_DEG;
	        break;
		case FLD_BRG_TO_HOME:
			val = navCalcBearing(homeLat, homeLon, logDumpGetValue(l, LOG_GPS_LAT), logDumpGetValue(l, LOG_GPS_LON));
			break;
	    case FLD_MAG_MAGNITUDE :
	        rpy[0] = l->data[LOG_IMU_MAGX];
	        rpy[1] = l->data[LOG_IMU_MAGY];
	        rpy[2] = l->data[LOG_IMU_MAGZ];
	        val = sqrt(rpy[0]*rpy[0] + rpy[1]*rpy[1] + rpy[2]*rpy[2]);
	        break;
	    case FLD_ACC_MAGNITUDE :
	        rpy[0] = l->data[LOG_IMU_ACCX];
	        rpy[1] = l->data[LOG_IMU_ACCY];
	        rpy[2] = l->data[LOG_IMU_ACCZ];
	        val = sqrt(rpy[0]*rpy[0] + rpy[1]*rpy[1] + rpy[2]*rpy[2]);
	        break;
		// logged values
		default:
			if (field < LOG_NUM_IDS)
				val = l->data[field];
			break;
	}
	return val;
}

void logDumpStats(loggerRecord_t *l) {
	int i, j;
	double val;

	for (i = 0; i < dumpNum; i++) {
		val = logDumpGetValue(l, dumpOrder[i]);
		if (val > dumpYMax[i])
			dumpYMax[i] = val;
		if (val < dumpYMin[i])
			dumpYMin[i] = val;
	}
}

void logDumpHeaders(void) {
	int i;

	if (dumpNum) {
		for (i = 0; i < dumpNum; i++) {
			printf("%s%c", dumpHeaders[dumpOrder[i]], (i < dumpNum-1 ? valueSep : 0));
		}
		printf("\n"); // end of export row
	}
}

void logDumpText(loggerRecord_t *l) {
	int i, mkwpt;
	double logVal, gpsFixTime;
	char outStr[31];
	char gpxTrkptOut[1000];
	char *trackName;
	char lclTrigWptName[40];
	static bool homeSet;
	unsigned trigCount;
	expFields_t exp;

	// check for home position being set
	if (homeSetChannel && posHoldChannel) {
		if (!homeSet && (l->radioChannels[homeSetChannel-1] > 250 ||
				(homeLat == 0.0f && l->radioChannels[posHoldChannel-1] > 250))) {
			homeLat = logDumpGetValue(l, LOG_GPS_LAT);
			homeLon = logDumpGetValue(l, LOG_GPS_LON);
			homeSet = true;
		}
		else if (l->radioChannels[homeSetChannel-1] < 250)
			homeSet = false;
	}

	// flat text format
	if (!exportGPX && !exportKML && !exportMAV) {

		for (i = 0; i < dumpNum; i++) {
			logVal = logDumpGetValue(l, dumpOrder[i]);

			if (dumpOrder[i] == FLD_GPS_UTC_TIME)
				formatIsoTime(outStr, logVal);
			else
				sprintf(outStr, "%.15G", logVal);

			if ((dumpOrder[i] == FLD_CAM_TRIGGER || dumpOrder[i] == LOG_GMBL_TRIGGER) && (bool)logVal)
				camTrigLastActive = logVal;

			printf(outStr);

			if (i < dumpNum-1)
				printf("%c", valueSep);

		}
		printf("\n"); // end of export row

	}
#ifdef USE_MAVLINK
	// mavlink log format (experimental)
	else if (exportMAV) {
		mavlinkDo(l);
	}
#endif
	// KML/GPX format
	else {

		strcpy(exp.name, "");
		strcpy(exp.wptstyle, "waypoint");
		trackName = (char *) calloc(strlen(logfilespec.name)+10, sizeof(char));
		strcpy(trackName, logfilespec.name);
		mkwpt = 0;

		exp.lat = logDumpGetValue(l, LOG_GPS_LAT);
		exp.lon = logDumpGetValue(l, LOG_GPS_LON);
		if (gpsTrackUsePresAlt)
			exp.alt = logDumpGetValue(l, LOG_UKF_PRES_ALT);
		else if (gpsTrackUseUkfAlt)
			exp.alt = logDumpGetValue(l, LOG_UKF_POSD);
		else
			exp.alt = logDumpGetValue(l, LOG_GPS_HEIGHT);
		exp.alt += gpsTrackAltOffset;
		exp.speed = logDumpGetValue(l, FLD_GPS_H_SPEED);
		exp.climb = logDumpGetValue(l, LOG_UKF_VELD);
		exp.hdg = logDumpGetValue(l, FLD_YAW);
		exp.roll = logDumpGetValue(l, FLD_ROLL);
		exp.pitch = logDumpGetValue(l, FLD_PITCH);
		exp.lat = logDumpGetValue(l, LOG_GPS_LAT);

		formatIsoTime(outStr, logDumpGetValue(l, FLD_GPS_UTC_TIME));
		strcpy(exp.time, outStr);

		if (dumpTrigger) {
			trigCount = (unsigned)logDumpGetValue(l, FLD_CAM_TRIGGER);
			if (trigCount) {
				sprintf(lclTrigWptName, "%s-%d", trigWptName, trigCount);
				sprintf(exp.name, lclTrigWptName);
				strcpy(exp.wptstyle, "trg_waypoint");
				mkwpt = 1;
				camTrigLastActive = trigCount;
			}
		}

		if ( (gpsTrackAsWpts || gpsTrackInclWpts) && !strlen(exp.name) ) {
			gpxWptCnt++;
			sprintf(lclTrigWptName, "%s-%d", "wpt", gpxWptCnt);
			sprintf(exp.name, lclTrigWptName);
		}

		if (!gpsTrackAsWpts) {
			// if exporting a track, check for gap in gps time and start a new track if gap is too large;
			// this is to avoid interpolated lines between the end/start points
			gpsFixTime = logDumpGetValue(l, FLD_GPS_UTC_TIME);
			if (lastGpsFixTime && gpsFixTime - lastGpsFixTime > GPS_TRACK_MAX_TM_GAP) {
				gpxTrkCnt++;
				sprintf(trackName, "%s-%d", logfilespec.name, gpxTrkCnt);
				if (exportGPX) {
					printf(gpxTrkEnd);
					printf(gpxTrkStart, trackName);
				} else {
					printf(kmlModel, trackModelURL);
					printf(kmlTrkEnd);
					printf(kmlTrkStart, trackName, trackAltMode);
				}
			}
			lastGpsFixTime = gpsFixTime;

			if (exportGPX)
				// template value order: lat, lon, ele, time, heading, speed
				printf(gpxTrkptTempl, exp.lat, exp.lon, exp.alt, exp.time, exp.hdg, exp.speed);
			else {
				printf(kmlTrkTimestamp, exp.time);
				// template value order: lon, lat, ele
				printf(kmlTrkCoords, exp.lon, exp.lat, exp.alt);
				// template value order: heading, tilt, roll
				printf(kmlTrkAngles, exp.hdg, exp.pitch, exp.roll);
			}

		}

		if (mkwpt || gpsTrackAsWpts || gpsTrackInclWpts) {
			if (exportGPX)
				// template value order: lat, lon, ele, time, heading, speed, name
				sprintf(gpxTrkptOut, gpxWptTempl, exp.lat, exp.lon, exp.alt, exp.time, exp.hdg, exp.speed, exp.name);
			else
				// str replace order: wpt name, date/time, lat, lon, alt, speed, speed (km/h), heading, climb rate,
				//		isotime, wpt style, alt. mode, lon, lat, alt
				sprintf(gpxTrkptOut, kmlWptTempl, exp.name, exp.time, exp.lat, exp.lon, exp.alt, exp.speed, exp.speed*3600/1000,
						exp.hdg, exp.roll, exp.pitch, -exp.climb, exp.time, exp.wptstyle, waypointAltMode, exp.lon, exp.lat, exp.alt );

			if (gpsTrackAsWpts)
				printf(gpxTrkptOut);
			else {
				gpxWaypoints = (char *) realloc(gpxWaypoints, (strlen(gpxWaypoints) + (strlen(gpxTrkptOut)+1)) * sizeof(char));
				strcat(gpxWaypoints, gpxTrkptOut);
			}
		}

	} // export format
}

bool logDumpCheckRecordForExport(const uint32_t count, loggerRecord_t *logEntry) {
	return count >= dumpRangeMin &&
		!(count % OUTPUT_FREQ_DIVISOR) &&
		( !dumpTriggeredOnly || logDumpGetValue(logEntry, FLD_CAM_TRIGGER) ) &&
		( !dumpGpsTrack || ( logDumpGetValue(logEntry, LOG_GPS_HACC) <= gpsTrackMinHAcc && logDumpGetValue(logEntry, LOG_GPS_VACC) <= gpsTrackMinVAcc) );
}

bool logDumpProgress(const uint32_t count) {
	// send progress indication
	if (!(count % 1000)) {
		fprintf(stderr, ".");
		fflush(stderr);
	}
	return !dumpRangeMax || count <= dumpRangeMax;
}

int main(int argc, char **argv) {
	FILE *lf;
	int i, j;
	uint32_t count = 0; // total log line counter
	uint32_t exp_count = 0; // total exported lines counter
	struct stat sbuf; // file stat() buffer

	outFP = stdout;
	dumpNum = 0;

	plotterOpts(argc, argv);
	logDumpOpts(argc, argv);
	argc -= optind;
	argv += optind;

	fprintf(stderr, "\n");
	if (argc < 1) {
		fprintf(stderr, "logDump: need log file argument. Type logDump --help for usage details.\n");
		exit(1);
	}
	if (dumpNum < 1) {
		fprintf(stderr, "logDump: need at least one value to export. Type logDump --help for usage details.\n");
		exit(1);
	}

	// init waypoint storage
	gpxWaypoints = (char *) calloc(1, sizeof(char));

	// determine output frequency
	if (dumpGpsTrack && !usrSpecOutFreq) // use lower default setting for gps track log
		outputFreq = gpsTrackFreq;

	// set up field labels (combine from logger.h and logdump.h)
	for (i=0; i < LOG_NUM_IDS; i++)
		dumpHeaders[i] = loggerFieldLabels[i];
	j = 0;
	for (i++; i < NUM_FIELDS; i++)
		dumpHeaders[i] = logDumpFieldLabels[j++];

	fprintf(stderr, "logDump: opening logfile: %s\n", argv[0]);

	if ( !stat(argv[0], &sbuf) ) {

		// if asked to output a real date column, need to get a base date to start from
		if (outputRealDate) {

			char fileDateStr[30] = "";
			time_t now = time(NULL);
			// init gps track log time with current UTC time
			struct tm* trackTime = gmtime(&now);

			strftime(fileDateStr, 100, "%d-%m-%Y %H:%M:%S", localtime(&sbuf.st_mtime));
			fprintf(stderr, "logDump: Logfile last modified: %s (UTC)\n", fileDateStr);

			// set track date to log file date unless date was specified via options
			if ( trackDateStr == NULL || strlen(trackDateStr) != 6 ) {
				trackTime = localtime(&sbuf.st_mtime); // log date is actually in UTC time even though system thinks it's local
			} else {
				char tday[3], tmon[3], tyr[3];
				strncpy(tday, trackDateStr, 2);
				strncpy(tmon, trackDateStr+2, 2);
				strncpy(tyr, trackDateStr+4, 2);
				trackTime->tm_year = atoi(tyr + 0) + 100;
				trackTime->tm_mon = atoi(tmon + 0) - 1;
				trackTime->tm_mday = atoi(tday + 0);
			}

			// zero time values for start of week calculation (GPS Time Of Week starts on each Sunday at 00:00:00)
			trackTime->tm_hour = 0;
			trackTime->tm_min = 0;
			trackTime->tm_sec = 0;

			// seconds to add to GPS ToW from log
			towStartTime = mktime(trackTime);

			// find the previous Sunday if we don't have it already
			while (trackTime->tm_wday != 0) {
				trackTime->tm_mday -= 1;
				// TOW always starts on a Sunday
				towStartTime = mktime(trackTime);
			}

			strftime(fileDateStr, 100, "%d-%b-%Y %H:%M:%S", trackTime);
			fprintf(stderr, "logDump: using GPS Time of Week reference date: %s\n", fileDateStr);

			if (utcToLocal) {
				// use local time in track log
				// towStartTime += getUTCOffset();
				fprintf(stderr, "logDump: adjusting date/time output to local time (UTC %.1fh).\n", getUTCOffset() / 3600.0f);
			}

		} // if outputRealDate

	} else {
		fprintf(stderr, "logDump: could not access logfile: %s\n", argv[0]);
		exit(1);
	}

#if defined (__WIN32__)
	lf = fopen(argv[0], "rb");
#else
	lf = fopen(argv[0], "r");
#endif

	logfilespec = extractFileName(argv[0]);

	if (lf) {
		fprintf(stderr, "\n");

#ifdef USE_MAVLINK
		if (exportMAV) {
			mavlinkInit();
			char outfileName[] = "test.mavlink";
			outFP = fopen(outfileName, "wb");
			if (outFP == NULL) {
				fprintf(stderr, "logDump: cannot open output file '%s'\n", outfileName);
				exit(0);
			}
		}
#endif

		if (includeHeaders && !exportGPX && !exportKML && !exportMAV && !dumpPlot) {
			// write text header
			logDumpHeaders();
		} else if (exportGPX) {
			// write GPX header
			printf(gpxHeader);
			if (!gpsTrackAsWpts)
				printf(gpxTrkStart, logfilespec.name);
			gpxTrkCnt++;
		} else if (exportKML) {
			// write KML header
			// str replace order: document title, wpt color, wpt icon, wpt color, wpt icon,
			// 		wpt trg color, wpt icon, wpt trg color, wpt icon, line color, line width (d), line color, line width (d)
			printf(kmlHeader, logfilespec.name, waypointColor, waypointIconURL, waypointColor, waypointIconURL,
					waypointTrigColor, waypointIconURL, waypointTrigColor, waypointIconURL, trackColor, trackWidth, trackColor, trackWidth);
			if (!gpsTrackAsWpts) {
				printf(kmlFolderHeader, "Track", "Track");
				// str replace order: track name, track ID, alt. mode
				printf(kmlTrkHeader, logfilespec.name, logfilespec.name);
				printf(kmlTrkStart, logfilespec.name, trackAltMode);
			} else
				printf(kmlFolderHeader, "Points", "Points");
			gpxTrkCnt++;
		}

		// plot output
		if (dumpPlot) {
			double *xVals, *yVals;

			// need to get X & Y extents for all plotted values to initialize plotter

			dumpYMin = (double *)calloc(dumpNum, sizeof(double));
			dumpYMax = (double *)calloc(dumpNum, sizeof(double));
			dumpXMin = (double *)calloc(dumpNum, sizeof(double));
			dumpXMax = (double *)calloc(dumpNum, sizeof(double));
			// initialize with bogus values
			std::fill(dumpYMin, dumpYMin + dumpNum, +9999999.99);
			std::fill(dumpYMax, dumpYMax + dumpNum, -9999999.99);

			// force header read
			loggerReadEntry(lf, &logEntry);
			rewind(lf);

			// read through entire log to update dumpYMin/dumpYMax extents for each value being exported
			while (loggerReadEntry(lf, &logEntry) != EOF) {
				if (logDumpCheckRecordForExport(count++, &logEntry)) {
					logDumpStats(&logEntry);
					exp_count++;
				}
				if (!logDumpProgress(count))
					break;
			}

			// NOTE: everything below assumes that all logged columns (values) have the same number of samples (exp_count).
			// We could do this per value instead (inside the next log reading loop) but at this point it's overkill.

			xVals = (double *)calloc(exp_count, sizeof(double));
			yVals = (double *)calloc(exp_count, sizeof(double));

			// populate X graph values with zero through n samples
			for (i = 0; i < exp_count; i++)
				xVals[i] = (double)(i * OUTPUT_FREQ_DIVISOR + dumpRangeMin);

			std::fill(dumpXMin, dumpXMin + dumpNum, *std::min_element(xVals, xVals + exp_count));
			std::fill(dumpXMax, dumpXMax + dumpNum, *std::max_element(xVals, xVals + exp_count));

			if (!plotterInit(dumpNum, dumpYMin, dumpYMax, dumpXMin, dumpXMax))
				exit(1);

			for (i = 0; i < dumpNum; i++) {
				rewind(lf);
				count = exp_count = 0;
				while (loggerReadEntry(lf, &logEntry) != EOF) {
					if (logDumpCheckRecordForExport(count++, &logEntry))
						yVals[exp_count++] = logDumpGetValue(&logEntry, dumpOrder[i]);

					if (!logDumpProgress(count))
						break;
				}
				plotterLine(exp_count, i, xVals, yVals, dumpHeaders[dumpOrder[i]]);
			}

			plotterEnd();

			free(dumpYMin);
			free(dumpYMax);
			free(dumpXMin);
			free(dumpXMax);
			free(xVals);
			free(yVals);
		}
		// file export
		else {
			while (loggerReadEntry(lf, &logEntry) != EOF) {
				if (logDumpCheckRecordForExport(count++, &logEntry)) {
					logDumpText(&logEntry);
					exp_count++;
				}
				if (!logDumpProgress(count))
					break;
			}
		}

		// finish up writing GPX/KML export
		if (exportGPX) {
			if (!gpsTrackAsWpts)
				// close track log
				printf(gpxTrkEnd);
			// write waypoints, if any
			if (strlen(gpxWaypoints))
				printf(gpxWaypoints);
			// close gpx
			printf(gpxFooter);
		}
		else if (exportKML) {
			if (!gpsTrackAsWpts) {
				// close track log
				printf(kmlModel, trackModelURL);
				printf(kmlTrkEnd);
				printf(kmlTrkFooter);
			}
			printf(kmlFolderFooter);
			// write waypoints, if any
			if (strlen(gpxWaypoints)) {
				printf(kmlFolderHeader, "Points", "Points");
				printf(gpxWaypoints);
				printf(kmlFolderFooter);
			}
			// close kml
			printf(kmlFooter);
		}


		fprintf(stderr, "\n\nlogDump: %d total records X %lu bytes = %4.1f MB\n", count, sizeof(logEntry), (float)count*sizeof(logEntry)/1024/1000);
		fprintf(stderr, "logDump: %d mins %d seconds @ %dHz exported %d records\n", count/200/60, count/200 % 60, outputFreq, exp_count);
		if (dumpTriggeredOnly)
			fprintf(stderr, "logDump: only triggered records were exported\n");
		if (dumpGpsTrack)
			fprintf(stderr, "logDump: GPS accuracy filters were applied (h=%.1fm; v=%.1fm); starttime: %u\n", gpsTrackMinHAcc, gpsTrackMinVAcc, towStartTime);
		if (gpxWptCnt)
			fprintf(stderr, "logDump: %d waypoints exported to GPX\n", gpxWptCnt);
	}
	else {
		fprintf(stderr, "logDump: cannot open logfile\n");
		exit(1);
	}
	exit(0);
}
