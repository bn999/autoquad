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

#include "logDump.h"
#include "logDump_mavlink.h"
#ifdef HAS_PLPLOT
	#include "plplot/plplot.h"
#endif
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>

// include export formatting templates (gpx/kml)
#include "logDump_templates.h"

#ifdef HAS_PLPLOT
        PLFLT *xVals;
        PLFLT *yVals;
#endif

time_t towStartTime;
FILE *outFP;

void usage(void) {
	char outTxt[8000] = "\n\
Usage: logDump [options] [values] logfile [ > outfile.ext ]\n\n\
Options Summary (see below for shorthand option names):\n\n\
	[ --help] [ --exp-format=(csv|tab|gpx|kml) ]\n\
	[ --out-freq=HZ ] [ --col-headers ]\n";

#ifdef HAS_PLPLOT
	strcat(outTxt, "	[ --plot [ --scale-min=N ] [ --scale-max=N ] ]\n");
#endif

	strcat(outTxt, "\
	[ --gps-track\n\
		[ --gps-wpoints=(include|only) ]\n\
		[ --alt-source=(press|ukf) ] [ --alt-offset=N ]\n\
		[ --track-min-hacc=M ] [ --track-min-vacc=M ]\n\
	]\n\
	[ --log-date=DDMMYY ] [ --localtime ]\n\
	[ --trig-chan=N ] [ --trig-val=N ] [ --trig-only ] [ --trig-delay=N ]\n\
\n\
Option Details:\n\
\n\
 --help	(-h)	Print this message.\n\
\n\
 --exp-format (-e)\n\
	Defines the export format. One of:\n\
		txt (default), csv, tab, gpx, or kml\n\
	(the last 2 choices only work with the --gps-track option).\n\
\n\
 --out-freq (-f)\n\
	Frequency of log dump output in whole Hz. Valid values are\n\
	1 to 200; default is 200 except when --gps-track is used,\n\
	in which case it is 5.\n\
\n\
 --col-headers (-c)\n\
	Include column headings row in the export.\n\
\n\
 --log-date (-d)\n\
	Use specified UTC date as actual date of log\n\
	instead of the log file modification date.\n\
	Used only with the --gps-track or --gps-time options.\n\
	Use DDMMYY format (eg. 250512).\n\
\n\
 --localtime (-l)\n\
	Output local time instead of UTC/GMT time (when used\n\
	with --gps-track or --gps-time).\n\
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
 --gps-track (-g)\n\
	Dumps a GPS track log in CSV format with date & time,\n\
	lat, lon, altitude, and horizontal velocity columns.\n\
\n\
Options only useful in conjunction with --gps-track:\n\
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
Values (at least one is required unless --gps-track is used):\n\
\n\
 --micros	Microseconds since AQ boot.\n\
 --voltages	Voltage readings 0-14.\n\
 --rates	IMU RATE X, Y, & Z.\n\
 --accs		IMU ACC X, Y, & Z.\n\
 --acc-bias	ACC_BIAS X, Y, & Z.\n\
 --mags		IMU MAG X, Y, & Z.\n\
 --mag-sign	ADC_MAG_SIGN.\n\
 --pressures	Pressure 1, pressure 2.\n\
 --pres-alt	UKF_ALT and UKF_PRES_ALT.\n\
 --temps	Board temperature\n\
 --vin		Battery voltage.\n\
 --quat		Quaternions.\n\
 --poss		UKF positions N, E, & D.\n\
 --vels		UKF velocity N, E, & D.\n\
 --motors	Motor port outputs 0-13.\n\
 --throttle	Output throttle.\n\
 --out-pry	Output pitch, roll, & yaw.\n\
 --attitude	Heading, roll, and pitch in degrees.\n\
 --gps-time	Timestamp (based on log date + logged GPS time of week).\n\
 --gps-lat-lon	GPS Latitude and Longitude.\n\
 --gps-pos-acc	GPS horizontal position accuracy in meters.\n\
 --gps-alt	GPS altitude in meters.\n\
 --gps-alt-acc	GPS vertical position accuracy in meters.\n\
 --gps-vels	Velocities N, E, & D.\n\
 --gps-vel-acc	GPS velocity accuracy in meters.\n\
 --gps-dops	PDOP, HDOP, VDOP, TDOP, NDOP, EDOP.\n\
 --gps-tow	GPS Time of Week.\n\
 --gps-pos-micros	\n\
 --gps-vel-micros	\n\
 --radio-chan-8	Radio channel inputs 0-7\n\
 --radio-chan-gt8 Radio channel inputs 8-17\n\
 --radio-quality	\n\
 --gmbl-trig	Trigger active state and activation count\n\
\n\
Examples:\n\
	logDump --rates AQL-001.LOG\n\
	logDump -c -e csv --radio-chan-8 AQL-001.LOG >output.csv\n\
	logDump -f 100 -c -l --gps-time --motors AQL-001.LOG >output.txt\n\
	logDump -g -e gpx -d 250812 AQL-001.LOG >gpslog-001.gpx\n\
");

// Depreciated options: --lat --lon --extras --aux-accs

	fprintf(stderr, "%s", outTxt);
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
		O_MAG_SIGN,
		O_PRESSURES,
		O_PRES_ALT,
		O_TEMPS,
		O_VIN,
		O_GPS_POS_MICROS,
		O_LAT_LON,
		O_GPS_ALT,
		O_GPS_POS_ACC,
		O_GPS_VEL_MICROS,
		O_GPS_VELS,
		O_GPS_VEL_ACC,
		O_GPS_VERT_ACC,
		O_GPS_UTC_TIME,
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
                {"scale-min",		required_argument,	NULL,		'n'},
                {"scale-max",		required_argument,	NULL,		'x'},
                {"gps-track",		no_argument,		NULL,		'g'},
                {"out-freq",		required_argument,	NULL,		'f'},
                {"track-min-hacc",	required_argument,	NULL,		'a'},
                {"track-min-vacc",	required_argument,	NULL,		'v'},
                {"track-date",		required_argument,	NULL,		'd'},
                {"trig-chan",		optional_argument,	NULL,		't'},
                {"trig",			optional_argument,	NULL,		't'},
                {"trig-val",		required_argument,	NULL,		'r'},
                {"out-trig-only",	no_argument,		NULL,		'y'},
                {"trig-only",		no_argument,		NULL,		'y'},
                {"trig-delay",		required_argument,	NULL,		'i'},
                {"localtime",		no_argument,		NULL,		'l'},
                {"col-headers",		no_argument,		NULL,		'c'},
                {"exp-format",		required_argument,	NULL,		'e'},
                {"gps-wpoints",		required_argument,	NULL,		'w'},
                {"alt-source",		required_argument,	NULL,		'A'},
                {"alt-offset",		required_argument,	NULL,		'O'},
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
                {"gps-lat-lon",		no_argument,		&longOpt,	O_LAT_LON},
                {"gps-pos-acc",		no_argument,		&longOpt,	O_GPS_POS_ACC},
                {"gps-pos-micros",	no_argument,		&longOpt,	O_GPS_POS_MICROS},
                {"gps-alt",			no_argument,		&longOpt,	O_GPS_ALT},
                {"gps-alt-acc",		no_argument,		&longOpt,	O_GPS_VERT_ACC},
                {"gps-vels",		no_argument,		&longOpt,	O_GPS_VELS},
                {"gps-vel-acc",		no_argument,		&longOpt,	O_GPS_VEL_ACC},
                {"gps-vel-micros",	no_argument,		&longOpt,	O_GPS_VEL_MICROS},
                {"gps-time",		no_argument,		&longOpt,	O_GPS_UTC_TIME},
                {"gps-dops",		no_argument,		&longOpt,	O_GPS_DOPS},
                {"gps-tow",			no_argument,		&longOpt,	O_GPS_ITOW},
                {"poss",			no_argument,		&longOpt,	O_POSS},
                {"vels",			no_argument,		&longOpt,	O_VELS},
                {"quat",			no_argument,		&longOpt,	O_QUAT},
                {"motors",			no_argument,		&longOpt,	O_MOTORS},
                {"throttle",		no_argument,		&longOpt,	O_THROTTLE},
                {"out-pry",			no_argument,		&longOpt,	O_PRY},
                {"radio-quality",	no_argument,		&longOpt,	O_RADIO_QUALITY},
                {"radio-chan-8",	no_argument,		&longOpt,	O_RADIO_CHAN_8},
                {"radio-chan-gt8",	no_argument,		&longOpt,	O_RADIO_CHAN_GT8},
                {"attitude",		no_argument,		&longOpt,	O_ATTITUDE},
                {"acc-bias",		no_argument,		&longOpt,	O_ACC_BIAS},
                {"gmbl-trig",		no_argument,		&longOpt,	O_GMBL_TRIG},
                {NULL,				0,					NULL,		0}
	};

	scaleMin = scaleMax = nan("");
	while ((ch = getopt_long(argc, argv, "hpglcyf:a:v:d:t::r:i:n:x:e:w:A:O:", longopts, NULL)) != -1) {
		switch (ch) {
			case 'h':
				usage();
				exit(0);
				break;
			case 'p':
				dumpPlot++;
				break;
			case 'g':
				dumpGpsTrack++;
				outputRealDate++;
				includeHeaders++;
				valueSep = ',';
				dumpHeaders[dumpNum] = "time";
				dumpOrder[dumpNum++] = GPS_UTC_TIME;
				dumpHeaders[dumpNum] = "lat";
				dumpOrder[dumpNum++] = LAT;
				dumpHeaders[dumpNum] = "lon";
				dumpOrder[dumpNum++] = LON;
				dumpHeaders[dumpNum] = "alt (m)";
				dumpOrder[dumpNum++] = GPS_ALT;
				dumpHeaders[dumpNum] = "speed (m/s)";
				dumpOrder[dumpNum++] = GPS_H_SPEED;
				dumpHeaders[dumpNum] = "climb rate (m/s)";
				dumpOrder[dumpNum++] = VELD;
				dumpHeaders[dumpNum] = "rotation";
				dumpOrder[dumpNum++] = YAW;
				dumpHeaders[dumpNum] = "roll";
				dumpOrder[dumpNum++] = ROLL;
				dumpHeaders[dumpNum] = "pitch";
				dumpOrder[dumpNum++] = PITCH;
				//dumpHeaders[dumpNum] = "trigger";
				//dumpOrder[dumpNum++] = CAM_TRIGGER;
				//dumpOrder[dumpNum++] = TEMP4;
				//dumpHeaders[dumpNum] = "GPS_ITOW";
				break;
			case 'f':
				if (atof(optarg)) {
					outputFreq = atoi(optarg);
					gpsTrackFreq = outputFreq;
					usrSpecOutFreq++;
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
				utcToLocal++;
				break;
			case 't':
				if (optarg)
					camTrigChannel = atoi(optarg);
				dumpTrigger = true;
				dumpHeaders[dumpNum] = "trigger";
				dumpOrder[dumpNum++] = CAM_TRIGGER;
				break;
			case 'r':
				camTrigValue = atoi(optarg);
				break;
			case 'y':
				dumpTriggeredOnly++;
				break;
			case 'i':
				camTrigDelay = atoi(optarg) * 1000;
				break;
			case 'c':
				includeHeaders++;
				break;
			case 'e':
				if (strcmp(optarg, "csv") == 0)
					valueSep = ',';
				else if (strcmp(optarg, "tab") == 0)
					valueSep = '	';
				else if (strcmp(optarg, "gpx") == 0)
					exportGPX++;
				else if (strcmp(optarg, "kml") == 0)
					exportKML++;
				else if (strcmp(optarg, "mav") == 0)
					exportMAV++;
				break;
			case 'w':
				if (!strcmp(optarg, "o") || !strcmp(optarg, "only"))
					gpsTrackAsWpts++;
				else if (!strcmp(optarg, "i") || !strcmp(optarg, "include"))
					gpsTrackInclWpts++;
				break;
			case 'n':
				scaleMin = atof(optarg);
				break;
			case 'x':
				scaleMax = atof(optarg);
				break;
			case 'A':
				if (!strcmp(optarg, "p") || !strcmp(optarg, "press"))
					gpsTrackUsePresAlt++;
				else if (!strcmp(optarg, "u") || !strcmp(optarg, "ukf"))
					gpsTrackUseUkfAlt++;
				break;
			case 'O':
				gpsTrackAltOffset = atof(optarg);
				break;
			case 0:
				switch (longOpt) {
					case O_MICROS:
						dumpHeaders[dumpNum] = "LASTUPDATE";
						dumpOrder[dumpNum++] = MICROS;
						break;
					case O_VOLTAGES:
						dumpHeaders[dumpNum] = "VOLTAGE0";
						dumpOrder[dumpNum++] = VOLTAGE1;
						dumpHeaders[dumpNum] = "VOLTAGE1";
						dumpOrder[dumpNum++] = VOLTAGE2;
						dumpHeaders[dumpNum] = "VOLTAGE2";
						dumpOrder[dumpNum++] = VOLTAGE3;
						dumpHeaders[dumpNum] = "VOLTAGE3";
						dumpOrder[dumpNum++] = VOLTAGE4;
						dumpHeaders[dumpNum] = "VOLTAGE4";
						dumpOrder[dumpNum++] = VOLTAGE5;
						dumpHeaders[dumpNum] = "VOLTAGE5";
						dumpOrder[dumpNum++] = VOLTAGE6;
						dumpHeaders[dumpNum] = "VOLTAGE6";
						dumpOrder[dumpNum++] = VOLTAGE7;
						dumpHeaders[dumpNum] = "VOLTAGE7";
						dumpOrder[dumpNum++] = VOLTAGE8;
						dumpHeaders[dumpNum] = "VOLTAGE8";
						dumpOrder[dumpNum++] = VOLTAGE9;
						dumpHeaders[dumpNum] = "VOLTAGE9";
						dumpOrder[dumpNum++] = VOLTAGE10;
						dumpHeaders[dumpNum] = "VOLTAGE10";
						dumpOrder[dumpNum++] = VOLTAGE11;
						dumpHeaders[dumpNum] = "VOLTAGE11";
						dumpOrder[dumpNum++] = VOLTAGE12;
						dumpHeaders[dumpNum] = "VOLTAGE12";
						dumpOrder[dumpNum++] = VOLTAGE13;
						dumpHeaders[dumpNum] = "VOLTAGE13";
						dumpOrder[dumpNum++] = VOLTAGE14;
						dumpHeaders[dumpNum] = "VOLTAGE14";
						dumpOrder[dumpNum++] = VOLTAGE15;
						break;
					case O_RATES:
						dumpHeaders[dumpNum] = "IMU_RATEX";
						dumpOrder[dumpNum++] = RATEX;
						dumpHeaders[dumpNum] = "IMU_RATEY";
						dumpOrder[dumpNum++] = RATEY;
						dumpHeaders[dumpNum] = "IMU_RATEZ";
						dumpOrder[dumpNum++] = RATEZ;
						break;
					case O_ACCS:
						dumpHeaders[dumpNum] = "IMU_ACCX";
						dumpOrder[dumpNum++] = ACCX;
						dumpHeaders[dumpNum] = "IMU_ACCY";
						dumpOrder[dumpNum++] = ACCY;
						dumpHeaders[dumpNum] = "IMU_ACCZ";
						dumpOrder[dumpNum++] = ACCZ;
						break;
					case O_MAGS:
						dumpHeaders[dumpNum] = "IMU_MAGX";
						dumpOrder[dumpNum++] = MAGX;
						dumpHeaders[dumpNum] = "IMU_MAGY";
						dumpOrder[dumpNum++] = MAGY;
						dumpHeaders[dumpNum] = "IMU_MAGZ";
						dumpOrder[dumpNum++] = MAGZ;
						break;
					case O_MAG_SIGN:
						dumpHeaders[dumpNum] = "ADC_MAG_SIGN";
						dumpOrder[dumpNum++] = ADC_MAG_SIGN;
						break;
					case O_PRESSURES:
						dumpHeaders[dumpNum] = "ADC_PRESSURE1";
						dumpOrder[dumpNum++] = PRESSURE1;
						dumpHeaders[dumpNum] = "ADC_PRESSURE2";
						dumpOrder[dumpNum++] = PRESSURE2;
						break;
					case O_PRES_ALT:
						dumpHeaders[dumpNum] = "UKF_ALT";
						dumpOrder[dumpNum++] = UKF_ALT;
						dumpHeaders[dumpNum] = "UKF_PRES_ALT";
						dumpOrder[dumpNum++] = UKF_PRES_ALT;
						break;
					case O_TEMPS:
						dumpHeaders[dumpNum] = "ADC_TEMP0";
						dumpOrder[dumpNum++] = TEMP1;
						break;
					case O_VIN:
						dumpHeaders[dumpNum] = "ADC_VIN";
						dumpOrder[dumpNum++] = VIN;
						break;
					case O_GPS_POS_MICROS:
						dumpHeaders[dumpNum] = "GPS_POS_UPDATE";
						dumpOrder[dumpNum++] = GPS_POS_MICROS;
						break;
					case O_LAT_LON:
						dumpHeaders[dumpNum] = "GPS_LAT";
						dumpOrder[dumpNum++] = LAT;
						dumpHeaders[dumpNum] = "GPS_LON";
						dumpOrder[dumpNum++] = LON;
//						dumpHeaders[dumpNum] = "BRG_TO_HOME";
//						dumpOrder[dumpNum++] = BRG_TO_HOME;
						break;
					case O_GPS_POS_ACC:
						dumpHeaders[dumpNum] = "GPS_HACC";
						dumpOrder[dumpNum++] = GPS_POS_ACC;
						break;
					case O_GPS_ALT:
						dumpHeaders[dumpNum] = "GPS_HEIGHT";
						dumpOrder[dumpNum++] = GPS_ALT;
						break;
					case O_GPS_VERT_ACC:
						dumpHeaders[dumpNum] = "GPS_VACC";
						dumpOrder[dumpNum++] = GPS_VACC;
						break;
					case O_GPS_VEL_MICROS:
						dumpHeaders[dumpNum] = "GPS_VEL_UPDATE";
						dumpOrder[dumpNum++] = GPS_VEL_MICROS;
						break;
					case O_GPS_VELS:
						dumpHeaders[dumpNum] = "GPS_VELN";
						dumpOrder[dumpNum++] = GPS_VELN;
						dumpHeaders[dumpNum] = "GPS_VELE";
						dumpOrder[dumpNum++] = GPS_VELE;
						dumpHeaders[dumpNum] = "GPS_VELD";
						dumpOrder[dumpNum++] = GPS_VELD;
						break;
					case O_GPS_VEL_ACC:
						dumpHeaders[dumpNum] = "GPS_SACC";
						dumpOrder[dumpNum++] = GPS_VEL_ACC;
						break;
					case O_GPS_UTC_TIME:
						outputRealDate++;
						dumpHeaders[dumpNum] = "time";
						dumpOrder[dumpNum++] = GPS_UTC_TIME;
						break;
					case O_GPS_DOPS:
						dumpHeaders[dumpNum] = "GPS_PDOP";
						dumpOrder[dumpNum++] = GPS_PDOP;
						dumpHeaders[dumpNum] = "GPS_HDOP";
						dumpOrder[dumpNum++] = GPS_HDOP;
						dumpHeaders[dumpNum] = "GPS_VDOP";
						dumpOrder[dumpNum++] = GPS_VDOP;
						dumpHeaders[dumpNum] = "GPS_TDOP";
						dumpOrder[dumpNum++] = GPS_TDOP;
						dumpHeaders[dumpNum] = "GPS_NDOP";
						dumpOrder[dumpNum++] = GPS_NDOP;
						dumpHeaders[dumpNum] = "GPS_EDOP";
						dumpOrder[dumpNum++] = GPS_EDOP;
						break;
					case O_GPS_ITOW:
						dumpHeaders[dumpNum] = "GPS_ITOW";
						dumpOrder[dumpNum++] = GPS_ITOW;
						break;
					case O_POSS:
						dumpHeaders[dumpNum] = "UKF_POSN";
						dumpOrder[dumpNum++] = POSN;
						dumpHeaders[dumpNum] = "UKF_POSE";
						dumpOrder[dumpNum++] = POSE;
						dumpHeaders[dumpNum] = "UKF_POSD";
						dumpOrder[dumpNum++] = POSD;
						break;
					case O_VELS:
						dumpHeaders[dumpNum] = "UKF_VELN";
						dumpOrder[dumpNum++] = VELN;
						dumpHeaders[dumpNum] = "UKF_VELE";
						dumpOrder[dumpNum++] = VELE;
						dumpHeaders[dumpNum] = "UKF_VELD";
						dumpOrder[dumpNum++] = VELD;
						break;
					case O_QUAT:
						dumpHeaders[dumpNum] = "UKF_Q1";
						dumpOrder[dumpNum++] = QUAT0;
						dumpHeaders[dumpNum] = "UKF_Q2";
						dumpOrder[dumpNum++] = QUAT1;
						dumpHeaders[dumpNum] = "UKF_Q3";
						dumpOrder[dumpNum++] = QUAT2;
						dumpHeaders[dumpNum] = "UKF_Q4";
						dumpOrder[dumpNum++] = QUAT3;
						break;
					case O_MOTORS:
						dumpHeaders[dumpNum] = "MOT_MOTOR0";
						dumpOrder[dumpNum++] = MOTOR1;
						dumpHeaders[dumpNum] = "MOT_MOTOR1";
						dumpOrder[dumpNum++] = MOTOR2;
						dumpHeaders[dumpNum] = "MOT_MOTOR2";
						dumpOrder[dumpNum++] = MOTOR3;
						dumpHeaders[dumpNum] = "MOT_MOTOR3";
						dumpOrder[dumpNum++] = MOTOR4;
						dumpHeaders[dumpNum] = "MOT_MOTOR4";
						dumpOrder[dumpNum++] = MOTOR5;
						dumpHeaders[dumpNum] = "MOT_MOTOR5";
						dumpOrder[dumpNum++] = MOTOR6;
						dumpHeaders[dumpNum] = "MOT_MOTOR6";
						dumpOrder[dumpNum++] = MOTOR7;
						dumpHeaders[dumpNum] = "MOT_MOTOR7";
						dumpOrder[dumpNum++] = MOTOR8;
						dumpHeaders[dumpNum] = "MOT_MOTOR8";
						dumpOrder[dumpNum++] = MOTOR9;
						dumpHeaders[dumpNum] = "MOT_MOTOR9";
						dumpOrder[dumpNum++] = MOTOR10;
						dumpHeaders[dumpNum] = "MOT_MOTOR10";
						dumpOrder[dumpNum++] = MOTOR11;
						dumpHeaders[dumpNum] = "MOT_MOTOR11";
						dumpOrder[dumpNum++] = MOTOR12;
						dumpHeaders[dumpNum] = "MOT_MOTOR12";
						dumpOrder[dumpNum++] = MOTOR13;
						dumpHeaders[dumpNum] = "MOT_MOTOR13";
						dumpOrder[dumpNum++] = MOTOR14;
						break;
					case O_THROTTLE:
						dumpHeaders[dumpNum] = "MOT_THROTTLE";
						dumpOrder[dumpNum++] = THROTTLE;
						break;
					case O_PRY:
						dumpHeaders[dumpNum] = "MOT_PITCH";
						dumpOrder[dumpNum++] = MOT_PITCH;
						dumpHeaders[dumpNum] = "MOT_ROLL";
						dumpOrder[dumpNum++] = MOT_ROLL;
						dumpHeaders[dumpNum] = "MOT_YAW";
						dumpOrder[dumpNum++] = MOT_YAW;
						break;
					case O_RADIO_QUALITY:
						dumpHeaders[dumpNum] = "RADIO_QUALITY";
						dumpOrder[dumpNum++] = RADIO_QUALITY;
						dumpHeaders[dumpNum] = "RADIO_ERRORS";
						dumpOrder[dumpNum++] = RADIO_ERRORS;
						break;
					case O_RADIO_CHAN_8:
						dumpHeaders[dumpNum] = "RADIO_CHANNEL0";
						dumpOrder[dumpNum++] = RADIO_CHANNEL1;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL1";
						dumpOrder[dumpNum++] = RADIO_CHANNEL2;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL2";
						dumpOrder[dumpNum++] = RADIO_CHANNEL3;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL3";
						dumpOrder[dumpNum++] = RADIO_CHANNEL4;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL4";
						dumpOrder[dumpNum++] = RADIO_CHANNEL5;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL5";
						dumpOrder[dumpNum++] = RADIO_CHANNEL6;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL6";
						dumpOrder[dumpNum++] = RADIO_CHANNEL7;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL7";
						dumpOrder[dumpNum++] = RADIO_CHANNEL8;
						break;
					case O_RADIO_CHAN_GT8:
						dumpHeaders[dumpNum] = "RADIO_CHANNEL8";
						dumpOrder[dumpNum++] = RADIO_CHANNEL9;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL9";
						dumpOrder[dumpNum++] = RADIO_CHANNEL10;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL10";
						dumpOrder[dumpNum++] = RADIO_CHANNEL11;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL11";
						dumpOrder[dumpNum++] = RADIO_CHANNEL12;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL12";
						dumpOrder[dumpNum++] = RADIO_CHANNEL13;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL13";
						dumpOrder[dumpNum++] = RADIO_CHANNEL14;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL14";
						dumpOrder[dumpNum++] = RADIO_CHANNEL15;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL15";
						dumpOrder[dumpNum++] = RADIO_CHANNEL16;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL16";
						dumpOrder[dumpNum++] = RADIO_CHANNEL17;
						dumpHeaders[dumpNum] = "RADIO_CHANNEL17";
						dumpOrder[dumpNum++] = RADIO_CHANNEL18;
						break;
					case O_ATTITUDE:
						dumpHeaders[dumpNum] = "heading";
						dumpOrder[dumpNum++] = YAW;
						dumpHeaders[dumpNum] = "roll";
						dumpOrder[dumpNum++] = ROLL;
						dumpHeaders[dumpNum] = "pitch";
						dumpOrder[dumpNum++] = PITCH;
						break;
					case O_ACC_BIAS:
						dumpHeaders[dumpNum] = "ACC_BIAS_X";
						dumpOrder[dumpNum++] = ACC_BIAS_X;
						dumpHeaders[dumpNum] = "ACC_BIAS_Y";
						dumpOrder[dumpNum++] = ACC_BIAS_Y;
						dumpHeaders[dumpNum] = "ACC_BIAS_Z";
						dumpOrder[dumpNum++] = ACC_BIAS_Z;
						break;
					case O_GMBL_TRIG:
						dumpTrigger = true;
						dumpHeaders[dumpNum] = "GMBL_TRIGGER";
						dumpOrder[dumpNum++] = GMBL_TRIGGER;
						break;
				} // longopt switch
				break;
			default:
				// fprintf(stderr, "logDump: logDumpOpts: error\n");
				break;
		} // getopt value switch
	} // while getopt has value loop
}

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
struct filespec extractFileName(char *fp) {
	// filespec = {path, name, ext}
	struct filespec r = {(char *)blnk ,(char *)blnk ,(char *)blnk };

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
	double val;
	float rpy[3];

	switch (field) {
	case MICROS:
		val = l->lastUpdate;
		break;
	case VOLTAGE1:
	case VOLTAGE2:
	case VOLTAGE3:
	case VOLTAGE4:
	case VOLTAGE5:
	case VOLTAGE6:
	case VOLTAGE7:
	case VOLTAGE8:
	case VOLTAGE9:
	case VOLTAGE10:
	case VOLTAGE11:
	case VOLTAGE12:
	case VOLTAGE13:
	case VOLTAGE14:
	case VOLTAGE15:
		val = l->voltages[field-VOLTAGE1];
		break;
	case RATEX:
	case RATEY:
	case RATEZ:
		val = l->rate[field-RATEX];
		break;
	case ACCX:
	case ACCY:
	case ACCZ:
		val = l->acc[field-ACCX];
		break;
	case MAGX:
	case MAGY:
	case MAGZ:
		val = l->mag[field-MAGX];
		break;
	case PRESSURE1:
	case PRESSURE2:
		val = l->pressure[field-PRESSURE1];
		break;
	case TEMP1:
		val = l->temp[0];
		break;
	case VIN:
		val = l->vIn;
		break;
	case ADC_MAG_SIGN:
		val = l->magSign;
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
	case GPS_VACC:
		val = l->gpsVAcc;
		break;
	case GPS_VEL_MICROS:
		val = l->gpsVelUpdate;
		break;
	case GPS_VELN:
	case GPS_VELE:
	case GPS_VELD:
		val = l->gpsVel[field-GPS_VELN];
		break;
	case GPS_VEL_ACC:
		val = l->gpsVelAcc;
		break;
	case GPS_PDOP:
	case GPS_HDOP:
	case GPS_VDOP:
	case GPS_TDOP:
	case GPS_NDOP:
	case GPS_EDOP:
		val = l->gpsDops[field-GPS_PDOP];
		break;
	case GPS_ITOW:
	case GPS_UTC_TIME:
		val = l->gpsItow;
		break;
	case POSN:
	case POSE:
	case POSD:
		val = l->pos[field-POSN];
		break;
	case UKF_ALT:
		val = l->ukfAlt;
		break;
	case UKF_PRES_ALT:
		val = l->pressAlt;
		break;
	case VELN:
	case VELE:
	case VELD:
		val = l->vel[field-VELN];
		break;
	case QUAT0:
	case QUAT1:
	case QUAT2:
	case QUAT3:
		val = l->quat[field-QUAT0];
		break;
	case MOTOR1:
	case MOTOR2:
	case MOTOR3:
	case MOTOR4:
	case MOTOR5:
	case MOTOR6:
	case MOTOR7:
	case MOTOR8:
	case MOTOR9:
	case MOTOR10:
	case MOTOR11:
	case MOTOR12:
	case MOTOR13:
	case MOTOR14:
		val = l->motors[field-MOTOR1];
		break;
	case THROTTLE:
		val = l->throttle;
		break;
	case MOT_PITCH:
	case MOT_ROLL:
	case MOT_YAW:
		val = l->motPRY[field-MOT_PITCH];
		break;
	case RADIO_CHANNEL1:
	case RADIO_CHANNEL2:
	case RADIO_CHANNEL3:
	case RADIO_CHANNEL4:
	case RADIO_CHANNEL5:
	case RADIO_CHANNEL6:
	case RADIO_CHANNEL7:
	case RADIO_CHANNEL8:
	case RADIO_CHANNEL9:
	case RADIO_CHANNEL10:
	case RADIO_CHANNEL11:
	case RADIO_CHANNEL12:
	case RADIO_CHANNEL13:
	case RADIO_CHANNEL14:
	case RADIO_CHANNEL15:
	case RADIO_CHANNEL16:
	case RADIO_CHANNEL17:
	case RADIO_CHANNEL18:
		val = l->radioChannels[field-RADIO_CHANNEL1];
		break;
	case RADIO_QUALITY:
		val = l->radioQuality;
		break;
	case RADIO_ERRORS:
		val = l->radioErrors;
		break;
	case ACC_BIAS_X:
	case ACC_BIAS_Y:
	case ACC_BIAS_Z:
		val = l->accBias[field-ACC_BIAS_X];
		break;

	// calculated values:
	case GPS_H_SPEED:
		val = (fabs(l->gpsVel[0]) + fabs(l->gpsVel[1])); // * 3600 / 1000; // km/h
		break;
	case CAM_TRIGGER:
	case GMBL_TRIGGER:
		// is trigger active?
		if ( l->gimbalTrig || (
				camTrigChannel > 0 && camTrigChannel < 19 && (
					(camTrigValue < 0 && l->radioChannels[camTrigChannel-1] < camTrigValue) ||
					(camTrigValue > 0 && l->radioChannels[camTrigChannel-1] > camTrigValue) ||
					(camTrigValue == 0 && l->radioChannels[camTrigChannel-1] > -TRIG_ZERO_BUFFER && l->radioChannels[camTrigChannel-1] < TRIG_ZERO_BUFFER) )
				)) {
			// first time this trigger is activated
			if (!camTrigActivatedTime)
				camTrigActivatedTime = l->lastUpdate;

			// use count in LOG_GMBL_TRIGGER if available
			val = (l->gimbalTrig) ? l->gimbalTrig : ++camTrigCnt;

			// if a shutter delay is configured, we only want one positive return value per trigger activation, after the specified delay time
			if (camTrigDelay && (val == camTrigLastActive || l->lastUpdate < camTrigActivatedTime + camTrigDelay))
				val = 0;
		}
		// trigger is not active
		else {
			val = 0;
			camTrigActivatedTime = 0;
		}
		break;
	case ROLL:
		attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
		val = rpy[0] * -1.0 * RAD_TO_DEG;
		break;
	case PITCH:
		attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
		val = rpy[1] * -1.0 * RAD_TO_DEG;
		break;
	case YAW:
		attitudeExtractEulerQuat(l->quat, &rpy[2], &rpy[1], &rpy[0]);
		val = rpy[2] * RAD_TO_DEG;
		if (val < 0) val = 360 + val;
		break;
	case BRG_TO_HOME:
		val = navCalcBearing(homeLat, homeLon, logDumpGetValue(l, LAT), logDumpGetValue(l, LON));
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

void logDumpHeaders(void) {
	int i;

	if (dumpNum) {
		for (i = 0; i < dumpNum; i++) {
			printf("%s%c", dumpHeaders[i], (i < dumpNum-1 ? valueSep : 0));
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
			homeLat = logDumpGetValue(l, LAT);
			homeLon = logDumpGetValue(l, LON);
			homeSet = true;
		}
		else if (l->radioChannels[homeSetChannel-1] < 250)
			homeSet = false;
	}

	// flat text format
	if (!exportGPX && !exportKML && !exportMAV) {

		for (i = 0; i < dumpNum; i++) {
			logVal = logDumpGetValue(l, dumpOrder[i]);

			if (dumpOrder[i] == GPS_UTC_TIME)
				formatIsoTime(outStr, logVal);
			else
				sprintf(outStr, "%.15G", logVal);

			if ((dumpOrder[i] == CAM_TRIGGER || dumpOrder[i] == GMBL_TRIGGER) && (bool)logVal)
				camTrigLastActive = logVal;

			printf(outStr);

			if (i < dumpNum-1)
				printf("%c", valueSep);

		}
		printf("\n"); // end of export row

	}
	// mavlink log format (experimental)
	else if (exportMAV) {
		mavlinkDo(l);
	}
	// KML/GPX format
	else {

		strcpy(exp.name, "");
		strcpy(exp.wptstyle, "waypoint");
		trackName = (char *) calloc(strlen(logfilespec.name)+10, sizeof(char));
		strcpy(trackName, logfilespec.name);
		mkwpt = 0;

		exp.lat = logDumpGetValue(l, LAT);
		exp.lon = logDumpGetValue(l, LON);
		if (gpsTrackUsePresAlt)
			exp.alt = logDumpGetValue(l, UKF_PRES_ALT);
		else if (gpsTrackUseUkfAlt)
			exp.alt = logDumpGetValue(l, POSD);
		else
			exp.alt = logDumpGetValue(l, GPS_ALT);
		exp.alt += gpsTrackAltOffset;
		exp.speed = logDumpGetValue(l, GPS_H_SPEED);
		exp.climb = logDumpGetValue(l, VELD);
		exp.hdg = logDumpGetValue(l, YAW);
		exp.roll = logDumpGetValue(l, ROLL);
		exp.pitch = logDumpGetValue(l, PITCH);
		exp.lat = logDumpGetValue(l, LAT);

		formatIsoTime(outStr, logDumpGetValue(l, GPS_UTC_TIME));
		strcpy(exp.time, outStr);

		if (dumpTrigger) {
			trigCount = (unsigned)logDumpGetValue(l, CAM_TRIGGER);
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
			gpsFixTime = logDumpGetValue(l, GPS_UTC_TIME);
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

			// write the export record to stdout
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

#ifdef HAS_PLPLOT

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

#endif

int main(int argc, char **argv) {
	FILE *lf;
	int i;
	int count; // total log line counter
	int outputFreqDivisor;
	struct stat sbuf; // file stat() buffer

	outFP = stdout;

#ifdef HAS_PLPLOT
	plparseopts(&argc, (const char**)argv, PL_PARSE_SKIP);
#endif
	logDumpOpts(argc, argv);
	argc -= optind;
	argv += optind;

	if (argc < 1) {
		fprintf(stderr, "logDump: need log file argument\n");
		usage();
		exit(1);
	}

	if (dumpNum < 1) {
		fprintf(stderr, "logDump: need at least one value to export\n");
		usage();
		exit(1);
	}

	// init waypoint storage
	gpxWaypoints = (char *) calloc(1, sizeof(char));

	// determine output frequency
	if (dumpGpsTrack && !usrSpecOutFreq) // use lower default setting for gps track log
		outputFreq = gpsTrackFreq;
	outputFreqDivisor = (int)(200 / outputFreq); // divide 200Hz logging rate by this to set output frequency (eg 200/40=5Hz)

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
		for (i = 0; i < NUM_FIELDS; i++) {
			dumpMin[i] = +9999999.99;
			dumpMax[i] = -9999999.99;
		}

		if (exportMAV) {
			mavlinkInit();
			char outfileName[] = "test.mavlink";
			outFP = fopen(outfileName, "wb");
			if (outFP == NULL) {
				fprintf(stderr, "logDump: cannot open output file '%s'\n", outfileName);
				exit(0);
			}
		}

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

#ifdef HAS_PLPLOT
		if (dumpPlot) {
			count = 0;
			// force header read
			loggerReadEntry(lf, &logEntry);
			rewind(lf);

			while (loggerReadEntry(lf, &logEntry) != EOF) {
				if ( !(count % outputFreqDivisor) && ( !dumpTriggeredOnly || logDumpGetValue(&logEntry, CAM_TRIGGER) ) ) {
					logDumpStats(&logEntry);
					cntExpSamples++; // count actual exported samples
				}
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
#endif
			count = 0;
			while (loggerReadEntry(lf, &logEntry) != EOF) {
				if ( !(count % outputFreqDivisor) &&
						( !dumpTriggeredOnly || logDumpGetValue(&logEntry, CAM_TRIGGER) ) &&
						( !dumpGpsTrack || ( logDumpGetValue(&logEntry, GPS_POS_ACC) <= gpsTrackMinHAcc && logDumpGetValue(&logEntry, GPS_VACC) <= gpsTrackMinVAcc) ) ) {
					logDumpText(&logEntry);
					cntExpSamples++; // count actual exported samples
					// send progress indication
					if (!(cntExpSamples % 200)) {
						fprintf(stderr, ".");
						fflush(stderr);
					}
				}
				count++;
			}
#ifdef HAS_PLPLOT
		}
#endif

		// finish up writing GPX export
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
		fprintf(stderr, "logDump: %d mins %d seconds @ %dHz exported %d records\n", count/200/60, count/200 % 60, outputFreq, cntExpSamples);
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
