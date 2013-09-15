/*
 * logDump.h
 *
 *  Created on: Dec 23, 2012
 *      Author: Max
 */

#ifndef LOGDUMP_H_
#define LOGDUMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "logger.h"
#include <stdint.h>
#include <time.h>

#define P0                  	101325.0	// standard static pressure at sea level
#define ADC_REF_VOLTAGE		3.3f
#define ADC_TEMP_OFFSET		1.25f		// volts (IDG500 PTATS)
#define ADC_TEMP_SLOPE		(1.0f / 0.004f)	// deg C / volts

#define ADC_TEMP_A		+167.5358f
#define ADC_TEMP_B		+6.6871f
#define ADC_TEMP_C		+0.0347f
#define ADC_TEMP_R2		100000.0f	// ohms
#define ADC_KELVIN_TO_CELCIUS	-273.0f

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

#define GPS_TRACK_MAX_TM_GAP	3000	// milliseconds w/out GPS position after which to start new track segment
#define TRIG_ZERO_BUFFER		100		// pulse width ms +/- buffer for zero (center) position

// defaults
static int outputFreq = 200;				// output frequency in Hz (log is written at 200Hz)
static int gpsTrackFreq = 5;				// default export output frequency when used with --gps-track option
static float gpsTrackMinHAcc = 2;			// gps track dump: minimum GPS_HACC (est. horizontal accuracy) in meters
static float gpsTrackMinVAcc = 2;			// gps track dump: minimum GPS_VACC (est. vertical accuracy) in meters
static bool gpsTrackAsWpts = 0;				// export gps track as waypoints (GPX/KML only)
static bool gpsTrackInclWpts = 0;			// export gps track AND waypoints (GPX/KML only)
static bool gpsTrackUsePresAlt = 0;			// use pressure sensor alt. instead of GPS alt, after adjusting by given offset
static bool gpsTrackUseUkfAlt = 0;			// use UKF derived altitude instead of raw GPS altitude
static float gpsTrackAltOffset = 0;			// offset in meters between reported pressure alt. and MSL
static int camTrigChannel = 0;				// trigger radio channel (zero for none)
static int camTrigValue = 250;				// trigger channel value above/below which means camera was triggered
static unsigned camTrigDelay = 0;			// delay micros between trigger activation and camera shutter opening
static int homeSetChannel = 7;
static int posHoldChannel = 6;
static char valueSep = ' ';					// export value delimiter (space, comma, tab, etc)

// GPX/KML export settings
static const char trigWptName[30] = "trig"; // what to name waypoints made from triggered track points
static const char trackColor[] = "ff007fef"; // KML track color
static const int trackWidth = 3;			// KML track width
static const char trackAltMode[] = "absolute"; // KML track altitude mode (clampToGround, relativeToGround, or absolute)
static const int trackExtrude = 0;			// KML extrude track (0 or 1)
static const int trackTesselate = 0;		// KML tesselate (break up in to smaller pieces) track (0 or 1); should = 1 if trackExtrude=1
static const char trackModelURL[] = "http://max.wdg.us/AQ/AQ.dae"; // KML COLLADA model file for tracklog
static const char waypointColor[] = "99fe6500"; // KML waypoint label color
static const char waypointTrigColor[] = "990000ca"; // KML triggered waypoint label color
static const char waypointIconURL[] = "http://maps.google.com/mapfiles/kml/shapes/arrow.png";
static const char waypointAltMode[] = "absolute"; // KML waypoint altitude mode (clampToGround, relativeToGround, or absolute)

enum fields {
	// logged:
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
	ADC_MAG_SIGN,
	GPS_POS_MICROS,
	LAT,
	LON,
	GPS_ALT,
	GPS_POS_ACC,
	GPS_VACC,
	GPS_VEL_MICROS,
	GPS_VELN,
	GPS_VELE,
	GPS_VELD,
	GPS_VEL_ACC,
    GPS_PDOP,
    GPS_HDOP,
    GPS_VDOP,
    GPS_TDOP,
    GPS_NDOP,
    GPS_EDOP,
	GPS_ITOW,
	POSN,
	POSE,
	POSD,
	UKF_PRES_ALT,
	UKF_ALT,
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
    MOT_PITCH,
    MOT_ROLL,
    MOT_YAW,
	RADIO_CHANNEL1,
	RADIO_CHANNEL2,
	RADIO_CHANNEL3,
	RADIO_CHANNEL4,
	RADIO_CHANNEL5,
	RADIO_CHANNEL6,
	RADIO_CHANNEL7,
	RADIO_CHANNEL8,
	RADIO_CHANNEL9,
	RADIO_CHANNEL10,
	RADIO_CHANNEL11,
	RADIO_CHANNEL12,
	RADIO_CHANNEL13,
	RADIO_CHANNEL14,
	RADIO_CHANNEL15,
	RADIO_CHANNEL16,
	RADIO_CHANNEL17,
	RADIO_CHANNEL18,
	RADIO_QUALITY,
	RADIO_ERRORS,
	GMBL_TRIGGER,
	ACC_BIAS_X,
	ACC_BIAS_Y,
	ACC_BIAS_Z,
	// calculated:
	GPS_H_SPEED,
	GPS_UTC_TIME,
	CAM_TRIGGER,
	ROLL,
	PITCH,
	YAW,
	BRG_TO_HOME,
	NUM_FIELDS
};

struct filespec {
	char *path, *name, *ext;
};

typedef struct {
	double lat, lon, alt, speed, climb, hdg, roll, pitch;
	char time[31], name[30], wptstyle[20];
} expFields_t;

static bool dumpPlot;
static bool dumpGpsTrack;
static bool utcToLocal;
static bool outputRealDate;
static bool includeHeaders;
static bool dumpTrigger;
static bool dumpTriggeredOnly;
static bool exportGPX;
static bool exportKML;
static bool exportMAV;
static int dumpOrder[NUM_FIELDS];
static int dumpNum;
static int cntExpSamples;
static int usrSpecOutFreq;
static int gpxWptCnt;
static int gpxTrkCnt;
static unsigned camTrigActivatedTime;
static unsigned camTrigLastActive;
static unsigned camTrigCnt;
static bool camTrigRecordExported;
static float dumpMin[NUM_FIELDS];
static float dumpMax[NUM_FIELDS];
static float scaleMin, scaleMax;
static double lastGpsFixTime;
static double homeLat, homeLon;
static const char *dumpHeaders[NUM_FIELDS];
static char *trackDateStr;
static char *gpxWaypoints;
static const char *blnk = "";
static struct filespec logfilespec;
static loggerRecord_t logEntry;

extern time_t towStartTime; // will hold date to add with GPS ToW to arrive at actual date/time
extern FILE *outFP;

extern double logDumpGetValue(loggerRecord_t *l, int field);

#ifdef __cplusplus
}
#endif

#endif /* LOGDUMP_H_ */
