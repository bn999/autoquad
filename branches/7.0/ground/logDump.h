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

#define AQ_LOGGING_FREQUENCY	200		// assume this logging rate for AQ logs
#define OUTPUT_FREQ_DIVISOR		(int)(AQ_LOGGING_FREQUENCY / outputFreq)	// divide 200Hz logging rate by this to set output frequency (eg 200/40=5Hz)

// default options
static int outputFreq = AQ_LOGGING_FREQUENCY; // output frequency in Hz
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
static unsigned homeSetChannel = 7;			// radio channel used to set home position
static unsigned posHoldChannel = 6;			// radio channel used to set position hold mode
static uint32_t dumpRangeMin = 1;			// start export at this record number
static uint32_t dumpRangeMax = 0;			// end export at this record number (zero for all)
static char valueSep = ' ';					// export value delimiter (space, comma, tab, etc)
static bool dumpPlot = 0;					// plot the results instead of exporting them
static bool utcToLocal = 0;					// convert to local time
static bool outputRealDate = 0;				// calculate and use actual date instead of time-of-week
static bool includeHeaders = 0;				// include column headers in flat text exports
static bool dumpTriggeredOnly = 0;			// only export records with trigger indicator (see help)
static bool exportGPX = 0;					// export GPX format
static bool exportKML = 0;					// export KML format

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

// this "extends" the log_fields enum from logger.h
enum calculated_fields {
	FLD_GPS_H_SPEED = LOG_NUM_IDS + 1,
	FLD_GPS_UTC_TIME,	// formatted time derived from GPS time and log file time
	FLD_CAM_TRIGGER,	// triggering action
	FLD_ROLL,			// roll, pitch, heading, in degrees
	FLD_PITCH,
	FLD_YAW,
	FLD_BRG_TO_HOME,
	FLD_MAG_MAGNITUDE,
	FLD_ACC_MAGNITUDE,
	FLD_ACC_PITCH,		// pure ACC-derived pitch
	FLD_ACC_ROLL,		// pure ACC-derived roll
	NUM_FIELDS
};

static const char *logDumpFieldLabels[] = {
	"GPS GND SPEED (m/s)",
	"TIME",
	"TRIG",
	"ROLL (deg)",
	"PITCH (deg)",
	"HEADING (deg)",
	"BRG TO HOME (deg)",
	"MAG Magnitude",
	"ACC Magnitude",
	"ACC Pitch (deg)",
	"ACC Roll (deg)",
	0  // terminate
};

typedef struct {
	char *path, *name, *ext;
} filespec_t;

typedef struct {
	double lat, lon, alt, speed, climb, hdg, roll, pitch;
	char time[31], name[30], wptstyle[20];
} expFields_t;

extern time_t towStartTime; // will hold date to add with GPS ToW to arrive at actual date/time
extern FILE *outFP;

extern double logDumpGetValue(loggerRecord_t *l, int field);

#ifdef __cplusplus
}
#endif

#endif /* LOGDUMP_H_ */
