
#include "plotter.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <string.h>
#include <math.h>
#include <algorithm>

#define LOG_NUM_DCA		8

enum fields {
	// logged
	QUAT_DES_0 = 0,
	QUAT_DES_1,
	QUAT_DES_2,
	QUAT_DES_3,
	WCD_0,
	WCD_1,
	WCD_2,
	QUAT_ACT_0,
	QUAT_ACT_1,
	QUAT_ACT_2,
	QUAT_ACT_3,
	RATE_ACT_0,
	RATE_ACT_1,
	RATE_ACT_2,
	RATE_DES_0,
	RATE_DES_1,
	RATE_DES_2,
	INERTIA_REQ_0,
	INERTIA_REQ_1,
	INERTIA_REQ_2,
	HOVER_THRUST,
	DCA_0,
	DCA_1,
	DCA_2,
	DCA_3,
	DCA_4,
	DCA_5,
	DCA_6,
	DCA_7,
	NUM_LOG_FIELDS,
	// calculated
	ROLL_DES,
	PITCH_DES,
	YAW_DES,
	ROLL_ACT,
	PITCH_ACT,
	YAW_ACT,

	NUM_FIELDS
};

const char *fieldLabels[] = {
	"Quat0 Desired",
	"Quat1 Desired",
	"Quat2 Desired",
	"Quat3 Desired",
	"wcd[0]",
	"wcd[1]",
	"wcd[2]",
	"Quat0 Actual",
	"Quat1 Actual",
	"Quat2 Actual",
	"Quat3 Actual",
	"Rate0 Actual",
	"Rate1 Actual",
	"Rate2 Actual",
	"Rate0 Desired",
	"Rate1 Desired",
	"Rate2 Desired",
	"Inertia0 Req",
	"Inertia1 Req",
	"Inertia2 Req",
	"Hover Thrust",
	"DCA[0]",
	"DCA[1]",
	"DCA[2]",
	"DCA[3]",
	"DCA[4]",
	"DCA[5]",
	"DCA[6]",
	"DCA[7]",
	"",
	// calculated
	"Roll Desired",
	"Pitch Desired",
	"Yaw Desired",
	"Roll Actual",
	"Pitch Actual",
	"Yaw Actual"
};

// option defaults
static char sep = ' ';
static bool dumpPlot = false;
static bool includeHeaders = false;
static uint32_t dumpRangeMin = 1;			// start export at this record number
static uint32_t dumpRangeMax = 0;			// end export at this record number (zero for all)
// runtime globals
float logRowData[NUM_FIELDS];
int dumpNum;
int dumpOrder[NUM_FIELDS];
double *dumpYMin, *dumpYMax;
double *dumpXMin, *dumpXMax;

void qLogDumpUsage(void) {
	fprintf(stderr,
"\n\
Usage: quatosLogDump [options] [values] logfile [ > outfile.ext ]\n\n\
Options Summary:\n\n\
   [-e (txt|csv|tab)] [-c] [-p] [-m number] [-M number]\n\
   [--all] [--rates] [--quat] [--att] [--inertia] [--thrust] [--wcd] [--dca]\n\
\n\
Option Details:\n\
\n\
 --exp-format (-e)  Export format. One of: txt (default), csv, or tab\n\
 --col-headers (-c) Include column headings row in the export.\n\
 --plot (-p)        Plot the data instead of exporting it (ignores -e & -c options).\n\
                      (See plotting options, below. Use -h to get details.)\n\
 --range-min (-m)   Start export at this record number (default is 1).\n\
 --range-max (-M)   End export at this record number (zero means all records until end).\n\
\n\
Values to export (at least one is required):\n\
\n\
 --all              Export all available values. MUST be last option before file name.\n\
 --rates            Desired and actual Rates.\n\
 --quat             Desired and actual Quaternions.\n\
 --att              Desired and actual Attitude (roll, pitch, & yaw, in rad).\n\
 --inertia          Requested Inertia array.\n\
 --thrust           Hover Thrust.\n\
 --wcd              WCD array.\n\
 --dca              DCA array.\n"
	);

	plotterUsage();

	fprintf(stderr,
"\n\
Examples:\n\
   quatosLogDump --all QUAT.LOG >output.txt\n\
   quatosLogDump -c -e csv --quat --rates QUAT.LOG >output.csv\n\
   quatosLogDump --plot --quat -dev svg -o quat.svg -geo 1200x1024 QUAT.LOG\n"
	);
}

void qLogDumpOpts(int argc, char **argv) {
	int ch, i;
	static int longOpt;

	enum longOptions {
		O_ALL,
		O_RATES,
		O_QUAT,
		O_ATT,
		O_INERTIA,
		O_THRUST,
		O_WCD,
		O_DCA
	};

	/* options descriptor */
	static struct option longopts[] = {
		{"help",			no_argument, 		NULL,		'h'},
		{"plot",			no_argument,		NULL,		'p'},
		{"pairs",			no_argument,		NULL,		'P'},
		{"col-headers",		no_argument,		NULL,		'c'},
		{"exp-format",		required_argument,	NULL,		'e'},
		{"range-min",		required_argument,	NULL,		'm'},
		{"range-max",		required_argument,	NULL,		'M'},
		{"all",				no_argument,		&longOpt,	O_ALL},
		{"rates",			no_argument,		&longOpt,	O_RATES},
		{"quat",			no_argument,		&longOpt,	O_QUAT},
		{"att",				no_argument,		&longOpt,	O_ATT},
		{"inertia",			no_argument,		&longOpt,	O_INERTIA},
		{"thrust",			no_argument,		&longOpt,	O_THRUST},
		{"wcd",				no_argument,		&longOpt,	O_WCD},
		{"dca",				no_argument,		&longOpt,	O_DCA},
		{NULL,				0,					NULL,		0}
	};

	while ((ch = getopt_long(argc, argv, "hpPce:m:M:", longopts, NULL)) != -1) {
		switch (ch) {
			case 'h':
				qLogDumpUsage();
				exit(0);
				break;
			case 'p':
				dumpPlot = true;
				break;
			case 'c':
				includeHeaders = true;
				break;
			case 'e':
				if (strcmp(optarg, "csv") == 0)
					sep = ',';
				else if (strcmp(optarg, "tab") == 0)
					sep = '	';
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
							if (i != NUM_LOG_FIELDS)
								dumpOrder[dumpNum++] = i;
						}
						return;  // prevent dumpOrder overflow
					case O_RATES:
						dumpOrder[dumpNum++] = RATE_DES_0;
						dumpOrder[dumpNum++] = RATE_ACT_0;
						dumpOrder[dumpNum++] = RATE_DES_1;
						dumpOrder[dumpNum++] = RATE_ACT_1;
						dumpOrder[dumpNum++] = RATE_DES_2;
						dumpOrder[dumpNum++] = RATE_ACT_2;
						break;
					case O_QUAT:
						dumpOrder[dumpNum++] = QUAT_DES_0;
						dumpOrder[dumpNum++] = QUAT_ACT_0;
						dumpOrder[dumpNum++] = QUAT_DES_1;
						dumpOrder[dumpNum++] = QUAT_ACT_1;
						dumpOrder[dumpNum++] = QUAT_DES_2;
						dumpOrder[dumpNum++] = QUAT_ACT_2;
						dumpOrder[dumpNum++] = QUAT_DES_3;
						dumpOrder[dumpNum++] = QUAT_ACT_3;
						break;
					case O_ATT:
						dumpOrder[dumpNum++] = ROLL_DES;
						dumpOrder[dumpNum++] = ROLL_ACT;
						dumpOrder[dumpNum++] = PITCH_DES;
						dumpOrder[dumpNum++] = PITCH_ACT;
						dumpOrder[dumpNum++] = YAW_DES;
						dumpOrder[dumpNum++] = YAW_ACT;
						break;
					case O_INERTIA:
						dumpOrder[dumpNum++] = INERTIA_REQ_0;
						dumpOrder[dumpNum++] = INERTIA_REQ_1;
						dumpOrder[dumpNum++] = INERTIA_REQ_2;
						break;
					case O_THRUST:
						dumpOrder[dumpNum++] = HOVER_THRUST;
						break;
					case O_WCD:
						dumpOrder[dumpNum++] = WCD_0;
						dumpOrder[dumpNum++] = WCD_1;
						dumpOrder[dumpNum++] = WCD_2;
						break;
					case O_DCA:
						for (i=0; i < LOG_NUM_DCA; i++)
							dumpOrder[dumpNum++] = DCA_0 + i;
						break;
				} // longopt switch
				break;
			default:
				// fprintf(stderr, "logDump: logDumpOpts: error\n");
				break;
		} // getopt value switch
	} // while getopt has value loop

}

float *qLogDumpMakeQarray(const char *typ) {
	static float q[4];
	if (!strcmp(typ, "a")) {
		q[0] = logRowData[QUAT_ACT_0];
		q[1] = logRowData[QUAT_ACT_1];
		q[2] = logRowData[QUAT_ACT_2];
		q[3] = logRowData[QUAT_ACT_3];
	} else {
		q[0] = logRowData[QUAT_DES_0];
		q[1] = logRowData[QUAT_DES_1];
		q[2] = logRowData[QUAT_DES_2];
		q[3] = logRowData[QUAT_DES_3];
	}
	return q;
}

void attitudeExtractEulerQuat(float *q, float *rpy) {
	float q0, q1, q2, q3;

	q0 = q[1];
	q1 = q[2];
	q2 = q[3];
	q3 = q[0];

	rpy[2] = atan2((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
	rpy[1] = asin(-2.0f * (q0 * q2 - q1 * q3));
	rpy[0] = atan((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));
}

double qLogDumpGetValue(int field) {
	double val = nan("");
	float rpy[3];

	switch (field) {
		case ROLL_DES:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("d"), rpy);
			val = rpy[0];
			break;
		case PITCH_DES:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("d"), rpy);
			val = rpy[1];
			break;
		case YAW_DES:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("d"), rpy);
			val = rpy[2];
			break;
		case ROLL_ACT:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("a"), rpy);
			val = rpy[0];
			break;
		case PITCH_ACT:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("a"), rpy);
			val = rpy[1];
			break;
		case YAW_ACT:
			attitudeExtractEulerQuat(qLogDumpMakeQarray("a"), rpy);
			val = rpy[2];
			break;
		default:
			val = (double)logRowData[field];
			break;
	}
	return val;
}

void qLogDumpStats() {
	int i;
	double val;

	for (i = 0; i < dumpNum; i++) {
		val = qLogDumpGetValue(dumpOrder[i]);
		if (val > dumpYMax[i])
			dumpYMax[i] = val;
		if (val < dumpYMin[i])
			dumpYMin[i] = val;
	}
}

void qLogDumpHeaders(void) {
	int i;

	if (dumpNum) {
		for (i = 0; i < dumpNum; i++) {
			printf("%s", fieldLabels[dumpOrder[i]]);
			if (i < dumpNum - 1)
				printf("%c", sep);
		}
		printf("\n"); // end of header row
	}
}

void qLogDumpText(void) {
	int i;

	for (i = 0; i < dumpNum; i++) {
		printf("%.15G", qLogDumpGetValue(dumpOrder[i]));
		if (i < dumpNum - 1)
			printf("%c", sep);
	}
	printf("\n"); // end of export row
}

bool qLogDumpProgress(const uint32_t count) {
	// send progress indication
	if (!(count % 1000)) {
		fprintf(stderr, ".");
		fflush(stderr);
	}
	return !dumpRangeMax || count <= dumpRangeMax;
}

int main(int argc, char *argv[]) {
	FILE *fp;
	unsigned int sync = 0;
	uint32_t rec = 0;
	uint32_t exp_count = 0;

	plotterOpts(argc, argv);
	qLogDumpOpts(argc, argv);
	argc -= optind;
	argv += optind;

	if (argc < 1) {
		fprintf(stderr, "quatosLogDump: need log file argument.  Type quatosLogDump --help for usage details.\n");
		exit(1);
	}

	if (dumpNum < 1) {
		fprintf(stderr, "quatosLogDump: need at least one value to export.  Type quatosLogDump --help for usage details.\n");
		exit(1);
	}

	fprintf(stderr, "quatosLogDump: opening logfile: %s\n", argv[0]);

	fp = fopen(argv[0], "rb");
	if (fp == NULL) {
		fprintf(stderr, "quatosLogDump: Cannot open file '%s', aborting...\n", argv[0]);
		return 0;
	}

	// plot output
	if (dumpPlot) {
		double *xVals, *yVals;
		int i, j;

		// need to get X & Y extents for all plotted values to initialize plotter

		dumpYMin = (double *)calloc(dumpNum, sizeof(double));
		dumpYMax = (double *)calloc(dumpNum, sizeof(double));
		dumpXMin = (double *)calloc(dumpNum, sizeof(double));
		dumpXMax = (double *)calloc(dumpNum, sizeof(double));
		// initialize with bogus values
		std::fill(dumpYMin, dumpYMin + dumpNum, +9999999.99);
		std::fill(dumpYMax, dumpYMax + dumpNum, -9999999.99);

		// read through entire log to update dumpYMin/dumpYMax extents for each value being exported
		while (fread(&sync, sizeof(sync), 1, fp) == 1) {
			if (sync == 0xffffffff) {
				if (fread(logRowData, sizeof(float), NUM_LOG_FIELDS, fp) == NUM_LOG_FIELDS) {
					if (rec++ >= dumpRangeMin) {
						qLogDumpStats();
						exp_count++;
						if (!qLogDumpProgress(rec))
							break;
					}
				} else
					fprintf(stderr, "quatosLogDump: field count error record # %d\n", rec);
			} else
				fprintf(stderr, "quatosLogDump: sync error record # %d\n", rec);
		}

		// NOTE: everything below assumes that all logged columns (values) have the same number of samples (rec).
		// We could do this per value instead (inside the next log reading loop) but at this point it's overkill.

		xVals = (double *)calloc(exp_count, sizeof(double));
		yVals = (double *)calloc(exp_count, sizeof(double));

		// populate X graph values with zero through n samples
		for (j = 0; j < exp_count; j++)
			xVals[j] = (double)j + dumpRangeMin;

		std::fill(dumpXMin, dumpXMin + dumpNum, *std::min_element(xVals, xVals + exp_count));
		std::fill(dumpXMax, dumpXMax + dumpNum, *std::max_element(xVals, xVals + exp_count));

		if (!plotterInit(dumpNum, dumpYMin, dumpYMax, dumpXMin, dumpXMax))
			exit(1);

		for (i = 0; i < dumpNum; i++) {
			rewind(fp);
			rec = exp_count = 0;
			while (fread(&sync, sizeof(sync), 1, fp) == 1) {
				if (sync == 0xffffffff && fread(logRowData, sizeof(float), NUM_LOG_FIELDS, fp) == NUM_LOG_FIELDS) {
					if (rec++ >= dumpRangeMin)
						yVals[exp_count++] = qLogDumpGetValue(dumpOrder[i]);
					if (!qLogDumpProgress(rec))
						break;
				}
			}
			plotterLine(exp_count, i, xVals, yVals, fieldLabels[dumpOrder[i]]);
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

		if (includeHeaders)
			qLogDumpHeaders();

		while (fread(&sync, sizeof(sync), 1, fp) == 1) {
			if (sync == 0xffffffff) {
				if (fread(logRowData, sizeof(float), NUM_LOG_FIELDS, fp) == NUM_LOG_FIELDS) {
					if (rec++ >= dumpRangeMin) {
						qLogDumpText();
						exp_count++;
					}
					if (!qLogDumpProgress(rec))
						break;
				}
			}
			else {
				fprintf(stderr, "sync error record # %d\n", rec);
			}
		}
	}

	fclose(fp);

	fprintf(stderr, "\nquatosLogDump: %d records dumped\n", exp_count);

	return 1;
}
