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
#include <getopt.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

typedef struct {
	float vIn;
	float soc;
} logData_t;

logData_t *logData;
double zeroSOC;
int numRecs;

void batCalPlot(MatrixXd &ab) {
	char s[256];
	float yMin, yMax;
	int i;

	PLFLT *xVals;
	PLFLT *yVals;

	xVals = (PLFLT *)calloc(numRecs, sizeof(PLFLT));
	yVals = (PLFLT *)calloc(numRecs, sizeof(PLFLT));

	yMin = +999999999.99;
	yMax = -999999999.99;

	for (i = 0; i < numRecs; i++) {
		if (logData[i].vIn < yMin)
			yMin = logData[i].vIn;
		if (logData[i].vIn > yMax)
			yMax = logData[i].vIn;
	}

	plinit();
	plcol0(15);
	plenv(100.0, 0.0, yMin, yMax, 0, 0);
	pllab("SOC (%)", "Volts", "Battery Voltage vs State of Charge");

	sprintf(s, "y = %5.1f + %5.1f*x + %5.1f*x^2 + %5.1f*x^3 + %5.1f*x^4 + %5.1f*x^5",
		ab(0, 0), ab(1, 0), ab(2, 0), ab(3, 0), ab(4, 0), ab(5, 0));
        plptex(95.0, yMax-(yMax-yMin)/10.0, -10, 0, 0, s);
	printf("%s\n", s);

	// plot data
	for (i = 0; i < numRecs; i++) {
		xVals[i] = logData[i].soc * 100.0;
		yVals[i] = logData[i].vIn;
	}
	plcol0(3);
	plline(numRecs, xVals, yVals);

	// plot curve
	for (i = 0; i < numRecs; i++) {
		double s = logData[i].soc;

		xVals[i] = s * 100;
		yVals[i] = ab(0, 0) + ab(1, 0)*s + ab(2, 0)*s*s + ab(3, 0)*s*s*s + ab(4, 0)*s*s*s*s + ab(5, 0)*s*s*s*s*s;
	}
	plcol0(1);
	plline(numRecs, xVals, yVals);

	plend();

	free(xVals);
	free(yVals);
}

void batCalInterpolate(void) {
	MatrixXd X;
	MatrixXd A;
	MatrixXd c;
	MatrixXd ab;
	int m,n;
	int i, j, k;

	n = numRecs;
	m = 6;		// 5th order poly

	X.setZero(n, m);
	A.setZero(m, m);
	c.setZero(m, 1);

	// y = a + bx + cx^2 + dx^3 + ex^4 fx^5
	for (k = 0; k < n; k++) {
		X(k, 0) = 1.0;
		X(k, 1) = logData[k].soc;
		X(k, 2) = logData[k].soc * logData[k].soc;
		X(k, 3) = logData[k].soc * logData[k].soc * logData[k].soc;
		X(k, 4) = logData[k].soc * logData[k].soc * logData[k].soc * logData[k].soc;
		X(k, 5) = logData[k].soc * logData[k].soc * logData[k].soc * logData[k].soc * logData[k].soc;
	}

	for (i = 0; i < m; i++) {
		for (j = 0; j < m; j++)
			for (k = 0; k < n; k++)
				A(i, j) += X(k, i) * X(k, j);

		for (k = 0; k < n; k++)
			c(i, 0) += X(k, i) * logData[k].vIn;
	}

	ab = A.inverse() * c;

	batCalPlot(ab);

	printf("#define DEFAULT_SPVR_BAT_CRV1	%+e\n", ab(0, 0));
	printf("#define DEFAULT_SPVR_BAT_CRV2	%+e\n", ab(1, 0));
	printf("#define DEFAULT_SPVR_BAT_CRV3	%+e\n", ab(2, 0));
	printf("#define DEFAULT_SPVR_BAT_CRV4	%+e\n", ab(3, 0));
	printf("#define DEFAULT_SPVR_BAT_CRV5	%+e\n", ab(4, 0));
	printf("#define DEFAULT_SPVR_BAT_CRV6	%+e\n", ab(5, 0));
}

int batCalLoadLog(char *logFile) {
        FILE *lf;
	loggerRecord_t l;

	lf = fopen(logFile, "r");

	if (lf == NULL) {
		fprintf(stderr, "batCal: cannot open logfile '%s'\n", logFile);
		return 0;
	}
	else {
		numRecs = 0;
		while (loggerReadEntry(lf, &l) != EOF) {
			if (l.vIn < zeroSOC)
				break;

			if (l.throttle > 0) {
				logData = (logData_t *)realloc(logData, sizeof(logData_t) * (numRecs+1));
				logData[numRecs].vIn = l.vIn;
				numRecs++;
			}
		}
	}

	fclose(lf);

	if (numRecs < 1) {
		fprintf(stderr, "batCal: aborting\n");
	}
	else {
		int i;

		for (i = 0; i < numRecs; i++) {
			logData[i].soc = 1.0 - ((double)i / (double)numRecs);
		}

		fprintf(stderr, "batCal: loaded %d records from '%s'\n", numRecs, logFile);
	}

	return numRecs;
}

void batCalUsage(void) {
	fprintf(stderr, "usage: batcal [--help] [--zero=value] <log_file>\n");
}

void batCalOpts(int argc, char **argv) {
        int bflag, ch;
	static int longOpt;
	int pCount;
	float variance;
	int i;

        /* options descriptor */
        static struct option longopts[] = {
                {"help",		no_argument,		NULL,		'h'},
                {"zero",		required_argument,	NULL,		'z'},
                {NULL,          	0,                      NULL,		0}
        };

        bflag = 0;
        while ((ch = getopt_long(argc, argv, "hz:", longopts, NULL)) != -1)
                switch (ch) {
		case 'h':
			batCalUsage();
			break;
		case 'z':
			zeroSOC = atof(optarg);
			break;
                default:
			batCalUsage();
                        fprintf(stderr, "sim2: calOpts: error\n");
                        break;
                }
}

int main(int argc, char **argv) {
        batCalOpts(argc, argv);
        argc -= optind;
        argv += optind;

	if (argc != 1) {
		fprintf(stderr, "batCal: need one log file argument, aborting\n");
		return 1;
	}
	

	if (!batCalLoadLog(*argv)) {
		fprintf(stderr, "batCal: no logs loaded, aborting...\n");
		return 1;
	}
	
	batCalInterpolate();

	return 0;
}
