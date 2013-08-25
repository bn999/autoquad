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

#include "telemetryDump.h"
#include "serial.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <errno.h>

static struct telemetryFieldStruct telemetryFields[] = {
		{"Roll Angle", FLOAT_T},
		{"Pitch Angle", FLOAT_T},
		{"Yaw Angle", FLOAT_T},
		{"PWM Throttle", INT_T},
		{"PWM Rudd", INT_T},
		{"PWM Pitch", INT_T},
		{"PWM Roll", INT_T},
		{"PWM PZT", INT_T},
		{"PWM PLT", INT_T},
		{"GYO X", FLOAT_T},
		{"GYO Y", FLOAT_T},
		{"GYO Z", FLOAT_T},
		{"ACC X", FLOAT_T},
		{"ACC Y", FLOAT_T},
		{"ACC Z", FLOAT_T},
		{"Hold Heading", FLOAT_T},
		{"Pressure", FLOAT_T},
		{"Temperature", FLOAT_T},
		{"Press Alt", FLOAT_T},
		{"Volts In", FLOAT_T},
		{"Micros per Sec", INT_T},
		{"Pos N", FLOAT_T},
		{"Pos E", FLOAT_T},
		{"Pos D", FLOAT_T},
		{"GPS Lat", FLOAT_T},
		{"GPS Lon", FLOAT_T},
		{"GPS HAcc", FLOAT_T},
		{"GPS COG", FLOAT_T},
		{"GPS Alt", FLOAT_T},
		{"GPS PDOP", FLOAT_T},
		{"Hold Bearing", FLOAT_T},
		{"Hold Dist", FLOAT_T},
		{"Hold Alt", FLOAT_T},
		{"Pitch Tilt", FLOAT_T},
		{"Roll Tilt", FLOAT_T},
		{"Vel N", FLOAT_T},
		{"Vel E", FLOAT_T},
		{"Vel D", FLOAT_T},
		{"MAG X", FLOAT_T},
		{"MAG Y", FLOAT_T},
		{"MAG Z", FLOAT_T},
		{"Loop Freq", FLOAT_T},
		{"Radio Signal", FLOAT_T},
		{"Motor 1", FLOAT_T},
		{"Motor 2", FLOAT_T},
		{"Motor 3", FLOAT_T},
		{"Motor 4", FLOAT_T},
		{"Idle Percent", FLOAT_T},
		{"Nav Bias X", FLOAT_T},
		{"Nav Bias Y", FLOAT_T},
		{"Nav Bias Z", FLOAT_T},
		{"AGL", FLOAT_T},
		{NULL, NO_T}
	};

#define DEFAULT_PORT            "/dev/ttyUSB0"
#define DEFAULT_BAUD            115200

char port[256];
unsigned int baud;
unsigned char parityA, parityB;

void telemetryDumpUsage(void) {
	fprintf(stderr, "usage: telemetryDump <-h> <-p device_file> <-b baud_rate>\n");
}

int telemetryDumpInit(int argc, char **argv) {
	int ch;

	strncpy(port, DEFAULT_PORT, sizeof(port));
	baud = DEFAULT_BAUD;

	/* options descriptor */
	static struct option longopts[] = {
		{ "help",       required_argument,      NULL,           'h' },
		{ "port",       required_argument,      NULL,           'p' },
		{ "baud",       required_argument,      NULL,           's' },
		{ NULL,         0,                      NULL,           0 }
		};

	while ((ch = getopt_long(argc, argv, "hp:b:f:o", longopts, NULL)) != -1)
	switch (ch) {
		case 'h':
			telemetryDumpUsage();
			exit(0);
			break;
		case 'p':
			strncpy(port, optarg, sizeof(port));
			break;
		case 'b':
			baud = atoi(optarg);
			break;
		default:
			telemetryDumpUsage();
			return 0;
	}
	argc -= optind;
	argv += optind;

	return 1;
}

void telemetryDumpHeaders(void) {
	int i = 0;

	while (telemetryFields[i].fieldName) {
		if (i)
			printf(", ");

		printf("\"%s\"", telemetryFields[i].fieldName);
		i++;
	}
	printf("\n");
}

void telemetryDumpFail(serialStruct_t *s) {
	fprintf(stderr, "telemetryDump: read 1 failed with errno = %d, aborting...\n", errno);
	serialFree(s);
	fflush(stdout);
	exit(1);
}

unsigned char telemetryDumpRead(serialStruct_t *s) {
	unsigned char c;

	if (read(s->fd, &c, 1) < 1)
		telemetryDumpFail(s);
	
	return c;
}

void telemetryDumpChecksum(unsigned char c) {
	parityA += c;
	parityB += parityA;
}

int telemetryDumpGetFloat(serialStruct_t *s, float *v) {
	unsigned int p = 0;
	unsigned char *c = (unsigned char *)v;
	unsigned int i;

	for (i = 0; i < sizeof(float); i++) {
		*c = telemetryDumpRead(s);
		telemetryDumpChecksum(*c++);
	}
}

int telemetryDumpGetInt(serialStruct_t *s, int *v) {
	unsigned int p = 0;
	unsigned char *c = (unsigned char *)v;
	unsigned int i;

	for (i = 0; i < sizeof(int); i++) {
		*c = telemetryDumpRead(s);
		telemetryDumpChecksum(*c++);
	}
}

void telemetryDump(serialStruct_t *s) {
	char outputBuf[2048];
	char buf[64];
	unsigned char c;
	float floatVal;
	int intVal;
	int i, j;

	while (1) {
		start:

		// look for 'Aq' to indicate new line
		c = telemetryDumpRead(s);
		if (c != 'A')
			goto start;

		c = telemetryDumpRead(s);
		if (c != 'q')
			goto start;

		parityA = parityB = 0;

		c = telemetryDumpRead(s);
		if (c == 'T') {
			*outputBuf = 0;
			i = 0;
			while (telemetryFields[i].fieldName) {
				if (i)
					strncat(outputBuf, ", ", sizeof(outputBuf));

				if (telemetryFields[i].fieldType == FLOAT_T) {
					telemetryDumpGetFloat(s, &floatVal);
					snprintf(buf, sizeof(buf), "%g", floatVal);
				}
				else if (telemetryFields[i].fieldType == INT_T) {
					telemetryDumpGetInt(s, &intVal);
					snprintf(buf, sizeof(buf), "%d", intVal);
				}
				strcat(outputBuf, buf);
				i++;
			}

			c = telemetryDumpRead(s);
			if (parityA != c) {
				fprintf(stderr, "telemetryDump: checksum error\n");
				goto start;
			}

			c = telemetryDumpRead(s);
			if (parityB != c) {
				fprintf(stderr, "telemetryDump: checksum error\n");
				goto start;
			}

			printf("%s\n", outputBuf);
			fflush(stdout);
		}
	}
}

int main(int argc, char **argv) {
	serialStruct_t *s;

	// init
	if (!telemetryDumpInit(argc, argv)) {
		fprintf(stderr, "Init failed, aborting...\n");
		return 0;
	}

	if (!(s = initSerial(port, baud, 1))) {
		fprintf(stderr, "Cannot open port '%s', aborting...\n", port);
		return 0;
	}

	telemetryDumpHeaders();

	telemetryDump(s);

	return 1;
}
