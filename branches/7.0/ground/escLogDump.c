#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t escId;
    uint32_t micros;
    uint32_t data[2];
} __attribute__((gcc_struct, packed)) motorsLog_t;  // gcc_struct attribute required for win/gcc/mingw, ignore on others

typedef struct {
    unsigned int state :    3;
    unsigned int vin :	    12;	// x 100
    unsigned int amps :	    14;	// x 100
    unsigned int rpm :	    15;
    unsigned int duty :	    8;	// x (255/100)
    unsigned int temp :     9;  // (Deg C + 32) * 4
    unsigned int errCode :  3;
} __attribute__((gcc_struct, packed)) esc32CanStatus_t;

int main(int argc, char *argv[]) {
	FILE *fp;
	uint8_t sync = 0;
	motorsLog_t logBuf;
	int rec = 0;
	int logdataVersion = 3;

	if (argc < 2 || !strcmp(argv[1], "-h")) {
		fprintf(stderr, "Usage: escLogDump [-v2] <log file> [ >output.txt ]\n\n");
		fprintf(stderr, "   Default is ESC32v3 log, use -v2 for ESC32v2.\n");
		exit(1);
	}

	if (!strcmp(argv[1], "-v2"))
		logdataVersion = 2;

	fp = fopen(argv[argc-1], "rb");
	if (fp == NULL) {
		fprintf(stderr, "Cannot open file '%s', aborting...\n", argv[argc-1]);
		exit(1);
	}

	fprintf(stderr, "Opening log file with ESC data version %d\n", logdataVersion);
	// column headers
	printf("micros id state vin amps rpm duty ");
	if (logdataVersion == 2)
		printf("errors ");
	else
		printf("temp ");
	printf("dsrm-code \n");

	while (fread(&sync, sizeof(sync), 1, fp) == 1) {
		if (sync == 0xff) {
			if (fread(&logBuf, 13, 1, fp) == 1) {
				// check bits 9 & 10 of the sync flag
				if ((logBuf.escId & 0xc0) == 0xc0) {
					esc32CanStatus_t *status = (esc32CanStatus_t *)&logBuf.data;

					printf("%d ", logBuf.micros);
					printf("%d ", logBuf.escId & 0x3f);
					printf("%d ", status->state);
					printf("%f ", status->vin / 100.0f);
					printf("%f ", status->amps / 100.0f);
					printf("%d ", status->rpm);
					printf("%f ", (float)status->duty / 255 * 100);
					if (logdataVersion == 2)
						printf("%d ", status->temp);  // actually the error count
					else
						printf("%f ", (float)status->temp / 4.0f - 32.0f);
					printf("%d ", status->errCode);
					printf("\n");
					rec++;
				} else
					fprintf(stderr, "invalid sync at record # %d (s: 0x%x)\n", rec, (logBuf.escId & 0xc0));
			} else
				fprintf(stderr, "invalid num fields at record # %d\n", rec);
		} else
			fprintf(stderr, "sync error record # %d (s: 0x%x)\n", rec, sync);
	}

	exit(0);
}
