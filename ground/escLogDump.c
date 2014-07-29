#include <stdio.h>
#include <stdint.h>

typedef struct {
    uint8_t escId;
    uint32_t micros;
    uint32_t data[2];
} __attribute__((packed)) motorsLog_t;

typedef struct {
    unsigned int state :    3;
    unsigned int vin :	    12;	// x 100
    unsigned int amps :	    14;	// x 100
    unsigned int rpm :	    15;
    unsigned int duty :	    8;	// x (255/100)
    unsigned int temp :     9;  // (Deg C + 32) * 4
    unsigned int errCode :  3;
} esc32CanStatus_t;

motorsLog_t logBuf;

int main(int argc, char *argv[]) {
	FILE *fp;
	uint8_t sync = 0;
	int rec = 0;
	int i;

	fp = fopen(argv[1], "rb");
	if (fp == NULL) {
		fprintf(stderr, "Cannot open file '%s', aborting...\n", argv[1]);
	}
	else {
		while (fread(&sync, sizeof(sync), 1, fp) == 1) {
			if (sync == 0xff) {
				if (fread(&logBuf, sizeof(motorsLog_t), 1, fp) == 1) {
					// check bits 9 & 10 of the sync flag
					if ((logBuf.escId & 0xc0) == 0xc0) {
						esc32CanStatus_t *status = (esc32CanStatus_t *)&logBuf.data;

						printf("%d ", logBuf.micros);
						printf("%d ", logBuf.escId & 0x3f);
						printf("%f ", status->vin / 100.0f);
						printf("%f ", status->amps / 100.0f);
						printf("%d ", status->rpm);
						printf("%f ", (float)status->duty / 255 * 100);
						printf("%f ", (32.0f + status->temp) / 4);
						printf("%d ", status->errCode);
						printf("\n");
						rec++;
					}
				}
			}
			else {
				fprintf(stderr, "sync error record # %d\n", rec);
			}
		}
	}
	return 1;
}
