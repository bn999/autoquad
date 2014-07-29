#include <stdio.h>

#define NUM_ROWS	29

float bsBuf[512];
static char sep = ' ';

int main(int argc, char *argv[]) {
	FILE *fp;
	unsigned int sync = 0;
	int rec = 0;
	int i;

	fp = fopen(argv[1], "rb");
	if (fp == NULL) {
		fprintf(stderr, "Cannot open file '%s', aborting...\n", argv[1]);
		return 0;
	}

	printf("Quat_Desired[0]%c", sep);
	printf("Quat_Desired[1]%c", sep);
	printf("Quat_Desired[2]%c", sep);
	printf("Quat_Desired[3]%c", sep);
	printf("wcd[0]%c", sep);
	printf("wcd[1]%c", sep);
	printf("wcd[2]%c", sep);
	printf("Quat_Actual[0]%c", sep);
	printf("Quat_Actual[1]%c", sep);
	printf("Quat_Actual[2]%c", sep);
	printf("Quat_Actual[3]%c", sep);
	printf("Rate_Actual[0]%c", sep);
	printf("Rate_Actual[1]%c", sep);
	printf("Rate_Actual[2]%c", sep);
	printf("Rate_Desired[0]%c", sep);
	printf("Rate_Desired[1]%c", sep);
	printf("Rate_Desired[2]%c", sep);
	printf("Inertia_Requested[0]%c", sep);
	printf("Inertia_Requested[1]%c", sep);
	printf("Inertia_Requested[2]%c", sep);
	printf("Hover_Thrust%c", sep);
	for (i = 22; i <= 29; i++)
		printf("DCA[%d]%c", i-22, sep);
	printf("\n");

	while (fread(&sync, sizeof(sync), 1, fp) == 1) {
		if (sync == 0xffffffff) {
			if (fread(bsBuf, sizeof(float), NUM_ROWS, fp) == NUM_ROWS) {
				for (i = 0; i < NUM_ROWS; i++)
					printf("%g%c", bsBuf[i], sep);
				printf("\n");
				rec++;
			}
		}
		else {
			fprintf(stderr, "sync error record # %d\n", rec);
		}
	}

	fclose(fp);

	fprintf(stderr, "%d records dumped\n", rec);

	return 1;
}
