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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "serial.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#define STM_RETRIES_SHORT	1000
#define STM_RETRIES_LONG	5000

unsigned char getResults[11];

unsigned char stmHexToChar(const char *hex) {
	char hex1, hex2;
	unsigned char nibble1, nibble2;
	
	// force data to upper case
	hex1 = toupper(hex[0]);
	hex2 = toupper(hex[1]);

	nibble1 = hex1 - ((hex1 <  65) ? 48 : 55);
	nibble2 = hex2 - ((hex2 <  65) ? 48 : 55);

	return (nibble1 << 4 | nibble2);
}


unsigned char stmWaitAck(serialStruct_t *s, int retries) {
	unsigned char c;
	unsigned int i;

	for (i = 0; i < retries; i++) {
		if (serialAvailable(s)) {
			c = serialRead(s);
			if (c == 0x79) {
//				putchar('+'); fflush(stdout);
				return 1;
			}
			if (c == 0x1f) {
				putchar('-'); fflush(stdout);
				return 0;
			}
			else {
				printf("?%02x?", c); fflush(stdout);
				return 0;
			}
		}
		usleep(1000);
	}

	return 0;
}

unsigned char stmWriteString(serialStruct_t *s, const char *hex) {
	unsigned char c;
	unsigned char ck;
	unsigned char i;

	ck = 0;
	i = 0;
	while (*hex) {
		c = stmHexToChar(hex);
		serialWrite(s, (char *)&c, 1);
		ck ^= c;
		hex += 2;
		i++;
	}
	if (i == 1)
		ck = 0xff ^ c;

	// send checksum
	serialWrite(s, (char *)&ck, 1);

	return stmWaitAck(s, STM_RETRIES_LONG);
}

int stmWriteLen(serialStruct_t *s, char *data, int len, unsigned char ck) {
	unsigned char c;
	int i;

	for (i = 0; i < len; i++) {
		c = data[i];
		ck ^= c;
		serialWrite(s, (char *)&c, 1);
	}
	serialWrite(s, (char *)&ck, 1);

	return stmWaitAck(s, STM_RETRIES_LONG);
}

void stmSendData(serialStruct_t *s, unsigned int addr, char *buf, int len) {
	unsigned char c;
	unsigned char ck;
	unsigned char a[4];

	printf("Writing address %x for %d bytes\n", addr, len);

	sendRetry:

	do {
		c = getResults[5];
		serialWrite(s, (char *)&c, 1);
		c = 0xff ^ c;
		serialWrite(s, (char *)&c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	// send address
	// reverse byte order
	a[0] = *(((unsigned char *)&addr)+3);
	a[1] = *(((unsigned char *)&addr)+2);
	a[2] = *(((unsigned char *)&addr)+1);
	a[3] = *(((unsigned char *)&addr)+0);
	if (!stmWriteLen(s, (char *)&a, 4, 0)) {
		printf("Address error\n");
		goto sendRetry;
	}

	// send len
	ck = 0;
	c = len - 1;
	serialWrite(s, (char *)&c, 1);
	ck ^= c;

	// send data
	if (!stmWriteLen(s, buf, len, ck)) {
		printf("Data error\n");
		goto sendRetry;
	}
}

void stmWriteBuffer(serialStruct_t *s, char *msb, char *lsb, char *len, char *data) {
	static unsigned char buf[256];
	static unsigned int startAddr = 0;
	static unsigned int length = 0;
	unsigned int newAddr;
	int offset;
	int n;
	int i;

	// flush if called with 0 address
	if (msb == 0  && lsb == 0 && startAddr) {
		stmSendData(s, startAddr, (char *)buf, length);
		startAddr = 0;
		length = 0;
		return;
	}

	n = stmHexToChar(len);

	newAddr = stmHexToChar(&msb[0])<<24 | stmHexToChar(&msb[2])<<16 | stmHexToChar(&lsb[0])<<8 | stmHexToChar(&lsb[2]);

	// is this new data within our current block?
	if (newAddr != (startAddr+length) || (newAddr+n) > (startAddr+256)) {
		// flush
		if (startAddr)
			stmSendData(s, startAddr, (char *)buf, length);

		// clear buffer
		memset(buf, 0xff, 256);

		startAddr = newAddr;
		length = 0;
	}

	offset = newAddr - startAddr;

	for (i = 0; i < n; i++)
		buf[offset+i] = stmHexToChar(&data[i*2]);

	length = offset + n;
}

// interpret Intel Hex file
char *stmHexLoader(serialStruct_t *s, FILE *fp) {
	char hexByteCount[3], hexAddressLSB[5], hexRecordType[3], hexData[128];
	char addressMSB[5];
	static char addressJump[9];

	memset(addressJump, 0, sizeof(addressJump));
	memset(addressMSB, 0, sizeof(addressMSB));

	while (fscanf(fp, ":%2s%4s%2s%s\n", hexByteCount, hexAddressLSB, hexRecordType, hexData) != EOF) {
		unsigned int byteCount, addressLSB, recordType;

		recordType = stmHexToChar(hexRecordType);
		hexData[stmHexToChar(hexByteCount) * 2] = 0;	// terminate at CHKSUM

		switch (recordType) {
			case 0x00:
				stmWriteBuffer(s, addressMSB, hexAddressLSB, hexByteCount, hexData);
				break;
			case 0x01:
				// EOF
				return addressJump;
				break;
			case 0x04:
				// MSB of destination 32 bit address
				strncpy(addressMSB, hexData, 4);
				break;
			case 0x05:
				// flush
				stmWriteBuffer(s, 0, 0, 0, 0);
				// 32 bit address to run after load
				strncpy(addressJump, hexData, 8);
				break;
		}
	}

	// flush
	stmWriteBuffer(s, 0, 0, 0, 0);

	return 0;
}

void stmLoader(serialStruct_t *s, FILE *fp, unsigned char overrideParity, unsigned char cont) {
	char c;
	unsigned char b1, b2, b3;
	unsigned char i, n;
	char *jumpAddress;

	// turn on parity generation
	if (!overrideParity)
		serialEvenParity(s);

	top:

	// only if not continuing previous session
	if (!cont) {
		printf("Place STM in bootloader mode and press any key >");
		getchar();
		printf("\n");

		serialFlush(s);

		// poke the MCU
		do {
			printf("p"); fflush(stdout);
			c = 0x7f;
			serialWrite(s, &c, 1);
		} while (!stmWaitAck(s, STM_RETRIES_SHORT));
		printf("STM bootloader alive...\n");
	}

	// send GET command
	do {
		c = 0x00;
		serialWrite(s, &c, 1);
		c = 0xff;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	b1 = serialRead(s);	// number of bytes
	b2 = serialRead(s);	// bootloader version
	printf("STM Bootloader version: %d.%d\n", (b2 & 0xf0) >> 4, (b2 & 0x0f));

	printf("Getting %d commands.\n", b1);
	for (i = 0; i < b1; i++)
		getResults[i] = serialRead(s);

	stmWaitAck(s, STM_RETRIES_LONG);
	printf("Commands received.\n");

	// send GET ID command
	printf("Getting ID\n");
	do {
		c = getResults[2];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	n = serialRead(s);
	printf("STM Device ID: 0x");
	for (i = 0; i <= n; i++)
		printf("%02x", serialRead(s));
	stmWaitAck(s, STM_RETRIES_LONG);
	printf("\n");

/*
	// Enable ROP
	printf("Sending enable ROP\n");
	c = getResults[9];
	serialWrite(s, &c, 1);
	c = 0xff ^ c;
	serialWrite(s, &c, 1);

	if (!stmWaitAck(s, STM_RETRIES_LONG))
		printf("ROP already active\n");
	else if (!stmWaitAck(s, STM_RETRIES_LONG))
		printf("Enable ROP failed\n");
	else
		goto top;
*/

	flash_size:

	// read Flash size
	c = getResults[3];
	serialWrite(s, &c, 1);
	c = 0xff ^ c;
	serialWrite(s, &c, 1);

	// if read not allowed, unprotect (which also erases)
	if (!stmWaitAck(s, STM_RETRIES_LONG)) {
		printf("ROP unprotect\n");

		// unprotect command
		c = getResults[10];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
		stmWaitAck(s, STM_RETRIES_LONG);

		// wait for results
		if (stmWaitAck(s, STM_RETRIES_LONG))
			goto top;
	}

	// send address
	if (!stmWriteString(s, "1FFFF7E0"))
		goto flash_size;

	// send # bytes (N-1 = 1)
	if (!stmWriteString(s, "01"))
		goto flash_size;

	b1 = serialRead(s);
	b2 = serialRead(s);
	printf("STM Flash Size: %dKB\n", b2<<8 | b1);

	// erase flash
	erase_flash:
	printf("Global flash erase [command 0x%x]...", getResults[6]); fflush(stdout);
	do {
		c = getResults[6];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	// global erase
	if (getResults[6] == 0x44) {
		// mass erase
		if (!stmWriteString(s, "FFFF"))
			goto erase_flash;
	}
	else {
		c = 0xff;
		serialWrite(s, &c, 1);
		c = 0x00;
		serialWrite(s, &c, 1);

		if (!stmWaitAck(s, STM_RETRIES_LONG))
			goto erase_flash;
	}

	printf("Done.\n");
	
	// upload hex file
	printf("Flashing device...\n");
	jumpAddress = stmHexLoader(s, fp);
	if (jumpAddress) {
		printf("\nFlash complete, restarting.\n");

		go:
		// send GO command
		do {
			c = getResults[4];
			serialWrite(s, &c, 1);
			c = 0xff ^ c;
			serialWrite(s, &c, 1);
		} while (!stmWaitAck(s, STM_RETRIES_LONG));

		// send address
		if (!stmWriteString(s, jumpAddress))
			goto go;
	}
	else {
		printf("\nFlash complete.\n");
	}
}
