#    This file is part of AutoQuad.
#
#    AutoQuad is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    AutoQuad is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    You should have received a copy of the GNU General Public License
#    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.
#
#    Copyright © 2011, 2012  Bill Nesbitt

CC = g++
CFLAGS = -g -O3
LDFLAGS =

ALL_CFLAGS = $(CFLAGS)

all: loader telemetryDump logDump batCal

loader: loader.o serial.o stmbootloader.o
	$(CC) -o loader $(ALL_CFLAGS) loader.o serial.o stmbootloader.o

telemetryDump: telemetryDump.o serial.o
	$(CC) -o telemetryDump $(ALL_CFLAGS) telemetryDump.o serial.o

logDump: logDump.o serial.o logger.o
	$(CC) -o logDump $(ALL_CFLAGS) logDump.o serial.o logger.o -L/opt/local/lib -lplplotd

batCal: batCal.o logger.o
	$(CC) -o batCal $(ALL_CFLAGS) batCal.o logger.o -L/opt/local/lib -lplplotd

loader.o: loader.c serial.h stmbootloader.h
	$(CC) -c $(ALL_CFLAGS) loader.c

stmbootloader.o: stmbootloader.c stmbootloader.h serial.o
	$(CC) -c $(ALL_CFLAGS) stmbootloader.c

serial.o: serial.c serial.h
	$(CC) -c $(ALL_CFLAGS) serial.c

telemetryDump.o: telemetryDump.c telemetryDump.h
	$(CC) -c $(ALL_CFLAGS) telemetryDump.c

logDump.o: logDump.c logger.h
	$(CC) -c $(ALL_CFLAGS) logDump.c -I/opt/local/include

batCal.o: batCal.cc
	$(CC) -c $(ALL_CFLAGS) batCal.cc -I/opt/local/include -I/usr/local/include/eigen3

logger.o: logger.c logger.h
	$(CC) -c $(ALL_CFLAGS) logger.c

clean:
	rm -f loader telemetryDump logDump batCal *.o
