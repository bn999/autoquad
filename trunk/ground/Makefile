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
#    Copyright © 2011-2014  Bill Nesbitt

# Include user-specific settings file, if any, in regular Makefile format.
# This file can set any default variable values you wish to override (all defaults are listed below).
# The .user file is not included with the source code distribution, so it will not be overwritten.
-include Makefile.user

CC ?= g++
CFLAGS ?= -g -O3
#LDFLAGS ?=

# BUILD_PATH = .
BUILD_PATH ?= ../build

# Linux/OS X
#LIBPATH ?= /opt/local/lib
#INCPATH ?= /opt/local/include
#MAVLINK ?= ../mavlink/include/autoquad
#EXPAT ?= $(LIBPATH)
#EXPAT_LIB ?= expat
#PLPLOT ?= $(LIBPATH)
#PLPLOT_LIB ?= plplotd
#PLPLOT_INC ?= $(INCPATH)
#EIGEN ?= /usr/local/include/eigen3

# Windows
LIBPATH ?= ../../../lib
INCPATH ?= .
MAVLINK ?= $(LIBPATH)/mavlink/include/autoquad
EXPAT ?= $(LIBPATH)/expat
EXPAT_LIB ?= libexpat
#PLPLOT ?= $(LIBPATH)/plplot/lib
#PLPLOT_LIB ?= libplplotd
#PLPLOT_INC ?= $(LIBPATH)
EIGEN ?= $(LIBPATH)/eigen

WITH_PLPLOT =
ifdef PLPLOT
	WITH_PLPLOT = -I$(PLPLOT_INC) -L$(PLPLOT) -l$(PLPLOT_LIB) -DHAS_PLPLOT
endif

ALL_CFLAGS = $(CFLAGS)

# Targets

all: loader telemetryDump logDump batCal quatosTool escLogDump quatosLogDump

all-win: logDump batCal quatosTool escLogDump quatosLogDump

loader: $(BUILD_PATH)/loader.o $(BUILD_PATH)/serial.o $(BUILD_PATH)/stmbootloader.o
	$(CC) -o $(BUILD_PATH)/loader $(ALL_CFLAGS) $(BUILD_PATH)/loader.o $(BUILD_PATH)/serial.o $(BUILD_PATH)/stmbootloader.o

telemetryDump: $(BUILD_PATH)/telemetryDump.o $(BUILD_PATH)/serial.o
	$(CC) -o $(BUILD_PATH)/telemetryDump $(ALL_CFLAGS) $(BUILD_PATH)/telemetryDump.o $(BUILD_PATH)/serial.o

logDump: $(BUILD_PATH)/logDump.o $(BUILD_PATH)/logger.o $(BUILD_PATH)/logDump_mavlink.o
	$(CC) -o $(BUILD_PATH)/logDump $(ALL_CFLAGS) $(BUILD_PATH)/logDump.o $(BUILD_PATH)/logger.o $(BUILD_PATH)/logDump_mavlink.o $(WITH_PLPLOT)

batCal: $(BUILD_PATH)/batCal.o $(BUILD_PATH)/logger.o
	$(CC) -o $(BUILD_PATH)/batCal $(ALL_CFLAGS) $(BUILD_PATH)/batCal.o $(BUILD_PATH)/logger.o $(WITH_PLPLOT)

quatosTool: $(BUILD_PATH)/quatosTool.o
	$(CC) -o $(BUILD_PATH)/quatosTool $(ALL_CFLAGS) $(BUILD_PATH)/quatosTool.o -L$(EXPAT) -l$(EXPAT_LIB)

escLogDump: $(BUILD_PATH)/escLogDump.o
	$(CC) -o $(BUILD_PATH)/escLogDump $(ALL_CFLAGS) $(BUILD_PATH)/escLogDump.o

quatosLogDump: $(BUILD_PATH)/quatosLogDump.o
	$(CC) -o $(BUILD_PATH)/quatosLogDump $(ALL_CFLAGS) $(BUILD_PATH)/quatosLogDump.o



$(BUILD_PATH)/loader.o: loader.c serial.h stmbootloader.h
	$(CC) -c $(ALL_CFLAGS) loader.c -o $@

$(BUILD_PATH)/stmbootloader.o: stmbootloader.c stmbootloader.h serial.o
	$(CC) -c $(ALL_CFLAGS) stmbootloader.c -o $@

$(BUILD_PATH)/serial.o: serial.c serial.h
	$(CC) -c $(ALL_CFLAGS) serial.c -o $@

$(BUILD_PATH)/telemetryDump.o: telemetryDump.c telemetryDump.h
	$(CC) -c $(ALL_CFLAGS) telemetryDump.c -o $@

$(BUILD_PATH)/logDump.o: logDump.c logDump_templates.h logDump.h logger.h logDump_mavlink.h
	$(CC) -c $(ALL_CFLAGS) logDump.c -o $@ -I$(INCPATH) -I$(MAVLINK) $(WITH_PLPLOT)

$(BUILD_PATH)/logDump_mavlink.o: logDump_mavlink.cpp logDump_mavlink.h
	$(CC) -c $(ALL_CFLAGS) logDump_mavlink.cpp -o $@ -I$(MAVLINK)

$(BUILD_PATH)/batCal.o: batCal.cc
	$(CC) -c $(ALL_CFLAGS) batCal.cc -o $@ -I$(INCPATH) -I$(EIGEN) $(WITH_PLPLOT)

$(BUILD_PATH)/quatosTool.o: quatosTool.cc
	$(CC) -c $(ALL_CFLAGS) quatosTool.cc -o $@ -I$(EXPAT)/src -I$(EIGEN)

$(BUILD_PATH)/logger.o: logger.c logger.h
	$(CC) -c $(ALL_CFLAGS) logger.c -o $@

$(BUILD_PATH)/escLogDump.o: escLogDump.c
	$(CC) -c $(ALL_CFLAGS) escLogDump.c -o $@

$(BUILD_PATH)/quatosLogDump.o: quatosLogDump.c
	$(CC) -c $(ALL_CFLAGS) quatosLogDump.c -o $@

clean:
	rm -f $(BUILD_PATH)/loader $(BUILD_PATH)/telemetryDump $(BUILD_PATH)/logDump $(BUILD_PATH)/batCal $(BUILD_PATH)/quatosTool $(BUILD_PATH)/*.o $(BUILD_PATH)/*.exe
