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

#ifndef _telemetry_dump_h
#define _telemetry_dump_h

enum telemetryTypes {
	DOUBLE_T,
	FLOAT_T,
	INT_T,
	SHORT_T,
	CHAR_T,
	NO_T
};

struct telemetryFieldStruct {
	const char *fieldName;
	enum telemetryTypes fieldType;
};

#endif
