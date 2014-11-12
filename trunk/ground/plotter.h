/*
 * plotter.h
 *
 *  Created on: Nov 7, 2014
 *      Author: Maxim Paperno

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

Usage example:
--------------
#include "plotter.h"

void usage(void) {
	...
	plotterUsage();  // to display plotter options and help
}

int main(int argc, char *argv[]) {

	plotterOpts(argc, argv);		// must call this to enable plotter and PLplot options processing
	myOptionsParser(argc, argv);	// if any, should be after plotterOpts(), or should ignore unknown options

	...

	plotterInit(nValues, 		<-- total number of items to be plotted
				minYValues[],	<-- array nValues long of minimum Y value for each item plotted
				maxYValues[], 	<-- array nValues long of maximum Y value for each item plotted
				minXValues[], 	<-- array nValues long of minimum X value for each item plotted
				maxXValues[]	<-- array nValues long of maximum X value for each item plotted
	);

	for each item {
		plotterLine(nrec, 		<-- total number of records in this graph
					nval,		<-- sequence number of this value (out of nValues in plotterInit())
					xVals[],	<-- array nrec long of graph X values
					yVals[],	<-- array nrec long of graph Y values
					label		<-- text description of this value (for legend)
		);
	}

	plotterEnd();  // must call to finish up
}
--------------
*/

#ifndef PLOTTER_H_
#define PLOTTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#define PLOTTER_COLOR0MAP_LEN		24				// number of colors in plplot color0 map

// option defaults
//
static bool plotNoLegend = false;					// if true, do not draw a legend
static bool plotWhiteBg = false;					// if true, use a white background and change color scheme to suit
static bool plotNoAlpha = false;					// if true, do not adjust the transparency of overlapping lines
static int plotValsPerPage = 0;						// if not zero, limit number of items shown per graph
static int plotMaxLegendValsOnTop = 3;				// if up to this many graph items, put legend on top as title (instead of on right side)
static int plotStartColor = 2;						// color index of first color to use for plot
static char plotDefaultSize[] = "1024x768";	// default canvas size (use PLPlot's -geometry to override)
// color values files for default and white backgrounds (must be in same folder as exe or in a "plplot standard path").
static char plotCmapFile[] = "plotter_colormap.pal";
static char plotCmapFileWbg[] = "plotter_colormap_whitebg.pal";
// graph minimum and maximum X and Y scales
static double plotScaleYMin = nan(""), plotScaleYMax = nan("");
static double plotScaleXMin = nan(""),plotScaleXMax = nan("");
// default output plplot device (use -dev to override)
#if defined (__WIN32__)
static char plotDefaultDevice[] = "wingcc";
#elif defined (__APPLE__)
static char plotDefaultDevice[] = "aqt";
#else
static char plotDefaultDevice[] = "xwin";
#endif

extern void plotterUsage(void);
extern void plotterOpts(int &argc, char **argv);
extern bool plotterInit(const int nValues, double *minYValues, double *maxYValues, double *minXValues, double *maxXValues);
extern void plotterNewPage(const int nvals, double ymin, double ymax, double xmin, double xmax, const char *title = 0);
extern void plotterLine(const int nrec, const int nval, const double xVals[], const double yVals[], const char *label);
extern void plotterEndPage();
extern void plotterEnd();

#ifdef __cplusplus
}
#endif

#endif /* PLOTTER_H_ */
