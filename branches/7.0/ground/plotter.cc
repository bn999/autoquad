/*
 * plotter.cc
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

*/

#include "plotter.h"
#include <stdio.h>
#include <string.h>
#include <algorithm>
#if defined (__WIN32__)
	#include "windows.h"
#endif
#ifdef HAS_PLPLOT
	#include "plplot/plplot.h"
#endif

// runtime defaults
bool plotLegendOnTop = false;
int plotMaxColors = PLOTTER_COLOR0MAP_LEN;
int plotNumValues = 0;
int plotNumPages = 1;
int plotCurrValIdx = -1;
int plotCurrPgIdx = -1;
char *plotCustomBgColor = NULL;
char *plotCustomCmapFile = NULL;
// runtime globals
int plotValsPerCurrPg;
int *plotColors, *plotLegendOptions;
double *plotMaxYValues, *plotMinYValues;
double *plotMaxXValues, *plotMinXValues;
const char *plotValueLabels[100];

#ifdef HAS_PLPLOT
	// extend PLPlot options
	static PLOptionTable plotOptions[] = {
		{
			"vpg",
			NULL, NULL,
			&plotValsPerPage,
			PL_OPT_INT | PL_OPT_ARG,
			"-vpg number",
			"Values Per Graph: limit number of items shown per graph (might create\n                            multiple graphs). 0 (default) to show all values on one graph."
		},
		{
			"ymin",
			NULL, NULL,
			&plotScaleYMin,
			PL_OPT_FLOAT | PL_OPT_ARG,
			"-ymin number",
			"Minimum plot Y scale (all graphs)."
		},
		{
			"ymax",
			NULL, NULL,
			&plotScaleYMax,
			PL_OPT_FLOAT | PL_OPT_ARG,
			"-ymax number",
			"Maximum plot Y scale (all graphs)."
		},
		{
			"xmin",
			NULL, NULL,
			&plotScaleXMin,
			PL_OPT_FLOAT | PL_OPT_ARG,
			"-xmin number",
			"Minimum plot X scale (all graphs)."
		},
		{
			"xmax",
			NULL, NULL,
			&plotScaleXMax,
			PL_OPT_FLOAT | PL_OPT_ARG,
			"-xmax number",
			"Maximum plot X scale (all graphs)."
		},
		{
			"noleg",
			NULL, NULL,
			&plotNoLegend,
			PL_OPT_BOOL,
			"-noleg",
			"Do not draw graph legend(s)."
		},
		{
			"max_top_leg",
			NULL, NULL,
			&plotMaxLegendValsOnTop,
			PL_OPT_INT | PL_OPT_ARG,
			"-max_top_leg number",
			"Maximum legend items to fit on top of graph (default 3)."
		},
		{
			"white",
			NULL, NULL,
			&plotWhiteBg,
			PL_OPT_BOOL,
			"-white",
			"Use a white background for the plot."
		},
		{
			"bgc",
			NULL, NULL,
			&plotCustomBgColor,
			PL_OPT_STRING | PL_OPT_ARG,
			"-bgc color",
			"Background color (use instead of PLplot -bg option, same syntax)."
		},
		{
			"color",
			NULL, NULL,
			&plotStartColor,
			PL_OPT_INT | PL_OPT_ARG,
			"-color num (2-23)",
			"Index of color to use as starting point for each new graph (def. 2)."
		},
		{
			"notrans",
			NULL, NULL,
			&plotNoAlpha,
			PL_OPT_BOOL,
			"-notrans",
			"Do not adjust the transparency of overlapping lines."
		},
		{
			"cmap0",
			NULL, NULL,
			&plotCustomCmapFile,
			PL_OPT_STRING | PL_OPT_ARG | PL_OPT_INVISIBLE,
			"-cmap0 file_name",
			"Use plot colors from a .pal format file. First color is BG, 2nd is FG."
		},
		{ NULL, NULL, NULL, NULL, 0, NULL, NULL } // terminate
	};
#endif


void plotterUsage(void) {
#ifdef HAS_PLPLOT
	plOptUsage();
#endif
}

void plotterOpts(int &argc, char **argv) {
#ifdef HAS_PLPLOT
	plSetUsage(argv[0], "");
	plMergeOpts(plotOptions, "Log plotting options", NULL);
	plsetopt("-dev", plotDefaultDevice);
	plsetopt("-geometry", plotDefaultSize);
	plparseopts(&argc, (const char**)argv, PL_PARSE_SKIP);
#endif

	if (plotStartColor >= PLOTTER_COLOR0MAP_LEN)
		plotStartColor = PLOTTER_COLOR0MAP_LEN - 1;
	else if (plotStartColor < 2)
		plotStartColor = 2;
}

void plotterSwapVals(double *a, double *b) {
	double tmp = *a;
	*a = *b;
	*b = tmp;
}

void plotterSetPaths(void) {
#if defined (__WIN32__)
	char *path, *ptr, *pos, cpath[MAX_PATH];
	GetModuleFileName(NULL, cpath, MAX_PATH);
	pos = strrchr(cpath, '\\');
	ptr = getenv("PATH");
	path = (char *)malloc(strlen(ptr)+MAX_PATH+25);
	strcpy(path, "PATH=");
	strcat(path, ptr);
	strcat(path, ";");
	strncat(path, cpath, pos-cpath+1);
	strcat(path, "plplot\\bin");
	putenv(path);
#endif
}

bool plotterInit(const int nValues, double *minYValues, double *maxYValues, double *minXValues, double *maxXValues) {
#ifdef HAS_PLPLOT
	int w = 1, h = 1; // page windows grid (w by h plots per page)
	int c, r, g, b, i;
	double a;

	plotterSetPaths();

	plotNumValues = nValues;
	plotMinYValues = minYValues;
	plotMaxYValues = maxYValues;
	plotMinXValues = minXValues;
	plotMaxXValues = maxXValues;

	if (plotValsPerPage) {
		plotNumPages = ceilf((float)nValues / (float)plotValsPerPage);
		w = ceilf(sqrtf(plotNumPages));
		h = rintf(sqrtf(plotNumPages));
	} else
		plotValsPerPage = nValues;
	//fprintf(stderr, "plotter: nValues: %d, plotNumPages: %d, w: %d, h: %d\n", nValues, plotNumPages, w, h);

	if (plotCustomCmapFile)
		plspal0(plotCustomCmapFile);
	else if (plotWhiteBg)
		plspal0(plotCmapFileWbg);
	else
		plspal0(plotCmapFile);

	if (plotCustomBgColor)
		plsetopt("-bg", plotCustomBgColor);

	if (!plotWhiteBg) {
		plgcolbg(&r, &g, &b);  					// test background color
		if (r > 175 && g > 175 && b > 175) {	// have light bg
			plotWhiteBg = true;
			if (!plotCustomCmapFile)
				plspal0(plotCmapFileWbg);
		}
	}

	// ensure enough colors for all possible values per page
	if (plotValsPerPage + plotStartColor > PLOTTER_COLOR0MAP_LEN) {
		plotMaxColors = plotValsPerPage + plotStartColor;
		plscmap0n(plotMaxColors);
		c = 2;
		for (i = PLOTTER_COLOR0MAP_LEN; i < plotMaxColors; i++) {
			//fprintf(stderr, "plotter: i: %d, c: %d\n", i, c);
			plgcol0a(c++, &r, &g, &b, &a);
			plscol0a(i, r, g, b, a);
			if (c >= PLOTTER_COLOR0MAP_LEN)
				c = 2;
		}
	}

	plstar(w, h);
	plsfont(PL_FCI_SANS, -1, -1);

	return true;
#else
	fprintf(stderr, "plotter: error -- no plotting library available\n");
	return false;
#endif
}

void plotterNewPage(const int nvals, double ymin, double ymax, double xmin, double xmax, const char *title) {
#ifdef HAS_PLPLOT
	// window margins
	double mleft = 0.04;
	double mright = 0.97;
	double mbot = 0.10;
	double mtop = 0.96;
	int r, g, b, i;

	plotCurrPgIdx++;
	plotValsPerCurrPg = nvals;
	plotColors = (int *)calloc(plotValsPerCurrPg, sizeof(int));
	plotLegendOptions = (int *)calloc(plotValsPerCurrPg, sizeof(int));
	//plotValueLabels = (char **)calloc(plotValsPerCurrPg, sizeof(char)*100);\

	plotLegendOnTop = nvals <= plotMaxLegendValsOnTop;
	if (title != 0 || plotLegendOnTop)
		mtop = 0.90;
	if (!plotLegendOnTop && !plotNoLegend)
		mright = 0.80;

	if (!isnan(plotScaleYMin))
		ymin = plotScaleYMin;
	if (!isnan(plotScaleYMax))
		ymax = plotScaleYMax;
	if (!isnan(plotScaleXMin))
		xmin = plotScaleXMin;
	if (!isnan(plotScaleXMax))
		xmax = plotScaleXMax;

	if (ymin > ymax)
		plotterSwapVals(&ymin, &ymax);
	else if (ymin == ymax)
		ymax += 0.1;
	if (xmin > xmax)
		plotterSwapVals(&xmin, &xmax);
	else if (xmin == xmax)
		xmax += 0.1;
	//fprintf(stderr, "plotter: nvals: %d, ymin: %f, ymax: %f, xmin: %f, xmax: %f, ttl: %s\n", nvals, ymin, ymax, xmin, xmax, title);

	pladv(0);								// advance to new graph page
	plvpor(mleft, mright, mbot, mtop);		// set suitable margins to allow for axis labels & legend on right side
	plwind(xmin, xmax, ymin, ymax);			// define graph window extents
	plschr(0.0, 0.5);						// scale fonts of labels
	plcol0(1);								// use fg color for grid lines and labels
	if (title != 0)
		plmtex("t", 3.0, 0.5, 0.5, title);	// graph title
	plbox("uwginst", 0.0, 0, "uwginst", 0.0, 0);	// define graph box (frame, tick marks, labels)
	plschr(0.0, 1.0);						// reset scale of fonts

#endif
}

void plotterLine(const int nrec, const int nval, const double xVals[], const double yVals[], const char *label) {
#ifdef HAS_PLPLOT
	static int nextn = plotValsPerPage;
	static int cmap0color = plotStartColor;
	double ymin, ymax, xmin, xmax;
	int r, g, b;
	double a;

	if (++plotCurrValIdx >= plotValsPerPage)
		plotCurrValIdx = 0;
	// start a new graph for each new set of values
	if (plotCurrValIdx == 0) {
		nextn = std::min(plotValsPerPage, plotNumValues - nval);
		ymin = *std::min_element(plotMinYValues + nval, plotMinYValues + nval + nextn);
		ymax = *std::max_element(plotMaxYValues + nval, plotMaxYValues + nval + nextn);
		xmin = *std::min_element(plotMinXValues + nval, plotMinXValues + nval + nextn);
		xmax = *std::max_element(plotMaxXValues + nval, plotMaxXValues + nval + nextn);
		cmap0color = plotStartColor;
		plotterNewPage(nextn, ymin, ymax, xmin, xmax);
	}
	else if (++cmap0color >= plotMaxColors) {
		cmap0color = 1;
	}

	// set line color transparency gradient
	if (!plotNoAlpha) {
		plgcol0a(cmap0color, &r, &g, &b, &a);
		a = 1.0 - 0.8 / (float)nextn * (float)plotCurrValIdx;
		plscol0a(cmap0color, r, g, b, a);
		//fprintf(stderr, "plotter: i: %d, cmap0color: %d, a: %f\n", plotCurrValIdx, cmap0color, a);
	}

	plotColors[plotCurrValIdx] = cmap0color;
	plotValueLabels[plotCurrValIdx] = label;
	plotLegendOptions[plotCurrValIdx] = PL_LEGEND_NONE;
	plcol0(cmap0color);
	plline(nrec, (PLFLT *)xVals, (PLFLT *)yVals);

	if (plotCurrValIdx == nextn - 1)
		plotterEndPage();

#else
	fprintf(stderr, "plotter: error -- no plotting library available\n");
#endif
}

void plotterEndPage(void) {
#ifdef HAS_PLPLOT
	PLFLT legend_width, legend_height;
	float txtpos;
	char buff[100];
	int r, g, b, i;

	if (!plotNoLegend) {
		if (plotLegendOnTop) {
			txtpos = 1.0 / (float)(plotValsPerCurrPg + 1);
			plschr(0.0, 0.5);					// scale fonts of labels
			for (i=0; i < plotValsPerCurrPg; i++) {
				plcol0(plotColors[i]);
				plmtex("t", 3.0, txtpos * (i+1), 0.5, plotValueLabels[i]);
			}
			plschr(0.0, 1.0);					// reset scale of fonts
		}
		else {
			pllegend( &legend_width, &legend_height,
				PL_LEGEND_BACKGROUND | PL_LEGEND_BOUNDING_BOX,	// plotOptions
				PL_POSITION_RIGHT | PL_POSITION_OUTSIDE,		// position
				0.03, 0.0, 0.0,							// x offset, y offset, plot_width
				0, 1, 1, 0, 0,							// bg_color, bb_color, bb_style,  nrow, ncolumn
				plotValsPerCurrPg, plotLegendOptions,	// num legend items, opt_array
				0.0, 0.5, 1.0, 0.,  					// text offset, scale, spacing, justification
				plotColors, plotValueLabels,			// legend colors array, titles array
				NULL, NULL, NULL, NULL,					// box colors, patterns, scales, line_widths
				NULL, NULL, NULL,						// line colors, styles, widths
				NULL, NULL, NULL, NULL 					// symbol colors, scales, numbers, symbols
			);
		}
	}
	// reset color transparencies
	if (!plotNoAlpha) {
		for (i=1; i < plotValsPerCurrPg; i++) {
			plgcol0(plotColors[i], &r, &g, &b);
			plscol0a(plotColors[i], r, g, b, 1.0);
		}
	}

	free(plotColors);
	free(plotLegendOptions);
	plotColors = NULL;
	plotLegendOptions = NULL;
#endif
}

void plotterEnd(void) {
#ifdef HAS_PLPLOT
	plend();
#endif
}

