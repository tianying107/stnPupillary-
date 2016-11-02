//
//  stnImgOperaters.h
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#ifndef stnImgOperaters_h
#define stnImgOperaters_h


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "functions.h"

#define stnRed (double color[3]={1,0,0})

typedef struct {
    int row;
    int col;
} stnPoint;

void filterBlobWithLabel(int **labelImage, int nrows, int ncols, int label);
void stnMedianFilter(int **inputImg, int nrows, int ncols, int filterWidth, int filterHeight);
void stnFindCentral(int **inputImg, int nrows, int ncols, stnPoint *centerPoint);
void stnBoundaryPoint(int **inputImg, int nrows, int ncols, stnPoint *centerPoint, stnPoint *leftPoint, stnPoint *rightPoint);
void stnContourBound(int **inputImg, int nrows, int ncols, stnPoint *leftPoint, stnArray *directionArray, stnArray *contourMapRow, stnArray *contourMapCol);
double *stnCurvature(stnArray *directionArray, int windowSize);
void stnSafePoints(stnArray *contourRows, stnArray *contourCols, stnArray *breakPoints, stnPoint *rightPoint, stnArray *safeRows, stnArray *safeCols);
void stnEllipseFitting(stnArray *pointRows, stnArray *pointCols, stnPoint *centerPoint, int parameters[5]);
void stnCircleFitting(stnArray *pointRows, stnArray *pointCols, int parameters[3]);
void stnCirclePoints(stnArray *pointRows, stnArray *pointCols, int parameters[3]);
void stnDrawPoints(stnArray *pointRows, stnArray *pointCols, unsigned char **inputImg, int nrows, int ncols, double **outputImg);
void stnDrawColorPoints(stnArray *pointRows, stnArray *pointCols, double **inputImg, int nrows, int ncols, double color[3]);
void stnGray2RGB(double **inputImg, int nrows, int ncols, double **outputImg);

void growthCircle(stnPoint *centerPoint, int **inputImg, int nrows, int ncols);
#endif /* stnImgOperaters_h */
