//
//  stnImgOperaters.h
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#ifndef stnImgOperaters_h
#define stnImgOperaters_h

#include <stdio.h>
#include <math.h>
#include "functions.h"
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
void stnEllipseFitting(stnArray *pointRows, stnArray *pointCols);
#endif /* stnImgOperaters_h */
