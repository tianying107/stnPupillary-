//
//  stnCurvatureAlg.c
//  pupilAlg
//
//  Created by st chen on 16/10/5.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#include "stnCurvatureAlg.h"
#include "stnImgOperaters.h"
#include "functions.h"
extern void imageHistogramEqualization(unsigned char **, int , int , int , int , double **);
extern void imageThreshold(double **, int , int , double, int **);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void imgInt2Char(int **, int , int , unsigned char **);
extern int connectivityLabel(int **, int , int , int **);

void stnCurvaturePro(unsigned char **inputImg, int nrows, int ncols, double **outputImg){

    int i,j;
    unsigned char **y;
    y = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
    
    double **interImg=outputImg;
    /*HISTOGRAM EQULIZATION*/
    int lower=0; //EQULIZATION LOWER BOUND
    int upper=255;  //EQULIZATION HIGHER BOUND
    imageHistogramEqualization(inputImg, nrows, ncols, lower, upper, interImg);
    
    /*Threshold*/
    int **binearImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    imageThreshold(interImg, nrows, ncols, (double)0.15, binearImg);
    
    /*Largest area blob*/
    int **labelImg =(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    int maxLabel = connectivityLabel(binearImg, nrows, ncols, labelImg);
    filterBlobWithLabel(labelImg, nrows, ncols, maxLabel);
    
    /*Median filter the result*/
    stnMedianFilter(labelImg, nrows, ncols, 21, 21);
    
    /*find the blob central*/
    stnPoint centerPoint;
    stnFindCentral(labelImg, nrows, ncols, &centerPoint);
    
    /*left point and right point*/
    stnPoint leftPoint, rightPoint;
    stnBoundaryPoint(labelImg, nrows, ncols, &centerPoint, &leftPoint, &rightPoint);
//    printf("left point at row:%d col:%d\n",leftPoint.row, leftPoint.col);
//    printf("right point at row:%d col:%d\n",rightPoint.row, rightPoint.col);
    
    /*contour*/
    stnArray directionArray,contourMapRow,contourMapCol;
    initStnArray(&directionArray, 1);
    initStnArray(&contourMapRow, 1);
    initStnArray(&contourMapCol, 1);
    stnContourBound(labelImg, nrows, ncols, &leftPoint,&directionArray,&contourMapRow,&contourMapCol);
    
    /*Curvature*/
    double *curvature = stnCurvature(&directionArray, 15);
    
    /*peaks and break points*/
    stnArray peaks;
    initStnArray(&peaks, 1);
    detect_peak(curvature, (int)directionArray.used, &peaks, 0.3, 0.73);

    /*safe points index, the index of contourMapRow and contourMapCol*/
    stnArray safeRows, safeCols;
    initStnArray(&safeRows, 1);
    initStnArray(&safeCols, 1);
    stnSafePoints(&contourMapRow, &contourMapCol, &peaks, &rightPoint, &safeRows, &safeCols);
    
    
    
//    /*ellipse fitting*/
    stnEllipseFitting(&safeRows, &safeCols);
    
    /*circle fitting*/
    int parameters[3];
    stnCircleFitting(&safeRows, &safeCols, parameters);
//    printf("center: %d,%d, radius:%d\n",parameters[1],parameters[0],parameters[2]);

    /*circle points*/
    stnArray circleRows, circleCols;
    initStnArray(&circleCols, 1);
    initStnArray(&circleRows, 1);
    stnCirclePoints(&circleRows, &circleCols, parameters);
    
    /*draw the circle*/
    stnDrawPoints(&circleRows, &circleCols, inputImg, nrows, ncols, outputImg);

    //release memory
    freeStnArray(&directionArray);
    freeStnArray(&contourMapRow);
    freeStnArray(&contourMapCol);
    freeStnArray(&peaks);
    freeStnArray(&safeRows);
    freeStnArray(&safeCols);
    freeStnArray(&circleRows);
    freeStnArray(&circleCols);
    
//    imgDouble2Char(outputImg, nrows, ncols, y);
////    imgInt2Char(outputImg, nrows, ncols, y);
//        /* WRITE THE IMAGE */
//        fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
//        for(i = 0; i < nrows; i++)
//            if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
//                error("can't write the image");
//    
//        /* CLOSE FILE & QUIT */
//        fclose(fpy);
}
