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

extern void imageHistogramEqualization(unsigned char **, int , int , int , int , double **);
extern void imageThreshold(double **, int , int , double, int **);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void imgInt2Char(int **, int , int , unsigned char **);
extern int connectivityLabel(int **, int , int , int **);

void stnCurvaturePro(unsigned char **inputImg, int nrows, int ncols, double **outputImg, double **outputppm, int allParameters[7]){

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
    imageThreshold(interImg, nrows, ncols, (double)0.16, binearImg);
    
    /*Median filter the result*/
    stnMedianFilter(binearImg, nrows, ncols, 21, 21);
    
    /*stnTricky clear*/
    for (i=0; i<nrows; i++) {
        for (j=0; j<100; j++) {
            binearImg[i][j]=1; 
        }
    }
    
    /*find the blob central*/
    stnPoint centerPoint;
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    
    /*grow_circle*/
    growthCircle(&centerPoint, binearImg, nrows, ncols);
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    

    /*left point and right point*/
    stnPoint leftPoint, rightPoint;
    stnBoundaryPoint(binearImg, nrows, ncols, &centerPoint, &leftPoint, &rightPoint);
    
    /*contour*/
    stnArray directionArray,contourMapRow,contourMapCol;
    initStnArray(&directionArray, 1);
    initStnArray(&contourMapRow, 1);
    initStnArray(&contourMapCol, 1);
    stnContourBound(binearImg, nrows, ncols, &leftPoint,&directionArray,&contourMapRow,&contourMapCol);
    
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
    
    
    /*******Ellipse*******/
    /*ellipse fitting*/
    int ellipseParameters[5];
    stnEllipseFitting(&safeRows, &safeCols, &centerPoint,ellipseParameters);
    int ellipse[3];
    ellipse[0]=ellipseParameters[0];ellipse[1]=ellipseParameters[1];ellipse[2]=ellipseParameters[4];
    /*ellipse points*/
    stnArray ellipseRows, ellipseCols;
    initStnArray(&ellipseCols, 1);
    initStnArray(&ellipseRows, 1);
    
    
    /*******Circle*******/
    /*circle fitting*/
    int parameters[3];
    stnCircleFitting(&safeRows, &safeCols, parameters);
    /*circle points*/
    stnArray circleRows, circleCols;
    initStnArray(&circleCols, 1);
    initStnArray(&circleRows, 1);
    
    
    /**********center point valid check*********/
    if (pow(ellipse[0]-parameters[0], 2)+pow(ellipse[1]-parameters[1], 2)>min(pow(ellipse[2],2), pow(parameters[2],2))) {
        ellipseParameters[1]=0;
        ellipseParameters[0]=0;
        ellipseParameters[2]=0;
        ellipseParameters[3]=0;
        parameters[1]=0;
        parameters[0]=0;
        parameters[2]=0;
    }
    
    /*******get circle points**********/
    stnCirclePoints(&ellipseRows, &ellipseCols, ellipse);
    stnCirclePoints(&circleRows, &circleCols, parameters);
    
    
    
    /*convert grayscale to rgb using equalized image*/
    if (!(int)safeRows.used) {
        imgInt2Double(binearImg, nrows, ncols, interImg);
    }
    stnGray2RGB(interImg, nrows, ncols, outputppm);
    
    
    /*draw the circle*/
    double redColor[3] = {1,0,0};
    double blueColor[3] = {0,0,1};
    stnDrawColorPoints(&circleRows, &circleCols, outputppm, nrows, ncols, redColor);
    stnDrawColorPoints(&ellipseRows, &ellipseCols, outputppm, nrows, ncols, blueColor);
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

    
    allParameters[0]=ellipseParameters[1];
    allParameters[1]=ellipseParameters[0];
    allParameters[2]=ellipseParameters[2];
    allParameters[3]=ellipseParameters[3];
    allParameters[4]=parameters[1];
    allParameters[5]=parameters[0];
    allParameters[6]=parameters[2];
}
