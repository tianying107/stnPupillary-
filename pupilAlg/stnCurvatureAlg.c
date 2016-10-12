//
//  stnCurvatureAlg.c
//  pupilAlg
//
//  Created by st chen on 16/10/5.
//  Copyright © 2016年 Star Chen. All rights reserved.
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
    FILE *fpy;
    int i,j;
    unsigned char **y;
    y = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
    fpy = fopen("/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/output.pgm", "w");
    double **interImg=outputImg;
    /*HISTOGRAM EQULIZATION*/
    int lower=0; //EQULIZATION LOWER BOUND
    int upper=255;  //EQULIZATION HIGHER BOUND
    imageHistogramEqualization(inputImg, nrows, ncols, lower, upper, interImg);
    
    /*Threshold*/
    int **binearImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    imageThreshold(interImg, nrows, ncols, (double)0.23, binearImg);
    
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
    printf("left point at row:%d col:%d\n",leftPoint.row, leftPoint.col);
    printf("right point at row:%d col:%d\n",rightPoint.row, rightPoint.col);
    
    /*contour*/
    stnArray directionArray,contourMapRow,contourMapCol;
    initStnArray(&directionArray, 1);
    initStnArray(&contourMapRow, 1);
    initStnArray(&contourMapCol, 1);
    stnContourBound(labelImg, nrows, ncols, &leftPoint,&directionArray,&contourMapRow,&contourMapCol);
    
    /*Curvature*/
    double *curvature = stnCurvature(&directionArray, 15);
    printf("index:%d curvature:%f\n",587,curvature[587]);
    
    
    
    outputImg = interImg;
    
    
    
    
    
    
    
    imgInt2Char(labelImg, nrows, ncols, y);
        /* WRITE THE IMAGE */
        fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
        for(i = 0; i < nrows; i++)
            if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
                error("can't write the image");
    
        /* CLOSE FILE & QUIT */
        fclose(fpy);
}
