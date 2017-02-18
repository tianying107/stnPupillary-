//
//  stnCurvatureAlg.c
//  pupilAlg
//
//  Created by st chen on 16/10/5.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//
#include <time.h>
#include "stnCurvatureAlg.h"
#include "stnImgOperaters.h"

extern void imageHistogramEqualization(unsigned char **, int , int , int , int , double **);
extern void imageThreshold(double **, int , int , double, int **);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void imgInt2Char(int **, int , int , unsigned char **);
extern int connectivityLabel(int **, int , int , int **);

void stnCurvaturePro(unsigned char **inputImg, int nrows, int ncols, double **outputImg, double **outputppm, double allParameters[7],int side){

    int i,j;
    unsigned char **y;
    y = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
    
    double **interImg=outputImg;
    
    
    
    bool binearyOutput = false;
    bool dynamicThresholdMode = true;
    bool useSafePointOnly = true;
    bool drawSafePoint = true;
    /*****
     *****
     *****HISTOGRAM EQULIZATION AND FIX THRESHOLD SECTION*****
     *****
     *****/
    
    int lower=0; //EQULIZATION LOWER BOUND
    int upper=255;  //EQULIZATION HIGHER BOUND
    imageHistogramEqualization(inputImg, nrows, ncols, lower, upper, interImg);
    int **binearImg1=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    imageThreshold(interImg, nrows, ncols, (double)0.19, binearImg1);//best:0.19

    
    /*****
     *****DYNAMIC THRESHOLD
     ***** version 1.1
     ***** get histogram first
     ***** 3 moving smooth second
     ***** get inverse
     ***** make decision based on number of peaks and the location of peaks
     *****/
    imgChar2Double(inputImg, nrows, ncols, interImg);
    int **binearImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    double *rHistogram = imageHistogram(inputImg, nrows, ncols);
    double *histogram = movingWindowSmooth(rHistogram, 3, 256);
    free(rHistogram);
    for (int h=0; h<256; h++) {
        histogram[h] = 1 - histogram[h];
    }
    stnArray histogramPeaks;
    initStnArray(&histogramPeaks, 1);
    detect_peak(histogram, 256, &histogramPeaks, 0.001, 0, 0);

    int decisionIndex=0;
    int candicate1, candicate2;
    if ((int)histogramPeaks.used > 1){
        if ((int)histogramPeaks.used > 2 && histogramPeaks.array[2]-histogramPeaks.array[1]<11){
            if (histogram[histogramPeaks.array[2]]>histogram[histogramPeaks.array[1]]) {
                candicate2 = histogramPeaks.array[2];
            }
            else candicate2 = histogramPeaks.array[1];
        }
        else candicate2 = histogramPeaks.array[1];
        candicate1 = histogramPeaks.array[0];
        decisionIndex = candicate2;
        double checkSum = 0;
        for (i=0; i<min(candicate2+1, 256); i++) {
            checkSum += 1-histogram[i];
        }
        if (checkSum>0.5) {
            decisionIndex = candicate1;
        }
    }else if ((int)histogramPeaks.used == 1){
        decisionIndex = histogramPeaks.array[0];
    }
    else{
        decisionIndex = 0;
    }
//    printf("decision1.0: %d\n",decisionIndex);
    double dynamicThreshold = ((double)decisionIndex+4)/256;
    imageThreshold(interImg, nrows, ncols, dynamicThreshold, binearImg);//best:0.19
    
    /*****
     *****DYNAMIC THRESHOLD
     ***** version 2.0
     *****/
    stnPoint tempCenterPoint;
    tempCenterPoint.col = ncols/2;
    tempCenterPoint.row = nrows/2;
    double threshold = stnDynamicThreshold2(inputImg, nrows, ncols, &tempCenterPoint);
//    imageThreshold(interImg, nrows, ncols, threshold, binearImg);
    
    
    
    if (!dynamicThresholdMode) {
        binearImg = binearImg1;
    }
    
    
    /*Median filter the result*/
    stnMedianFilter(binearImg, nrows, ncols, 21, 21);//21

    
    /*stnTricky clear*/
//    for (i=0; i<nrows; i++) {
//        for (j=0; j<30; j++) {
//            binearImg[i][j]=1;
//            binearImg[i][ncols-j]=1;
//        }
//    }
    
    
    
    
    
    
    
    
    /*find the blob central*/
    stnPoint centerPoint;
    if (allParameters[4]==0||allParameters[5]==0) {
        stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    }else{
        centerPoint.col=allParameters[4];
        centerPoint.row=allParameters[5];
    }

    /*grow_circle*/
    bool invalidCenter=false;
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    if (centerPoint.row == -3340 && centerPoint.col == -3012)
        invalidCenter = true;
    
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    if (centerPoint.row == -3340 && centerPoint.col == -3012)
        invalidCenter = true;
    
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    if (centerPoint.row == -3340 && centerPoint.col == -3012)
        invalidCenter = true;
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    if (invalidCenter) {
        allParameters[0]=0;
        allParameters[1]=0;
        allParameters[2]=0;
        allParameters[3]=0;
        allParameters[4]=0;
        allParameters[5]=0;
        allParameters[6]=0;
        
        return;
    }
    
    //    printf("%d , %d \n",centerPoint.row,centerPoint.col);
    

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
    double *curvature = stnCurvature(&directionArray, 15);//15
    
    /*peaks and break points*/
    stnArray peaks;
    initStnArray(&peaks, 1);
    detect_peak(curvature, (int)directionArray.used, &peaks, 1.1, 0.3, 0);//0.3,0.3//0.3,0.73
    /*safe points index, the index of contourMapRow and contourMapCol*/
    stnArray safeRows, safeCols;
    initStnArray(&safeRows, 1);
    initStnArray(&safeCols, 1);
    if (useSafePointOnly) {
        stnSafePoints(&contourMapRow, &contourMapCol, &directionArray,&peaks, &rightPoint, &safeRows, &safeCols);
    }
    else{
        for (i=0; i<(int)contourMapCol.used; i++) {
            insertStnArray(&safeRows, contourMapRow.array[i]);
            insertStnArray(&safeCols, contourMapCol.array[i]);
        }
    }
    
    /*******Circle*******/
    /*circle fitting*/
    double *parameters = malloc(3*sizeof(double));
    stnCircleFitting(&safeRows, &safeCols, parameters);
    centerPoint.col=parameters[1];
    centerPoint.row=parameters[0];
    //    stnMLSCircleFitting(&safeRows, &safeCols, parameters);
    /*******Ellipse*******/
    /*ellipse fitting*/
    double *ellipseParameters = malloc(6*sizeof(double));
    for (i=0; i<6; i++) {
        ellipseParameters[i]=0;
    }
    stnEllipseFitting(&safeRows, &safeCols, &centerPoint,ellipseParameters);
    double *ellipse = malloc(3*sizeof(double));
    ellipse[0]=ellipseParameters[0];ellipse[1]=ellipseParameters[1];ellipse[2]=ellipseParameters[4];
    
    
    /*ellipse points*/
    stnArray ellipseRows, ellipseCols, averageRows, averageCols;
    initStnArray(&ellipseCols, 1);
    initStnArray(&ellipseRows, 1);
    initStnArray(&averageRows, 1);
    initStnArray(&averageCols, 1);

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
//        printf("%f, %f, %f\n",parameters[1],parameters[0],parameters[2]);
        printf("invalid center point.\n");
    }
    if (fabs(parameters[0])>max(nrows, ncols) || fabs(parameters[1])>max(nrows, ncols) || fabs(parameters[2])>max(nrows, ncols)) {
        parameters[1]=0;
        parameters[0]=0;
        parameters[2]=0;
    }
    if ( isnan(ellipseParameters[0]) ||  isnan(ellipseParameters[1]) ||  isnan(ellipseParameters[2]) ||  isnan(ellipseParameters[3])) {
        ellipseParameters[1]=0;
        ellipseParameters[0]=0;
        ellipseParameters[2]=0;
        ellipseParameters[3]=0;
    }
    
    /*******get circle points**********/
    stnCirclePoints(&averageRows, &averageCols, ellipse);
    stnCirclePoints(&circleRows, &circleCols, parameters);
    stnEllipsePoints(&ellipseRows, &ellipseCols, ellipseParameters);

    
    /*convert grayscale to rgb using equalized image*/
    if (!(int)safeRows.used || binearyOutput) {
        imgInt2Double(binearImg, nrows, ncols, interImg);
    }
    else
        imgChar2Double(inputImg, nrows, ncols, interImg);
    stnGray2RGB(interImg, nrows, ncols, outputppm);
    
    
    /*draw the circle*/
    double redColor[3] = {1,0,0};
    double blueColor[3] = {0,0,1};
    double yellowColor[3] = {1,1,0};
    stnDrawColorPoints(&circleRows, &circleCols, outputppm, nrows, ncols, redColor);
    stnDrawColorPoints(&averageRows, &averageCols, outputppm, nrows, ncols, blueColor);
    stnDrawColorPoints(&ellipseRows, &ellipseCols, outputppm, nrows, ncols, yellowColor);
    if (drawSafePoint) {
        double greenColor[3] = {0.2,1,0.2};
        stnDrawColorPoints(&safeRows, &safeCols, outputppm, nrows, ncols, greenColor);
    }
    // stnDrawPoints(&circleRows, &circleCols, inputImg, nrows, ncols, outputImg);

    
    
    
    allParameters[0]=ellipseParameters[1];
    allParameters[1]=ellipseParameters[0];
    allParameters[2]=ellipseParameters[2];
    allParameters[3]=ellipseParameters[3];
    allParameters[4]=parameters[1];
    allParameters[5]=parameters[0];
    allParameters[6]=parameters[2];
    
    
    //release memory
    freeStnArray(&directionArray);
    freeStnArray(&contourMapRow);
    freeStnArray(&contourMapCol);
    freeStnArray(&peaks);
    freeStnArray(&safeRows);
    freeStnArray(&safeCols);
    freeStnArray(&circleRows);
    freeStnArray(&circleCols);
    freeStnArray(&averageRows);
    freeStnArray(&averageCols);
    freeStnArray(&ellipseRows);
    freeStnArray(&ellipseCols);
    
    freeStnMatrix((void**)y);
    freeStnMatrix((void**)binearImg);
    freeStnMatrix((void**)binearImg1);

    free(histogram);
    free(curvature);
    free(parameters);
    free(ellipseParameters);
    
    
}
