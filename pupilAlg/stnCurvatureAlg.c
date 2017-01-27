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
//    unsigned char **imgQ1 = (unsigned char **)matrix(nrows/2, ncols/2, 0, 0, sizeof(unsigned char));

//    clock_t begin = clock();
    imageHistogramEqualization(inputImg, nrows, ncols, lower, upper, interImg);
// //    dynamicHistogramEqualization(inputImg, nrows, ncols, lower, upper, 20, interImg);
// //    clock_t end = clock();
// //    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
// //    printf("equalization spend %f seconds\n",time_spent);
//    imgChar2Double(inputImg, nrows, ncols, interImg);
//    int **binearImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
//    if (side==1) {
//        imageThreshold(interImg, nrows, ncols, (double)0.37, binearImg);//best:0.19
//    }
//    else{
//        imageThreshold(interImg, nrows, ncols, (double)0.18, binearImg);//best:0.19
//    }
    int **binearImg1=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    imageThreshold(interImg, nrows, ncols, (double)0.19, binearImg1);//best:0.19

    
    /*****
     *****DYNAMIC THRESHOLD
     *****
     ***** get histogram first
     ***** 3 moving smooth second
     ***** get inverse
     ***** make decision based on number of peaks and the location of peaks
     *****/
    imgChar2Double(inputImg, nrows, ncols, interImg);
    int **binearImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    double *histogram = imageHistogram(inputImg, nrows, ncols);
    histogram = movingWindowSmooth(histogram, 3, 256);
    for (int h=0; h<256; h++) {
        histogram[h] = 1 - histogram[h];
//        printf("%f\n",histogram[h]);
    }
    stnArray histogramPeaks;
    initStnArray(&histogramPeaks, 1);
    detect_peak(histogram, 256, &histogramPeaks, 0.001, 0, 0);
//    for (i = 0; i<(int)histogramPeaks.used; i++) {
//        printf("%d\n",histogramPeaks.array[i]);
//    }
    int decisionIndex=0;
    int candicate1, candicate2;
    if ((int)histogramPeaks.used > 1){
        if ((int)histogramPeaks.used > 2 && histogramPeaks.array[2]-histogramPeaks.array[1]<11){
            if (histogram[histogramPeaks.array[2]]>histogram[histogramPeaks.array[1]]) {
                candicate2 = histogramPeaks.array[2];
            }
            else{
                candicate2 = histogramPeaks.array[1];
            }
//            candicate2 = histogramPeaks.array[2];
//            printf("candidate1:%d,candidate2:%d\n",histogramPeaks.array[1],histogramPeaks.array[2]);
        }
        else candicate2 = histogramPeaks.array[1];
        candicate1 = histogramPeaks.array[0];
        decisionIndex = candicate2;
//        printf("%d,%d\n",candicate1,candicate2);
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
//    printf("%d\n",decisionIndex);
    double dynamicThreshold = ((double)decisionIndex+2)/256;
    imageThreshold(interImg, nrows, ncols, dynamicThreshold, binearImg);//best:0.19
    
    
    
    
    
    if (!dynamicThresholdMode) {
        binearImg = binearImg1;
    }
    
    
    /*Median filter the result*/
//    begin = clock();
    stnMedianFilter(binearImg, nrows, ncols, 21, 21);//21
//    end = clock();
//    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
//    printf("median filter spend %f seconds\n",time_spent);
    
    /*stnTricky clear*/
//    for (i=0; i<30; i++) {
//        for (j=0; j<30; j++) {
//            binearImg[i][j]=1;
//            binearImg[i][ncols-j]=1;
//            binearImg[nrows-i-1][j]=1;
//            binearImg[nrows-i-1][ncols-j]=1;
//        }
//    }
    for (i=0; i<nrows; i++) {
        for (j=0; j<30; j++) {
            binearImg[i][j]=1;
            binearImg[i][ncols-j]=1;
        }
    }
    
    
    
    
    
    
    
    
    /*find the blob central*/
    stnPoint centerPoint;
    if (allParameters[4]==0||allParameters[5]==0) {
        stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    }else{
        centerPoint.col=allParameters[4];
        centerPoint.row=allParameters[5];
    }
//    printf("%d , %d \n",centerPoint.row,centerPoint.col);
    
    /*grow_circle*/
//    begin = clock();
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
    growthCircle(&centerPoint, binearImg, nrows, ncols,allParameters[6]);
    stnFindCentral(binearImg, nrows, ncols, &centerPoint);
//    printf("%d , %d \n",centerPoint.row,centerPoint.col);
//    printf("point=%d\n",binearImg[centerPoint.row][centerPoint.col]);
//    end = clock();
//    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
//    printf("blob spend %f seconds\n",time_spent);
    

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
        stnSafePoints(&contourMapRow, &contourMapCol, &peaks, &rightPoint, &safeRows, &safeCols);
    }
    else{
        for (i=0; i<(int)contourMapCol.used; i++) {
            insertStnArray(&safeRows, contourMapRow.array[i]);
            insertStnArray(&safeCols, contourMapCol.array[i]);
        }
    }
    
        
    
    
    
    /*******Ellipse*******/
    /*ellipse fitting*/
    double ellipseParameters[6];
//    begin = clock();
    stnEllipseFitting(&safeRows, &safeCols, &centerPoint,ellipseParameters);
    double ellipse[3];
    ellipse[0]=ellipseParameters[0];ellipse[1]=ellipseParameters[1];ellipse[2]=ellipseParameters[4];
//    end = clock();
//    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    /*ellipse points*/
    stnArray ellipseRows, ellipseCols, averageRows, averageCols;
    initStnArray(&ellipseCols, 1);
    initStnArray(&ellipseRows, 1);
    initStnArray(&averageRows, 1);
    initStnArray(&averageCols, 1);
    
    
    /*******Circle*******/
    /*circle fitting*/
    double parameters[3];
    stnCircleFitting(&safeRows, &safeCols, parameters);

//    stnMLSCircleFitting(&safeRows, &safeCols, parameters);

    
    
    
    /**compare parameters from fix threshold and dynamic threshold**/
    
    
    
    
    
    
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
    
    /*******get circle points**********/
//    begin = clock();
    stnCirclePoints(&averageRows, &averageCols, ellipse);
    stnCirclePoints(&circleRows, &circleCols, parameters);
    stnEllipsePoints(&ellipseRows, &ellipseCols, ellipseParameters);
//    end = clock();
//    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

    
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
    allParameters[0]=ellipseParameters[1];
    allParameters[1]=ellipseParameters[0];
    allParameters[2]=ellipseParameters[2];
    allParameters[3]=ellipseParameters[3];
    allParameters[4]=parameters[1];
    allParameters[5]=parameters[0];
    allParameters[6]=parameters[2];
    
    free(histogram);
}
