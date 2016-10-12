//
//  stnImgOperaters.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include "stnImgOperaters.h"
#include <stdlib.h>
extern void flood_fill(int ,int ,int , int **, int , int);
extern void **matrix(int, int, int, int, int);
static int count;
void imageSplit(unsigned char **combinedImg, int nrows, int newWidth, unsigned char **img1, unsigned char **img2){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<newWidth; j++) {
            img1[i][j] = combinedImg[i][j];
            img2[i][j] = combinedImg[i][j+newWidth];
        }
    }
}

void imageHistogramEqualization(unsigned char **inputImg, int nrows, int ncols, int lower, int upper, double **outputImg){
    int i, j;
    int Num = 0;
    //calculate the hist
    double h[256];
    for (i=0; i<256; i++) {
        h[i]=0;
    }
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            int va;
            va=inputImg[i][j];
            if ((va>=lower)&&(va<=upper)) {
                h[va]+=1;
                Num+=1;
            }
        }
    }
    
    //calculate the smallest gray level
    int XL=0;
    double H[256];
    for (i=lower; i<upper+1; i++) {
        if (i==lower) {
            H[i]=h[i];
        }
        else{
            H[i]=H[i-1]+h[i];
        }
        if (H[i]==0) {
            XL=i;
        }
    }
    //Map to new gray level
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            int va;
            va=inputImg[i][j];
            if (va<lower) {
                outputImg[i][j]=0;
            }
            else if (va>upper){
                outputImg[i][j]=255;
            }
            else{
                outputImg[i][j]=(H[va]-H[XL])/((double)Num-H[XL]);
            }
        }
    }
}

void imageThreshold(double **inputImg, int nrows, int ncols, double threshold, int **binearImg){
    int i, j;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (inputImg[i][j]<threshold) {
                binearImg[i][j]=0;
            }
            else{
                binearImg[i][j]=1;
            }
        }
    }
}

/**
 *Version 1.0
 *connectivityLabel is a function to isolate blobs and label from the input image using 8-neighbour connectivity analysis
 *inputImg is the input binary image
 *labeledImg is the output image. Each blob in the image was labeled. The label start from 1.
 *This function return an int maxLabel, which indicate the largest area of blob.
 *
 */
int connectivityLabel(int **inputImg, int nrows, int ncols, int **labeledImg){
    int i, j, label, labelCount,maxLabel,maxCount;
    
    //set pixel 1 to -1, set pixel 0 to 0 as unlabeled pixel
    labelCount = 0;
    maxLabel=0;
    maxCount=0;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (inputImg[i][j]==1) {
                labeledImg[i][j]=-1;
            }
            else {
                labeledImg[i][j]=0;
                labelCount++;
            }
        }
    }

    //search unlabeled pixels
    label=1;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (labeledImg[i][j]==0) {
                count=0;
                //flood fill unlabeled pixels
                flood_fill(i,j,label,labeledImg,nrows,ncols);
                if (count>maxCount) {
                    maxCount = count;
                    maxLabel = label;
                }
                label++;
            }
        }
    }

    return maxLabel;
    
}

/**
 *flood_fill algorithm, here use 6-neighbour connectivity analysis to avoid block
 *A static variable count to count the number of current label
 */
void flood_fill(int x,int y,int label, int **labelImg, int nrows, int ncols){
    labelImg[x][y]=label;
    count++;
    
    if(x>0&&labelImg[x-1][y]==0)flood_fill(x-1,y,label,labelImg,nrows,ncols);
    if(y>0&&labelImg[x][y-1]==0)flood_fill(x,y-1,label,labelImg,nrows,ncols);
    if(y<ncols-1&&labelImg[x][y+1]==0)flood_fill(x,y+1,label,labelImg,nrows,ncols);
    if(y>0&&x<nrows-1&&labelImg[x+1][y-1]==0)flood_fill(x+1,y-1,label,labelImg,nrows,ncols);
    if(x<nrows-1&&labelImg[x+1][y]==0)flood_fill(x+1,y,label,labelImg,nrows,ncols);
    if(y<ncols-1&&x<nrows-1&&labelImg[x+1][y+1]==0)flood_fill(x+1,y+1,label,labelImg,nrows,ncols);
    
}

/**
 *filterBlobWithLabel is a function can pick up the blob in a well labeled image using a specific label
 *labelImage is a well labeled image, with nrows rows and ncols column. It is both input and output.
 *label is the target label
 *
 *The target area is 0(black), background is 1(white).
 */
void filterBlobWithLabel(int **labelImage, int nrows, int ncols, int label){
    int i,j;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (labelImage[i][j]==label) labelImage[i][j]=0;
            else labelImage[i][j]=1;
        }
    }
}
/**
 *stnMedianFilter is a 2-D median filter that only work on binary image
 */
void stnMedianFilter(int **inputImg, int nrows, int ncols, int filterWidth, int filterHeight){
    int edgeX, edgeY, i, j, filterX, filterY, sum;
    int **filteredImg;
    
    filteredImg = inputImg;
    edgeX = (filterWidth-1)/2;
    edgeY = (filterHeight-1)/2;
    for (i=edgeY; i<nrows-edgeY; i++) {
        for (j=edgeX; j<ncols-edgeX; j++) {
            sum = 0;
            for (filterY=i-edgeY; filterY<i+edgeY+1;filterY++) {
                for (filterX = j-edgeX; filterX<j+edgeX+1; filterX++) {
                    sum += inputImg[filterY][filterX];
                }
            }
            filteredImg[i][j]=(int)(sum/ceil((((float)(filterWidth*filterHeight))/2)));
        }
    }
    inputImg = filteredImg;
}

/**
 *stnFindCentral
 */
void stnFindCentral(int **inputImg, int nrows, int ncols, stnPoint *centerPoint){
    double count = 0;
    int i,j;
    int x=0;
    int y=0;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (!inputImg[i][j]) {
                y+=i;
                x+=j;
                count++;
            }
        }
    }
    centerPoint->col = round(((double)x)/count);
    centerPoint->row = round(((double)y)/count);
    printf("center at row:%d col:%d\n",centerPoint->row,centerPoint->col);
}

/**
 *stnBoundaryPoints
 */
void stnBoundaryPoint(int **inputImg, int nrows, int ncols, stnPoint *centerPoint, stnPoint *leftPoint, stnPoint *rightPoint){
    int x = centerPoint->col;
    int y = centerPoint->row;
    int windowSize = 30;
    //any holes within the pupil blob were classified as a corneal relection if the hole width was less than this value
    //true point at here is black (i.e. the value of center point is 0)
    
    /**left boundary point*/
    int sum = 0;
    for (int i=0; i<windowSize && x-i>=0; i++) {
        sum += inputImg[y][x-i];
    }
    //sum < windowSize means there is at least one point in the window is black
    while (sum<windowSize && x) {
        x--;
        sum=0;
        for (int i=0; (i<windowSize && x-i>=0); i++) {
            sum += inputImg[y][x-i];
        }
    }
    leftPoint->col = x+1;
    leftPoint->row = y;

    /**right boundary point*/
    sum = 0;
    for (int i=0; i<windowSize && x+i<ncols; i++) {
        sum += inputImg[y][x+i];
    }
    while (sum<windowSize && x) {
        x++;
        sum=0;
        for (int i=0; i<windowSize && x+i<ncols; i++) {
            sum += inputImg[y][x+i];
        }
    }
    rightPoint->col = x-1;
    rightPoint->row = y;
}

/**
 *stnContourBound
 */
void stnContourBound(int **inputImg, int nrows, int ncols, stnPoint *leftPoint, stnArray *directionArray, stnArray *contourMapRow, stnArray *contourMapCol){
    int mapPlus2[8]={2,3,4,5,6,7,0,1};
    int mapMinus1[8]={7,0,1,2,3,4,5,6};
    int direction = 1; //initial direction is 1
    int x = leftPoint->col;
    int y = leftPoint->row;
    initStnArray(directionArray, 1);
    initStnArray(contourMapRow, 1);
    initStnArray(contourMapCol, 1);
    
    int map[16] = {y-1,x+1,y-1,x,y-1,x-1,y,x-1,y+1,x-1,y+1,x,y+1,x+1,y,x+1};
    direction = mapPlus2[direction];
    for (int j=0; j<8; j++) {
        if (inputImg[map[direction*2]][map[direction*2+1]]==1) {
            direction = mapMinus1[direction];
        }
        else{
            
            insertStnArray(directionArray, direction);
            insertStnArray(contourMapRow, y);
            insertStnArray(contourMapCol, x);
            y = map[direction*2];
            x = map[direction*2+1];
            break;
        }
    }
    
    while ((y-1) && (x-1) && (y+1<nrows) && (x+1<ncols) && (x!=leftPoint->col || y!=leftPoint->row)) {
        int map[16] = {y-1,x+1,y-1,x,y-1,x-1,y,x-1,y+1,x-1,y+1,x,y+1,x+1,y,x+1};
        direction = mapPlus2[direction];
        for (int j=0; j<8; j++) {
            if (inputImg[map[direction*2]][map[direction*2+1]]==1) {
                direction = mapMinus1[direction];
            }
            else{
                insertStnArray(directionArray, direction);
                insertStnArray(contourMapRow, y);
                insertStnArray(contourMapCol, x);
                y = map[direction*2];
                x = map[direction*2+1];
                break;
            }
        }
    }

}
/**
 *stnCurvature
 */
double *stnCurvature(stnArray *directionArray, int windowSize){
    int circleMap[8][8] = {{0,1,2,3,4,-3,-2,-1},
                            {-1,0,1,2,3,4,-3,-2},
                            {-2,-1,0,1,2,3,4,-3},
                            {-3,-2,-1,0,1,2,3,4},
                            {4,-3,-2,-1,0,1,2,3},
                            {3,4,-3,-2,-1,0,1,2},
                            {2,3,4,-3,-2,-1,0,1},
                            {1,2,3,4,-3,-2,-1,0}};
    int length =(int)directionArray->used;

    
//    double curvature[directionArray->used];
    double *curvature = (double *)malloc(length*sizeof(double));
    
    stnArray newDirection;
    initStnArray(&newDirection, 1);
    for (int i=-(windowSize-1); i<length+windowSize-1; i++) {
        if (i<0) {
            insertStnArray(&newDirection, directionArray->array[length+i]);
        }
        else if (i>=length){
            insertStnArray(&newDirection, directionArray->array[i-length]);
        }
        else insertStnArray(&newDirection, directionArray->array[i]);
    }
    for (int i=windowSize-1; i<length+windowSize-1; i++) {
        int alp1=0, alp2=0;
        for (int j=i-windowSize+1; j<i+1; j++) {
            alp1 += newDirection.array[j];
        }
        for (int j=i; j<i+windowSize; j++) {
            alp2 += newDirection.array[j];
        }
        curvature[i-windowSize+1]=stnInterp2(8, 8, circleMap, ((double)alp1)/windowSize, ((double)alp2)/windowSize);
    }
    return curvature;
}





