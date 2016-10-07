//
//  stnImgOperaters.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include "stnImgOperaters.h"

extern void flood_fill(int ,int ,int , int **, int , int);
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
void stnFindCentral(int **inputImg, int nrows, int ncols, int *centerX, int *centerY){
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
    centerX[0] = round(((double)x)/count);
    centerY[0] = round(((double)y)/count);
    printf("center at row:%d col:%d\n",centerY[0],centerX[0]);
}
