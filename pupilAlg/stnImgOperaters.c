//
//  stnImgOperaters.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include "stnImgOperaters.h"
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
