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

void stnCurvaturePro(unsigned char **inputImg, int nrows, int ncols, double **outputImg){
    FILE *fpy;
    int i;
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
    
    outputImg = interImg;
    
    
    
    
    
    
    
    
    imgInt2Char(binearImg, nrows, ncols, y);
        /* WRITE THE IMAGE */
        fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
        for(i = 0; i < nrows; i++)
            if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
                error("can't write the image");
    
        /* CLOSE FILE & QUIT */
        fclose(fpy);
}
