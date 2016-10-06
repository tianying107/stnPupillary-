//
//  main.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h> /* needed for exit() */
#include <math.h>
#include "functions.h"
#include "stnCurvatureAlg.h"
#include "stnCurvatureAlg.h"

extern int read_pgm_hdr(FILE *, int *, int *);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void converInd(unsigned char **, int, int, double **);
extern void imageSplit(unsigned char **, int , int , unsigned char **, unsigned char **);
extern void stnCurvaturePro(unsigned char **, int , int , double **);
extern void imgInt2Char(int **, int , int , unsigned char **);
extern void imgDouble2Char(double **, int , int , unsigned char **);
int main(int argc, const char * argv[]) {
    FILE *fpx, *fpy;
    int nrows, ncols, i;//, j, k1, k2;
    unsigned char **x, **y,**img1,**img2;
    double **doubleImage, **px, **py, **out1, **out2;
    /* OPEN FILES */
    fpx = fopen("/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_0001_image.pgm","r");
    fpy = fopen("/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/output.pgm", "w");
    /* READ HEADER */
    if(read_pgm_hdr(fpx, &nrows, &ncols) < 0)
        error("not a PGM image or bpp > 8");
    /* ALLOCATE ARRAYS */
    x = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
    y = (unsigned char **)matrix(nrows, ncols/2, 0, 0, sizeof(char));
    doubleImage = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    img1 = (unsigned char **)matrix(nrows, ncols/2, 0, 0, sizeof(unsigned char));
    img2 = (unsigned char **)matrix(nrows, ncols/2, 0, 0, sizeof(unsigned char));
    out1 =(double **)matrix(nrows, ncols/2, 0, 0, sizeof(double));
    out2 =(double **)matrix(nrows, ncols/2, 0, 0, sizeof(double));
    px = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    py = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    if(x == NULL) error("can't allocate memory");
    /* READ THE IMAGE */
    for(i = 0; i < nrows; i++)
        if(fread(&x[i][0], sizeof(char), ncols, fpx) != ncols)
            error("can't read the image");
//    converInd(x, nrows, ncols, doubleImage);
    
    imageSplit(x, nrows, ncols, img1, img2);
    
    stnCurvaturePro(img1, nrows, ncols/2, out1);
    
    
    printf("vv %f\n",out1[100][100]);
    
//    imgInt2Char(, <#int nrows#>, <#int ncols#>, <#unsigned char **charImage#>)
//    /* WRITE THE IMAGE */
//    fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
//    for(i = 0; i < nrows; i++)
//        if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
//            error("can't write the image");
//    
//    /* CLOSE FILE & QUIT */
//    fclose(fpx);
    return 0;
}
