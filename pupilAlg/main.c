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

extern int read_pgm_hdr(FILE *, int *, int *);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void converInd(unsigned char **, int, int, double **);
extern void imageSplit(double **, int , int , double **, double **);

int main(int argc, const char * argv[]) {
    FILE *fpx;//, *fpy;
    int nrows, ncols, i;//, j, k1, k2;
    unsigned char **x;
    double **doubleImage, **px, **py,**img1,**img2;
    /* OPEN FILES */
    fpx = fopen("/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_0001_image.pgm","r");
    
    /* READ HEADER */
    if(read_pgm_hdr(fpx, &nrows, &ncols) < 0)
        error("not a PGM image or bpp > 8");
    /* ALLOCATE ARRAYS */
    x = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
    doubleImage = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    img1 = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    img2 = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    px = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    py = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    if(x == NULL) error("can't allocate memory");
    /* READ THE IMAGE */
    for(i = 0; i < nrows; i++)
        if(fread(&x[i][0], sizeof(char), ncols, fpx) != ncols)
            error("can't read the image");
    converInd(x, nrows, ncols, doubleImage);
    
    printf("vv %f\n",doubleImage[100][100]);
    
    return 0;
}
