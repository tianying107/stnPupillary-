//
//  main.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#include <stdio.h>
#include <stdlib.h> /* needed for exit() */
#include <math.h>
#include "functions.h"
#include "stnCurvatureAlg.h"
#include "stnImgOperaters.h"


extern int read_pgm_hdr(FILE *, int *, int *);
extern void **matrix(int, int, int, int, int);
extern void error(const char *);
extern void converInd(unsigned char **, int, int, double **);
extern void imageSplit(unsigned char **, int , int , unsigned char **, unsigned char **);
extern void imgInt2Char(int **, int , int , unsigned char **);
extern void imgDouble2Char(double **, int , int , unsigned char **);
int main(int argc, const char * argv[]) {
    FILE *fpx, *fpy;
    int nrows, ncols, i;//, j, k1, k2;
    unsigned char **x, **y,**img1,**img2;
    double **doubleImage, **px, **py, **out1, **out2, **ppm1, **ppm2;
    /* OPEN FILES */
    for (int index=1; index<2; index++) {

        
        char name[] = "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_0001_image.pgm";
        snprintf(name, sizeof(name), "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_%.4d_image.pgm", index);
//        printf("%s\n",name);
        fpx = fopen(name,"r");
        
        char nameout[] = "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/frame_0001_image.ppm";
        snprintf(nameout, sizeof(nameout), "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/frame_%4d_image.ppm", index);
        fpy = fopen(nameout, "w");
        /* READ HEADER */
        if(read_pgm_hdr(fpx, &nrows, &ncols) < 0)
            error("not a PGM image or bpp > 8");
        
        
        /* ALLOCATE ARRAYS */
        x = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
        y = (unsigned char **)matrix(nrows, 3*ncols, 0, 0, sizeof(char));
//        y = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
        doubleImage = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
        img1 = (unsigned char **)matrix(nrows, ncols/2, 0, 0, sizeof(unsigned char));
        img2 = (unsigned char **)matrix(nrows, ncols/2, 0, 0, sizeof(unsigned char));
        out1 =(double **)matrix(nrows, ncols/2, 0, 0, sizeof(double));
        out2 =(double **)matrix(nrows, ncols/2, 0, 0, sizeof(double));
        ppm1 =(double **)matrix(nrows, 3*ncols/2, 0, 0, sizeof(double));
        ppm2 =(double **)matrix(nrows, 3*ncols/2, 0, 0, sizeof(double));
        px = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
        py = (double **)matrix(nrows, ncols, 0, 0, sizeof(double));
        if(x == NULL) error("can't allocate memory");
        /* READ THE IMAGE */
        for(i = 0; i < nrows; i++)
            if(fread(&x[i][0], sizeof(char), ncols, fpx) != ncols)
                error("can't read the image");
        
        
        imageSplit(x, nrows, ncols/2, img1, img2);
        
        stnCurvaturePro(img1, nrows, ncols/2, out1, ppm1);
        stnCurvaturePro(img2, nrows, ncols/2, out2, ppm2);
        imgCombineDouble2Char(ppm1, ppm2, nrows, 3*ncols/2, y);
//        imgCombineDouble2Char(out1, out2, nrows, ncols/2, y);
        
        /* WRITE THE IMAGE */
        fprintf(fpy, "P6\n%d %d\n255\n", ncols, nrows);
        for(i = 0; i < nrows; i++)
            if(fwrite(&y[i][0], sizeof(char), 3*ncols, fpy) != 3*ncols)
                error("can't write the image");
//        /* WRITE THE IMAGE */
//        fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
//        for(i = 0; i < nrows; i++)
//            if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
//                error("can't write the image");
        
        /* CLOSE FILE & QUIT */
        fclose(fpy);
        printf("processed: %d\n",index);


    }
    return 0;
}
