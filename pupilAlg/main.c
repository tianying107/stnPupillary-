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
#include <time.h>
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
    FILE *fpx, *fpy, *fParameter;
    int nrows, ncols, i;
    unsigned char **x, **y,**img1,**img2;
    double **doubleImage, **px, **py, **out1, **out2, **ppm1, **ppm2;
    double paraLeft[7]={0,0,0,0,0,0,0},paraRight[7]={0,0,0,0,0,0,0};
    /* OPEN FILES */
    fParameter = fopen("/Users/stn/Documents/PHD/output/parameters.txt", "w");
    for (int index=1; index<101; index++) {

        
        char name[] = "/Users/stn/Documents/PHD/pupil_data/dataset4/frame_0001_image.pgm";
        snprintf(name, sizeof(name), "/Users/stn/Documents/PHD/pupil_data/dataset4/frame_%.4d_image.pgm", index);
//        printf("%s\n",name);
        fpx = fopen(name,"r");
        
        char nameout[] = "/Users/stn/Documents/PHD/output/frame_0001_image.ppm";
        snprintf(nameout, sizeof(nameout), "/Users/stn/Documents/PHD/output/frame_%4d_image.ppm", index);
        fpy = fopen(nameout, "w");
        /* READ HEADER */
        if(read_pgm_hdr(fpx, &nrows, &ncols) < 0)
            error("not a PGM image or bpp > 8");
        
        
        /* ALLOCATE ARRAYS */
        x = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
        y = (unsigned char **)matrix(nrows, 3*ncols, 0, 0, sizeof(char));
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
        
        
//        clock_t begin = clock();
        
        imageSplit(x, nrows, ncols/2, img1, img2);
        
        stnCurvaturePro(img1, nrows, ncols/2, out1, ppm1, paraLeft,1);
        stnCurvaturePro(img2, nrows, ncols/2, out2, ppm2, paraRight,2);
//        clock_t end = clock();
//        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
//        printf("both side of image spend %f seconds\n",time_spent);
        
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
        
        fprintf(fParameter, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n", (double)paraRight[0],(double)paraRight[1],(double)paraRight[2],(double)paraRight[3],(double)paraRight[4],(double)paraRight[5],(double)paraRight[6],(double)paraLeft[0],(double)paraLeft[1],(double)paraLeft[2],(double)paraLeft[3],(double)paraLeft[4],(double)paraLeft[5],(double)paraLeft[6]);
        
        printf("processed: %d\n",index);

        freeStnMatrix((void**)x);
        freeStnMatrix((void**)y);
        freeStnMatrix((void**)doubleImage);
        freeStnMatrix((void**)img1);
        freeStnMatrix((void**)img2);
        freeStnMatrix((void**)out1);
        freeStnMatrix((void**)out2);
        freeStnMatrix((void**)ppm1);
        freeStnMatrix((void**)ppm2);
        freeStnMatrix((void**)px);
        freeStnMatrix((void**)py);
        
        fclose(fpx);
        fclose(fpy);
    }
    
    fclose(fParameter);
    return 0;
}
