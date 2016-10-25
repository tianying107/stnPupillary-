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
#include "stnCurvatureAlg.h"

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
    double **doubleImage, **px, **py, **out1, **out2;
    /* OPEN FILES */
    for (int index=186; index<209; index++) {
        
        double **test =(double **)matrix(6, 6, 0, 0, sizeof(double));

//        double circleMap[6][6] = {{1,2,3,4,5,6},
//            {2,3,4,5,6,1},
//            {3,4,5,6,1,2},
//            {4,5,6,1,2,3},
//            {5,6,1,2,3,4},
//            {6,1,2,3,4,5}};
//        double circleMap[5][5] = {{1,2,3,4,5},
//                        {2,3,4,5,6},
//                        {3,4,5,6,1},
//                        {4,5,6,1,2},
//                        {5,6,1,2,3}};
        test[0][0]=1;
        test[0][1]=-2;
        test[0][2]=4;
        test[1][0]=-5;
        test[1][1]=2;
        test[1][2]=0;
        test[2][0]=1;
        test[2][1]=0;
        test[2][2]=3;
//        double det = Determinant(test, 3);
//        printf("det = %f\n",det);
//        stnMatrixInverse(6, circleMap);
        
        
        
        
        char name[] = "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_0001_image.pgm";
        snprintf(name, sizeof(name), "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/image/frame_%.4d_image.pgm", index);
//        printf("%s\n",name);
        fpx = fopen(name,"r");
        
        char nameout[] = "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/frame_0001_image.pgm";
        snprintf(nameout, sizeof(nameout), "/Users/stn/Documents/Group/Pupilary/pupilAlg/pupilAlg/output/frame_%4d_image.pgm", index);
        fpy = fopen(nameout, "w");
        /* READ HEADER */
        if(read_pgm_hdr(fpx, &nrows, &ncols) < 0)
            error("not a PGM image or bpp > 8");
        /* ALLOCATE ARRAYS */
        x = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
        y = (unsigned char **)matrix(nrows, ncols, 0, 0, sizeof(char));
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
        
        imageSplit(x, nrows, ncols/2, img1, img2);
        
        
        stnCurvaturePro(img1, nrows, ncols/2, out1);
        stnCurvaturePro(img2, nrows, ncols/2, out2);
        
        imgCombineDouble2Char(out1, out2, nrows, ncols/2, y);

        
        
        /* WRITE THE IMAGE */
        fprintf(fpy, "P5\n%d %d\n255\n", ncols, nrows);
        for(i = 0; i < nrows; i++)
            if(fwrite(&y[i][0], sizeof(char), ncols, fpy) != ncols)
                error("can't write the image");
        
        /* CLOSE FILE & QUIT */
        fclose(fpy);
        printf("processed: %d\n",index);
        
        
        
        
        
        

    }
    return 0;
}
