//
//  functions.h
//  ImageSnake
//
//  Created by st chen on 16/6/14.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#ifndef functions_h
#define functions_h

#include <stdio.h>
void inverseMat(int points, double Amat[points][points]);
double detrminant(double a[250][250], int k);
void cofactors(double num[250][250], int f, double inv[f][f]);
void trans(double num[250][250], double fac[250][250], int r, double inv[r][r]);
void converInd(unsigned char **x, int nrows, int ncols, double **intImage);

/**
 *stn Dynamic array struct and relate functions
 */
typedef struct {
    int *array;
    size_t used;
    size_t size;
} stnArray;
void initStnArray(stnArray *a, size_t initialSize);
void insertStnArray(stnArray *a, int element);
void freeStnArray(stnArray *a);

#define max(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })

double sum(double x[], int arr_count);
double stnInterp2(int nrows, int ncols, int intMatrix[nrows][ncols], double row, double col);
void detect_peak(
                 const double*   data, /* the data */
                 int             data_count, /* row count of data */
                 stnArray*       peaks, /* emission peaks will be put here */
                 double          delta, /* delta used for distinguishing peaks */
                 double          threshold   /*threshold used for filter peaks below the value*/
);
void stnMatrixSquare(int nrows, int ncols, double matrix[nrows][ncols]);
void stnEigenVector(int nSize, double intMatrix[nSize][nSize]);
#endif /* functions_h */
