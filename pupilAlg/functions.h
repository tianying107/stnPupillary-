//
//  functions.h
//  ImageSnake
//
//  Created by st chen on 16/6/14.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#ifndef functions_h
#define functions_h

#include <stdio.h>
void freeStnMatrix(void** matrix);
void inverseMat(int points, double Amat[points][points]);
double detrminant(double a[250][250], int k);
void cofactors(double num[250][250], int f, double inv[f][f]);
void trans(double num[250][250], double fac[250][250], int r, double inv[r][r]);
void converInd(unsigned char **x, int nrows, int ncols, double **intImage);
void imgDouble2Char(double **doubleImage, int nrows, int ncols, unsigned char **charImage);
void imgCombineDouble2Char(double **doubleImage1, double **doubleImage2, int nrows, int ncols, unsigned char **charImage);
void imgInt2Double(int **intImage, int nrows, int ncols, double **doubleImage);
void imgChar2Double(unsigned char **intImage, int nrows, int ncols, double **doubleImage);
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
double sumStnArray(stnArray *a);
double sumToStnArray(stnArray *a, int sumToIndex);
double sumSquaredStnArray(stnArray *a);
double sumPoweredStnArray(stnArray *a, double power_a);
double sumProductStnArray(stnArray *a, double power_a, stnArray *b, double power_b);

/**
 *funtion max(a,b)
 */
#define max(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })

#define min(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

double stnInterp2(int nrows, int ncols, int intMatrix[nrows][ncols], double row, double col);
void detect_peak(
                 const double*   data, /* the data */
                 int             data_count, /* row count of data */
                 stnArray*       peaks, /* emission peaks will be put here */
                 double          delta, /* delta used for distinguishing peaks */
                 double          threshold,   /*threshold used for filter peaks below the value*/
                 int             deltaMode        /*1:use delta on the right side of peak; 2:use delta on both sides of peak*/
);
void stnMatrixSquare(int nrows, int ncols, double **matrix, double **multiply);
void stnMatrixMultiply(int nrows1, int nrows2, int ncols, double **matrix1, double **matrix2, double **multiply);
double *stnEigenVector(int nSize, double **intMatrix);
void stnMatrixInverse(int nrows, double **squareMatrix);



double Determinant(double **a,int n);
void CoFactor(double **a,int n,double **b);
void Transpose(double **a,int n);

double *firstDev(double *input, int size);
#endif /* functions_h */
