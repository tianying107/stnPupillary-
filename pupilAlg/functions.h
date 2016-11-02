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


/**
 *funtion max(a,b)
 */
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
void stnMatrixSquare(int nrows, int ncols, double matrix[nrows][ncols], double multiply[nrows][nrows]);
void stnMatrixMultiply(int nrows1, int nrows2, int ncols, double matrix1[nrows1][ncols], double matrix2[ncols][nrows2], double multiply[nrows1][nrows2]);
double *stnEigenVector(int nSize, double intMatrix[nSize][nSize]);
void stnMatrixInverse(int nrows, double squareMatrix[nrows][nrows]);



double Determinant(double **a,int n);
void CoFactor(double **a,int n,double **b);
void Transpose(double **a,int n);
#endif /* functions_h */
