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

#endif /* functions_h */
