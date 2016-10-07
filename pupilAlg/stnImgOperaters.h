//
//  stnImgOperaters.h
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#ifndef stnImgOperaters_h
#define stnImgOperaters_h

#include <stdio.h>
#include <math.h>
#include "functions.h"
void filterBlobWithLabel(int **labelImage, int nrows, int ncols, int label);
void stnMedianFilter(int **inputImg, int nrows, int ncols, int filterWidth, int filterHeight);
void stnFindCentral(int **inputImg, int nrows, int ncols, int *centerX, int *centerY);
#endif /* stnImgOperaters_h */
