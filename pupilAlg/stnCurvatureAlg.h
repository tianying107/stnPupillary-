//
//  stnCurvatureAlg.h
//  pupilAlg
//
//  Created by st chen on 16/10/5.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#ifndef stnCurvatureAlg_h
#define stnCurvatureAlg_h

#include <stdio.h>
#include "functions.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
void stnCurvaturePro(unsigned char **inputImg, int nrows, int ncols, double **outputImg, double **outputppm, double allParameters[7],int side);
//#include "stnImgOperaters.h"
#endif /* stnCurvatureAlg_h */
