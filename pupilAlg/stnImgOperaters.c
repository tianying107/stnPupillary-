//
//  stnImgOperaters.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include "stnImgOperaters.h"
void imageSplit(double **combinedImg, int nrows, int newWidth, double **img1, double **img2){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<newWidth; j++) {
            img1[i][j] = combinedImg[i][j];
            img2[i][j] = combinedImg[i][j+newWidth];
        }
    }
}
