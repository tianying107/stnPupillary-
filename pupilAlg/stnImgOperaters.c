//
//  stnImgOperaters.c
//  pupilAlg
//
//  Created by st chen on 16/10/3.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#include "stnImgOperaters.h"
#include <stdlib.h>
#define PI 3.14159265
extern void flood_fill(int ,int ,int , int **, int , int);
extern void **matrix(int, int, int, int, int);
static int count;
void imageSplit(unsigned char **combinedImg, int nrows, int newWidth, unsigned char **img1, unsigned char **img2){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<newWidth; j++) {
            img1[i][j] = combinedImg[i][j];
            img2[i][j] = combinedImg[i][j+newWidth];
        }
    }
}

double *movingWindowSmooth(double *input, int windowSize, int inputSize){
    int i;
    int startIndex = (windowSize-1)/2;
    double *output = (double *)malloc(256*sizeof(double));
    output[0] = input[0];
    output[255] = input[255];
    for (i=startIndex; i<inputSize-startIndex; i++) {
        output[i] = (input[i-1]+input[i]+input[i+1])/(double)windowSize;
    }
    return output;
}

double *imageHistogram(unsigned char **inputImg, int nrows, int ncols){
    
    //calculate the normalized histogram
    int i, j;
    double Num = 0;
    double *histogram = (double *)malloc(256*sizeof(double));
    for (i=0; i<256; i++) {
        histogram[i]=0;
    }
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            int va;
            va=inputImg[i][j];
            histogram[va] += 1;
            Num += 1;
        }
    }
    for (i=0; i<256; i++) {
        histogram[i]=histogram[i]/Num;
        
    }
    return histogram;
}
void imageHistogramEqualization(unsigned char **inputImg, int nrows, int ncols, int lower, int upper, double **outputImg){
    int i, j;
    int Num = 0;
    //calculate the hist
    double h[256];
    for (i=0; i<256; i++) {
        h[i]=0;
    }
    //left half side
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            int va;
            va=inputImg[i][j];
            if ((va>=lower)&&(va<=upper)) {
                h[va]+=1;
                Num+=1;
            }
        }
    }
    
    //calculate the smallest gray level
    int XL=0;
    double H[256];
    for (i=lower; i<upper+1; i++) {
        if (i==lower) {
            H[i]=h[i];
        }
        else{
            H[i]=H[i-1]+h[i];
        }
        if (H[i]==0) {
            XL=i;
        }
    }
    //Map to new gray level
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            int va;
            va=inputImg[i][j];
            if (va<lower) {
                outputImg[i][j]=0;
            }
            else if (va>upper){
                outputImg[i][j]=upper;
            }
            else{
                outputImg[i][j]=(H[va]-H[XL])/((double)Num-H[XL]);
            }
        }
    }
}

/**
 *deprecated function
void dynamicHistogramEqualization(unsigned char **inputImg, int nrows, int ncols, int lower, int upper, int radius, double **outputImg){
    int i, j,ii,jj;
    int Num,XL,H[256];
    double h[256];
    
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            outputImg[i][j]=(double)inputImg[i][j];
        }
    }
    
    for (i=radius; i<nrows-radius; i++) {
        for (j=radius; j<ncols-radius; j++) {
            //calculate the hist
            Num = 0;
            for (ii=0; ii<256; ii++) {
                h[ii]=0;
            }
            for (ii=i-radius; ii<i+radius; ii++) {
                for (jj=j-radius; jj<j+radius; jj++) {
                    int va;
                    va=inputImg[ii][jj];
                    if ((va>=lower)&&(va<=upper)) {
                        h[va]+=1;
                        Num+=1;
                    }
                }
            }
            
            //calculate the smallest gray level
            XL=0;
            for (ii=lower; ii<upper+1; ii++) {
                if (ii==lower) {
                    H[ii]=h[ii];
                }
                else{
                    H[ii]=H[ii-1]+h[ii];
                }
                if (H[ii]==0) {
                    XL=ii;
                }
            }
            //Map to new gray level
            int va;
            va=inputImg[i][j];
            if (va<lower) {
                outputImg[i][j]=0;
            }
            else if (va>upper){
                outputImg[i][j]=255;
            }
            else{
                outputImg[i][j]=(H[va]-H[XL])/((double)Num-H[XL]);
            }

        }
    }

}
*/
 
void imageThreshold(double **inputImg, int nrows, int ncols, double threshold, int **binearImg){
    int i, j;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (inputImg[i][j]<threshold) {
                binearImg[i][j]=0;
            }
            else{
                binearImg[i][j]=1;
            }
        }
    }
}

/**DEPRECATED
 *Version 1.0
 *connectivityLabel is a function to isolate blobs and label from the input image using 8-neighbour connectivity analysis
 *inputImg is the input binary image
 *labeledImg is the output image. Each blob in the image was labeled. The label start from 1.
 *This function return an int maxLabel, which indicate the largest area of blob.
 *
 */
int connectivityLabel(int **inputImg, int nrows, int ncols, int **labeledImg){
    int i, j, label, labelCount,maxLabel,maxCount;
    
    //set pixel 1 to -1, set pixel 0 to 0 as unlabeled pixel
    labelCount = 0;
    maxLabel=0;
    maxCount=0;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (inputImg[i][j]==1) {
                labeledImg[i][j]=-1;
            }
            else {
                labeledImg[i][j]=0;
                labelCount++;
            }
        }
    }

    //search unlabeled pixels
    label=1;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (labeledImg[i][j]==0) {
                count=0;
                //flood fill unlabeled pixels
                flood_fill(i,j,label,labeledImg,nrows,ncols);
                if (count>maxCount) {
                    maxCount = count;
                    maxLabel = label;
                }
                label++;
            }
        }
    }

    return maxLabel;
    
}

/**
 *Growth Circle algorithm, an algorithm to label the largest blob
 *Version 1.2
 *NEW FEAUTRES: using previous radius result to guess the starting point
 */
void growthCircle(stnPoint *centerPoint, int **inputImg, int nrows, int ncols, int radius){
    bool allWhite = false;
    int i,j,count;
    //test previous radius
    count = 0;
    
    if ((centerPoint->col)>=ncols || (centerPoint->row)>=nrows || centerPoint->col<0 || centerPoint->row<0 || centerPoint == NULL){
        centerPoint->row = -3340; centerPoint->col = -3012;
        printf("center point invalid.");
        return;
    }
    for (i=0; i<500*PI; i++) {
        int col = floor(radius*cos(((double)i)/1000) + centerPoint->col);
        int row = floor(radius*sin(((double)i)/1000) + centerPoint->row);
        int value1 = inputImg[row][col];
        int value2 = inputImg[row][-col+2*centerPoint->col];
        int value3 = inputImg[-row+2*centerPoint->row][col];
        int value4 = inputImg[-row+2*centerPoint->row][-col+2*centerPoint->col];
        if (value1&&value2&&value3&&value4) {
            count++;
        }
        else break;
    }
    if (count>=1500) {
        for (radius=radius; radius>0; radius--) {
            count = 0;
            for (i=0; i<500*PI; i++) {
                int col = floor(radius*cos(((double)i)/1000) + centerPoint->col);
                int row = floor(radius*sin(((double)i)/1000) + centerPoint->row);
                int value1 = inputImg[row][col];
                int value2 = inputImg[row][-col+2*centerPoint->col];
                int value3 = inputImg[-row+2*centerPoint->row][col];
                int value4 = inputImg[-row+2*centerPoint->row][-col+2*centerPoint->col];
                if (value1&&value2&&value3&&value4) {
                    count++;
                }
                else break;
            }
            
            if (count>=1500) {
                allWhite = true;
                break;
            }
        }
    }else{
        for (radius=radius; radius<min(min(centerPoint->col, ncols-centerPoint->col), min(centerPoint->row, nrows-centerPoint->row)); radius++) {
            count = 0;
            for (i=0; i<500*PI; i++) {
                int col = floor(radius*cos(((double)i)/1000) + centerPoint->col);
                int row = floor(radius*sin(((double)i)/1000) + centerPoint->row);
                int value1 = inputImg[row][col];
                int value2 = inputImg[row][-col+2*centerPoint->col];
                int value3 = inputImg[-row+2*centerPoint->row][col];
                int value4 = inputImg[-row+2*centerPoint->row][-col+2*centerPoint->col];
                if (value1&&value2&&value3&&value4) {
                    count++;
                }
                else break;
            }
            
            if (count>=1500) {
                allWhite = true;
                break;
            }
        }
    }
    
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if ((pow(i-centerPoint->row, 2)+pow(j-centerPoint->col, 2))>radius*radius) {
                inputImg[i][j] = 1;
            }
        }
    }
}
/*old version growth circle without guess
void growthCircle(stnPoint *centerPoint, int **inputImg, int nrows, int ncols, int radius){
    bool allWhite = false;
    int i,j,count;
        for (radius=40; radius<min(min(centerPoint->col, ncols-centerPoint->col), min(centerPoint->row, nrows-centerPoint->row)); radius++) {
            count = 0;
            for (i=0; i<500*PI; i++) {
                int col = floor(radius*cos(((double)i)/1000) + centerPoint->col);
                int row = floor(radius*sin(((double)i)/1000) + centerPoint->row);
                int value1 = inputImg[row][col];
                int value2 = inputImg[row][-col+2*centerPoint->col];
                int value3 = inputImg[-row+2*centerPoint->row][col];
                int value4 = inputImg[-row+2*centerPoint->row][-col+2*centerPoint->col];
                if (value1&&value2&&value3&&value4) {
                    count++;
                }
                else break;
            }
            
            if (count>=1500) {
                allWhite = true;
                break;
            }
        }
    
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if ((pow(i-centerPoint->row, 2)+pow(j-centerPoint->col, 2))>radius*radius) {
                inputImg[i][j] = 1;
            }
        }
    }
}
*/





/**DEPRECATED
 *flood_fill algorithm, here use 6-neighbour connectivity analysis to avoid block
 *A static variable count to count the number of current label
 */
void flood_fill(int x,int y,int label, int **labelImg, int nrows, int ncols){
    labelImg[x][y]=label;
    count++;
    if (count>170000) {
        return;
    }
    if(x>0&&labelImg[x-1][y]==0)flood_fill(x-1,y,label,labelImg,nrows,ncols);
    if(y>0&&labelImg[x][y-1]==0)flood_fill(x,y-1,label,labelImg,nrows,ncols);
    if(y<ncols-1&&labelImg[x][y+1]==0)flood_fill(x,y+1,label,labelImg,nrows,ncols);
//    if(y>0&&x<nrows-1&&labelImg[x+1][y-1]==0)flood_fill(x+1,y-1,label,labelImg,nrows,ncols);
    if(x<nrows-1&&labelImg[x+1][y]==0)flood_fill(x+1,y,label,labelImg,nrows,ncols);
//    if(y<ncols-1&&x<nrows-1&&labelImg[x+1][y+1]==0)flood_fill(x+1,y+1,label,labelImg,nrows,ncols);
    
}





/**DEPRECATED
 *filterBlobWithLabel is a function can pick up the blob in a well labeled image using a specific label
 *labelImage is a well labeled image, with nrows rows and ncols column. It is both input and output.
 *label is the target label
 *
 *The target area is 0(black), background is 1(white).
 */
void filterBlobWithLabel(int **labelImage, int nrows, int ncols, int label){
    int i,j;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (labelImage[i][j]==label) labelImage[i][j]=0;
            else labelImage[i][j]=1;
        }
    }
}




/**
 *version 1.1
 *stnMedianFilter is a 2-D median filter that only work on binary image
 */
void stnMedianFilter(int **inputImg, int nrows, int ncols, int filterWidth, int filterHeight){
    int edgeX, edgeY, i, j, filterX, filterY, sum;
    
    int **filteredImg=(int **)matrix(nrows, ncols, 0, 0, sizeof(int));
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            filteredImg[i][j]=inputImg[i][j];
        }
    }
    edgeX = (filterWidth-1)/2;
    edgeY = (filterHeight-1)/2;
    for (i=edgeY; i<nrows-edgeY; i++) {
        for (j=edgeX; j<ncols-edgeX; j++) {
            sum = 0;
            for (filterY=i-edgeY; filterY<i+edgeY+1 ;filterY++) {
                for (filterX = j-edgeX; filterX<j+edgeX+1; filterX++) {
                    sum += inputImg[filterY][filterX];
                }
            }
            filteredImg[i][j]=(int)(sum/ceil((((float)(filterWidth*filterHeight))/2)));
        }
    }
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            inputImg[i][j]=filteredImg[i][j];
        }
    }
    freeStnMatrix((void**)filteredImg);
}





/**
 *stnFindCentral
 */
void stnFindCentral(int **inputImg, int nrows, int ncols, stnPoint *centerPoint){
    double count = 0;
    int i,j;
    int x=0;
    int y=0;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            if (!inputImg[i][j]) {
                y+=i;
                x+=j;
                count++;
            }
        }
    }
    centerPoint->col = round(((double)x)/count);
    centerPoint->row = round(((double)y)/count);
}

/**
 *version 1.1
 *stnBoundaryPoints
 */
void stnBoundaryPoint(int **inputImg, int nrows, int ncols, stnPoint *centerPoint, stnPoint *leftPoint, stnPoint *rightPoint){
    int x = centerPoint->col;
    int y = centerPoint->row;
    int windowSize = nrows/10;
    if (y>=0&&y<nrows&&x>=0&&x<ncols) {
        //any holes within the pupil blob were classified as a corneal relection if the hole width was less than this value
        //true point at here is black (i.e. the value of center point is 0)
        
        /**left boundary point*/
        int sum = 0;
        for (int i=0; i<windowSize && x-i>=0; i++) {
            sum += inputImg[y][x-i];
        }
        //sum < windowSize means there is at least one point in the window is black
        while (sum<windowSize && x) {
            x--;
            sum=0;
            for (int i=0; (i<windowSize && x-i>=0); i++) {
                sum += inputImg[y][x-i];
            }
        }
        leftPoint->col = x+1;
        leftPoint->row = y;
        
        /**right boundary point*/
        sum = 0;
        for (int i=0; i<windowSize && x+i<ncols; i++) {
            sum += inputImg[y][x+i];
        }
        while (sum<min(windowSize, ncols-x) && x<ncols) {
            x++;
            sum=0;
            for (int i=0; i<windowSize && x+i<ncols; i++) {
                sum += inputImg[y][x+i];
            }
        }
        rightPoint->col = x-1;
        rightPoint->row = y;
    }
}

/**
 *version 1.1
 *stnContourBound
 */
void stnContourBound(int **inputImg, int nrows, int ncols, stnPoint *leftPoint, stnArray *directionArray, stnArray *contourMapRow, stnArray *contourMapCol){
    int *mapPlus2=malloc(8*sizeof(int));//{2,3,4,5,6,7,0,1};
    mapPlus2[0]=2;mapPlus2[1]=3;mapPlus2[2]=4;mapPlus2[3]=5;mapPlus2[4]=6;mapPlus2[5]=7;mapPlus2[6]=0;mapPlus2[7]=1;
    int *mapMinus1=malloc(8*sizeof(int));//{7,0,1,2,3,4,5,6};
    mapMinus1[0]=7;mapMinus1[1]=0;mapMinus1[2]=1;mapMinus1[3]=2;mapMinus1[4]=3;mapMinus1[5]=4;mapMinus1[6]=5;mapMinus1[7]=6;
    int direction = 1; //initial direction is 1
    int x = leftPoint->col;
    int y = leftPoint->row;
    
    int *map = malloc(16*sizeof(int));//{y-1,x+1,y-1,x,y-1,x-1,y,x-1,y+1,x-1,y+1,x,y+1,x+1,y,x+1};
    map[0]=y-1;map[1]=x+1;map[2]=y-1;map[3]=x;map[4]=y-1;map[5]=x-1;map[6]=y;map[7]=x-1;map[8]=y+1;map[9]=x-1;map[10]=y+1;map[11]=x;map[12]=y+1;map[13]=x+1;map[14]=y;map[15]=x+1;
    direction = mapPlus2[direction];
    
    for (int j=0; j<8; j++) {
            if (inputImg[map[direction*2]][map[direction*2+1]]==1) {
                direction = mapMinus1[direction];
            }
            else{
                insertStnArray(directionArray, direction);
                insertStnArray(contourMapRow, y);
                insertStnArray(contourMapCol, x);
                y = map[direction*2];
                x = map[direction*2+1];
                break;
            }
    }
    
    while ((y-1) && (x-1) && (y+1<nrows) && (x+1<ncols) && (x!=leftPoint->col || y!=leftPoint->row)) {
        int map[16] = {y-1,x+1,y-1,x,y-1,x-1,y,x-1,y+1,x-1,y+1,x,y+1,x+1,y,x+1};
        direction = mapPlus2[direction];
        for (int j=0; j<8; j++) {
                if (inputImg[map[direction*2]][map[direction*2+1]]==1) {
                    direction = mapMinus1[direction];
                }
                else{
                    insertStnArray(directionArray, direction);
                    insertStnArray(contourMapRow, y);
                    insertStnArray(contourMapCol, x);
                    y = map[direction*2];
                    x = map[direction*2+1];
                    break;
                }
            }
    }

}
/**
 *version 1.1
 *stnCurvature
 */
double *stnCurvature(stnArray *directionArray, int windowSize){
    int circleMap[8][8] = {{0,1,2,3,4,-3,-2,-1},
                            {-1,0,1,2,3,4,-3,-2},
                            {-2,-1,0,1,2,3,4,-3},
                            {-3,-2,-1,0,1,2,3,4},
                            {4,-3,-2,-1,0,1,2,3},
                            {3,4,-3,-2,-1,0,1,2},
                            {2,3,4,-3,-2,-1,0,1},
                            {1,2,3,4,-3,-2,-1,0}};
    int length =(int)directionArray->used;
    
    
//    double curvature[directionArray->used];
    double *curvature = (double *)malloc(length*sizeof(double));
    if (!length) return curvature;
    
    stnArray newDirection;
    initStnArray(&newDirection, 1);
    for (int i=-(windowSize-1); i<length+windowSize-1; i++) {
        if (i<0) {
            insertStnArray(&newDirection, directionArray->array[length+i]);
        }
        else if (i>=length){
            insertStnArray(&newDirection, directionArray->array[i-length]);
        }
        else insertStnArray(&newDirection, directionArray->array[i]);
    }
    for (int i=windowSize-1; i<length+windowSize-1; i++) {
        int alp1=0, alp2=0;
        for (int j=i-windowSize+1; j<i+1; j++) {
            alp1 += newDirection.array[j];
        }
        for (int j=i; j<i+windowSize; j++) {
            alp2 += newDirection.array[j];
        }
//        printf("%f, %f\n",((double)alp1)/windowSize,((double)alp2)/windowSize);
        curvature[i-windowSize+1]=stnInterp2(8, 8, circleMap, min(((double)alp1)/windowSize, 6) , min(((double)alp2)/windowSize, 6));
    }
//    for (int i=0; i<length; i++) {
//        printf("%f\n",curvature[i]);
//    }
    freeStnArray(&newDirection);
    return curvature;
}

/** INCOMPLETE
 *stnDirectionDifference
 */
double stnDirectionDifference(double direction1, double direction2){
    int circleMap[8][8] = {{0,1,2,3,4,-3,-2,-1},
        {-1,0,1,2,3,4,-3,-2},
        {-2,-1,0,1,2,3,4,-3},
        {-3,-2,-1,0,1,2,3,4},
        {4,-3,-2,-1,0,1,2,3},
        {3,4,-3,-2,-1,0,1,2},
        {2,3,4,-3,-2,-1,0,1},
        {1,2,3,4,-3,-2,-1,0}};
    
    
    double diff=0;
    if (direction1>=0 && direction1<8 && direction2>=0 && direction2<8) {
        diff = stnInterp2(8, 8, circleMap, direction1 , direction2);
    }
    return diff;
}

/**
 *stnSafePoints
 *version 1.1
 */
void stnSafePoints(stnArray *contourRows, stnArray *contourCols, stnArray *directionArray, stnArray *breakPoints, stnPoint *rightPoint, stnArray *safeRows, stnArray *safeCols){
    /*calculate left safe points upper and lower boundary index first*/
    int leftUpperIndex = breakPoints->array[0];
    int leftLowerIndex = breakPoints->array[(int)breakPoints->used - 1];
    /*then find out right point index in contour*/
    int rightIndex = 0;
    for (int i = 0; i<(int)contourRows->used; i++) {
        if ((contourRows->array[i]==rightPoint->row)&&(contourCols->array[i]==rightPoint->col)) {
            rightIndex = i;
            break;
        }
    }
    
    /*using right point index to find right upper boundary index and right lower boundary index*/
    int rightUpperIndex=0, rightLowerIndex=0;
    for (int i = 0; i<(int)breakPoints->used; i++) {
        if (rightIndex<breakPoints->array[i] && i>0) {
            rightUpperIndex = breakPoints->array[i];
            rightLowerIndex = breakPoints->array[i-1];
            break;
        }
    }
    //left safe boundary
    int leftSafeCount = 0;
    for (int i=0; i<(int)contourRows->used; i++) {
        if (i<leftUpperIndex||i>leftLowerIndex) {
            insertStnArray(safeRows, contourRows->array[i]);
            insertStnArray(safeCols, contourCols->array[i]);
            leftSafeCount += 1;
        }
    }

    //right safe boundary
    for (int i=rightLowerIndex; i<=rightUpperIndex; i++) {
            insertStnArray(safeRows, contourRows->array[i]);
            insertStnArray(safeCols, contourCols->array[i]);
    }
    if (rightIndex == 0) {
        insertStnArray(safeRows, rightPoint->row);
        insertStnArray(safeCols, rightPoint->col);
    }
    
}

/**
 *version 1.1
 *stnEllipseFitting
 */
void stnEllipseFitting(stnArray *pointRows, stnArray *pointCols, stnPoint *centerPoint, double parameters[6]){
    int i;
    int length = (int)pointRows->used;
    double **D=(double **)matrix(6, length, 0, 0, sizeof(double));
    double *eigenParameter;
    double **squareMatrix=(double **)matrix(6, 6, 0, 0, sizeof(double));//a matrix that d*d'(aka the square matrix used in least square, inverse)
    
    
    for (i=0; i<length; i++) {
        D[0][i]=pow((double)pointCols->array[i] - centerPoint->col,2);
        D[1][i]=((double)pointCols->array[i] - centerPoint->col)* ((double)pointRows->array[i] - centerPoint->row);
        D[2][i]=pow((double)pointRows->array[i] - centerPoint->row,2);
        D[3][i]=(double)pointCols->array[i] - centerPoint->col;
        D[4][i]=(double)pointRows->array[i] - centerPoint->row;
        D[5][i]=1;
    }
    stnMatrixSquare(6, length, D, squareMatrix);
    stnMatrixInverse(6, squareMatrix);
    
//    double C[6][6]={{0,0,2,0,0,0},
//                    {0,-1,0,0,0,0},
//                    {2,0,0,0,0,0},
//                    {0,0,0,0,0,0},
//                    {0,0,0,0,0,0},
//                    {0,0,0,0,0,0}};
    double **C=(double **)matrix(6, 6, 0, 0, sizeof(double));
    for (i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            C[i][j]=0;
        }
    }
    C[0][2]=2;C[2][0]=2;C[1][1]=-1;
    double **pMatrix=(double **)matrix(6, 6, 0, 0, sizeof(double));
    stnMatrixMultiply(6, 6, 6, squareMatrix, C, pMatrix);
    eigenParameter=stnEigenVector(6, pMatrix);
    double a=eigenParameter[0],b=eigenParameter[1]/2,c=eigenParameter[2],d=eigenParameter[3]/2,e=eigenParameter[4]/2,f=eigenParameter[5];
    double det0,detm,l1,l2;
    double ra,rb,radius,centerX,centerY;
    detm = a*c-b*b;
    det0 = a*c*f+d*b*e+e*d*b-a*e*e-c*d*d-f*b*b;
    l2=(-(a+c)+sqrt(pow(a+c, 2)-4*(a*c-b*b)))/2;
    l1=(-(a+c)-sqrt(pow(a+c, 2)-4*(a*c-b*b)))/2;
    
    ra=sqrt((det0/detm)/l1);
    rb=sqrt((det0/detm)/l2);
    radius = (ra+rb)/2;
    centerX = (4*b*e-c*d)/(4*a*c-4*pow(b, 2))+centerPoint->col;
    centerY = (4*b*d-a*e)/(4*a*c-4*pow(b, 2))+centerPoint->row;
    
    parameters[0]=centerY;
    parameters[1]=centerX;
    parameters[2]=rb;
    parameters[3]=ra;
    parameters[4]=radius;
    parameters[5]=atan2(2, (a-c)/(2*b));
    free(eigenParameter);
    freeStnMatrix((void**)D);
    freeStnMatrix((void**)squareMatrix);
    freeStnMatrix((void**)C);
    freeStnMatrix((void**)pMatrix);
}

/**
 *version 1.1
 *stnCircleFitting
 */
void stnCircleFitting(stnArray *pointRows, stnArray *pointCols, double parameters[3]){
    int i;
    
    int length = (int)pointRows->used;
    double **d=(double **)matrix(3, length, 0, 0, sizeof(double));//a matrix of [x; y; 1];
    double **squareMatrix=(double **)matrix(3, 3, 0, 0, sizeof(double));//a matrix that d*d'(aka the square matrix used in least square, inverse)
    double **rMatrix=(double **)matrix(length, 1, 0, 0, sizeof(double));
    for (i=0; i<length; i++) {
        d[0][i]=(double)pointCols->array[i];
        d[1][i]=(double)pointRows->array[i];
        d[2][i]=1;
        rMatrix[i][0] = pow((double)pointCols->array[i], 2) + pow((double)pointRows->array[i], 2);
    }


    stnMatrixSquare(3, length, d,squareMatrix);
    
    /*inverse correct*/
    stnMatrixInverse(3, squareMatrix);

    
    double **pMatrix=(double **)matrix(3, length, 0, 0, sizeof(double));// the matrix (U'*U)\U'
    stnMatrixMultiply(3, length, 3, squareMatrix, d, pMatrix);

    double **paraMatrix=(double **)matrix(3, 1, 0, 0, sizeof(double));;
    stnMatrixMultiply(3, 1, length, pMatrix, rMatrix, paraMatrix);

    double xhat = paraMatrix[0][0]/2;//col
    double yhat = paraMatrix[1][0]/2;//row
    
    parameters[0]=yhat;
    parameters[1]=xhat;
    parameters[2]=sqrt(pow(xhat, 2)+pow(yhat, 2)+paraMatrix[2][0]);
    
    freeStnMatrix((void**)d);
    freeStnMatrix((void**)squareMatrix);
    freeStnMatrix((void**)rMatrix);
    freeStnMatrix((void**)pMatrix);
    freeStnMatrix((void**)paraMatrix);
//    printf("regular cx: %d, cy: %d, r: %d\n",centerX,centerY,radius);
    
}
/** unused
 *stnMLSCircleFitting
 *Modified Least Squares Method
 */
void stnMLSCircleFitting(stnArray *pointRows, stnArray *pointCols, double parameters[3]){
    int i;
    double length = (double)pointRows->used;//    n
    double A,B,C,D,E;
    
    /*******for loop version*******/
    double sx=0,sx2=0,sy=0,sy2=0,sxy=0,sxy2=0,sx2y=0,sx3=0,sy3=0;
    for (i=0; i<(int)length; i++) {
        double x=(double)pointCols->array[i];
        double y=(double)pointRows->array[i];
        
        sx += x;
        sy += y;
        sx2 += pow(x, 2);
        sy2 += pow(y, 2);
        sxy += x*y;
        sxy2 += x*y*y;
        sx2y += x*x*y;
        sx3 += pow(x, 3);
        sy3 += pow(y, 3);
    }
    A = length*sx2 - pow(sx, 2);
    B = length*sxy - sx*sy;
    C = length*sy2 - pow(sy, 2);
    D = 0.5*(length*sxy2-sx*sy2+length*sx3-sx*sx2);
    E = 0.5*(length*sx2y-sy*sx2+length*sy3-sy*sy2);
    
    /*
    
    A = length*sumSquaredStnArray(pointCols)-pow(sumStnArray(pointCols), 2);
    B = length*sumProductStnArray(pointCols, 1, pointRows, 1)-sumStnArray(pointRows)*sumStnArray(pointCols);
    C = length*sumSquaredStnArray(pointRows)-pow(sumStnArray(pointRows), 2);
    D = 0.5*(length*sumProductStnArray(pointCols, 1, pointRows, 2) - sumStnArray(pointCols)*sumSquaredStnArray(pointRows)+length*sumPoweredStnArray(pointCols, 3)-sumSquaredStnArray(pointCols)*sumSquaredStnArray(pointCols));
    E = 0.5*(length*sumProductStnArray(pointRows, 1, pointCols, 2) - sumStnArray(pointRows)*sumSquaredStnArray(pointCols)+length*sumPoweredStnArray(pointRows, 3)-sumSquaredStnArray(pointRows)*sumSquaredStnArray(pointRows));
    */
    double centerCol = (D*C-B*E)/(A*C-B*B);
    double centerRow = (A*E-B*D)/(A*C-B*B);
    centerRow=centerRow;
    centerCol=centerCol;
    double radius = 0;
    for (i=0; i<length; i++) {
        if (i<(int)pointCols->used) {
            radius += sqrt(pow((double)pointCols->array[i]-centerCol, 2)+pow((double)pointRows->array[i]-centerRow, 2));
        }
    }
    radius = radius/((double)length);
    
    int radiusInt = round(radius);
    int centerX = round(centerCol);
    int centerY = round(centerRow);
    parameters[0]=centerY;
    parameters[1]=centerX;
    parameters[2]=radiusInt;
}
/**
 *stnCirclePoints
 */
void stnCirclePoints(stnArray *pointRows, stnArray *pointCols, double parameters[3]){
    int i;
    for (i=0; i<1000*2*PI; i++) {
        int col = parameters[2]*cos(((double)i)/1000) + parameters[1];
        int row = parameters[2]*sin(((double)i)/1000) + parameters[0];
        insertStnArray(pointRows, row);
        insertStnArray(pointCols, col);
    }
}
/**
 *stnEllipsePoints
 */
void stnEllipsePoints(stnArray *pointRows, stnArray *pointCols, double parameters[6]){
    int i;
    for (i=0; i<1000*2*PI; i++) {
        int col = parameters[1]+cos(parameters[5])*parameters[2]*cos(((double)i)/1000)-sin(parameters[5])*parameters[3]*sin(((double)i)/1000);
        int row = parameters[0]+sin(parameters[5])*parameters[2]*cos(((double)i)/1000)+cos(parameters[5])*parameters[3]*sin(((double)i)/1000);
        insertStnArray(pointRows, row);
        insertStnArray(pointCols, col);
    }
}

/**
 *stnDrawPoints
 */
void stnDrawPoints(stnArray *pointRows, stnArray *pointCols, unsigned char **inputImg, int nrows, int ncols, double **outputImg){
    int i,j,row,col;
    
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            outputImg[i][j]=((double)inputImg[i][j])/255;
        }
    }
    for (i=0; i<(int)pointCols->used; i++) {
        row = pointRows->array[i];
        col = pointCols->array[i];
        if (row>=0 && row<nrows && col>=0 && col<ncols) {
            outputImg[row][col]=1;
        }
    }
}

void stnDrawColorPoints(stnArray *pointRows, stnArray *pointCols, double **inputImg, int nrows, int ncols, double color[3]){
    int i,row,col;
    for (i=0; i<(int)pointCols->used; i++) {
        row = pointRows->array[i];
        col = pointCols->array[i];
        if (row>=0 && row<nrows && col>=0 && col<ncols) {
            
            inputImg[row][3*col]=color[0];
            inputImg[row][3*col+1]=color[1];
            inputImg[row][3*col+2]=color[2];
        }
    }
}

void stnGray2RGB(double **inputImg, int nrows, int ncols, double **outputImg){
    int i,j;
    for (i=0; i<nrows; i++) {
        for (j=0; j<ncols; j++) {
            outputImg[i][3*j] = inputImg[i][j];
            outputImg[i][3*j+1] = inputImg[i][j];
            outputImg[i][3*j+2] = inputImg[i][j];
        }
    }
}




/**
 *version 0.9
 *stnDynamicThreshold
 *version 2.0
 */
double stnDynamicThreshold2(unsigned char **inputImg,int nrows,int ncols, stnPoint *centerPoint){
    int i, j, midY;
    double **interImg =(double **)matrix(nrows, ncols, 0, 0, sizeof(double));
    imgChar2Double(inputImg, nrows, ncols, interImg);
    
    midY = (centerPoint->row<=0||centerPoint->row>nrows)?nrows/2:centerPoint->row;
    
    /*middle line in horizontal way*/
    double *midLine = (double *)malloc(ncols*sizeof(double));
    for (i=0; i<ncols; i++) {
        midLine[i] = 255*interImg[midY][i];
    }
//    double *midSmooth = movingWindowSmooth(midLine, 3, ncols);
    
    double *integration = (double *)malloc(256*sizeof(double));
    for (i=0; i<256; i++) {
        double sum = 0;
        for (j=0; j<ncols; j++) {
            if (midLine[j]<i+1) {
                sum++;
            }
        }
        integration[i] = sum;
    }
    double *smoothIntegration = movingWindowSmooth(integration, 3, 256);
    double *deriv = firstDev(smoothIntegration, 256);
    double maxDev = 0;
    for (i=0; i<256; i++) {
        maxDev = max(deriv[i], maxDev);
    }
    for (i=0; i<256; i++) {
        deriv[i] = maxDev - deriv[i];
    }
    
    stnArray peaks;
    initStnArray(&peaks, 1);
    detect_peak(deriv, 256, &peaks, maxDev/15, 0, 0);
//        for (i = 0; i<(int)peaks.used; i++) {
//            printf("%d\n",peaks.array[i]);
//        }

    double decision = ((double)peaks.array[2])/256;
    if (decision>0.18) {
        decision = ((double)peaks.array[1])/256;
    }
//    printf("decison: %f\n",decision);
    freeStnArray(&peaks);
    freeStnMatrix((void**)interImg);
    free(midLine);
    free(integration);
    free(smoothIntegration);
    free(deriv);
    return decision;
}

















