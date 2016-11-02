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
//            printf("col1:%d, col2:%d\n",j,j+newWidth);
            img1[i][j] = combinedImg[i][j];
            img2[i][j] = combinedImg[i][j+newWidth];
        }
    }
}

void imageHistogramEqualization(unsigned char **inputImg, int nrows, int ncols, int lower, int upper, double **outputImg){
    int i, j;
    int Num = 0;
    //calculate the hist
    double h[256];
    for (i=0; i<256; i++) {
        h[i]=0;
    }
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
                outputImg[i][j]=255;
            }
            else{
                outputImg[i][j]=(H[va]-H[XL])/((double)Num-H[XL]);
            }
        }
    }
}

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

///**
// *Growth Circle algorithm, an algorithm to label the largest blob
// *
// */
//bool growthCircle(stnPoint *centerPoint, int **inputImg, int nrows, int ncols){
//    bool allWhite = false;
//    int radius=0;
//    int i;
//    for (radius=0; radius<ncols; radius++) {
//        
//        for (i=0; i<500*PI; i++) {
//            int col = floor(radius*cos(((double)i)/1000) + centerPoint->col);
//            int row = floor(radius*sin(((double)i)/1000) + centerPoint->row);
//            int value1 = inputImg[row][col];
//            int value2 = inputImg[row][col-centerPoint->col];
//            int value3 = inputImg[row-centerPoint->row][col];
//            int value4 = inputImg[row-centerPoint->row][col-centerPoint->col];
//            if (!(value1||value2||value3||value4)) {
//                break;
//            }
//            
//            
//        }
//        
//        
//    }
//    
//    radius++;
//    return allWhite;
//}

/**
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
 *
 */
void growthCircle(stnPoint *centerPoint, int **inputImg, int nrows, int ncols){
    bool allWhite = false;
    int radius=0;
    int i,j,count;
    
    for (radius=50; radius<min(min(centerPoint->col, ncols-centerPoint->col), min(centerPoint->row, nrows-centerPoint->row)); radius++) {
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


/**
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

/**
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
 *stnMedianFilter is a 2-D median filter that only work on binary image
 */
void stnMedianFilter(int **inputImg, int nrows, int ncols, int filterWidth, int filterHeight){
    int edgeX, edgeY, i, j, filterX, filterY, sum;
    int **filteredImg;
    
    filteredImg = inputImg;
    edgeX = (filterWidth-1)/2;
    edgeY = (filterHeight-1)/2;
    for (i=edgeY; i<nrows-edgeY; i++) {
        for (j=edgeX; j<ncols-edgeX; j++) {
            sum = 0;
            for (filterY=i-edgeY; filterY<i+edgeY+1;filterY++) {
                for (filterX = j-edgeX; filterX<j+edgeX+1; filterX++) {
                    sum += inputImg[filterY][filterX];
                }
            }
            filteredImg[i][j]=(int)(sum/ceil((((float)(filterWidth*filterHeight))/2)));
        }
    }
    inputImg = filteredImg;
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
//    printf("center at row:%d col:%d\n",centerPoint->row,centerPoint->col);
}

/**
 *stnBoundaryPoints
 */
void stnBoundaryPoint(int **inputImg, int nrows, int ncols, stnPoint *centerPoint, stnPoint *leftPoint, stnPoint *rightPoint){
    int x = centerPoint->col;
    int y = centerPoint->row;
    int windowSize = 30;
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
    while (sum<windowSize && x) {
        x++;
        sum=0;
        for (int i=0; i<windowSize && x+i<ncols; i++) {
            sum += inputImg[y][x+i];
        }
    }
    rightPoint->col = x-1;
    rightPoint->row = y;
}

/**
 *stnContourBound
 */
void stnContourBound(int **inputImg, int nrows, int ncols, stnPoint *leftPoint, stnArray *directionArray, stnArray *contourMapRow, stnArray *contourMapCol){
    int mapPlus2[8]={2,3,4,5,6,7,0,1};
    int mapMinus1[8]={7,0,1,2,3,4,5,6};
    int direction = 1; //initial direction is 1
    int x = leftPoint->col;
    int y = leftPoint->row;
    initStnArray(directionArray, 1);
    initStnArray(contourMapRow, 1);
    initStnArray(contourMapCol, 1);
    
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
    return curvature;
}

/**
 *stnSafePoints
 */
void stnSafePoints(stnArray *contourRows, stnArray *contourCols, stnArray *breakPoints, stnPoint *rightPoint, stnArray *safeRows, stnArray *safeCols){
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
    
    for (int i=0; i<(int)contourRows->used; i++) {
        if (i<leftUpperIndex||i>leftLowerIndex||i>rightLowerIndex||i<rightUpperIndex) {
            insertStnArray(safeRows, contourRows->array[i]);
            insertStnArray(safeCols, contourCols->array[i]);
        }
    }
}

/**
 *stnEllipseFitting
 */
void stnEllipseFitting(stnArray *pointRows, stnArray *pointCols, stnPoint *centerPoint, int parameters[5]){
    int i;
    int length = (int)pointRows->used;
    double D[6][length];
    double *eigenParameter;
    double squareMatrix[6][6];//a matrix that d*d'(aka the square matrix used in least square, inverse)
    
    
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
    
    double C[6][6]={{0,0,2,0,0,0},
                    {0,-1,0,0,0,0},
                    {2,0,0,0,0,0},
                    {0,0,0,0,0,0},
                    {0,0,0,0,0,0},
                    {0,0,0,0,0,0}};
    double pMatrix[6][6];
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
}

/**
 *stnCircleFitting
 */
void stnCircleFitting(stnArray *pointRows, stnArray *pointCols, int parameters[3]){
    int i;
    
    int length = (int)pointRows->used;
    double d[3][length];//a matrix of [x; y; 1];
    double squareMatrix[3][3];//a matrix that d*d'(aka the square matrix used in least square, inverse)
    double rMatrix[length][1];
    for (i=0; i<length; i++) {
        d[0][i]=(double)pointCols->array[i];
        d[1][i]=(double)pointRows->array[i];
        d[2][i]=1;
        rMatrix[i][0] = pow((double)pointCols->array[i], 2) + pow((double)pointRows->array[i], 2);
    }


    stnMatrixSquare(3, length, d,squareMatrix);
    
    /*inverse correct*/
    stnMatrixInverse(3, squareMatrix);

    
    double pMatrix[3][length];// the matrix (U'*U)\U'
    stnMatrixMultiply(3, length, 3, squareMatrix, d, pMatrix);

    double paraMatrix[3][1];
    stnMatrixMultiply(3, 1, length, pMatrix, rMatrix, paraMatrix);

    double xhat = round(paraMatrix[0][0]/2);//col
    double yhat = round(paraMatrix[1][0]/2);//row
    
    int radius = round(sqrt(pow(xhat, 2)+pow(yhat, 2)+paraMatrix[2][0]));
    int centerX = round(xhat);
    int centerY = round(yhat);
    parameters[0]=centerY;
    parameters[1]=centerX;
    parameters[2]=radius;
    
}

/**
 *stnCirclePoints
 */
void stnCirclePoints(stnArray *pointRows, stnArray *pointCols, int parameters[3]){
    int i;
    for (i=0; i<1000*2*PI; i++) {
        int col = parameters[2]*cos(((double)i)/1000) + parameters[1];
        int row = parameters[2]*sin(((double)i)/1000) + parameters[0];
        insertStnArray(pointRows, row);
        insertStnArray(pointCols, col);
    }
}

/**
 *stnCirclePoints
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
