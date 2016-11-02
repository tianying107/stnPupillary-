//
//  functions.c
//  ImageSnake
//
//  Created by st chen on 16/6/14.
//  Copyright © 2016 Siteng Chen. All rights reserved.
//  For Visual and Autonomous Exploration Systems Research Laboratory, University of Arizona
//  Prof. Wolfgang Fink
//

#include "functions.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h> /* needed for malloc(), exit() */
#include <string.h> /* needed for strcmp() */

/* SKIP COMMENT */

/* This function skips past a comment in a file. The comment */
/* begins with a '#' character and ends with a newline character. */
/* The function returns EOF if there's an error. */

int skipcomment(FILE *fp)
{
    int i;
    
    if((i = getc(fp)) == '#')
        while((i = getc(fp)) != '\n' && i != EOF);
    return(ungetc(i, fp));
}

/* READ PGM HEADER */

/* This function reads the header of a PGM image. */
/* The dimensions are returned as arguments. */
/* This function ensures that there's no more than 8 bpp. */
/* The return value is negative if there's an error. */

int read_pgm_hdr(FILE *fp, int *nrows, int *ncols)
{
    char filetype[3];
    int maxval;
    
    if(skipcomment(fp) == EOF
       || fscanf(fp, "%2s\n", filetype) != 1
       || strcmp(filetype, "P5")
       || skipcomment(fp) == EOF
       || fscanf(fp, "%d", ncols) != 1
       || skipcomment(fp) == EOF
       || fscanf(fp, "%d", nrows) != 1
       || skipcomment(fp) == EOF
       || fscanf(fp, "%d%*c", &maxval) != 1
       || maxval > 255)
        return(-1);
    else return(0);
}


/* ERROR HANDLER */

void error(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
    exit(1);
}


/* DYNAMICALLY ALLOCATE A PSEUDO 2-D ARRAY */

/* This function allocates a pseudo 2-D array of size nrows x ncols. */
/* The coordinates of the first pixel will be first_row_coord and */
/* first_col_coord. The data structure consists of one contiguous */
/* chunk of memory, consisting of a list of row pointers, followed */
/* by the array element values. */
/* Assumption:  nrows*ncols*element_size, rounded up to a multiple  */
/* of sizeof(long double), must fit in a long type.  If not, then */
/* the "i += ..." step could overflow.                              */

void **matrix(int nrows, int ncols, int first_row_coord,
              int first_col_coord, int element_size)
{
    void **p;
    int alignment;
    long i;
    
    if(nrows < 1 || ncols < 1) return(NULL);
    i = nrows*sizeof(void *);
    /* align the addr of the data to be a multiple of sizeof(long double) */
    alignment = i % sizeof(long double);
    if(alignment != 0) alignment = sizeof(long double) - alignment;
    i += nrows*ncols*element_size+alignment;
    if((p = (void **)malloc((size_t)i)) != NULL)
    {
        /* compute the address of matrix[first_row_coord][0] */
        p[0] = (char *)(p+nrows)+alignment-first_col_coord*element_size;
        for(i = 1; i < nrows; i++)
        /* compute the address of matrix[first_row_coord+i][0] */
            p[i] = (char *)(p[i-1])+ncols*element_size;
        /* compute the address of matrix[0][0] */
        p -= first_row_coord;
    }
    return(p);
}

void freeStnMatrix(void** matrix){
    free(matrix);
}
/* REFLECT AN IMAGE ACROSS ITS BORDERS */

/* The parameter "amount" tells the number of rows or columns to be */
/* reflected across each of the borders. */
/* It is assumed that the data type is unsigned char. */
/* It is assumed that the array was allocated to be of size at least */
/* (nrows+2*amount) by (ncols+2*amount), and that the image was loaded */
/* into the middle portion of the array, with coordinates, */
/* 	0 <= row < nrows, 0 <= col < ncols */
/* thereby leaving empty elements along the borders outside the image */
/* The "reflect" function will then fill in those empty */
/* elements along the borders with the reflected image pixel values. */
/* For example, x[0][-1] will be assigned the value of x[0][0], */
/* and x[0][-2] will be assigned the value of x[0][1], if amount=2. */

void reflect(double **matrix, int nrows, int ncols, int amount)
{
    int i, j;
    
    if(matrix == NULL || nrows < 1 || ncols < 1 || amount < 1
       || amount > (nrows+1)/2 || amount > (ncols+1)/2)
        error("reflect: bad args");
    
    for(i = -amount; i < 0; i++)
    {
        for(j = -amount; j < 0; j++)
            matrix[i][j] = matrix[-i-1][-j-1];
        for(j = 0; j < ncols; j++)
            matrix[i][j] = matrix[-i-1][j];
        for(j = ncols; j < ncols+amount; j++)
            matrix[i][j] = matrix[-i-1][ncols+ncols-j-1];
    }
    for(i = 0; i < nrows; i++)
    {
        for(j = -amount; j < 0; j++)
            matrix[i][j] = matrix[i][-j-1];
        for(j = ncols; j < ncols+amount; j++)
            matrix[i][j] = matrix[i][ncols+ncols-j-1];
    }
    for(i = nrows; i < nrows+amount; i++)
    {
        for(j = -amount; j < 0; j++)
            matrix[i][j] = matrix[nrows+nrows-i-1][-j-1];
        for(j = 0; j < ncols; j++)
            matrix[i][j] = matrix[nrows+nrows-i-1][j];
        for(j = ncols; j < ncols+amount; j++)
            matrix[i][j] = matrix[nrows+nrows-i-1][ncols+ncols-j-1];
    }
}
/* Matrix Inverse Function max order: 250*/
void inverseMat(int points, double Amat[points][points]){
    double a[250][250], d;
    double inv[points][points];
    int i, j, n;
    n = points;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            
            a[i][j] = Amat[i][j];
//            printf("%lf\t", a[i][j]);
        }
//        printf("\n");
    }
    d = detrminant(a, n);
//    printf("\nTHE DETERMINANT IS=%lf", d);
    if (d == 0)
        printf("\nMATRIX IS NOT INVERSIBLE\n");
    else
        cofactors(a, n, inv);
    
//    printf("\nTHE INVERSE OF THE MATRIX:\n");
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            Amat[i][j] = inv[i][j];
//            printf("\t%2f", Amat[i][j]);
        }
//        printf("\n");
    }
    
}
double detrminant(double a[250][250], int k) {
    double s = 1, det = 0, b[250][250];
    int i, j, m, n, c;
    if (k == 1) {
        return (a[0][0]);
    } else {
//        printf("%d\n",k);
        det = 0;
        for (c = 0; c < k; c++) {
            m = 0;
            n = 0;
            for (i = 0; i < k; i++) {
                for (j = 0; j < k; j++) {
                    b[i][j] = 0;
                    if (i != 0 && j != c) {
                        b[m][n] = a[i][j];
                        if (n < (k - 2))
                            n++;
                        else {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            det = det + s * (a[0][c] * detrminant(b, k - 1));
            s = -1 * s;
        }
    }
    return (det);
}
void cofactors(double num[250][250], int f, double inv[f][f]) {
    double b[250][250], fac[250][250];
    int p, q, m, n, i, j;
    for (q = 0; q < f; q++) {
        for (p = 0; p < f; p++) {
            m = 0;
            n = 0;
            for (i = 0; i < f; i++) {
                for (j = 0; j < f; j++) {
                    b[i][j] = 0;
                    if (i != q && j != p) {
                        b[m][n] = num[i][j];
                        if (n < (f - 2))
                            n++;
                        else {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            fac[q][p] = pow(-1, q + p) * detrminant(b, f - 1);
        }
    }
    trans(num, fac, f,inv);
}
void trans(double num[250][250], double fac[250][250], int r, double inv[r][r])

{
    int i, j;
    double b[250][250], d;//, inv[r][r]
    for (i = 0; i < r; i++) {
        for (j = 0; j < r; j++) {
            b[i][j] = fac[j][i];
        }
    }
    
    d = detrminant(num, r);
    //    inv[i][j] = 0;
    for (i = 0; i < r; i++) {
        for (j = 0; j < r; j++) {
            inv[i][j] = b[i][j] / d;
        }
    }
    
    
}

void converInd(unsigned char **x, int nrows, int ncols, double **intImage){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<ncols; j++) {
            intImage[i][j] = x[i][j];
            intImage[i][j] = (double)(intImage[i][j])/255;
        }
    }

}

void imgDouble2Char(double **doubleImage, int nrows, int ncols, unsigned char **charImage){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<ncols; j++) {
            int value = floor(doubleImage[i][j]*255);
            charImage[i][j] = value;
        }
    }
}

void imgInt2Char(int **intImage, int nrows, int ncols, unsigned char **charImage){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<ncols; j++) {
            int value = floor(intImage[i][j]*255);
            charImage[i][j] = value;
        }
    }
}
void imgInt2Double(int **intImage, int nrows, int ncols, double **doubleImage){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<ncols; j++) {
            doubleImage[i][j] = intImage[i][j];
        }
    }
}

/*Dynamic grow array*/


void initStnArray(stnArray *a, size_t initialSize) {
    a->array = (int *)malloc(initialSize * sizeof(int));
    a->used = 0;
    a->size = initialSize;
}

void insertStnArray(stnArray *a, int element) {
    if (a->used == a->size) {
        a->size *= 2;
        a->array = (int *)realloc(a->array, a->size * sizeof(int));
    }
    a->array[a->used++] = element;
}

void freeStnArray(stnArray *a) {
    free(a->array);
    a->array = NULL;
    a->used = a->size = 0;
}

double sum(double x[], int arr_count){
    int i = 0;
    double my_sum = 0;
    
    for (i = 0; i < arr_count; i++){
        my_sum += x[i];
    }
    return my_sum;
}

double stnInterp2(int nrows, int ncols, int intMatrix[nrows][ncols], double row, double col){
    int x_min=0, x_max=7, y_min=0, y_max=7;
    double mf, nf; // fractional parts
    int m, n;// m and n are row and column indices
   
    // Find integer and fractional part of column index
    nf = (ncols-1) * (col - x_min) / (x_max - x_min);
    n = (int)nf;
    nf = nf - n;
    
    // Find integer and fractional part of row index
    mf = (nrows-1) * (row - y_min) / (y_max - y_min);
    m = (int)mf;
    mf = mf - m;
    
    // Calculate interpolated estimated
    double result = (1-nf)*(1-mf)*intMatrix[m][n] + nf*(1-mf)*intMatrix[m][n+1]
    + (1-nf)*mf*intMatrix[m+1][n] + nf*mf*intMatrix[m+1][n+1];
    
    return result;
    
}

/*detect peak*/
void detect_peak(
                const double*   data, /* the data */
                int             data_count, /* row count of data */
                stnArray*       peaks, /* emission peaks will be put here */
                double          delta, /* delta used for distinguishing peaks */
                double          threshold   /*threshold used for filter peaks below the value*/
)
{
    int     i;
    double  mx;
    int     mx_pos = 0;
    bool    positionUpdate=false;

    mx = threshold;
    
    for(i = 0; i < data_count; i++)
    {
        if(data[i] > mx)
        {
            mx_pos = i;
            mx = data[i];
            positionUpdate = true;
        }
        
        if(data[i] < mx - delta) {
            if(positionUpdate){
                insertStnArray(peaks, mx_pos);
                positionUpdate = false;
            }
            
            mx = max(data[i],threshold);
        }
    }
    
}

/*eigen vector and eigen value using power method*/
double *stnEigenVector(int nSize, double intMatrix[nSize][nSize]){
    int i,j,n;
    n=nSize;
    double z[nSize], e[nSize],zmax,emax;//,x[nSize];
    double *x = (double *)malloc(nSize*sizeof(double));
    x[0]=1;
    for (i=1; i<nSize; i++) {
        x[i]=0;
    }
    
    do
    {
        for(i=0; i<n; i++)
        {
            z[i]=0;
            for(j=0; j<n; j++)
            {
                z[i]=z[i]+intMatrix[i][j]*x[j];
            }
        }
        zmax=fabs(z[0]);
        for(i=1; i<n; i++)
        {
            if((fabs(z[i]))>zmax)
                zmax=fabs(z[i]);
        }
        for(i=0; i<n; i++)
        {
            z[i]=z[i]/zmax;
        }
        for(i=0; i<n; i++)
        {
            e[i]=0;
            e[i]=fabs((fabs(z[i]))-(fabs(x[i])));
        }
        emax=e[0];
        for(i=1; i<n; i++)
        {
            if(e[i]>emax)
                emax=e[i];
        }
        for(i=0; i<n; i++)
        {
            x[i]=z[i];
        }
    }while(emax>0.001);
    return x;
}

/**
 *do the matrix multiply U * U'
 */
void stnMatrixSquare(int nrows, int ncols, double matrix[nrows][ncols], double multiply[nrows][nrows]){
    int i, j, k;
    double sum = 0;
    
//    double multiply[nrows][nrows];
    
        
    for (i = 0; i < nrows; i++) {
        for (j = 0; j < nrows; j++) {
            for (k = 0; k < ncols; k++) {
                sum = sum + matrix[i][k]*matrix[j][k];
        }
            
            multiply[i][j] = sum;
            sum = 0;
        }
    }

}
/**
 *do the matrix multiply 
 */
void stnMatrixMultiply(int nrows1, int nrows2, int ncols, double matrix1[nrows1][ncols], double matrix2[ncols][nrows2], double multiply[nrows1][nrows2]){
    int i, j, k;
    double sum = 0;
    
    //    double multiply[nrows][nrows];
    
    
    for (i = 0; i < nrows1; i++) {
        for (j = 0; j < nrows2; j++) {
            for (k = 0; k < ncols; k++) {
                sum = sum + matrix1[i][k]*matrix2[k][j];
            }
            
            multiply[i][j] = sum;
            sum = 0;
        }
    }
    
    
    
}



/*
 Find the cofactor matrix of a square matrix
 */
void CoFactor(double **a,int n,double **b){
    int i,j,ii,jj,i1,j1;
    double det;
    double **c;
    
    c = malloc((n-1)*sizeof(double *));
    for (i=0;i<n-1;i++)
        c[i] = malloc((n-1)*sizeof(double));
    
    for (j=0;j<n;j++) {
        for (i=0;i<n;i++) {
            
            /* Form the adjoint a_ij */
            i1 = 0;
            for (ii=0;ii<n;ii++) {
                if (ii == i)
                    continue;
                j1 = 0;
                for (jj=0;jj<n;jj++) {
                    if (jj == j)
                        continue;
                    c[i1][j1] = a[ii][jj];
                    j1++;
                }
                i1++;
            }
            
            /* Calculate the determinate */
            det = Determinant(c,n-1);
            
            /* Fill in the elements of the cofactor */
            b[i][j] = pow(-1.0,i+j+2.0) * det;
        }
    }
    for (i=0;i<n-1;i++)
        free(c[i]);
    free(c);
}

/*
 Transpose of a square matrix, do it in place
 */
void Transpose(double **a,int n)
{
    int i,j;
    double tmp;
    
    for (i=1;i<n;i++) {
        for (j=0;j<i;j++) {
            tmp = a[i][j];
            a[i][j] = a[j][i];
            a[j][i] = tmp;
        }
    }
}
double Determinant(double **a,int n){
    int i,j,j1,j2;
    double det = 0;
    double **m = NULL;

    if (n < 1) { /* Error */

    } else if (n == 1) { /* Shouldn't get used */
        det = a[0][0];
    } else if (n == 2) {
        det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
    } else {
        det = 0;
        for (j1=0;j1<n;j1++) {
            m = malloc((n-1)*sizeof(double *));
            for (i=0;i<n-1;i++)
                m[i] = malloc((n-1)*sizeof(double));
            for (i=1;i<n;i++) {
                j2 = 0;
                for (j=0;j<n;j++) {
                    if (j == j1)
                        continue;
                    m[i-1][j2] = a[i][j];
                    j2++;
                }
            }
            det += pow(-1.0,j1+2.0) * a[0][j1] * Determinant(m,n-1);
            for (i=0;i<n-1;i++)
                free(m[i]);
            free(m);
        }
    }
    return(det);
}

void stnMatrixInverse(int nrows, double squareMatrix[nrows][nrows]){
    if (nrows==3) {
        double a11,a12,a13,a21,a22,a23,a31,a32,a33;
        a11 = squareMatrix[0][0];a12 = squareMatrix[0][1];a13 = squareMatrix[0][2];
        a21 = squareMatrix[1][0];a22 = squareMatrix[1][1];a23 = squareMatrix[1][2];
        a31 = squareMatrix[2][0];a32 = squareMatrix[2][1];a33 = squareMatrix[2][2];
        double deta = a11*a22*a33+a21*a32*a13+a31*a12*a23-a11*a32*a23-a31*a22*a13-a21*a12*a33;
        if (deta) {
            squareMatrix[0][0] = (a22*a33-a23*a32)/deta;
            squareMatrix[0][1] = (a13*a32-a12*a33)/deta;
            squareMatrix[0][2] = (a12*a23-a13*a22)/deta;
            squareMatrix[1][0] = (a23*a31-a21*a33)/deta;
            squareMatrix[1][1] = (a11*a33-a13*a31)/deta;
            squareMatrix[1][2] = (a13*a21-a11*a23)/deta;
            squareMatrix[2][0] = (a21*a32-a22*a31)/deta;
            squareMatrix[2][1] = (a12*a31-a11*a32)/deta;
            squareMatrix[2][2] = (a11*a22-a12*a21)/deta;
        }
    }
    else if (nrows<25){
        int i,j;
        double **squarePoint =(double **)matrix(nrows, nrows, 0, 0, sizeof(double));
        for (i=0; i<nrows; i++) {
            for (j=0; j<nrows; j++) {
                squarePoint[i][j]=squareMatrix[i][j];
            }
        }
        
        
        double det = Determinant(squarePoint, nrows);
        double **bMatrix =(double **)matrix(nrows, nrows, 0, 0, sizeof(double));
        if (det!=0) {
            CoFactor(squarePoint, nrows, bMatrix);
//            Transpose(bMatrix, nrows);
            for (i=0; i<nrows; i++) {
                for (j=0; j<nrows; j++) {
                    squareMatrix[i][j]=bMatrix[j][i]/det;
                }
            }
        }
    }
    
}

void imgCombineDouble2Char(double **doubleImage1, double **doubleImage2, int nrows, int ncols, unsigned char **charImage){
    for (int i = 0; i<nrows; i++) {
        for (int j = 0; j<ncols; j++) {
            int value1 = floor(doubleImage1[i][j]*255);
            int value2 = floor(doubleImage2[i][j]*255);
            charImage[i][j] = value1;
            charImage[i][j+ncols]=value2;
        }
    }
}
