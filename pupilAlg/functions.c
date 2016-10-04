//
//  functions.c
//  ImageSnake
//
//  Created by st chen on 16/6/14.
//  Copyright © 2016年 Star Chen. All rights reserved.
//

#include "functions.h"

#include <stdio.h>
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

